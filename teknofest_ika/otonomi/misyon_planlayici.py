#!/usr/bin/env python3
"""
misyon_planlayici.py  —  LYDİA Waypoint + Misyon Planlayıcı Node
=================================================================

Görev:
  /yolo/trigger topic'ini dinler, gelen sınıf adına göre bir sonraki
  aşamaya geçer ve Nav2'ye waypoint hedefi gönderir.

Protokol:
  /yolo/trigger  (std_msgs/String)  → Görüntü ekibinden tabela sınıfı
  /fsm_state     (std_msgs/String)  → Mevcut aşama adı (log/izleme için)
  /cmd_vel       (geometry_msgs/Twist) → Dur komutu (acil stop / timeout)

Akış:
  IDLE → [otonom_baslat servisi] → ASAMA_1 → Nav2 goal → goal tamam
       → /yolo/trigger bekle → ASAMA_2 → Nav2 goal → ... → FINISH

Pas Geçme Mantığı:
  Eğer bir aşama asama_timeout_saniye içinde tamamlanamazsa ve
  pas_gecilir: true ise bir sonraki aşamaya geçilir.
  pas_gecilir: false ise araç yerinde bekler, log yazılır.

KURULUM:
  sudo apt install ros-humble-nav2-msgs
  Waypoint dosyasını setup.py data_files'a ekle.

ÇALIŞTIRMA:
  ros2 run teknofest_ika misyon_planlayici \
       --ros-args -p waypoint_dosyasi:=config/waypoints.yaml

MOCK TEST (donanım olmadan):
  Terminal 1: ros2 run teknofest_ika misyon_planlayici
  Terminal 2: ros2 service call /otonom_baslat std_srvs/srv/Trigger
  Terminal 3: ros2 topic pub /yolo/trigger std_msgs/msg/String \
              "{data: 'su_gecisi'}" --once
"""

import math
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import yaml


# ─── Durum Sabitleri ────────────────────────────────────────────────────────
class Durum:
    IDLE      = "IDLE"
    BEKLIYOR  = "BEKLIYOR"    # Nav2 hedefe gidiyor
    TETIK_BEK = "TETIK_BEK"  # Yolo trigger bekleniyor
    FINISH    = "FINISH"
    HATA      = "HATA"


class MisyonPlanlayici(Node):

    def __init__(self):
        super().__init__('misyon_planlayici')

        # ── Parametreler ───────────────────────────────────────────────────
        self.declare_parameter('waypoint_dosyasi', 'config/waypoints.yaml')
        self.declare_parameter('sim_mode', False)

        wp_dosya = self.get_parameter('waypoint_dosyasi').value
        self._sim_mode = self.get_parameter('sim_mode').value

        # ── Waypoint yükleme ───────────────────────────────────────────────
        self._asamalar = []
        self._parametreler = {}
        self._waypoint_yukle(wp_dosya)

        # ── Durum ─────────────────────────────────────────────────────────
        self._durum          = Durum.IDLE
        self._asama_indeks   = -1          # -1: henüz başlamadı
        self._asama_baslama  = None        # aşama başlangıç zamanı
        self._yolo_sayac     = {}          # {sınıf: ardışık frame sayısı}
        self._hedef_gonderildi = False
        self._lock = threading.Lock()

        # ── QoS ───────────────────────────────────────────────────────────
        qos_sub = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        qos_pub = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)

        # ── ROS arayüzleri ─────────────────────────────────────────────────
        self._fsm_pub   = self.create_publisher(String, '/fsm_state', qos_pub)
        self._dur_pub   = self.create_publisher(Twist, '/cmd_vel', qos_pub)

        self.create_subscription(String, '/yolo/trigger', self._yolo_cb, qos_sub)

        self._otonom_srv = self.create_service(
            Trigger, 'otonom_baslat', self._otonom_baslat_cb)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Periyodik kontrol (timeout, durum yayını)
        self.create_timer(1.0, self._periyodik_kontrol)

        self.get_logger().info(
            f'MisyonPlanlayici baslatildi. {len(self._asamalar)} asama yuklendi.')
        self._fsm_yayinla(Durum.IDLE)

    # ─── Waypoint Dosyası Yükleme ──────────────────────────────────────────
    def _waypoint_yukle(self, dosya_yolu: str):
        try:
            yol = Path(dosya_yolu)
            if not yol.is_absolute():
                # Workspace share dizinini dene
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('teknofest_ika')
                yol = Path(pkg) / dosya_yolu

            with open(yol, 'r', encoding='utf-8') as f:
                veri = yaml.safe_load(f)

            self._asamalar    = veri.get('asamalar', [])
            self._parametreler = veri.get('parametreler', {})
            self.get_logger().info(f'Waypoint dosyasi yuklendi: {yol}')

        except Exception as e:
            self.get_logger().error(f'Waypoint dosyasi yuklenemedi: {e}')
            self._asamalar = []
            self._parametreler = {}

    # ─── /otonom_baslat Servisi ────────────────────────────────────────────
    def _otonom_baslat_cb(self, req, res):
        with self._lock:
            if self._durum != Durum.IDLE:
                res.success = False
                res.message = f'Zaten calisiyor: {self._durum}'
                return res

            if not self._asamalar:
                res.success = False
                res.message = 'Waypoint dosyasi bos veya yuklenemedi.'
                return res

            bekleme = self._parametreler.get('baslangic_bekleme', 3.0)
            self.get_logger().info(
                f'Otonom mod {bekleme:.0f}s icinde basliyor...')
            threading.Thread(
                target=self._baslangic_gecikme,
                args=(bekleme,), daemon=True).start()

            res.success = True
            res.message = f'Otonom mod {bekleme:.0f}s sonra baslar.'
        return res

    def _baslangic_gecikme(self, bekleme: float):
        time.sleep(bekleme)
        with self._lock:
            self._asama_indeks = 0
            self._sonraki_asamaya_gec()

    # ─── /yolo/trigger Callback ────────────────────────────────────────────
    def _yolo_cb(self, msg: String):
        sinif = msg.data.strip().lower()

        with self._lock:
            if self._durum != Durum.TETIK_BEK:
                return

            if self._asama_indeks < 0 or self._asama_indeks >= len(self._asamalar):
                return

            beklenen = self._asamalar[self._asama_indeks]['yolo_sinifi'].lower()
            if sinif != beklenen:
                # Yanlış sınıf — sayacı sıfırla
                if sinif in self._yolo_sayac:
                    del self._yolo_sayac[sinif]
                return

            # Ardışık frame sayacı
            self._yolo_sayac[sinif] = self._yolo_sayac.get(sinif, 0) + 1
            esik = self._parametreler.get('ardisik_frame_sayisi', 3)

            self.get_logger().debug(
                f'[YOLO] {sinif}: {self._yolo_sayac[sinif]}/{esik}')

            if self._yolo_sayac[sinif] >= esik:
                self._yolo_sayac = {}
                self.get_logger().info(
                    f'[YOLO] Tetiklendi: {sinif} — sonraki asamaya geciliyor')
                self._asama_indeks += 1
                self._sonraki_asamaya_gec()

    # ─── Aşama Geçişi ─────────────────────────────────────────────────────
    def _sonraki_asamaya_gec(self):
        """
        Mevcut _asama_indeks'e karşılık gelen waypoint'i Nav2'ye gönderir.
        _lock alınmış halde çağrılmalı.
        """
        if self._asama_indeks >= len(self._asamalar):
            self._durum = Durum.FINISH
            self._fsm_yayinla(Durum.FINISH)
            self.get_logger().info('TUM ASAMALAR TAMAMLANDI — FINISH')
            self._dur_komutu_gonder()
            return

        asama = self._asamalar[self._asama_indeks]
        self._durum         = Durum.BEKLIYOR
        self._asama_baslama = self.get_clock().now()
        self._hedef_gonderildi = False
        self._yolo_sayac    = {}

        self._fsm_yayinla(asama['isim'])
        self.get_logger().info(
            f'[ASAMA {self._asama_indeks + 1}/{len(self._asamalar)}] '
            f'{asama["isim"]} — Nav2 hedef gonderiliyor')

        # Nav2'ye gönderimi ayrı thread'de yap (action client blocking)
        threading.Thread(
            target=self._nav2_gonder,
            args=(asama,), daemon=True).start()

    # ─── Nav2 Hedef Gönderimi ──────────────────────────────────────────────
    def _nav2_gonder(self, asama: dict):
        if self._sim_mode:
            self.get_logger().warn(
                f'[SIM] Nav2 hedef: {asama["isim"]} '
                f'({asama["waypoint"]["x"]:.1f}, {asama["waypoint"]["y"]:.1f})')
            time.sleep(3.0)   # Simülasyon: 3 saniyede "ulaşıldı"
            with self._lock:
                if self._durum == Durum.BEKLIYOR:
                    self._durum = Durum.TETIK_BEK
                    self.get_logger().info(
                        f'[SIM] Hedefe ulasildi: {asama["isim"]} — trigger bekleniyor')
            return

        # Nav2 action server hazır mı?
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server bulunamadi!')
            with self._lock:
                self._durum = Durum.HATA
            return

        # Hedef pose oluştur
        hedef = NavigateToPose.Goal()
        hedef.pose = self._pose_olustur(
            asama['waypoint']['x'],
            asama['waypoint']['y'],
            asama['waypoint']['yaw'])

        tolerans = self._parametreler.get('hedef_tolerans_metre', 0.30)

        future = self._nav_client.send_goal_async(hedef)
        future.add_done_callback(
            lambda f: self._goal_kabul_cb(f, asama))

    def _goal_kabul_cb(self, future, asama: dict):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Nav2 hedef reddedildi: {asama["isim"]}')
            with self._lock:
                self._durum = Durum.HATA
            return

        self.get_logger().debug(f'Nav2 hedef kabul edildi: {asama["isim"]}')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._goal_sonuc_cb(f, asama))

    def _goal_sonuc_cb(self, future, asama: dict):
        result = future.result()
        status = result.status

        with self._lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Hedefe ulasildi: {asama["isim"]} — trigger bekleniyor')
                self._durum = Durum.TETIK_BEK
            else:
                self.get_logger().warn(
                    f'Nav2 hedef basarisiz (status={status}): {asama["isim"]}')
                # Başarısız olursa tetik beklemeye geç (pas geçme zaten timeout'ta
                if self._durum == Durum.BEKLIYOR:
                    self._durum = Durum.TETIK_BEK

    # ─── Periyodik Kontrol (Timeout) ───────────────────────────────────────
    def _periyodik_kontrol(self):
        with self._lock:
            if self._durum not in (Durum.BEKLIYOR, Durum.TETIK_BEK):
                return
            if self._asama_indeks < 0 or self._asama_baslama is None:
                return

            gecen = (self.get_clock().now() - self._asama_baslama).nanoseconds * 1e-9
            timeout = self._parametreler.get('asama_timeout_saniye', 120.0)

            if gecen > timeout:
                asama = self._asamalar[self._asama_indeks]
                if asama.get('pas_gecilir', False):
                    self.get_logger().warn(
                        f'TIMEOUT ({timeout:.0f}s): {asama["isim"]} atlaniyor')
                    self._asama_indeks += 1
                    self._sonraki_asamaya_gec()
                else:
                    self.get_logger().warn(
                        f'TIMEOUT ({timeout:.0f}s): {asama["isim"]} pas gecilemez, bekleniyor')
                    # Zaman damgasını sıfırla ki her saniye uyarmaya devam etmesin
                    self._asama_baslama = self.get_clock().now()

    # ─── Yardımcılar ──────────────────────────────────────────────────────
    def _pose_olustur(self, x: float, y: float, yaw: float) -> PoseStamped:
        """x, y [m] ve yaw [rad] → PoseStamped (map frame)"""
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # Yaw → quaternion (sadece z-w)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _fsm_yayinla(self, durum: str):
        msg = String()
        msg.data = durum
        self._fsm_pub.publish(msg)

    def _dur_komutu_gonder(self):
        msg = Twist()   # Tüm sıfır = dur
        self._dur_pub.publish(msg)


# ─── Main ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MisyonPlanlayici()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
