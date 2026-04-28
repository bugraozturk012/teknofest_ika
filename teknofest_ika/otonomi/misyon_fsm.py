#!/usr/bin/env python3
"""
misyon_fsm.py — LYDİA İKA Görev Yöneticisi (SMACH FSM)
=========================================================
v2.0 — Ekip Protokol v1.0 ile uyumlu

NE YAPAR:
  Parkur boyunca görev sırasını yönetir. Nav2'ye "şimdi şu noktaya git"
  komutunu verir. Atış bölgesinde durur, nişan alır, ateş eder.

  Görüntü ekibinden /ika/detections (JSON) alır:
    - hedef_var / hedef_hata_x / hedef_hata_y → nişan doğrulama
    - kayar_yon                                 → KAYAR_ENGEL geçiş kararı
    - tabela                                    → FSM bilgi (terrain_adapter halleder)

PROTOKOL ENTEGRASYONU (Ekip Protokol v1.0):
  Giriş : /ika/detections (JSON String) — Görüntü İşleme → Otomasyon
  Çıkış : Nav2 NavigateToPose action   — Otomasyon navigasyon
  Çıkış : /shoot_command (Bool)        — Otomasyon → Gömülü Sistemler köprüsü
           seri_kopru.py bu komutu LAZER,1 / LAZER,0 seri komutuna çevirir

ÖZEL DURUM MANTIĞI:
  DIK_EGIM waypoint : Şartname gereği hedefe varışta 2 saniye dur, sonra devam.
  KAYAR_ENGEL waypoint: kayar_yon != 'bilinmiyor' olana kadar bekle (max 10s).
  ATIS waypoint    : hedef_hata_x/y ±5 piksel toleransa girince ateş et.

DURUM DİYAGRAMI:
  [IDLE] → [NAVIGATE] → [SHOOT_APPROACH] → [SHOOT] → [NAVIGATE]
               ↓                                           ↓
         [ERROR_RECOVERY] ←─────────────────────────────── ┘
               ↓
         [MISSION_COMPLETE]

BAĞIMLILIKLAR:
  sudo apt install ros-humble-smach ros-humble-smach-ros
  pip3 install smach --break-system-packages

ÇALIŞTIRMA:
  ros2 run teknofest_ika misyon_fsm

TEST (kamerasız):
  ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
  ros2 topic pub /ika/detections std_msgs/msg/String \
    'data: "{\"tabela\":9,\"hedef_var\":true,\"hedef_hata_x\":2.0,\"hedef_hata_y\":1.0,\"kayar_yon\":\"sol\",\"fps\":30}"' \
    --rate 10
"""

import json
import math
import time
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool, String, Int16
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import smach


# ═══════════════════════════════════════════════════════════════════════════════
# DETECTIONS STORE — görüntü ekibinden gelen son paketi thread-safe tutar
# ═══════════════════════════════════════════════════════════════════════════════

class DetectionsStore:
    """
    /ika/detections topic'inden gelen son JSON paketini saklar.
    SMACH state'leri bu store'dan okur — doğrudan topic callback'e bağımlı değiller.
    threading.Lock ile thread-safe erişim sağlanır.
    """

    _DEFAULT = {
        'tabela':        0,
        'koni_var':      False,
        'koni_hata_x':   0.0,
        'kayar_engel_x': 0.5,
        'kayar_yon':     'bilinmiyor',
        'hedef_var':     False,
        'hedef_hata_x':  0.0,
        'hedef_hata_y':  0.0,
        'bariyer_sol_m': 1.5,
        'bariyer_sag_m': 1.5,
        'fps':           0.0,
    }

    def __init__(self):
        self._lock = threading.Lock()
        self._data = dict(self._DEFAULT)

    def update(self, data: dict):
        with self._lock:
            self._data = {**self._DEFAULT, **data}

    def get(self) -> dict:
        with self._lock:
            return dict(self._data)

    def get_field(self, key, default=None):
        with self._lock:
            return self._data.get(key, default)


# ═══════════════════════════════════════════════════════════════════════════════
# NAV2 CLIENT
# ═══════════════════════════════════════════════════════════════════════════════

class Nav2Client:
    """
    Nav2 NavigateToPose action'ını saran yardımcı sınıf.
    SMACH state'leri bu sınıf üzerinden Nav2 ile konuşur.
    """

    def __init__(self, node: Node):
        self.node = node
        self._client = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=ReentrantCallbackGroup()
        )

    def go_to(self, x: float, y: float, yaw: float) -> bool:
        """
        Verilen (x, y, yaw) noktasına git. Bloklar.
        Başarı → True, hata → False.
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                'Nav2 /navigate_to_pose servisi bulunamadı!'
            )
            return False

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(x, y, yaw)

        self.node.get_logger().info(
            f'Nav2 hedef → ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)'
        )

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.node.get_logger().warn('Nav2 hedefi reddetti.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result()
        success = (result.status == 4)  # GoalStatus.STATUS_SUCCEEDED = 4

        if success:
            self.node.get_logger().info(f'Hedefe ulaşıldı ✓ ({x:.2f}, {y:.2f})')
        else:
            self.node.get_logger().warn(
                f'Hedefe ulaşılamadı. Status={result.status}'
            )
        return success

    def _build_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.node.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # yaw → quaternion (2D: sadece z ve w)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 1 — IDLE
# ═══════════════════════════════════════════════════════════════════════════════

class IdleState(smach.State):
    """
    Başlangıç durumu. /mission_start → True gelince çıkar.
    Araç güçlenince hemen hareket etmemesi için bu bekleme zorunludur.

    Geçişler: 'started' → NavigateState
    """

    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=['started'])
        self.node = node
        self._start_received = False

        self.node.create_subscription(
            Bool, '/mission_start', self._on_start, 10
        )
        # Veri paketi kaydedici için misyon aktif yayıncısı
        self._misyon_pub = self.node.create_publisher(Bool, '/misyon/aktif', 10)
        self.node.get_logger().info('[IDLE] Hazır. /mission_start bekleniyor...')

    def _on_start(self, msg: Bool):
        if msg.data:
            self._start_received = True

    def execute(self, userdata):
        self._start_received = False
        rate = self.node.create_rate(10)
        while rclpy.ok() and not self._start_received:
            rate.sleep()
        self.node.get_logger().info('[IDLE] Görev başlıyor.')
        # Veri kaydını başlat
        self._misyon_pub.publish(Bool(data=True))
        return 'started'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 2 — NAVIGATE
# ═══════════════════════════════════════════════════════════════════════════════

class NavigateState(smach.State):
    """
    Waypoint listesini sırayla işler.

    Her waypoint için:
      1. Nav2'ye hedef gönder, bekle.
      2. Varışta waypoint etiketine göre özel mantık uygula:
           DIK_EGIM   → 2 saniye dur (şartname gereği)
           KAYAR_ENGEL → kayar_yon netleşene kadar bekle (max 10s)
           ATIS        → SHOOT_APPROACH state'ine geç
      3. Sonraki waypointı yükle.

    Geçişler:
      'next_waypoint'    → NavigateState (döngü)
      'shoot_waypoint'   → ShootApproachState
      'mission_complete' → MissionCompleteState
      'failed'           → ErrorRecoveryState
    """

    def __init__(self, node: Node, nav: Nav2Client, det_store: DetectionsStore):
        smach.State.__init__(
            self,
            outcomes=['next_waypoint', 'shoot_waypoint',
                      'mission_complete', 'failed'],
            input_keys=['waypoints', 'wp_index'],
            output_keys=['wp_index', 'current_wp']
        )
        self.node      = node
        self.nav       = nav
        self.det_store = det_store

    def execute(self, userdata):
        waypoints = userdata.waypoints
        idx       = userdata.wp_index

        if idx >= len(waypoints):
            self.node.get_logger().info('[NAVIGATE] Tüm waypointler tamamlandı.')
            return 'mission_complete'

        wp    = waypoints[idx]
        label = wp.get('label', '')

        self.node.get_logger().info(
            f'[NAVIGATE] {idx+1}/{len(waypoints)} → {label} '
            f'({wp["x"]:.2f}, {wp["y"]:.2f})'
        )

        # Nav2'ye git
        success = self.nav.go_to(wp['x'], wp['y'], wp.get('yaw', 0.0))

        if not success:
            if wp.get('pas_gecilir', False):
                self.node.get_logger().warn(
                    f'[NAVIGATE] {label} başarısız — pas_gecilir=True, atlıyorum.'
                )
                userdata.wp_index  = idx + 1
                userdata.current_wp = wp
                return 'next_waypoint'
            else:
                self.node.get_logger().warn(
                    f'[NAVIGATE] {label} başarısız — hata kurtarma.'
                )
                return 'failed'

        # ── Varış sonrası özel mantık ─────────────────────────────────

        if label == 'DIK_EGIM':
            # Şartname: dik eğim çıkışında 2 saniye dur, sonra devam
            self.node.get_logger().info(
                '[NAVIGATE] DIK_EGIM: şartname gereği 2 saniye bekleniyor...'
            )
            time.sleep(2.0)
            self.node.get_logger().info('[NAVIGATE] DIK_EGIM: devam.')

        elif label == 'KAYAR_ENGEL':
            # Kayar engelin hangi tarafa geçtiğini bekle (max 10s)
            self.node.get_logger().info(
                '[NAVIGATE] KAYAR_ENGEL: geçiş yönü bekleniyor...'
            )
            deadline = time.time() + 10.0
            while time.time() < deadline:
                yon = self.det_store.get_field('kayar_yon', 'bilinmiyor')
                if yon != 'bilinmiyor':
                    self.node.get_logger().info(
                        f'[NAVIGATE] Kayar engel → {yon} tarafa geçti.'
                    )
                    break
                time.sleep(0.2)
            else:
                self.node.get_logger().warn(
                    '[NAVIGATE] Kayar engel yönü alınamadı (timeout) — devam ediliyor.'
                )

        # İndeksi artır
        userdata.wp_index  = idx + 1
        userdata.current_wp = wp

        # Atış noktasına geldik mi?
        if wp.get('type') == 'shoot':
            return 'shoot_waypoint'

        return 'next_waypoint'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 3 — SHOOT_APPROACH
# ═══════════════════════════════════════════════════════════════════════════════

class ShootApproachState(smach.State):
    """
    Atış pozisyonu doğrulama.

    Protokol (Madde 3.3):
      hedef_hata_x piksel cinsindendir.
      ±5 pikselin altında "nişan tamam" sayılır.

    Görüntü ekibinden /ika/detections üzerinden hedef_var ve
    hedef_hata_x/y okunur. Tolerans sağlandığında ShootState'e geçilir.
    Max 15 saniye beklenir — timeout'ta yine de ateş edilir.

    Geçişler: 'in_position' → ShootState
              'failed'      → ErrorRecoveryState
    """

    HEDEF_TOLERANS_PX = 5.0   # Protokol Madde 3.3 — ±5 piksel
    TIMEOUT_S         = 15.0

    # Servo PID katsayısı: hata_px → açı değişimi
    # Deneysel: 1 piksel hata ≈ 0.3° servo hareketi
    KP_SERVO = 0.3

    def __init__(self, node: Node, det_store: DetectionsStore):
        smach.State.__init__(
            self,
            outcomes=['in_position', 'failed'],
            input_keys=['current_wp']
        )
        self.node      = node
        self.det_store = det_store

        # Servo komut publisher'ları — seri_kopru.py dinler
        self._pan_pub  = node.create_publisher(Int16, '/servo/pan',  10)
        self._tilt_pub = node.create_publisher(Int16, '/servo/tilt', 10)

        # Mevcut servo açıları (başlangıç: merkez)
        self._pan_aci  = 90
        self._tilt_aci = 90

    def _servo_gonder(self):
        self._pan_pub.publish(Int16(data=self._pan_aci))
        self._tilt_pub.publish(Int16(data=self._tilt_aci))

    def execute(self, userdata):
        self.node.get_logger().info('[SHOOT_APPROACH] Servo nişan başlıyor...')

        # Başlangıçta merkeze al
        self._pan_aci  = 90
        self._tilt_aci = 90
        self._servo_gonder()

        deadline = time.time() + self.TIMEOUT_S
        while time.time() < deadline:
            det = self.det_store.get()

            if det.get('hedef_var', False):
                hata_x = det.get('hedef_hata_x', 0.0)
                hata_y = det.get('hedef_hata_y', 0.0)

                # Proportional servo düzeltme
                # hata_x > 0 → hedef sağda → pan artır
                # hata_y > 0 → hedef aşağıda → tilt azalt
                self._pan_aci  += int(hata_x * self.KP_SERVO)
                self._tilt_aci -= int(hata_y * self.KP_SERVO)

                # Sınır kontrolü [0-180°]
                self._pan_aci  = max(0, min(180, self._pan_aci))
                self._tilt_aci = max(0, min(180, self._tilt_aci))

                self._servo_gonder()

                self.node.get_logger().info(
                    f'[SHOOT_APPROACH] hata=({hata_x:.1f},{hata_y:.1f})px '
                    f'servo=({self._pan_aci}°,{self._tilt_aci}°)'
                )

                if (abs(hata_x) <= self.HEDEF_TOLERANS_PX and
                        abs(hata_y) <= self.HEDEF_TOLERANS_PX):
                    self.node.get_logger().info('[SHOOT_APPROACH] Nişan tamam ✓')
                    return 'in_position'
            else:
                self.node.get_logger().info(
                    '[SHOOT_APPROACH] Hedef görüntüde yok, bekleniyor...'
                )

            time.sleep(0.05)   # 20 Hz servo döngüsü

        self.node.get_logger().warn(
            '[SHOOT_APPROACH] Timeout — nişan alınamadı, yine de ateş ediliyor.'
        )
        return 'in_position'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 4 — SHOOT
# ═══════════════════════════════════════════════════════════════════════════════

class ShootState(smach.State):
    """
    Atış yapma durumu — Şartname 6.10 uyumlu.

    Şartname kuralları:
      - En fazla 3 deneme hakkı, en iyi puan geçerli.
      - Lazer min 1 saniye hedefte kalacak (NANO sayar).
      - Lazer aktifken araç hareket etmeyecek (seri_kopru.py PKT_LAZER=1'de bloklar).

    Akış her deneme için:
      /shoot_command → True (seri_kopru PKT_LAZER=1 gönderir)
      /shoot_confirmed → True beklenir (seri_kopru PKT_LAZER=0 echo'sunda publish eder)
      Timeout → bir sonraki denemeye geç

    Geçişler: 'shot_fired' → NavigateState
              'failed'     → ErrorRecoveryState
    """

    MAX_DENEME = 3       # Şartname: en fazla 3 deneme
    TIMEOUT_S  = 8.0     # Her deneme için max bekleme

    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=['shot_fired', 'failed'])
        self.node       = node
        self._confirmed = False

        self._shoot_pub = node.create_publisher(Bool, '/shoot_command', 10)
        node.create_subscription(
            Bool, '/shoot_confirmed', self._on_confirmed, 10
        )

    def _on_confirmed(self, msg: Bool):
        if msg.data:
            self._confirmed = True

    def execute(self, userdata):
        basarili_deneme = 0

        for deneme in range(1, self.MAX_DENEME + 1):
            self.node.get_logger().info(
                f'[SHOOT] Deneme {deneme}/{self.MAX_DENEME}'
            )
            self._confirmed = False

            # Ateş komutu — seri_kopru PKT_LAZER=1 → MEGA → NANO → Lazer
            self._shoot_pub.publish(Bool(data=True))

            # Onay bekle (seri_kopru PKT_LAZER=0 echo'sunda publish eder)
            start = time.time()
            while not self._confirmed and (time.time() - start) < self.TIMEOUT_S:
                time.sleep(0.05)

            if self._confirmed:
                basarili_deneme = deneme
                self.node.get_logger().info(
                    f'[SHOOT] Deneme {deneme} onaylandı ✓'
                )
            else:
                self.node.get_logger().warn(
                    f'[SHOOT] Deneme {deneme} onayı gelmedi (timeout).'
                )

            # Son deneme değilse ShootApproach'a dönmek yerine
            # kısa bir bekleme sonrası tekrar dene
            if deneme < self.MAX_DENEME:
                time.sleep(1.0)

        if basarili_deneme > 0:
            self.node.get_logger().info(
                f'[SHOOT] {self.MAX_DENEME} denemeden {basarili_deneme}. onaylandı. Devam.'
            )
        else:
            self.node.get_logger().warn(
                '[SHOOT] Hiçbir deneme onaylanmadı — devam ediliyor.'
            )

        return 'shot_fired'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 5 — MISSION_COMPLETE
# ═══════════════════════════════════════════════════════════════════════════════

class MissionCompleteState(smach.State):
    """
    Görev tamamlandı. /mission_status → 'COMPLETE' yayınlanır.
    Geçişler: 'done' → (terminal)
    """

    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=['done'])
        self.node        = node
        self._status_pub = node.create_publisher(String, '/mission_status', 10)
        self._misyon_pub = node.create_publisher(Bool, '/misyon/aktif', 10)

    def execute(self, userdata):
        self.node.get_logger().info('═══ GÖREV TAMAMLANDI ═══')
        self._status_pub.publish(String(data='COMPLETE'))
        # Veri kaydını durdur
        self._misyon_pub.publish(Bool(data=False))
        return 'done'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 6 — ERROR_RECOVERY
# ═══════════════════════════════════════════════════════════════════════════════

class ErrorRecoveryState(smach.State):
    """
    Hata kurtarma. Nav2 başarısız olduğunda buraya düşülür.
    3 deneme başarısız olursa görev sonlandırılır.

    Geçişler: 'recovered' → NavigateState
              'abort'     → MissionCompleteState
    """

    MAX_RETRIES = 3

    def __init__(self, node: Node):
        smach.State.__init__(
            self,
            outcomes=['recovered', 'abort'],
            input_keys=['wp_index'],
            output_keys=['wp_index']
        )
        self.node         = node
        self._retry_count = 0

    def execute(self, userdata):
        self._retry_count += 1
        self.node.get_logger().warn(
            f'[ERROR_RECOVERY] Deneme {self._retry_count}/{self.MAX_RETRIES}'
        )

        if self._retry_count >= self.MAX_RETRIES:
            self.node.get_logger().error(
                '[ERROR_RECOVERY] Max deneme aşıldı. Görev sonlandırılıyor.'
            )
            self._retry_count = 0
            return 'abort'

        # Önceki waypointten tekrar dene
        if userdata.wp_index > 0:
            userdata.wp_index -= 1

        time.sleep(2.0)
        self._retry_count = 0
        return 'recovered'


# ═══════════════════════════════════════════════════════════════════════════════
# WAYPOINT YÜKLEYİCİ
# ═══════════════════════════════════════════════════════════════════════════════

def load_waypoints(yaml_path: str) -> list:
    """
    waypoints.yaml → waypoint listesi.

    'ATIS' isimli aşama type='shoot' olarak işaretlenir.
    'DIK_EGIM' ve 'KAYAR_ENGEL' etiketleri NavigateState'te özel işlenir.
    """
    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        asamalar  = data.get('asamalar', [])
        waypoints = []
        for a in asamalar:
            wp    = a.get('waypoint', {})
            isim  = a.get('isim', '')
            waypoints.append({
                'x':           wp.get('x', 0.0),
                'y':           wp.get('y', 0.0),
                'yaw':         wp.get('yaw', 0.0),
                'type':        'shoot' if isim == 'ATIS' else 'nav',
                'label':       isim,
                'pas_gecilir': a.get('pas_gecilir', False),
            })

        print(f'[WAYPOINTS] {len(waypoints)} waypoint yüklendi.')
        return waypoints

    except FileNotFoundError:
        print(f'[WAYPOINTS] UYARI: {yaml_path} bulunamadı. Boş liste.')
        return []
    except Exception as e:
        print(f'[WAYPOINTS] HATA: {e}')
        return []


# ═══════════════════════════════════════════════════════════════════════════════
# ANA ÇALIŞMA
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = rclpy.create_node('misyon_fsm')

    # ── Detections Store ──────────────────────────────────────────────
    det_store = DetectionsStore()

    def _on_detections(msg: String):
        try:
            data = json.loads(msg.data)
            det_store.update(data)
        except json.JSONDecodeError:
            pass

    node.create_subscription(String, '/ika/detections', _on_detections, 10)

    # ── Nav2 Client ───────────────────────────────────────────────────
    nav = Nav2Client(node)

    # ── Waypoint yükle ────────────────────────────────────────────────
    import os
    from ament_index_python.packages import get_package_share_directory
    try:
        pkg_share = get_package_share_directory('teknofest_ika')
        wp_path   = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    except Exception:
        wp_path = os.path.expanduser(
            '~/ika_ws/src/teknofest_ika/config/waypoints.yaml'
        )

    waypoints = load_waypoints(wp_path)

    # ── SMACH FSM Kurulumu ────────────────────────────────────────────
    sm = smach.StateMachine(outcomes=['GOREV_TAMAMLANDI', 'GOREV_IPTAL'])

    sm.userdata.waypoints  = waypoints
    sm.userdata.wp_index   = 0
    sm.userdata.current_wp = None

    with sm:
        smach.StateMachine.add(
            'IDLE',
            IdleState(node),
            transitions={'started': 'NAVIGATE'}
        )

        smach.StateMachine.add(
            'NAVIGATE',
            NavigateState(node, nav, det_store),
            transitions={
                'next_waypoint':    'NAVIGATE',
                'shoot_waypoint':   'SHOOT_APPROACH',
                'mission_complete': 'MISSION_COMPLETE',
                'failed':           'ERROR_RECOVERY',
            }
        )

        smach.StateMachine.add(
            'SHOOT_APPROACH',
            ShootApproachState(node, det_store),
            transitions={
                'in_position': 'SHOOT',
                'failed':      'ERROR_RECOVERY',
            }
        )

        smach.StateMachine.add(
            'SHOOT',
            ShootState(node),
            transitions={
                'shot_fired': 'NAVIGATE',
                'failed':     'ERROR_RECOVERY',
            }
        )

        smach.StateMachine.add(
            'MISSION_COMPLETE',
            MissionCompleteState(node),
            transitions={'done': 'GOREV_TAMAMLANDI'}
        )

        smach.StateMachine.add(
            'ERROR_RECOVERY',
            ErrorRecoveryState(node),
            transitions={
                'recovered': 'NAVIGATE',
                'abort':     'MISSION_COMPLETE',
            }
        )

    # ROS2 spin ayrı thread'de (FSM bloklayıcı olduğu için)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    node.get_logger().info('═══ LYDİA İKA Görev FSM v2.0 başlatıldı ═══')
    node.get_logger().info(f'Toplam waypoint : {len(waypoints)}')
    node.get_logger().info("'/mission_start' → True gönder → görev başlar")

    outcome = sm.execute()

    node.get_logger().info(f'FSM sonucu: {outcome}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
