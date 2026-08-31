#!/usr/bin/env python3
"""
misyon_fsm.py — LYDİA İKA Görev Yöneticisi (SMACH FSM)
=========================================================
v2.0 — Ekip Protokol v1.0 ile uyumlu

NE YAPAR:
  Parkur boyunca görev sırasını yönetir. Nav2'ye "şimdi şu noktaya git"
  komutunu verir. Atış bölgesinde durur, nişan alır, ateş eder.

  Görüntü ekibinden /ika/detections (JSON) alır:
    - kayar_yon → KAYAR_ENGEL geçiş kararı
    - tabela    → FSM bilgi (terrain_adapter halleder)

  NOT — hedef_var / hedef_hata_x / hedef_hata_y bu JSON'da hesaplanıp
  taşınıyor ama FSM bunları HİÇ okumuyor; bilinçli bir tasarım kararı
  (ekip: "hedef tahtasını tanıyacak, nişanlama yapmayacak"). YOLO'nun
  hedef_tahtasi tespiti sadece kayıt/bilgi amaçlıdır. Gerçek nişan alma
  tamamen ayrı ve YOLO'dan bağımsızdır — bkz. aşağıdaki ATIS waypoint notu.

PROTOKOL ENTEGRASYONU (Ekip Protokol v1.0):
  Giriş : /ika/detections (JSON String) — Görüntü İşleme → Otomasyon
  Çıkış : Nav2 NavigateToPose action   — Otomasyon navigasyon
  Çıkış : /shoot_command (Bool)        — Otomasyon → Gömülü Sistemler köprüsü
           seri_kopru.py bu komutu LAZER,1 / LAZER,0 seri komutuna çevirir

ÖZEL DURUM MANTIĞI:
  DIK_EGIM waypoint : Şartname gereği hedefe varışta 2 saniye dur, sonra devam.
  KAYAR_ENGEL waypoint: kayar_yon != 'bilinmiyor' olana kadar bekle (max 10s).
  ATIS waypoint    : targeting_node etkinleştirilir (kendi HSV+Hough+PID'i
                      ile hedef kilitler, YOLO hedef verisi kullanılmaz),
                      "/targeting/status" == "ALIGNED" gelince ateş edilir.

DURUM DİYAGRAMI:
  [IDLE] → [NAVIGATE] ─┬─ type=shoot ──→ [SHOOT_APPROACH] → [SHOOT] ─┐
                       │                                             │
                       ├─ type=hizlanma → [HIZLANMA] ────────────────┤
                       │                                             │
                       ├─ type=rampa ──→ [RAMPA] ────────────────────┤
                       │   (§6.10 Nav2 baypas: doğrudan /cmd_vel)     │
                       ├─ liste bitti ──→ [MISSION_COMPLETE] → GOREV_TAMAMLANDI
                       │                                             │
                       └───────────── [NAVIGATE] ←───────────────────┘
                       ↓ failed                 ↑ recovered (aynı wp)
                 [ERROR_RECOVERY] ──────────────┤
                       ↓ 2 deneme               ↑ sonraki_asama (wp+1)
                       └────────────────────────┘

  E-STOP → [MISSION_ABORT]         §6.12 süre doldu → [MISSION_ABORT]

  Görevi bitiren yollar YALNIZ ikisidir: E-STOP ve §6.12 süre dolması.
  İkisi de kurtarmaya uğramadan doğrudan MISSION_ABORT'a gider.

  Bir aşama 2 denemede de olmazsa o aşamadan VAZGEÇİLİR ve sıradaki
  waypoint'e geçilir — görev sürer. Eskiden görev iptal ediliyordu ve
  5 puanlık bir istasyonda takılmak, aşağı akıştaki koniler (50), kayar
  engel (50), atış (50), hızlanma (25) ve sıralamaya giren koşu süresi
  (100) puanlarının tamamını götürüyordu. Vazgeçmek §9 pası DEĞİLDİR:
  pas, aracın aşama sonuna elle taşınmasıdır ve puanı eksiye yazar.

  MISSION_COMPLETE ve MISSION_ABORT AYRI durumlardır: ilki
  /mission_status = 'COMPLETE', ikincisi 'ABORTED' yayınlar. Kurtarma
  bütçesi (2) aşama başınadır — NavigateState her tamamlanan waypoint'te
  sayacı sıfırlar.

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
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import Bool, Float32, String, UInt8
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from nav2_msgs.action import NavigateToPose

import smach

from teknofest_ika.otonomi.topics import (
    FSM_STATE_TOPIC, DETECTIONS_TOPIC,
    MOVING_OBS_DIR_TOPIC, E_STOP_TOPIC, MOD_AKTIF_TOPIC,
    MISSION_START_TOPIC, MISYON_AKTIF_TOPIC,
    TARGETING_ENABLE_TOPIC, TARGETING_STATUS_TOPIC,
    SHOOT_CMD_TOPIC, SHOOT_RESULT_TOPIC, LASER_FIRE_DURATION,
    MISSION_STATUS_TOPIC, MISYON_WP_INDEX_TOPIC,
    CMD_VEL_TOPIC, ODOM_TOPIC, RAMP_STOP_DURATION,
    MISYON_KALAN_SURE_TOPIC, KOSU_SURESI_S, PAS_HAKKI, PAS_GECILEMEZ,
    SCAN_FILTERED_TOPIC, ENGEBELI_SAPMA_TOPIC, KORIDOR_GENISLIGI_M,
    IMU_TOPIC, IMU_PITCH_RAMP_THRESHOLD,
    KORIDOR_SAPMA_UYARI_M, KORIDOR_TOPLAM_TOLERANS, KORIDOR_PENCERE_RAD,
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore, stop_check as _stop_check_pure, hizlanma_hiz_profili,
    durma_degerlendir, tarama_yan_mesafe, koridor_sapmasi, aci_sarmala,
    kosu_butcesi, pas_verilebilir as _pas_verilebilir_pure,
    kurtarma_karari as _kurtarma_karari_pure,
    quat_to_roll_pitch_deg, rollback_riskli, rampa_faz_gecisi,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')

# §6.9 koridor ortalamasının izlendiği aşama.
ENGEBELI_ETIKET = 'ENGEBELİ_ARAZİ'


class KoridorIzleyici:
    """
    §6.9: tümsekli bölüm geçilirken parkurun ortalanması gerekiyor, aracın
    tamamının aynı tümsek dizisi üzerinden geçirilmesi §9'da −5 puan.

    Tümsekler 20 cm taban / 5 cm yükseklik (Şekil 4) — LiDAR düzlemi zeminden
    55 cm'de olduğu için tümsekler taramada HİÇ görünmez, tekerlek yerleşimi
    planlanamaz. Ölçülebilen ve kuralın ilk cümlesinin istediği şey koridorda
    ortalanmak; §6.1 koridoru 3 m ve iki yanı 80 ± 10 cm bariyerli, iki duvar
    da tarama düzleminde.

    Bu sınıf yalnız ÖLÇER, direksiyona karışmaz. Gereksinim bugün sistemde
    görünmüyor: ne loglanıyor ne panoda var, dolayısıyla ortalanıp
    ortalanmadığımız ancak hakem kağıdından öğrenilirdi. Düzeltici katmanın
    gerekip gerekmediğine buradan çıkan ölçümle karar verilecek.
    """

    def __init__(self, node, lidar_yaw_rad: float = 1.6284):
        self.node = node
        self._pub = node.create_publisher(Float32, ENGEBELI_SAPMA_TOPIC, 10)

        # LiDAR gövdeye dönük monte; düzeltme TF'te uygulanıyor, ham taramada
        # değil. Araç ekseni açıları bu yüzden tarama çerçevesine kaydırılır.
        self._sol_merkez = aci_sarmala(math.pi / 2.0 - lidar_yaw_rad)
        self._sag_merkez = aci_sarmala(-math.pi / 2.0 - lidar_yaw_rad)

        self._lock   = threading.Lock()
        self._aktif  = False
        self._ornek  = []      # geçerli sapmalar [m]
        self._atlanan = 0      # koridor okunamayan tarama sayısı

        node.create_subscription(LaserScan, SCAN_FILTERED_TOPIC, self._on_scan, 10)

    def basla(self):
        with self._lock:
            self._aktif   = True
            self._ornek   = []
            self._atlanan = 0

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            if not self._aktif:
                return
        sol = tarama_yan_mesafe(
            list(msg.ranges), msg.angle_min, msg.angle_increment,
            self._sol_merkez, KORIDOR_PENCERE_RAD,
            msg.range_min, msg.range_max,
        )
        sag = tarama_yan_mesafe(
            list(msg.ranges), msg.angle_min, msg.angle_increment,
            self._sag_merkez, KORIDOR_PENCERE_RAD,
            msg.range_min, msg.range_max,
        )
        sapma, durum = koridor_sapmasi(
            sol, sag, KORIDOR_GENISLIGI_M, KORIDOR_TOPLAM_TOLERANS,
        )
        with self._lock:
            if durum != 'gecerli':
                self._atlanan += 1
                return
            self._ornek.append(sapma)
        self._pub.publish(Float32(data=float(sapma)))
        if abs(sapma) > KORIDOR_SAPMA_UYARI_M:
            yon = 'sağa' if sapma > 0 else 'sola'
            self.node.get_logger().warn(
                f'[KORİDOR] §6.9 merkezden {abs(sapma):.2f}m {yon} kaymış '
                f'(sol {sol:.2f}m / sağ {sag:.2f}m).',
                throttle_duration_sec=2.0,
            )

    def bitir(self):
        """Aşama sonunda özet loglar ve izlemeyi kapatır."""
        with self._lock:
            self._aktif = False
            ornek   = list(self._ornek)
            atlanan = self._atlanan

        if not ornek:
            self.node.get_logger().warn(
                f'[KORİDOR] §6.9 hiç geçerli koridor ölçümü alınamadı '
                f'({atlanan} tarama elendi) — ortalanma doğrulanamadı.'
            )
            return

        ortalama = sum(ornek) / len(ornek)
        enbuyuk  = max(ornek, key=abs)
        self.node.get_logger().info(
            f'[KORİDOR] §6.9 özet: {len(ornek)} ölçüm, ortalama sapma '
            f'{ortalama:+.2f}m, en büyük {enbuyuk:+.2f}m, '
            f'{atlanan} tarama elendi (+ sağa kayma).'
        )


# DetectionsStore → teknofest_ika.otonomi.pure_logic (rclpy bağımsız, DRY +
# test_birim.py gerçek sınıfı doğrudan import edip test eder).


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

    def go_to(self, x: float, y: float, yaw: float,
              timeout_sec: float = 120.0,
              stop_check_fn=None) -> str:
        """
        Verilen (x, y, yaw) noktasına git. Bloklar.
        Dönüş: 'success' | 'failed' | 'timeout' | 'stop_requested'

        stop_check_fn: callable → bool. True döndürünce goal iptal edilir,
        'stop_requested' döner. NavigateState §6.10 STOP mantığı için kullanır.
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                'Nav2 /navigate_to_pose servisi bulunamadı!'
            )
            return 'failed'

        goal = NavigateToPose.Goal()
        goal.pose = self._build_pose(x, y, yaw)

        self.node.get_logger().info(
            f'Nav2 hedef → ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad) '
            f'timeout={timeout_sec:.0f}s'
        )

        deadline = time.time() + timeout_sec

        future = self._client.send_goal_async(goal)
        while not future.done():
            if time.time() > deadline:
                self.node.get_logger().warn('Nav2 hedef gönderme timeout.')
                return 'timeout'
            if stop_check_fn and stop_check_fn():
                return 'stop_requested'
            time.sleep(0.05)

        try:
            goal_handle = future.result()
        except Exception as e:
            self.node.get_logger().error(f'Nav2 hedef gönderilemedi: {e}')
            return 'failed'

        if not goal_handle.accepted:
            self.node.get_logger().warn('Nav2 hedefi reddetti.')
            return 'failed'

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if time.time() > deadline:
                self.node.get_logger().warn(
                    f'Nav2 aşama timeout ({timeout_sec:.0f}s) — hedef iptal ediliyor.'
                )
                goal_handle.cancel_goal_async()
                return 'timeout'
            if stop_check_fn and stop_check_fn():
                goal_handle.cancel_goal_async()
                return 'stop_requested'
            time.sleep(0.05)

        try:
            result = result_future.result()
        except Exception as e:
            self.node.get_logger().error(f'Nav2 sonuç alınamadı: {e}')
            return 'failed'

        success = (result.status == 4)  # GoalStatus.STATUS_SUCCEEDED = 4

        if success:
            self.node.get_logger().info(f'Hedefe ulaşıldı ✓ ({x:.2f}, {y:.2f})')
        else:
            self.node.get_logger().warn(
                f'Hedefe ulaşılamadı. Status={result.status}'
            )
        return 'success' if success else 'failed'

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
# KOŞU SAATİ
# ═══════════════════════════════════════════════════════════════════════════════

class MisyonSaati:
    """
    Şartname §6.12 koşu saati: atış dahil tüm parkur 15 dakika içinde
    tamamlanmalıdır.

    Saat hakem kronometresini taklit eder, yani MANUEL MODDA DA İŞLER. FSM'in
    kendi bekleme döngüleri (manuel duraklama, STOP bekleme) kendi bütçelerini
    uzatabilir; koşu saatinin uzatılabilir bir karşılığı sahada yoktur.

    Aşama timeout'u tek başına yetmiyordu: 11 aşama × 120 s = 22 dakikalık bir
    bütçe, koşu limitinin bir buçuk katı. Aşama timeout'ları `kalan()` ile
    kırpılmadığında araç, hakem parkurdan çıkarttığı için hiç puan getirmeyecek
    bir aşamanın içinde dakikalarca bekleyebiliyordu.
    """

    def __init__(self, node: Node, sure_s: float = KOSU_SURESI_S):
        self.node    = node
        self.sure_s  = sure_s
        self._t0     = None      # None = koşu henüz başlamadı
        self._uyarildi = set()
        self._pub    = node.create_publisher(Float32, MISYON_KALAN_SURE_TOPIC, 10)

    def baslat(self) -> None:
        self._t0 = time.time()
        self._uyarildi.clear()
        self.node.get_logger().info(
            f'[SAAT] Koşu saati başladı — §6.12 limiti {self.sure_s / 60.0:.0f} dk.'
        )
        self.yayinla()

    def basladi_mi(self) -> bool:
        return self._t0 is not None

    def kalan(self) -> float:
        """Kalan saniye. Saat başlamadıysa tüm süre kalmış sayılır."""
        if self._t0 is None:
            return self.sure_s
        return max(0.0, self.sure_s - (time.time() - self._t0))

    def doldu(self) -> bool:
        return self.basladi_mi() and self.kalan() <= 0.0

    def yayinla(self) -> None:
        self._pub.publish(Float32(data=float(self.kalan())))

    def butce(self, istenen: float) -> float:
        """
        Bir aşamaya verilebilecek gerçek süre — hesap pure_logic'te,
        test_birim.py oradaki fonksiyonu doğrudan sürer.
        """
        return kosu_butcesi(istenen, self.kalan())

    # Kalan süre uyarı eşikleri (saniye), azalan sırada.
    ESIKLER = (300.0, 120.0, 60.0)

    def rapor(self) -> None:
        """Eşik geçildikçe bir kez uyarır; her çağrıda kalan süreyi yayınlar."""
        self.yayinla()
        if not self.basladi_mi():
            return
        kalan   = self.kalan()
        gecilen = [e for e in self.ESIKLER if kalan <= e]
        if not gecilen:
            return
        # İki rapor arasında birden fazla eşik atlanmış olabilir (uzun süren
        # tek bir aşama). Yalnız en acil eşik duyurulur; atlanan üst eşikler
        # işaretlenir ki geriye dönük uyarı yağmuru olmasın.
        esik = min(gecilen)
        if esik in self._uyarildi:
            return
        self._uyarildi.update(gecilen)
        self.node.get_logger().warn(
            f'[SAAT] Koşu süresinin son {esik / 60.0:.0f} dakikası '
            f'(kalan {kalan:.0f} s).'
        )


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 1 — IDLE
# ═══════════════════════════════════════════════════════════════════════════════

class IdleState(smach.State):
    """
    Başlangıç durumu. /mission_start → True gelince çıkar.
    Araç güçlenince hemen hareket etmemesi için bu bekleme zorunludur.

    Geçişler: 'started' → NavigateState
    """

    def __init__(self, node: Node, saat: 'MisyonSaati',
                 baslangic_bekleme: float = 3.0):
        smach.State.__init__(self, outcomes=['started'])
        self.node = node
        self.saat = saat
        self._start_received = False
        self._baslangic_bekleme = baslangic_bekleme

        self.node.create_subscription(
            Bool, MISSION_START_TOPIC, self._on_start, 10
        )
        # Veri paketi kaydedici için misyon aktif yayıncısı
        self._misyon_pub = self.node.create_publisher(Bool, MISYON_AKTIF_TOPIC, 10)
        self.node.get_logger().info('[IDLE] Hazır. /mission_start bekleniyor...')

    def _on_start(self, msg: Bool):
        if msg.data:
            self._start_received = True

    def execute(self, userdata):
        self._start_received = False
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='IDLE'))
        rate = self.node.create_rate(10)
        while rclpy.ok() and not self._start_received:
            rate.sleep()
        # Saat /mission_start ile başlar, başlangıç beklemesinden ÖNCE:
        # o 3 saniye de hakem kronometresinde işliyor (§6.12).
        self.saat.baslat()
        if self._baslangic_bekleme > 0:
            self.node.get_logger().info(
                f'[IDLE] Başlangıç bekleme: {self._baslangic_bekleme:.0f}s '
                f'(SLAM + Nav2 stabilizasyonu)...'
            )
            time.sleep(self._baslangic_bekleme)
        self.node.get_logger().info('[IDLE] Görev başlıyor.')
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

    # Bir waypoint içinde kaç kez STOP tetiklenebilir. Tabela görüş alanında
    # kalıp araç ilerlemezse (cooldown dolar, tabela hâlâ görünür) dur-kalk
    # döngüsü sonsuza gider; sınıra ulaşınca STOP tespiti bu waypoint için
    # devre dışı bırakılır.
    MAX_STOP = 3

    def __init__(self, node: Node, nav: Nav2Client, det_store: DetectionsStore,
                 saat: 'MisyonSaati', stage_timeout: float = 120.0):
        smach.State.__init__(
            self,
            outcomes=['next_waypoint', 'shoot_waypoint', 'hizlanma_waypoint',
                      'mission_complete', 'sure_doldu', 'failed', 'e_stop',
                      'rampa_waypoint'],
            input_keys=['waypoints', 'wp_index'],
            output_keys=['wp_index', 'current_wp', 'retry_count']
        )
        self.node                  = node
        self.nav                   = nav
        self.det_store             = det_store
        self.saat                  = saat
        self.stage_timeout         = stage_timeout
        # §9: pas hakkı koşu başına, aşama başına değil. Sayaç state örneği
        # üzerinde tutulur; NavigateState her waypoint için yeniden execute
        # edilir ama nesne aynı kalır.
        self._pas_kullanildi       = 0
        self._wp_index_pub         = node.create_publisher(UInt8, MISYON_WP_INDEX_TOPIC, 10)
        self._stop_cooldown_bitis  = 0.0   # §6.10: son STOP sonrası tekrar tetiklenme engeli
        # §6.9 koridor ortalaması — yalnız ölçer, direksiyona karışmaz.
        self._koridor              = KoridorIzleyici(node)

    def _stop_check(self) -> bool:
        """Nav2Client.go_to() callback'i: True dönünce goal iptal edilir (§6.10 STOP)."""
        return _stop_check_pure(
            self.det_store.get_field('stop_var', False),
            self._stop_cooldown_bitis,
            time.time(),
        )

    def _pas_verilebilir(self, wp: dict, label: str) -> bool:
        """
        §9 pas kapısı. Karar pure_logic.pas_verilebilir'de; burada yalnız
        reddin gerekçesi log'a yazılır.
        """
        izin, gerekce = _pas_verilebilir_pure(
            wp.get('pas_gecilir', False), label,
            self._pas_kullanildi, PAS_HAKKI, PAS_GECILEMEZ,
        )
        if gerekce == 'sartname_yasak':
            self.node.get_logger().error(
                f'[NAVIGATE] {label} §9 gereği pas geçilemez — '
                'pas_gecilir=True yok sayılıyor.'
            )
        elif gerekce == 'hak_bitti':
            self.node.get_logger().error(
                f'[NAVIGATE] Pas hakkı bitti ({PAS_HAKKI}/koşu) — '
                f'{label} atlanmıyor, hata kurtarmaya düşülüyor.'
            )
        return izin

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='NAVIGATE'))

        if self.det_store.get_field('e_stop', False):
            self.node.get_logger().error('[NAVIGATE] E-STOP aktif — navigasyon iptal.')
            return 'e_stop'

        self.saat.rapor()
        if self.saat.doldu():
            self.node.get_logger().error(
                '[NAVIGATE] §6.12 koşu süresi doldu — yeni aşamaya başlanmıyor.'
            )
            return 'sure_doldu'

        waypoints = userdata.waypoints
        idx       = userdata.wp_index

        if self.det_store.get_field('manual_mod', False):
            self.node.get_logger().warn('[NAVIGATE] Manuel mod aktif — navigasyon bekleniyor.')
            while rclpy.ok() and self.det_store.get_field('manual_mod', False):
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error('[NAVIGATE] E-STOP — manuel mod bekleme iptal.')
                    return 'e_stop'
                time.sleep(0.2)
            self.node.get_logger().info('[NAVIGATE] Tam otonom moda geri dönüldü.')

        if idx >= len(waypoints):
            self.node.get_logger().info('[NAVIGATE] Tüm waypointler tamamlandı.')
            return 'mission_complete'

        wp    = waypoints[idx]
        label = wp.get('label', '')

        self.node.get_logger().info(
            f'[NAVIGATE] {idx+1}/{len(waypoints)} → {label} '
            f'({wp["x"]:.2f}, {wp["y"]:.2f})'
        )

        # Nav2'ye git — STOP işareti görülürse ortada kesilir, 2s beklenir, tekrar gönderilir.
        # Aşama timeout'u waypoint'in TAMAMINI kapsar: her STOP'tan sonra go_to'yu
        # taze stage_timeout ile çağırmak toplam süreyi sınırsız bırakıyordu.
        stop_uygulandi = False
        stop_sayaci    = 0
        # Aşamanın kendi süresi varsa o kullanılır; yoksa ortak tavana düşülür.
        # Tek bir ortak sayı (120 s) her aşamaya aynı süreyi veriyordu: 2,5 m'lik
        # rampa inişi de, 22 m'lik sol U dönüşü de. Süreler waypoints.yaml'da
        # CAD'den ölçülen istasyonlar arası mesafeden türetildi (mesafe ÷ zemine
        # göre ortalama hız); türetimin tamamı her aşamanın yanında yazılı.
        istenen = float(wp.get('timeout_saniye', self.stage_timeout))
        # Aşama bütçesi koşuda kalan süreden uzun olamaz (§6.12). Kırpılmazsa
        # araç, hakem çoktan parkurdan çıkarttığı için puan getirmeyecek bir
        # aşamanın içinde dakikalarca bekler.
        asama_butcesi  = self.saat.butce(istenen)
        if asama_butcesi < istenen:
            self.node.get_logger().warn(
                f'[NAVIGATE] {label}: aşama bütçesi koşu saatiyle '
                f'{istenen:.0f}s → {asama_butcesi:.0f}s kırpıldı.'
            )
        wp_deadline    = time.time() + asama_butcesi

        # §6.9: tümsekli bölümde ortalanma izlenir. go_to() bloklayıcı, ölçüm
        # spin thread'indeki tarama callback'inde birikir.
        izleniyor = (label == ENGEBELI_ETIKET)
        if izleniyor:
            self._koridor.basla()

        try:
            while True:
                kalan = wp_deadline - time.time()
                if kalan <= 0.0:
                    result = 'timeout'
                else:
                    result = self.nav.go_to(
                        wp['x'], wp['y'], wp.get('yaw', 0.0),
                        timeout_sec=kalan,
                        stop_check_fn=(self._stop_check
                                       if stop_sayaci < self.MAX_STOP else None),
                    )

                if result == 'stop_requested':
                    stop_sayaci += 1
                    self.node.get_logger().info(
                        f'[NAVIGATE] §6.10 STOP işareti ({stop_sayaci}/{self.MAX_STOP}) '
                        '— araç durduruluyor, 2s bekleniyor.'
                    )
                    # Cooldown: aynı STOP işaretinin hemen tekrar tetiklenmesini engeller
                    # 3s: rampada üst-STOP ile alt-STOP arası için yeterli
                    self._stop_cooldown_bitis = time.time() + 3.0
                    self.det_store.update_field('stop_var', False)
                    stop_uygulandi = True
                    # Zorunlu bekleme navigasyon süresinden sayılmaz.
                    wp_deadline += RAMP_STOP_DURATION
                    deadline_stop = time.time() + RAMP_STOP_DURATION
                    while time.time() < deadline_stop:
                        if self.det_store.get_field('e_stop', False):
                            self.node.get_logger().error('[NAVIGATE] E-STOP — STOP bekleme iptal.')
                            return 'e_stop'
                        time.sleep(0.1)
                    self.node.get_logger().info('[NAVIGATE] §6.10 STOP bekleme tamam — devam ediliyor.')
                    if stop_sayaci >= self.MAX_STOP:
                        self.node.get_logger().warn(
                            f'[NAVIGATE] {label}: STOP sınırı ({self.MAX_STOP}) doldu — '
                            'bu waypoint için STOP tespiti devre dışı, dur-kalk döngüsü kesildi.'
                        )
                    continue

                if result == 'success':
                    break

                # 'failed' veya 'timeout'
                if self._pas_verilebilir(wp, label):
                    self._pas_kullanildi += 1
                    bedel = float(wp.get('pas_puan', 0.0))
                    self.node.get_logger().warn(
                        f'[NAVIGATE] {label} başarısız — pas geçiliyor '
                        f'({self._pas_kullanildi}/{PAS_HAKKI} hak). '
                        f'§9 bedeli: -{bedel:.0f} puan. Araç bir sonraki hedefe '
                        'sürüyor; geçerli bir pas için aşama sonuna ELLE '
                        'taşınması gerekir.'
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

        finally:
            # Hangi yoldan çıkılırsa çıkılsın izleme kapanmalı: açık
            # kalırsa sonraki aşamanın taramaları da §6.9 ölçümüne
            # yazılır ve özet yanlış aşamayı anlatır.
            if izleniyor:
                self._koridor.bitir()

        # §6.10 yedek tetikleyici: dik eğim giriş/çıkış waypoint'ine
        # YOLO'nun STOP tabelasını kaçırması nedeniyle hiç STOP
        # tetiklenmeden varılmışsa, zorunlu 2s bekleme burada mesafe/
        # konum tabanlı olarak (waypoint varışı = tetikleyici) uygulanır.
        # Tek bir görüntü tespitine bağımlılığı ortadan kaldırır.
        if (label in DIK_EGIM_ETIKETLERI and not stop_uygulandi
                and wp.get('type') != 'rampa'):
            self.node.get_logger().warn(
                f'[NAVIGATE] {label}: STOP tabelası tespit edilmeden hedefe '
                'varıldı — §6.10 yedek (konum tabanlı) 2s bekleme uygulanıyor.'
            )
            deadline_stop = time.time() + RAMP_STOP_DURATION
            while time.time() < deadline_stop:
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error(
                        '[NAVIGATE] E-STOP — yedek STOP bekleme iptal.'
                    )
                    return 'e_stop'
                time.sleep(0.1)
            self.node.get_logger().info(
                '[NAVIGATE] §6.10 yedek STOP bekleme tamam — devam ediliyor.'
            )

        if label == 'KAYAR_ENGEL':
            # Kayar engelin hangi tarafa geçtiğini bekle (max 10s)
            self.node.get_logger().info(
                '[NAVIGATE] KAYAR_ENGEL: geçiş yönü bekleniyor...'
            )
            deadline = time.time() + 10.0
            while time.time() < deadline:
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error('[NAVIGATE] E-STOP — KAYAR_ENGEL bekleme iptal.')
                    return 'e_stop'
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

        # İndeksi artır ve GCS'e bildir
        userdata.wp_index  = idx + 1
        userdata.current_wp = wp
        # Bir aşama tamamlandığı an kurtarma bütçesi tazelenir; sayaç görev
        # boyunca birikirse parkurun farklı yerlerindeki üç bağımsız hata
        # görevi iptal ettirir.
        userdata.retry_count = 0
        self._wp_index_pub.publish(UInt8(data=min(idx + 1, 255)))

        if wp.get('type') == 'shoot':
            return 'shoot_waypoint'

        if wp.get('type') == 'hizlanma':
            return 'hizlanma_waypoint'

        if wp.get('type') == 'rampa':
            return 'rampa_waypoint'

        return 'next_waypoint'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 3 — SHOOT_APPROACH
# ═══════════════════════════════════════════════════════════════════════════════

class ShootApproachState(smach.State):
    """
    Atış pozisyonu doğrulama.

    targeting_node'u aktifleştirir (HSV+Hough+PID ile hedef kilitler).
    Servo kontrolü ve hizalama targeting_node tarafından yapılır.
    "/targeting/status" == "ALIGNED" gelince ShootState'e geçilir.
    Max 15 saniye beklenir — timeout'ta yine de ateş edilir.

    Geçişler: 'in_position' → ShootState
              'failed'      → ErrorRecoveryState
    """

    TIMEOUT_S = 15.0

    def __init__(self, node: Node, det_store: DetectionsStore):
        smach.State.__init__(
            self,
            outcomes=['in_position', 'failed', 'e_stop'],
            input_keys=['current_wp']
        )
        self.node      = node
        self.det_store = det_store

        self._targeting_lock   = threading.Lock()
        self._targeting_status = "STANDBY"
        self._targeting_enable_pub = node.create_publisher(
            Bool, TARGETING_ENABLE_TOPIC, 10
        )
        node.create_subscription(
            String, TARGETING_STATUS_TOPIC, self._on_targeting_status, 10
        )

    def _on_targeting_status(self, msg: String):
        with self._targeting_lock:
            self._targeting_status = msg.data

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='SHOOT_APPROACH'))
        self.node.get_logger().info('[SHOOT_APPROACH] targeting_node aktifleştiriliyor...')

        with self._targeting_lock:
            self._targeting_status = "STANDBY"
        self._targeting_enable_pub.publish(Bool(data=True))

        deadline = time.time() + self.TIMEOUT_S
        while time.time() < deadline:
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error('[SHOOT_APPROACH] E-STOP.')
                self._targeting_enable_pub.publish(Bool(data=False))
                return 'e_stop'

            with self._targeting_lock:
                status = self._targeting_status

            if status == 'ALIGNED':
                self.node.get_logger().info('[SHOOT_APPROACH] Nişan tamam ✓')
                self._targeting_enable_pub.publish(Bool(data=False))
                return 'in_position'

            self.node.get_logger().debug(
                f'[SHOOT_APPROACH] targeting={status}',
                throttle_duration_sec=2.0,
            )
            time.sleep(0.05)

        self.node.get_logger().warn('[SHOOT_APPROACH] Timeout — yine de ateş ediliyor.')
        self._targeting_enable_pub.publish(Bool(data=False))
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
      Onay geldiğinde kalan denemeler kullanılmaz (şartname "en fazla 3" der,
      "her zaman 3" değil; boşa harcanan her deneme 9 saniyeye mal olur).

    Geçişler: 'shot_fired' → NavigateState
              'failed'     → ErrorRecoveryState (E-STOP)
    """

    MAX_DENEME = 3       # Şartname: en fazla 3 deneme
    TIMEOUT_S  = 8.0     # Her deneme için max bekleme

    def __init__(self, node: Node, det_store: DetectionsStore,
                 saat: 'MisyonSaati'):
        smach.State.__init__(self, outcomes=['shot_fired', 'failed', 'e_stop'])
        self.saat = saat
        self.node              = node
        self.det_store         = det_store
        self._confirmed_event  = threading.Event()

        self._shoot_pub = node.create_publisher(Bool, SHOOT_CMD_TOPIC, 10)
        node.create_subscription(
            Bool, SHOOT_RESULT_TOPIC, self._on_confirmed, 10
        )

    def _on_confirmed(self, msg: Bool):
        if msg.data:
            self._confirmed_event.set()

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='SHOOT'))
        basarili_deneme = 0

        for deneme in range(1, self.MAX_DENEME + 1):
            # Her denemenin BAŞINDA kontrol edilir. Kontrol yalnız döngü
            # sonunda olsaydı E-STOP aktifken bu state'e girilmesi lazeri
            # ateşlerdi.
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error(
                    '[SHOOT] E-STOP aktif — ateş edilmiyor.'
                )
                self._shoot_pub.publish(Bool(data=False))
                return 'e_stop'

            # Koşu saati yalnız denemeler ARASINDA bakılır: başlamış bir atış
            # yarıda kesilemez, §6.10 lazerin en az 1 s aktif kalmasını ve o
            # süre boyunca araca hareket verilmemesini şart koşuyor.
            if deneme > 1 and self.saat.doldu():
                self.node.get_logger().error(
                    '[SHOOT] §6.12 koşu süresi doldu — kalan denemeler '
                    'kullanılmıyor.'
                )
                break

            self.node.get_logger().info(
                f'[SHOOT] Deneme {deneme}/{self.MAX_DENEME}'
            )
            self._confirmed_event.clear()

            # Ateş komutu — seri_kopru PKT_LAZER=1 → MEGA → NANO → Lazer
            # NOT: seri_kopru bu True yayınını aldığı an _lazer_aktif=True
            # yapar ve TÜM hareket komutlarını PKT_DUR'a çevirir (movement
            # lock). Bu kilit, aşağıda False yayınlanana kadar açık kalır.
            self._shoot_pub.publish(Bool(data=True))
            ates_baslangic = time.time()

            # Onay bekle (seri_kopru PKT_LAZER=0 echo'sunda publish eder)
            # threading.Event.wait — spin thread'den gelen set() atomik olarak yakalanır
            onaylandi = self._confirmed_event.wait(timeout=self.TIMEOUT_S)

            # Şartname §6.10: lazer aktif olduktan sonra EN AZ 1 saniye
            # (LASER_FIRE_DURATION) hedefte/aktif kalmalı. Donanım (Nano)
            # zamanlamasına tek başına güvenmek yerine yazılım seviyesinde
            # de minimum süreyi garanti ediyoruz — hareket kilidi bu süre
            # boyunca kesinlikle açık kalır.
            gecen = time.time() - ates_baslangic
            if gecen < LASER_FIRE_DURATION:
                time.sleep(LASER_FIRE_DURATION - gecen)

            # Hareket kilidini aç — bir sonraki deneme/parkura geçiş için
            # seri_kopru._lazer_aktif'i False'a çeker. Bu çağrı yapılmazsa
            # kilit kalıcı olarak açık kalır ve araç bir daha hiç hareket
            # edemez (kritik güvenlik/görev hatası).
            self._shoot_pub.publish(Bool(data=False))

            if onaylandi:
                basarili_deneme = deneme
                self.node.get_logger().info(
                    f'[SHOOT] Deneme {deneme} onaylandı ✓'
                )
                break

            self.node.get_logger().warn(
                f'[SHOOT] Deneme {deneme} onayı gelmedi (timeout).'
            )

            # Son deneme değilse kısa bekleme sonrası tekrar dene
            if deneme < self.MAX_DENEME:
                time.sleep(1.0)

        if basarili_deneme > 0:
            self.node.get_logger().info(
                f'[SHOOT] {basarili_deneme}. denemede onaylandı. Devam.'
            )
        else:
            self.node.get_logger().warn(
                '[SHOOT] Hiçbir deneme onaylanmadı — devam ediliyor.'
            )

        return 'shot_fired'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 5 — HIZLANMA
# ═══════════════════════════════════════════════════════════════════════════════

class HizlanmaState(smach.State):
    """
    §6.11 Hızlanma Parkuru — Nav2 bypass, doğrudan /cmd_vel → mod_yoneticisi.

    NavigateState buraya waypoint'in `type: hizlanma` alanıyla geçer, tabela
    tespitiyle değil: hızlanma parkuru koordinattan biliniyor ve tek bir YOLO
    karesine bağlanamayacak kadar pahalı (§6.11 ilk altı takıma puan veriyor).
    Tabela_11 bu state'te yalnız `terrain_adapter`'ın `fast` profilini
    tetikler; Tabela_11_son ise aşağıdaki erken çıkışı.
    Nav2 path planner devre dışı — düz pistte gereksiz overhead.
    RC override hâlâ çalışır (mod_yoneticisi /cmd_vel'i dinlemeye devam eder).

    /cmd_vel'in ikinci yayıncısı Nav2'nin velocity_smoother'ıdır. Girdisi
    kesildiğinde smoother susmaz: araç hareket hâlindeyse komutu sıfıra çekip
    max_decel ile yavaşlayan bir rampa yayınlar, ancak sıfıra ulaşınca
    (stopped_) yayını tamamen keser. Bu state'e araç HAREKET HÂLİNDEYKEN
    girilirse iki yayıncı yaklaşık bir saniye boyunca aynı topic'e yazar ve
    mod_yoneticisi son geleni geçirdiği için gaz titrer. Normal akışta sorun
    çıkmaz: NavigateState hedefe varınca Nav2 aracı zaten durdurur ve smoother
    susmuş olur. Buraya "araç yürürken" atlanacak bir yol eklenirse önce
    smoother'ın susması beklenmelidir.

    Hız profili:
      0  .. OLCUM_MESAFE (30m) → MAX_HIZ — puanlanan bölüm sonuna kadar tam gaz
      çizgide                  → 0 komutu, fren zinciri devralır
      30 .. +DURMA_MESAFE(10m) → duruş izlenir, gerçekleşen mesafe loglanır
      Erken çıkış: Tabela_11_son tespiti VEYA timeout

    Geçişler: 'completed' → NavigateState
              'failed'    → ErrorRecoveryState
    """

    # Ölçüm (2026-07-28): gaz voltajı = 0.90 + hız[m/s]; tam gaz 2.84 V ≈
    # 1.94 m/s. Eski 3.0 değeri seri_kopru.MAX_HIZ_MS donanım kelepçesiydi,
    # aracın ulaşabildiği bir hız değil — 3.90 V ister, DAC/sürücü doyar.
    MAX_HIZ          = 1.90   # m/s — ölçülen tam gazın hemen altı
    # Şartname §6.11 iki ayrı mesafe tanımlıyor: 30 m puanlanan hızlanma
    # bölümü, ardından 10 m emniyetli durma payı. Fren payın içinde yapılır;
    # ölçülen bölümde yavaşlamak yalnız ilk altı takıma puan veren bir
    # kalemde sıralama kaybettirir. 1.90 m/s'den 10 m'de durmak 0.18 m/s²
    # ister, yani pay fazlasıyla geniş.
    OLCUM_MESAFE     = 30.0   # m — puanlanan bölüm
    DURMA_MESAFE     = 10.0   # m — çizgi sonrası emniyetli durma payı
    DURMA_HIZ_ESIGI  = 0.05   # m/s — bunun altı "durdu" sayılır
    # Payı tam hızda katetmek 5.3 s sürer, duruş bundan kısadır. Bu guard
    # odometri hareket halindeyken donarsa döngünün asılı kalmasını önler.
    DURMA_TIMEOUT_S  = 10.0   # s
    # Timeout mesafeden türetilir. Sabit 15 s, 30 m için 2.0 m/s ORTALAMA
    # gerektiriyordu; ölçülen tepe hız 1.94 m/s olduğundan parkur ivmelenme
    # süresi sıfır olsa bile her koşuda timeout'la bitiyordu. Bu bir kontrol
    # parametresi değil, donanım arızasına karşı üst limit: en kötü hâlde
    # ortalama hızın MAX_HIZ'in %40'ı olduğu varsayılır (kalkış sürtünmesi +
    # durma payı).
    TIMEOUT_S        = (OLCUM_MESAFE + DURMA_MESAFE) / (MAX_HIZ * 0.40)
    CMD_HZ           = 10.0   # Hz
    # Odom donmuşsa (enkoder yok/kopuk) /odom sabit (0,0) yayınlar; dist hep 0
    # kalır, profil hep MAX_HIZ verir ve araç timeout'a kadar tam gaz gider.
    ODOM_ILERLEME_S  = 4.0    # s — bu süre sonunda ilerleme yoksa iptal
    ODOM_ILERLEME_M  = 0.20   # m
    ODOM_BAYATLAMA_S = 1.0    # s — /odom mesajı bu süre gelmezse iptal

    def __init__(self, node: Node, det_store: DetectionsStore,
                 saat: 'MisyonSaati'):
        smach.State.__init__(self, outcomes=['completed', 'failed', 'e_stop'])
        self.saat = saat
        self.node      = node
        self.det_store = det_store

        self._cmd_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._lock     = threading.Lock()
        self._pos      = None   # (x, y) — son odom konumu
        self._hiz      = 0.0    # m/s — durma payının izlenmesi için
        self._son_odom = 0.0    # wall-clock — son /odom mesajının geliş anı

        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
            self._hiz = msg.twist.twist.linear.x
            self._son_odom = time.time()

    def _hiz_oku(self) -> float:
        with self._lock:
            return self._hiz

    def _odom_yasi(self) -> float:
        with self._lock:
            return time.time() - self._son_odom

    def _mesafe(self, baslangic) -> float:
        with self._lock:
            if self._pos is None:
                return 0.0
            dx = self._pos[0] - baslangic[0]
            dy = self._pos[1] - baslangic[1]
        return math.sqrt(dx * dx + dy * dy)

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='HIZLANMA'))

        # Odom başlangıç konumunu al (maks 2s bekle)
        deadline_init = time.time() + 2.0
        while time.time() < deadline_init:
            with self._lock:
                if self._pos is not None:
                    baslangic = self._pos
                    break
            time.sleep(0.05)
        else:
            self.node.get_logger().error('[HIZLANMA] Odom verisi yok — iptal.')
            return 'failed'

        self.node.get_logger().info(
            f'[HIZLANMA] Başladı. Hedef: {self.OLCUM_MESAFE:.0f}m @ {self.MAX_HIZ:.1f} m/s'
        )

        twist    = Twist()
        dt       = 1.0 / self.CMD_HZ
        baslama  = time.time()
        deadline = baslama + self.TIMEOUT_S
        sonuc    = 'completed'

        while rclpy.ok():
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error('[HIZLANMA] E-STOP — durduruluyor.')
                sonuc = 'e_stop'
                break

            if self.det_store.get_field('manual_mod', False):
                self.node.get_logger().warn(
                    '[HIZLANMA] Manuel mod — RC devraldı, bekleniyor.',
                    throttle_duration_sec=2.0,
                )
                twist.linear.x = 0.0
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                # Manuel duraklama timeout bütçesinden sayılmaz; sayılsaydı
                # deadline kontrolü bu daldan sonra geldiği için timeout hiç
                # dolmaz ve RC manuelde kaldığı sürece döngü sonsuza giderdi.
                deadline += 0.1
                baslama  += 0.1
                continue

            if time.time() > deadline:
                self.node.get_logger().warn('[HIZLANMA] Timeout — durduruluyor.')
                break

            # §6.12: koşu saati manuel duraklamalarla uzayan aşama bütçesinden
            # bağımsız işler; dolduysa 30 m tamamlanmamış olsa da fren.
            if self.saat.doldu():
                self.node.get_logger().error(
                    '[HIZLANMA] §6.12 koşu süresi doldu — fren.'
                )
                break

            # Odom sağlığı: kopuk/donmuş enkoder mesafeyi hep 0 gösterir,
            # profil hep MAX_HIZ verir ve araç timeout'a kadar tam gaz gider.
            yas = self._odom_yasi()
            if yas > self.ODOM_BAYATLAMA_S:
                self.node.get_logger().error(
                    f'[HIZLANMA] /odom {yas:.1f}s bayat — iptal.'
                )
                sonuc = 'failed'
                break

            dist = self._mesafe(baslangic)

            if (time.time() - baslama > self.ODOM_ILERLEME_S
                    and dist < self.ODOM_ILERLEME_M):
                self.node.get_logger().error(
                    f'[HIZLANMA] {self.ODOM_ILERLEME_S:.0f}s tam gazda ilerleme '
                    f'{dist:.2f}m — odometri donmuş veya araç hareket etmiyor, iptal.'
                )
                sonuc = 'failed'
                break

            if self.det_store.get_field('hizlanma_bitti', False):
                self.node.get_logger().info(
                    f'[HIZLANMA] Tabela_11_son tespit ({dist:.1f}m) — fren.'
                )
                break

            if dist >= self.OLCUM_MESAFE:
                self.node.get_logger().info(
                    f'[HIZLANMA] {self.OLCUM_MESAFE:.0f}m tamamlandı — fren.'
                )
                break

            # Hız profili — pure_logic.hizlanma_hiz_profili (test_birim.py
            # bu fonksiyonu doğrudan test eder).
            hiz = hizlanma_hiz_profili(dist, self.MAX_HIZ, self.OLCUM_MESAFE)

            twist.linear.x  = hiz
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)

            self.node.get_logger().info(
                f'[HIZLANMA] dist={dist:.1f}m  hız={hiz:.1f}m/s',
                throttle_duration_sec=0.5,
            )
            time.sleep(dt)

        # Durma payı — §6.11 bitiş çizgisinin ötesinde 10 m veriyor ve o pay
        # içinde duramayan araca ceza yazıyor. Sıfır komutunu birkaç kez
        # yayınlayıp çıkmak duruşu kimsenin izlemediği anlamına gelirdi:
        # kontrol bir sonraki state'e geçer, gerçekleşen durma mesafesi hiç
        # bilinmez ve ceza ancak hakem masasında öğrenilir.
        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        cizgi      = self._mesafe(baslangic)
        durma_basi = time.time()
        durum      = 'devam'

        while rclpy.ok():
            self._cmd_pub.publish(twist)
            asilan = max(0.0, self._mesafe(baslangic) - cizgi)
            durum  = durma_degerlendir(
                asilan, self._hiz_oku(), self.DURMA_MESAFE,
                self.DURMA_HIZ_ESIGI,
            )
            if durum != 'devam':
                break
            if time.time() - durma_basi > self.DURMA_TIMEOUT_S:
                durum = 'butce_asildi'
                break
            time.sleep(dt)

        if durum == 'butce_asildi':
            self.node.get_logger().error(
                f'[HIZLANMA] §6.11 durma payı aşıldı — {asilan:.1f}m/'
                f'{self.DURMA_MESAFE:.0f}m, hız {self._hiz_oku():.2f} m/s. '
                f'Ceza puanı riski.'
            )
        else:
            self.node.get_logger().info(
                f'[HIZLANMA] {asilan:.1f}m içinde durdu '
                f'(pay {self.DURMA_MESAFE:.0f}m).'
            )

        self.node.get_logger().info(f'[HIZLANMA] Tamamlandı → {sonuc}.')
        return sonuc


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 6 — RAMPA  (§6.10 dik eğim, Nav2 baypas)
# ═══════════════════════════════════════════════════════════════════════════════

class RampaState(smach.State):
    """
    §6.10 dik eğimi Nav2'ye sormadan sürer. HizlanmaState ile aynı desen:
    doğrudan /cmd_vel, planlayıcı devre dışı, güvenlik zinciri (mod_yoneticisi
    E-STOP sıfırlaması + speed_limit kırpması + ackermann_converter kelepçeleri)
    aynen mirasta kalır.

    NEDEN BAYPAS
    ─────────────────────────────────────────────────────────────────────────
    Rampada planlayıcının verecek kararı yok: koridor 3 m, direksiyon kararı
    yok, ve rampanın ayak izinde bariyer yok (parkur CAD'inde dört bariyerin
    dördü de giriş/çıkışta). Buna karşılık Nav2 rampada iki ayrı yanlış
    üretiyor:

      1. 0,55 m'deki yatay tarama düzlemi %45 eğimli yüzeyi tabanından
         0,55/0,45 = 1,22 m içeride keser ve orada DUVAR okur. 2B taramada
         eğimli zeminle dik duvar geometrik olarak ayırt edilemez.
      2. base_footprint→base_link statik (0 0 0) ve ekf two_d_mode roll/pitch'i
         her döngüde sıfırlıyor: TF ağacı aracın eğildiğini hiç bilmez, rampa
         üzerindeyken her tarama dönüşü düz zemine izdüşürülür.

    FAZ SİNYALİ PİTCH, KONUM DEĞİL
    ─────────────────────────────────────────────────────────────────────────
    Waypoint koordinatları kayabilir ve odometri sapar; ama araç eğimdeyse IMU
    bunu mutlak olarak söyler. Karar pure_logic.rampa_faz_gecisi'nde, pitch'in
    MUTLAK değeriyle — böylece aynı mantık hem tırmanışta hem inişte çalışır.
    Aşamanın tırmanış mı iniş mi olduğu waypoint etiketinden okunur, çünkü
    rampanın tepesinde (atış noktası, rampa ayak izinin tam ortası) pitch iki
    durumda da ~0'dır ve tek başına ayırt etmez.

    DURUŞ
    ─────────────────────────────────────────────────────────────────────────
    §6.10 iki saniyelik duruş istiyor. Sayaç, sıfır komutu basıldığı an değil
    araç GERÇEKTEN durduktan sonra başlar: hakemin kronometresi aracı ölçüyor,
    bizim niyetimizi değil. Yavaşlama velocity_smoother'ın ivme rampasına tabi
    olduğu için ikisi arasında ölçülmemiş bir gecikme var.

    Geçişler: 'completed' → NavigateState
              'failed'    → ErrorRecoveryState
              'e_stop'    → MissionAbortState
    """

    CMD_HZ            = 10.0    # Hz
    PITCH_ESIK_DEG    = IMU_PITCH_RAMP_THRESHOLD
    DURMA_HIZ_ESIGI   = 0.05    # m/s — bunun altı "durdu" sayılır
    DURMA_TIMEOUT_S   = 8.0     # s — bu sürede duramazsa yine de devam et
    YAKLASMA_MAX_M    = 3.0     # m — bu kadar sürüldüğü hâlde eğim yoksa iptal
    EGIM_MAX_M        = 6.0     # m — CAD: tırmanış koşusu 4,13 m
    # Donanım arızası korumaları — HizlanmaState'ten. Rampada daha kritik:
    # kopuk enkoderle mesafe hep 0 okunur, faz hiç bitmez ve araç eğimde
    # timeout'a kadar gaz vermeye devam eder.
    ODOM_BAYATLAMA_S  = 1.0     # s
    ODOM_ILERLEME_S   = 4.0     # s
    ODOM_ILERLEME_M   = 0.20    # m
    # Geri kayma bu süre boyunca sürerse tahrik yetmiyor demektir; daha fazla
    # gaz vermek çözmez. anti_rollback düğümü ackermann_converter üzerinden
    # zaten karşı komut basıyor — burada onunla yarışılmaz, yalnız iptal edilir.
    GERI_KAYMA_S      = 1.5     # s

    def __init__(self, node: Node, det_store: DetectionsStore,
                 saat: 'MisyonSaati',
                 tirmanis_hiz: float = 0.6, inis_hiz: float = 0.4):
        smach.State.__init__(self, outcomes=['completed', 'failed', 'e_stop'],
                             input_keys=['current_wp'])
        self.node          = node
        self.det_store     = det_store
        self.saat          = saat
        self.tirmanis_hiz  = tirmanis_hiz
        self.inis_hiz      = inis_hiz



        self._cmd_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._lock     = threading.Lock()
        self._pos      = None
        self._hiz      = 0.0
        self._pitch    = 0.0
        self._son_odom = 0.0

        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        # IMU BEST_EFFORT yayınlanıyor (anti_rollback ile aynı profil).
        # RELIABLE abone olunursa eşleşme kurulmaz ve callback HİÇ çağrılmaz —
        # hata basılmaz, pitch sonsuza kadar 0 kalır ve faz hiç değişmez.
        node.create_subscription(
            Imu, IMU_TOPIC, self._on_imu,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    # ── Sensör callback'leri ────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self._hiz = msg.twist.twist.linear.x
            self._son_odom = time.time()

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        _, pitch = quat_to_roll_pitch_deg(q.w, q.x, q.y, q.z)
        with self._lock:
            self._pitch = pitch

    def _oku(self):
        with self._lock:
            return self._pos, self._hiz, self._pitch, self._son_odom

    def _timeout(self, hedef_hiz: float) -> float:
        """Aşama üst sınırı — HizlanmaState ile aynı gerekçe.

        Sabit bir sayı hıza göre ya hiç dolmaz ya her koşuda dolar. Bu bir
        kontrol parametresi değil, donanım arızasına karşı üst sınır: en kötü
        hâlde ortalama hızın hedefin %40'ı olduğu varsayılır (kalkış
        sürtünmesi, eğimde tork düşüşü), üstüne duruş payı eklenir. Tırmanış
        ve iniş farklı hızlarda sürüldüğü için her faz kendi bütçesini alır.
        """
        return ((self.YAKLASMA_MAX_M + self.EGIM_MAX_M)
                / max(0.05, hedef_hiz * 0.40)
                + self.DURMA_TIMEOUT_S + RAMP_STOP_DURATION)

    @staticmethod
    def _mesafe(a, b):
        if a is None or b is None:
            return 0.0
        return math.hypot(b[0] - a[0], b[1] - a[1])

    # ── Duruş ───────────────────────────────────────────────────────────────

    def _dur_ve_bekle(self, sure_s: float, etiket: str) -> str:
        """Sıfır komutu basar, GERÇEKTEN durmayı bekler, sonra sure_s sayar."""
        twist = Twist()
        durma_basi = time.time()
        durdu_mu   = False

        while rclpy.ok():
            self._cmd_pub.publish(twist)
            if self.det_store.get_field('e_stop', False):
                return 'e_stop'
            _, hiz, _, _ = self._oku()
            if abs(hiz) < self.DURMA_HIZ_ESIGI:
                durdu_mu = True
                break
            if time.time() - durma_basi > self.DURMA_TIMEOUT_S:
                self.node.get_logger().error(
                    f'[RAMPA] {etiket}: {self.DURMA_TIMEOUT_S:.0f}s içinde '
                    f'durulamadı (hız={hiz:.2f} m/s) — §6.10 duruşu şüpheli.'
                )
                break
            time.sleep(1.0 / self.CMD_HZ)

        if durdu_mu:
            self.node.get_logger().info(
                f'[RAMPA] {etiket}: durdu ({time.time() - durma_basi:.1f}s), '
                f'§6.10 {sure_s:.0f}s bekleme başlıyor.'
            )

        bekleme_bitis = time.time() + sure_s
        while time.time() < bekleme_bitis:
            self._cmd_pub.publish(twist)
            if self.det_store.get_field('e_stop', False):
                return 'e_stop'
            time.sleep(1.0 / self.CMD_HZ)
        return 'ok'

    # ── Ana akış ────────────────────────────────────────────────────────────

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='RAMPA'))

        wp     = userdata.current_wp or {}
        etiket = wp.get('label', 'DIK_EGIM')
        iniyor = 'CIKIS' in etiket           # CIKIS = rampadan iniş aşaması
        hedef_hiz = self.inis_hiz if iniyor else self.tirmanis_hiz
        yon = 'iniş' if iniyor else 'tırmanış'

        # Odom başlangıcı — sensör yoksa hiç başlama
        deadline_init = time.time() + 2.0
        while time.time() < deadline_init:
            pos, _, _, _ = self._oku()
            if pos is not None:
                break
            time.sleep(0.05)
        else:
            self.node.get_logger().error('[RAMPA] /odom yok — iptal.')
            return 'failed'

        self.node.get_logger().info(
            f'[RAMPA] {etiket} ({yon}) başladı — hedef hız {hedef_hiz:.2f} m/s, '
            f'pitch eşiği {self.PITCH_ESIK_DEG:.0f}°. Nav2 baypas.'
        )

        twist         = Twist()
        dt            = 1.0 / self.CMD_HZ
        baslama       = time.time()
        timeout_s     = self._timeout(hedef_hiz)
        deadline      = baslama + timeout_s
        faz           = 'yaklasma'
        baslangic_pos = self._oku()[0]
        faz_basi      = baslangic_pos
        kayma_basi    = None
        sonuc         = None

        while rclpy.ok():
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error('[RAMPA] E-STOP — durduruluyor.')
                sonuc = 'e_stop'
                break

            if self.det_store.get_field('manual_mod', False):
                self.node.get_logger().warn(
                    '[RAMPA] Manuel mod — RC devraldı, bekleniyor.',
                    throttle_duration_sec=2.0,
                )
                twist.linear.x = 0.0
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                # Manuel duraklama bütçeden sayılmaz (HizlanmaState ile aynı).
                deadline += 0.1
                baslama  += 0.1
                continue

            if time.time() > deadline:
                self.node.get_logger().error(
                    f'[RAMPA] {timeout_s:.0f}s timeout — iptal.'
                )
                sonuc = 'failed'
                break

            if self.saat.doldu():
                self.node.get_logger().error(
                    '[RAMPA] §6.12 koşu süresi doldu — durduruluyor.'
                )
                sonuc = 'failed'
                break

            pos, hiz, pitch, son_odom = self._oku()

            yas = time.time() - son_odom
            if yas > self.ODOM_BAYATLAMA_S:
                self.node.get_logger().error(
                    f'[RAMPA] /odom {yas:.1f}s bayat — eğimde körlemesine '
                    'sürülmez, iptal.'
                )
                sonuc = 'failed'
                break

            gidilen = self._mesafe(faz_basi, pos)
            # İlerleme koruması TOPLAM yola bakar: gidilen faz değişiminde
            # sıfırlandığı için onunla ölçmek her faz geçişinde sahte alarm verir.
            toplam = self._mesafe(baslangic_pos, pos)
            if (time.time() - baslama > self.ODOM_ILERLEME_S
                    and toplam < self.ODOM_ILERLEME_M):
                self.node.get_logger().error(
                    f'[RAMPA] {self.ODOM_ILERLEME_S:.0f}s gazda ilerleme '
                    f'{toplam:.2f}m — takılma veya donmuş odometri, iptal.'
                )
                sonuc = 'failed'
                break

            # Geri kayma: yalnız tırmanışta anlamlı (rollback_riskli pozitif
            # pitch şartı arar). Sürekliyse tahrik yetmiyordur.
            if rollback_riskli(pitch, hiz, self.PITCH_ESIK_DEG, 0.05):
                if kayma_basi is None:
                    kayma_basi = time.time()
                    self.node.get_logger().warn(
                        f'[RAMPA] GERİ KAYMA (pitch={pitch:.1f}°, '
                        f'hız={hiz:.2f} m/s) — anti_rollback devrede olmalı.'
                    )
                elif time.time() - kayma_basi > self.GERI_KAYMA_S:
                    self.node.get_logger().error(
                        f'[RAMPA] Geri kayma {self.GERI_KAYMA_S:.1f}s sürdü — '
                        'tahrik yetersiz, gaz kesiliyor. Aracı fren tutmalı.'
                    )
                    sonuc = 'failed'
                    break
            else:
                kayma_basi = None

            yeni_faz = rampa_faz_gecisi(
                faz, abs(pitch), self.PITCH_ESIK_DEG, gidilen,
                self.YAKLASMA_MAX_M, self.EGIM_MAX_M,
            )
            if yeni_faz != faz:
                self.node.get_logger().info(
                    f'[RAMPA] faz {faz} → {yeni_faz} '
                    f'(pitch={pitch:.1f}°, {gidilen:.2f}m)'
                )
                faz      = yeni_faz
                faz_basi = pos

            if faz == 'bitti':
                break
            if faz == 'bulunamadi':
                self.node.get_logger().error(
                    f'[RAMPA] {self.YAKLASMA_MAX_M:.0f}m sürüldü, eğim '
                    f'başlamadı (pitch={pitch:.1f}°) — araç rampanın önünde '
                    'değil ya da IMU pitch okuması yanlış. İptal.'
                )
                sonuc = 'failed'
                break
            if faz == 'asildi':
                self.node.get_logger().error(
                    f'[RAMPA] Eğim {self.EGIM_MAX_M:.0f}m sürdü ve bitmedi — '
                    'pitch okuması şüpheli. İptal.'
                )
                sonuc = 'failed'
                break

            twist.linear.x  = hedef_hiz
            twist.angular.z = 0.0        # koridor düz, direksiyon kararı yok
            self._cmd_pub.publish(twist)

            self.node.get_logger().info(
                f'[RAMPA] {yon} faz={faz} pitch={pitch:.1f}° '
                f'{gidilen:.2f}m (toplam {toplam:.2f}m) hız={hiz:.2f}',
                throttle_duration_sec=0.5,
            )
            time.sleep(dt)

        # Hangi yoldan çıkılırsa çıkılsın gaz kesilir.
        twist.linear.x  = 0.0
        twist.angular.z = 0.0
        self._cmd_pub.publish(twist)

        if sonuc is not None:
            return sonuc

        # §6.10 zorunlu duruş — araç gerçekten durduktan SONRA sayılır.
        if self._dur_ve_bekle(RAMP_STOP_DURATION, etiket) == 'e_stop':
            return 'e_stop'

        self.node.get_logger().info(f'[RAMPA] {etiket} tamamlandı.')
        return 'completed'

# ═══════════════════════════════════════════════════════════════════════════════
# STATE 7 — MISSION_COMPLETE
# ═══════════════════════════════════════════════════════════════════════════════

class MissionCompleteState(smach.State):
    """
    Görevin terminal durumu. İki örneği kurulur:

      basarili=True  → 'MISSION_COMPLETE', /mission_status = 'COMPLETE'
      basarili=False → 'MISSION_ABORT',    /mission_status = 'ABORTED'

    Ayrım şart: iptal de 'COMPLETE' yayınlarsa, Nav2 ayakta değilken FSM
    ~21 saniyede (wait_for_server 5s → kurtarma 2s, üç tur) hiç hareket
    etmeden "görev tamamlandı" der ve bu logda başarıdan ayırt edilemez.

    Geçişler: 'done' → (terminal)
    """

    def __init__(self, node: Node, basarili: bool = True):
        smach.State.__init__(self, outcomes=['done'])
        self.node        = node
        self._basarili   = basarili
        self._status_pub = node.create_publisher(String, MISSION_STATUS_TOPIC, 10)
        self._misyon_pub = node.create_publisher(Bool, MISYON_AKTIF_TOPIC, 10)

    def execute(self, userdata):
        durum = 'MISSION_COMPLETE' if self._basarili else 'MISSION_ABORT'
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data=durum))
        if self._basarili:
            self.node.get_logger().info('═══ GÖREV TAMAMLANDI ═══')
            self._status_pub.publish(String(data='COMPLETE'))
        else:
            self.node.get_logger().error('═══ GÖREV İPTAL EDİLDİ ═══')
            self._status_pub.publish(String(data='ABORTED'))
        # Veri kaydını durdur
        self._misyon_pub.publish(Bool(data=False))
        return 'done'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 8 — ERROR_RECOVERY
# ═══════════════════════════════════════════════════════════════════════════════

class ErrorRecoveryState(smach.State):
    """
    Hata kurtarma. Bir aşama başarısız olduğunda buraya düşülür.

    Aşama MAX_RETRIES kez denenir; hâlâ olmuyorsa o aşamadan VAZGEÇİLİR ve
    sıradaki waypoint'e geçilir. Görev iptal edilmez: tek bir istasyonda
    takılmak koşunun tamamını götürüyordu — 5 puanlık bir aşama uğruna
    koniler (50), kayar engel (50), atış (50), hızlanma (25) ve sıralamaya
    giren koşu süresi (100) kaybediliyordu.

    Vazgeçmek §9 pası DEĞİLDİR: pas, takımın parkura girip aracı aşamanın
    sonuna elle taşımasıdır ve o aşamanın puanını eksiye yazar. Burada
    yapılan yalnız "bu hedefe ulaşmayı bırak, sıradakine nişan al" —
    istasyonlar tek koridorda arka arkaya dizili olduğu için araç engelin
    içinden geçmeyi fiilen denemeye devam eder.

    Görevi bitiren iki yol buraya HİÇ uğramaz: E-STOP ('e_stop' çıktısı) ve
    §6.12 süre dolması ('sure_doldu'). İkisi de doğrudan MISSION_ABORT'a
    gider — E-STOP'un kurtarma denemesi harcaması aracı saniyelerce
    duraklatıyordu.

    Sayaç userdata'da tutulur; NavigateState bir aşamayı tamamladığında
    sıfırlar (bütçe aşama başınadır, görev başına değil).

    Geçişler: 'recovered'     → NavigateState (aynı waypoint tekrar)
              'sonraki_asama' → NavigateState (bu aşamadan vazgeçildi)
    """

    # Her deneme koşu saatinden bir aşama bütçesi yiyor; koşu süresi 100
    # puanla tablodaki en büyük kalem olduğu için deneme sayısı düşük tutuldu.
    MAX_RETRIES = 2

    def __init__(self, node: Node):
        smach.State.__init__(
            self,
            outcomes=['recovered', 'sonraki_asama'],
            input_keys=['wp_index', 'retry_count', 'waypoints'],
            output_keys=['wp_index', 'retry_count']
        )
        self.node = node
        self.birakilan = 0          # bu koşuda vazgeçilen aşama sayısı

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='ERROR_RECOVERY'))
        sayac = userdata.retry_count + 1
        userdata.retry_count = sayac
        self.node.get_logger().warn(
            f'[ERROR_RECOVERY] Deneme {sayac}/{self.MAX_RETRIES}'
        )

        if _kurtarma_karari_pure(sayac, self.MAX_RETRIES) == 'sonraki':
            idx   = userdata.wp_index
            wpler = getattr(userdata, 'waypoints', None) or []
            etiket = (wpler[idx].get('label', f'#{idx}')
                      if idx < len(wpler) else f'#{idx}')
            self.birakilan += 1
            self.node.get_logger().error(
                f'[ERROR_RECOVERY] {etiket}: {self.MAX_RETRIES} deneme de '
                f'başarısız — bu aşamadan vazgeçilip sıradakine geçiliyor '
                f'(bu koşuda bırakılan aşama: {self.birakilan}). '
                'Görev SÜRÜYOR; bu bir §9 pası değildir.'
            )
            userdata.wp_index    = idx + 1
            userdata.retry_count = 0
            return 'sonraki_asama'

        # Başarısız olan waypoint TEKRAR denenir. wp_index navigasyon
        # başarısızlığında henüz artmamıştır; indeksi geri almak bir ÖNCEKİ
        # (tamamlanmış) aşamaya döner ve varışta o aşamanın tipi yeniden
        # okunur — ATIS_BOLGESI'ne dönülürse lazer ikinci kez ateşlenir,
        # HIZLANMA_PARKURU'na dönülürse 30 m'lik koşu baştan başlar.
        time.sleep(2.0)
        return 'recovered'


# ═══════════════════════════════════════════════════════════════════════════════
# WAYPOINT YÜKLEYİCİ
# ═══════════════════════════════════════════════════════════════════════════════

def load_waypoints(yaml_path: str):
    """
    waypoints.yaml → (waypoint listesi, parametreler dict).

    'ATIS_BOLGESI' isimli aşama type='shoot' olarak işaretlenir.
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
            if isim == 'ATIS_BOLGESI':
                tip = 'shoot'
            elif isim == 'HIZLANMA_PARKURU':
                tip = 'hizlanma'
            elif isim in DIK_EGIM_ETIKETLERI:
                # §6.10 eğimi RampaState sürüyor; Nav2 yalnız rampanın
                # başına kadar getiriyor.
                tip = 'rampa'
            else:
                tip = 'nav'
            waypoints.append({
                'x':           wp.get('x', 0.0),
                'y':           wp.get('y', 0.0),
                'yaw':         wp.get('yaw', 0.0),
                'type':        tip,
                'label':       isim,
                'pas_gecilir': a.get('pas_gecilir', False),
                # §9: pas geçilen aşamanın alınabilecek en yüksek puanı eksi
                # olarak yazılır. Log'da bedelin görünmesi için taşınıyor.
                'pas_puan':    a.get('pas_puan', 0.0),
            })

        parametreler = data.get('parametreler', {})
        print(f'[WAYPOINTS] {len(waypoints)} waypoint yüklendi.')

        # waypoints.yaml deposa placeholder (hepsi 0.0) olarak giriyor. Dolmadan
        # çalıştırılırsa NavigateState her istasyon için aynı hedefi (map
        # orijini) gönderir: araç ilk hedefe varır, sonraki aşamalar anında
        # "ulaşıldı" sayılır ve FSM parkuru hiç sürmeden tamamlar. Log'da
        # tek bir hata satırı bile görünmez — o yüzden burada yüksek sesle
        # uyarılıyor.
        sifir = [w for w in waypoints if w['x'] == 0.0 and w['y'] == 0.0]
        if sifir and len(sifir) == len(waypoints):
            print('[WAYPOINTS] ' + '!' * 60)
            print('[WAYPOINTS] TÜM KOORDİNATLAR PLACEHOLDER (0,0) — parkur '
                  'sürülmez.')
            print('[WAYPOINTS] SLAM haritası alınıp rviz2 "2D Nav Goal" ile '
                  f'{yaml_path} doldurulmalı.')
            print('[WAYPOINTS] ' + '!' * 60)
        elif sifir:
            isimler = ', '.join(w['label'] for w in sifir)
            print(f'[WAYPOINTS] UYARI: {len(sifir)} waypoint hâlâ (0,0) — '
                  f'{isimler}')

        return waypoints, parametreler

    except FileNotFoundError:
        print(f'[WAYPOINTS] UYARI: {yaml_path} bulunamadı. Boş liste.')
        return [], {}
    except Exception as e:
        print(f'[WAYPOINTS] HATA: {e}')
        return [], {}


# ═══════════════════════════════════════════════════════════════════════════════
# ANA ÇALIŞMA
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = rclpy.create_node('misyon_fsm')

    # ── Detections Store ──────────────────────────────────────────────
    det_store = DetectionsStore()

    # FSM durum yayını: tüm ekiplerin izleyebileceği string durum
    fsm_state_pub = node.create_publisher(String, FSM_STATE_TOPIC, 10)
    # node üzerinden state'lerin erişmesi için referans
    node._fsm_state_pub = fsm_state_pub

    def _on_detections(msg: String):
        try:
            data = json.loads(msg.data)
            det_store.update(data)
        except (json.JSONDecodeError, ValueError):
            pass

    node.create_subscription(String, DETECTIONS_TOPIC, _on_detections, 10)

    def _on_obs_direction(msg: String):
        det_store.update_field('kayar_yon', msg.data)

    node.create_subscription(String, MOVING_OBS_DIR_TOPIC, _on_obs_direction, 10)

    def _on_e_stop(msg: Bool):
        det_store.update_field('e_stop', msg.data)
        if msg.data:
            node.get_logger().error('!!! E-STOP ALINDI — TÜM NAVIGASYON DURDURULUYOR !!!')

    node.create_subscription(Bool, E_STOP_TOPIC, _on_e_stop, 10)

    def _on_mod(msg: UInt8):
        # 0=MANUAL → FSM navigasyonu bekletir (RC kumanda sürüyor)
        # 2=FULL_AUTO → tam otonom
        det_store.update_field('manual_mod', msg.data == 0)

    node.create_subscription(UInt8, MOD_AKTIF_TOPIC, _on_mod, 10)

    # ── Nav2 Client ───────────────────────────────────────────────────
    nav = Nav2Client(node)

    # ── Waypoint yükle ────────────────────────────────────────────────
    import os
    from ament_index_python.packages import get_package_share_directory
    try:
        pkg_share = get_package_share_directory('teknofest_ika')
        wp_path   = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    except Exception:
        wp_path = os.path.expanduser('~/lydia_ws/config/waypoints.yaml')

    waypoints, parametreler = load_waypoints(wp_path)

    stage_timeout      = float(parametreler.get('asama_timeout_saniye', 120.0))
    baslangic_bekleme  = float(parametreler.get('baslangic_bekleme',    3.0))
    kosu_suresi        = float(parametreler.get('kosu_suresi_saniye', KOSU_SURESI_S))
    # §6.10 rampa hızları — sahada ölçülecek, waypoints.yaml'dan canlı okunur.
    rampa_tirmanis     = float(parametreler.get('rampa_tirmanis_hiz', 0.60))
    rampa_inis         = float(parametreler.get('rampa_inis_hiz',     0.40))
    node.get_logger().info(f'Aşama timeout: {stage_timeout:.0f}s | Başlangıç bekleme: {baslangic_bekleme:.0f}s (waypoints.yaml)')

    saat = MisyonSaati(node, kosu_suresi)
    node.get_logger().info(
        f'Koşu limiti: {kosu_suresi / 60.0:.0f} dk (§6.12) | '
        f'Pas hakkı: {PAS_HAKKI}/koşu (§9) | '
        f'Rampa (Nav2 baypas): tırmanış {rampa_tirmanis:.2f} / '
        f'iniş {rampa_inis:.2f} m/s'
    )
    # Aşama bütçelerinin TOPLAMI koşu limitini aşarsa tek bir takılan aşama
    # koşunun geri kalanını yiyebilir. Toplam artık aşama başına yazılan
    # sürelerden hesaplanıyor; ortak tavan yalnız süresi olmayan aşamalar için.
    butce_toplami = sum(float(w.get('timeout_saniye', stage_timeout))
                        for w in waypoints) if waypoints else stage_timeout
    node.get_logger().info(
        f'Aşama bütçeleri toplamı: {butce_toplami:.0f}s / {kosu_suresi:.0f}s '
        f'(pay {kosu_suresi - butce_toplami:.0f}s). Atış, hızlanma ve zorunlu '
        'beklemeler bu toplamın DIŞINDA — ayrı durumlarda geçiyor.'
    )
    if butce_toplami > kosu_suresi:
        node.get_logger().warn(
            f'Aşama bütçesi toplamı ({butce_toplami:.0f}s) koşu limitini '
            f'({kosu_suresi:.0f}s) aşıyor — aşama süreleri koşu saatiyle '
            'kırpılacak.'
        )

    # ── SMACH FSM Kurulumu ────────────────────────────────────────────
    sm = smach.StateMachine(outcomes=['GOREV_TAMAMLANDI', 'GOREV_IPTAL'])

    sm.userdata.waypoints   = waypoints
    sm.userdata.wp_index    = 0
    sm.userdata.current_wp  = None
    sm.userdata.retry_count = 0

    with sm:
        smach.StateMachine.add(
            'IDLE',
            IdleState(node, saat, baslangic_bekleme),
            transitions={'started': 'NAVIGATE'}
        )

        smach.StateMachine.add(
            'NAVIGATE',
            NavigateState(node, nav, det_store, saat, stage_timeout),
            transitions={
                'next_waypoint':    'NAVIGATE',
                'shoot_waypoint':   'SHOOT_APPROACH',
                'hizlanma_waypoint':'HIZLANMA',
                'mission_complete': 'MISSION_COMPLETE',
                # §6.12 süre dolduğunda görev tamamlanmış değildir: araç
                # parkurdan çıkarılır. 'COMPLETE' yayınlamak logda gerçek
                # bitişten ayırt edilemez olurdu.
                'sure_doldu':       'MISSION_ABORT',
                'failed':           'ERROR_RECOVERY',
                # E-STOP kurtarmaya UĞRAMAZ: kurtarma denemesi harcamak
                # aracı saniyelerce duraklatıyordu.
                'e_stop':           'MISSION_ABORT',
                'rampa_waypoint':   'RAMPA',
            }
        )

        smach.StateMachine.add(
            'RAMPA',
            RampaState(node, det_store, saat, rampa_tirmanis, rampa_inis),
            transitions={
                'completed': 'NAVIGATE',
                'failed':    'ERROR_RECOVERY',
                'e_stop':    'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'HIZLANMA',
            HizlanmaState(node, det_store, saat),
            transitions={
                'completed': 'NAVIGATE',
                'failed':    'ERROR_RECOVERY',
                'e_stop':    'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'SHOOT_APPROACH',
            ShootApproachState(node, det_store),
            transitions={
                'in_position': 'SHOOT',
                'failed':      'ERROR_RECOVERY',
                'e_stop':      'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'SHOOT',
            ShootState(node, det_store, saat),
            transitions={
                'shot_fired': 'NAVIGATE',
                'failed':     'ERROR_RECOVERY',
                'e_stop':     'MISSION_ABORT',
            }
        )

        smach.StateMachine.add(
            'MISSION_COMPLETE',
            MissionCompleteState(node, basarili=True),
            transitions={'done': 'GOREV_TAMAMLANDI'}
        )

        smach.StateMachine.add(
            'MISSION_ABORT',
            MissionCompleteState(node, basarili=False),
            transitions={'done': 'GOREV_IPTAL'}
        )

        smach.StateMachine.add(
            'ERROR_RECOVERY',
            ErrorRecoveryState(node),
            transitions={
                'recovered':     'NAVIGATE',
                # Aşamadan vazgeçildi — görev İPTAL DEĞİL, sıradakine devam.
                'sonraki_asama': 'NAVIGATE',
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
