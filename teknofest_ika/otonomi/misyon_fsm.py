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

from std_msgs.msg import Bool, String, UInt8
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
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
)
from teknofest_ika.otonomi.pure_logic import (
    DetectionsStore, stop_check as _stop_check_pure, hizlanma_hiz_profili,
)

# §6.10: dik eğim çıkış/iniş noktalarında STOP tabelası kaçırılsa bile
# zorunlu 2s bekleme uygulanması gereken waypoint etiketleri.
DIK_EGIM_ETIKETLERI = ('DIK_EGIM_GIRIS', 'DIK_EGIM_CIKIS', 'DIK_EGIM')


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
# STATE 1 — IDLE
# ═══════════════════════════════════════════════════════════════════════════════

class IdleState(smach.State):
    """
    Başlangıç durumu. /mission_start → True gelince çıkar.
    Araç güçlenince hemen hareket etmemesi için bu bekleme zorunludur.

    Geçişler: 'started' → NavigateState
    """

    def __init__(self, node: Node, baslangic_bekleme: float = 3.0):
        smach.State.__init__(self, outcomes=['started'])
        self.node = node
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

    def __init__(self, node: Node, nav: Nav2Client, det_store: DetectionsStore,
                 stage_timeout: float = 120.0):
        smach.State.__init__(
            self,
            outcomes=['next_waypoint', 'shoot_waypoint', 'hizlanma_waypoint',
                      'mission_complete', 'failed'],
            input_keys=['waypoints', 'wp_index'],
            output_keys=['wp_index', 'current_wp']
        )
        self.node                  = node
        self.nav                   = nav
        self.det_store             = det_store
        self.stage_timeout         = stage_timeout
        self._wp_index_pub         = node.create_publisher(UInt8, MISYON_WP_INDEX_TOPIC, 10)
        self._stop_cooldown_bitis  = 0.0   # §6.10: son STOP sonrası tekrar tetiklenme engeli

    def _stop_check(self) -> bool:
        """Nav2Client.go_to() callback'i: True dönünce goal iptal edilir (§6.10 STOP)."""
        return _stop_check_pure(
            self.det_store.get_field('stop_var', False),
            self._stop_cooldown_bitis,
            time.time(),
        )

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='NAVIGATE'))

        if self.det_store.get_field('e_stop', False):
            self.node.get_logger().error('[NAVIGATE] E-STOP aktif — navigasyon iptal.')
            return 'failed'

        waypoints = userdata.waypoints
        idx       = userdata.wp_index

        if self.det_store.get_field('manual_mod', False):
            self.node.get_logger().warn('[NAVIGATE] Manuel mod aktif — navigasyon bekleniyor.')
            while rclpy.ok() and self.det_store.get_field('manual_mod', False):
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error('[NAVIGATE] E-STOP — manuel mod bekleme iptal.')
                    return 'failed'
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

        # Nav2'ye git — STOP işareti görülürse ortada kesilir, 2s beklenir, tekrar gönderilir
        stop_uygulandi = False
        while True:
            result = self.nav.go_to(
                wp['x'], wp['y'], wp.get('yaw', 0.0),
                timeout_sec=self.stage_timeout,
                stop_check_fn=self._stop_check,
            )

            if result == 'stop_requested':
                self.node.get_logger().info(
                    f'[NAVIGATE] §6.10 STOP işareti — araç durduruluyor, 2s bekleniyor.'
                )
                # Cooldown: aynı STOP işaretinin hemen tekrar tetiklenmesini engeller
                # 3s: rampada üst-STOP ile alt-STOP arası için yeterli
                self._stop_cooldown_bitis = time.time() + 3.0
                self.det_store.update_field('stop_var', False)
                stop_uygulandi = True
                deadline_stop = time.time() + RAMP_STOP_DURATION
                while time.time() < deadline_stop:
                    if self.det_store.get_field('e_stop', False):
                        self.node.get_logger().error('[NAVIGATE] E-STOP — STOP bekleme iptal.')
                        return 'failed'
                    time.sleep(0.1)
                self.node.get_logger().info('[NAVIGATE] §6.10 STOP bekleme tamam — devam ediliyor.')
                continue

            if result == 'success':
                break

            # 'failed' veya 'timeout'
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

        # §6.10 yedek tetikleyici: dik eğim giriş/çıkış waypoint'ine
        # YOLO'nun STOP tabelasını kaçırması nedeniyle hiç STOP
        # tetiklenmeden varılmışsa, zorunlu 2s bekleme burada mesafe/
        # konum tabanlı olarak (waypoint varışı = tetikleyici) uygulanır.
        # Tek bir görüntü tespitine bağımlılığı ortadan kaldırır.
        if label in DIK_EGIM_ETIKETLERI and not stop_uygulandi:
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
                    return 'failed'
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
                    return 'failed'
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
        self._wp_index_pub.publish(UInt8(data=min(idx + 1, 255)))

        if wp.get('type') == 'shoot':
            return 'shoot_waypoint'

        if wp.get('type') == 'hizlanma':
            return 'hizlanma_waypoint'

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
            outcomes=['in_position', 'failed'],
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
                return 'failed'

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

    Geçişler: 'shot_fired' → NavigateState
              'failed'     → ErrorRecoveryState
    """

    MAX_DENEME = 3       # Şartname: en fazla 3 deneme
    TIMEOUT_S  = 8.0     # Her deneme için max bekleme

    def __init__(self, node: Node, det_store: DetectionsStore):
        smach.State.__init__(self, outcomes=['shot_fired'])
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

            if onaylandi:
                basarili_deneme = deneme
                self.node.get_logger().info(
                    f'[SHOOT] Deneme {deneme} onaylandı ✓'
                )
            else:
                self.node.get_logger().warn(
                    f'[SHOOT] Deneme {deneme} onayı gelmedi (timeout).'
                )

            # Hareket kilidini aç — bir sonraki deneme/parkura geçiş için
            # seri_kopru._lazer_aktif'i False'a çeker. Bu çağrı yapılmazsa
            # kilit kalıcı olarak açık kalır ve araç bir daha hiç hareket
            # edemez (kritik güvenlik/görev hatası).
            self._shoot_pub.publish(Bool(data=False))

            # Son deneme değilse kısa bekleme sonrası tekrar dene
            if deneme < self.MAX_DENEME:
                if self.det_store.get_field('e_stop', False):
                    self.node.get_logger().error('[SHOOT] E-STOP — atış iptal.')
                    break
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
# STATE 5 — HIZLANMA
# ═══════════════════════════════════════════════════════════════════════════════

class HizlanmaState(smach.State):
    """
    §6.11 Hızlanma Parkuru — Nav2 bypass, doğrudan /cmd_vel → mod_yoneticisi.

    Tabela_11 tespit edince NavigateState bu state'e geçer.
    Nav2 path planner devre dışı — düz pistte gereksiz overhead.
    RC override hâlâ çalışır (mod_yoneticisi /cmd_vel'i dinlemeye devam eder).

    Hız profili:
      0  .. FREN_BASI_MESAFE (25m) → MAX_HIZ (10 m/s)
      25 .. TOPLAM_MESAFE    (30m) → doğrusal düşüş 10→0 m/s
      Erken çıkış: Tabela_11_son tespiti VEYA timeout

    Geçişler: 'completed' → NavigateState
              'failed'    → ErrorRecoveryState
    """

    MAX_HIZ          = 3.0    # m/s — seri_kopru.MAX_HIZ_MS ile eşleşmeli
    TOPLAM_MESAFE    = 30.0   # m
    FREN_BASI_MESAFE = 25.0   # m — son 5m'de yavaşla
    TIMEOUT_S        = 15.0   # s — donanım arızasına karşı üst limit
    CMD_HZ           = 10.0   # Hz

    def __init__(self, node: Node, det_store: DetectionsStore):
        smach.State.__init__(self, outcomes=['completed', 'failed'])
        self.node      = node
        self.det_store = det_store

        self._cmd_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._lock   = threading.Lock()
        self._pos    = None   # (x, y) — son odom konumu

        node.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )

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
            f'[HIZLANMA] Başladı. Hedef: {self.TOPLAM_MESAFE:.0f}m @ {self.MAX_HIZ:.1f} m/s'
        )

        twist   = Twist()
        dt      = 1.0 / self.CMD_HZ
        deadline = time.time() + self.TIMEOUT_S
        sonuc   = 'completed'

        while rclpy.ok():
            if self.det_store.get_field('e_stop', False):
                self.node.get_logger().error('[HIZLANMA] E-STOP — durduruluyor.')
                sonuc = 'failed'
                break

            if self.det_store.get_field('manual_mod', False):
                self.node.get_logger().warn('[HIZLANMA] Manuel mod — RC devraldı, bekleniyor.')
                twist.linear.x = 0.0
                self._cmd_pub.publish(twist)
                time.sleep(0.1)
                continue

            if time.time() > deadline:
                self.node.get_logger().warn('[HIZLANMA] Timeout — durduruluyor.')
                break

            dist = self._mesafe(baslangic)

            if self.det_store.get_field('hizlanma_bitti', False):
                self.node.get_logger().info(
                    f'[HIZLANMA] Tabela_11_son tespit ({dist:.1f}m) — fren.'
                )
                break

            if dist >= self.TOPLAM_MESAFE:
                self.node.get_logger().info(
                    f'[HIZLANMA] {self.TOPLAM_MESAFE:.0f}m tamamlandı — fren.'
                )
                break

            # Hız profili — pure_logic.hizlanma_hiz_profili (test_birim.py
            # bu fonksiyonu doğrudan test eder).
            hiz = hizlanma_hiz_profili(
                dist, self.MAX_HIZ, self.TOPLAM_MESAFE, self.FREN_BASI_MESAFE,
            )

            twist.linear.x  = hiz
            twist.angular.z = 0.0
            self._cmd_pub.publish(twist)

            self.node.get_logger().info(
                f'[HIZLANMA] dist={dist:.1f}m  hız={hiz:.1f}m/s',
                throttle_duration_sec=0.5,
            )
            time.sleep(dt)

        # Dur
        twist.linear.x = 0.0
        for _ in range(5):
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self.node.get_logger().info(f'[HIZLANMA] Tamamlandı → {sonuc}.')
        return sonuc


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 6 — MISSION_COMPLETE
# ═══════════════════════════════════════════════════════════════════════════════

class MissionCompleteState(smach.State):
    """
    Görev tamamlandı. /mission_status → 'COMPLETE' yayınlanır.
    Geçişler: 'done' → (terminal)
    """

    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=['done'])
        self.node        = node
        self._status_pub = node.create_publisher(String, MISSION_STATUS_TOPIC, 10)
        self._misyon_pub = node.create_publisher(Bool, MISYON_AKTIF_TOPIC, 10)

    def execute(self, userdata):
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='MISSION_COMPLETE'))
        self.node.get_logger().info('═══ GÖREV TAMAMLANDI ═══')
        self._status_pub.publish(String(data='COMPLETE'))
        # Veri kaydını durdur
        self._misyon_pub.publish(Bool(data=False))
        return 'done'


# ═══════════════════════════════════════════════════════════════════════════════
# STATE 7 — ERROR_RECOVERY
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
        if hasattr(self.node, '_fsm_state_pub'):
            self.node._fsm_state_pub.publish(String(data='ERROR_RECOVERY'))
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
            else:
                tip = 'nav'
            waypoints.append({
                'x':           wp.get('x', 0.0),
                'y':           wp.get('y', 0.0),
                'yaw':         wp.get('yaw', 0.0),
                'type':        tip,
                'label':       isim,
                'pas_gecilir': a.get('pas_gecilir', False),
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
        # 1=SEMI_AUTO → FSM çalışmaya devam eder, mod_yoneticisi RC override yapar
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
    node.get_logger().info(f'Aşama timeout: {stage_timeout:.0f}s | Başlangıç bekleme: {baslangic_bekleme:.0f}s (waypoints.yaml)')

    # ── SMACH FSM Kurulumu ────────────────────────────────────────────
    sm = smach.StateMachine(outcomes=['GOREV_TAMAMLANDI', 'GOREV_IPTAL'])

    sm.userdata.waypoints  = waypoints
    sm.userdata.wp_index   = 0
    sm.userdata.current_wp = None

    with sm:
        smach.StateMachine.add(
            'IDLE',
            IdleState(node, baslangic_bekleme),
            transitions={'started': 'NAVIGATE'}
        )

        smach.StateMachine.add(
            'NAVIGATE',
            NavigateState(node, nav, det_store, stage_timeout),
            transitions={
                'next_waypoint':    'NAVIGATE',
                'shoot_waypoint':   'SHOOT_APPROACH',
                'hizlanma_waypoint':'HIZLANMA',
                'mission_complete': 'MISSION_COMPLETE',
                'failed':           'ERROR_RECOVERY',
            }
        )

        smach.StateMachine.add(
            'HIZLANMA',
            HizlanmaState(node, det_store),
            transitions={
                'completed': 'NAVIGATE',
                'failed':    'ERROR_RECOVERY',
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
            ShootState(node, det_store),
            transitions={
                'shot_fired': 'NAVIGATE',
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
