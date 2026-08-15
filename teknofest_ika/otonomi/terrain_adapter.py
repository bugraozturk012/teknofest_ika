#!/usr/bin/env python3
"""
terrain_adapter.py — LYDİA İKA Arazi Parametre Adaptörü
=========================================================
v3.0 — Integer tabanlı protokol (optimizasyon)

─────────────────────────────────────────────────────────────────────────────
NEDEN INTEGER?
─────────────────────────────────────────────────────────────────────────────
String tabanlı mesajlar her callback'te heap allocation + karakter karşılaştırması
gerektirir. UInt8 ise tek byte, sıfır parse maliyeti, doğrudan dict lookup.

YOLO çıkışı zaten integer class_id üretir (0-14). yolo_adapter_node bunu
/yolo/class_id topic'ine UInt8 olarak yayınlar.

─────────────────────────────────────────────────────────────────────────────
TOPIC ARAYÜZÜ
─────────────────────────────────────────────────────────────────────────────
  Giriş  : /yolo/class_id  (std_msgs/UInt8)

           Eşleme models/best.pt içindeki names sözlüğünden okunmuştur —
           model 15 sınıfla eğitildi ve Tabela_12 İÇERMİYOR. Sıra alfabetik
           olduğu için class_id ile tabela numarası örtüşmez; listeyi elle
           kaydırmak (ör. araya Tabela_12 eklemek) index 4'ten sonrasının
           tamamını bozar ve her tabela komşusunun profilini uygular.

             0 = SULU_YOL       (Tabela_1)        → wet
             1 = DIK_EGIM_CIKIS (Tabela_10)       → normal
             2 = HIZLANMA       (Tabela_11)       → fast   (HizlanmaState Nav2'yi bypass eder)
             3 = HIZLANMA_SON   (Tabela_11_son)   → normal
             4 = TASLI_YOL      (Tabela_2)        → gravel
             5 = YAN_EGIM       (Tabela_3)        → slope
             6 = DIK_ENGEL      (Tabela_4)        → obstacle
             7 = KONİLİ_YOL     (Tabela_5)        → normal (Nav2 halleder)
             8 = KAYAR_ENGEL    (Tabela_6)        → normal (Nav2 halleder)
             9 = ENGEBELİ_ARAZİ (Tabela_7)        → rough
            10 = DIK_EGIM       (Tabela_8)        → rough
            11 = ATIS_BOLGESI   (Tabela_9)        → slow
            12 = Tabela_stop                      → normal (FSM halleder)
            13 = hedef_tahtasi                    → slow
            14 = trafik_huni                      → normal (Nav2 costmap halleder)
           255 = tespit yok                       → normal (varsayılan profil)

  Çıkış  : Nav2 /controller_server/set_parameters RPC çağrısı
           (Node yeniden başlatılmaz — anlık etkili)

─────────────────────────────────────────────────────────────────────────────
ARDIŞIK FRAME FİLTRESİ
─────────────────────────────────────────────────────────────────────────────
Tek frame'deki yanlış tespit Nav2 parametrelerini bozmasın diye
aynı class_id art arda CONSECUTIVE_FRAMES kez gelmeden profil uygulanmaz.
Eşik: topics.py → YOLO_CONSECUTIVE_FRAMES = 3

─────────────────────────────────────────────────────────────────────────────
PARAMETRE DEĞİŞİMİ
─────────────────────────────────────────────────────────────────────────────
Nav2 RegulatedPurePursuitController ve local_costmap parametreleri
/controller_server/set_parameters servisi üzerinden çalışma anında güncellenir.
Node yeniden başlatılmaz.

  Lookahead temeli: L_d = k_ld × v  (RPP temel denklemi)
  Düşük μ yüzeyinde (wet, gravel): v ve max_accel küçültülür
  Yüksek engel riskinde: inflation_radius büyütülür

─────────────────────────────────────────────────────────────────────────────
TEST (görüntü ekibi olmadan):
  # Tabela 1 = su_gecisi
  ros2 topic pub /yolo/class_id std_msgs/msg/UInt8 "data: 1" --once

  # Tabela 255 = tespit yok → normal profil
  ros2 topic pub /yolo/class_id std_msgs/msg/UInt8 "data: 255" --once
─────────────────────────────────────────────────────────────────────────────
"""

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8, Float32
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory

from teknofest_ika.otonomi.topics import YOLO_CLASS_ID_TOPIC, SPEED_LIMIT_TOPIC

# Yok tespit sentinel değeri (255: UInt8 max → "boş")
NO_DETECTION = 255

# ─────────────────────────────────────────────────────────────────────────────
# INTEGER CLASS_ID → TERRAIN PROFİLİ
# ─────────────────────────────────────────────────────────────────────────────
# Model alfabetik sırayla eğitildi — class_id Tabela numarasıyla örtüşmüyor.
# Doğrulama: python3 -c "import torch; print(torch.load('models/best.pt',
#   map_location='cpu', weights_only=False)['model'].names)"
# Gerçek eşleme (best.pt names, 15 sınıf):
#   0=Tabela_1  1=Tabela_10  2=Tabela_11  3=Tabela_11_son
#   4=Tabela_2  5=Tabela_3   6=Tabela_4   7=Tabela_5       8=Tabela_6
#   9=Tabela_7  10=Tabela_8  11=Tabela_9  12=Tabela_stop   13=hedef_tahtasi
#   14=trafik_huni
# Sıra topics.YOLO_CLASSES ile birebir aynı olmalıdır.
CLASS_TO_TERRAIN = {
    0:  'wet',      # Tabela_1   SULU_YOL        — μ≈0.3, fren mesafesi artar
    1:  'normal',   # Tabela_10  DIK_EGIM_CIKIS  — rampa çıkış, düz zemin
    2:  'fast',     # Tabela_11  HIZLANMA        — HizlanmaState Nav2'yi bypass eder
    3:  'normal',   # Tabela_11_son HIZLANMA_SON — hızlanma bitiyor, normale dön
    4:  'gravel',   # Tabela_2   TASLI_YOL       — lateral stabilite azalır
    5:  'slope',    # Tabela_3   YAN_EGIM        — F_lat = m·g·sin(θ)
    6:  'obstacle', # Tabela_4   DIK_ENGEL       — lokal costmap kaçınır
    7:  'normal',   # Tabela_5   KONİLİ_YOL     — Nav2 local costmap yeterli
    8:  'normal',   # Tabela_6   KAYAR_ENGEL     — Nav2 local costmap yeterli
    9:  'rough',    # Tabela_7   ENGEBELİ_ARAZİ — titreşim + düzensiz zemin
    10: 'rough',    # Tabela_8   DIK_EGIM        — eğim + zemin: en kısıtlı profil
    11: 'slow',     # Tabela_9   ATIS_BOLGESI    — dur, nişan al
    12: 'normal',   # Tabela_stop STOP işareti   — terrain değişmez, FSM halleder
    13: 'slow',     # hedef_tahtasi              — atış bölgesine yakın, yavaş
    14: 'normal',   # trafik_huni                — Nav2 costmap halleder
    NO_DETECTION: 'normal',
}

# ─────────────────────────────────────────────────────────────────────────────
# YOL TİPİ → NAV2 PARAMETRE PROFİLLERİ
#
# desired_linear_vel   [m/s]    : Hedef ileri hız
# lookahead_dist       [m]      : RPP lookahead mesafesi L_d
# max_accel / max_decel[m/s²]   : İvme/yavaşlama limiti
# max_robot_pose_search_dist [m]: Rota üzerinde araç pozisyonu arama mesafesi
# inflation_radius     [m]      : Costmap engel tamponu
#
# Formüller:
#   Fren mesafesi  : d = v² / (2·μ·g)
#   Min lookahead  : L_d_min = v × t_lookahead (t≈0.5s)
#   Turning radius : R = v / ω (RPP regulated_linear_scaling_min_radius ile kısıtlı)
# ─────────────────────────────────────────────────────────────────────────────
TERRAIN_PROFILES = {

    'normal': {
        # Düz, kuru zemin. Hız, otonom sürüşte sahada doğrulanmış en yüksek
        # değere sabitlendi: 2026-07-28'de parkur turu 0.65 m/s tavanıyla
        # tamamlandı. Aracın tam gaz karşılığı 1.94 m/s ölçüldü, ama o hız
        # otonomda hiç denenmedi — tavanı ölçülmemiş bir değere açmak,
        # planlayıcının frenleyemeyeceği bir hızda engele girmesi demek.
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.65,
            'FollowPath.lookahead_dist':                1.5,
            'FollowPath.max_angular_accel':             3.2,
            'FollowPath.max_robot_pose_search_dist':    5.0,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.40,
        },
    },

    'wet': {
        # Su geçişi: μ≈0.3 → d_fren = 0.8²/(2·0.3·9.8) ≈ 0.11m (kabul edilebilir)
        # Düşük angular accel: ıslak zeminde ani yön değişimi kayma yaratır
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.50,
            'FollowPath.lookahead_dist':                1.0,
            'FollowPath.max_angular_accel':             1.0,
            'FollowPath.max_robot_pose_search_dist':    2.5,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.55,
        },
    },

    'gravel': {
        # Taşlı yol: lateral temas düzensiz, engebeli zemin kayma riski
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.50,
            'FollowPath.lookahead_dist':                0.9,
            'FollowPath.max_angular_accel':             1.5,
            'FollowPath.max_robot_pose_search_dist':    2.5,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.45,
        },
    },

    'slope': {
        # Yan eğim: F_lat = m·g·sin(θ) → devrilme momenti
        # Küçük lookahead → direksiyon açısı minimize edilir
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.45,
            'FollowPath.lookahead_dist':                0.7,
            'FollowPath.max_angular_accel':             1.0,
            'FollowPath.max_robot_pose_search_dist':    2.0,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.65,
        },
    },

    'obstacle': {
        # Dik engel: Nav2 lokal costmap'i kaçınır, araç yavaşlar
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.60,
            'FollowPath.lookahead_dist':                1.2,
            'FollowPath.max_angular_accel':             2.0,
            'FollowPath.max_robot_pose_search_dist':    4.0,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.50,
        },
    },

    'rough': {
        # Dik eğim: eğim + engebeli zemin kombine → en kısıtlı profil
        # IMU gürültüsü artar, EKF kovaryansı büyür
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.45,
            'FollowPath.lookahead_dist':                0.6,
            'FollowPath.max_angular_accel':             1.0,
            'FollowPath.max_robot_pose_search_dist':    1.5,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.60,
        },
    },

    'slow': {
        # Atış öncesi: araç durmaya hazırlanır, taret hizalanır
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.45,
            'FollowPath.lookahead_dist':                0.5,
            'FollowPath.max_angular_accel':             0.8,
            'FollowPath.max_robot_pose_search_dist':    1.5,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.40,
        },
    },

    'fast': {
        # Hızlanma bölgesi: düz zemin, maksimum performans
        'controller_server': {
            'FollowPath.desired_linear_vel':            0.90,
            'FollowPath.lookahead_dist':                2.5,
            'FollowPath.max_angular_accel':             3.2,
            'FollowPath.max_robot_pose_search_dist':    8.0,
        },
        'local_costmap/local_costmap': {
            'inflation_layer.inflation_radius': 0.35,
        },
    },
}


class TerrainAdapter(Node):

    def __init__(self):
        super().__init__('terrain_adapter')

        self._current_terrain    = 'normal'
        self._last_class_id     = NO_DETECTION
        self._consecutive       = 0
        self._speed_limit       = float('inf')   # imu_guvenlik'ten gelir
        self._last_applied_speed_limit = float('inf')
        self._last_speed_limit_apply_t = 0.0
        self._consecutive_frames = self._load_consecutive_frames()
        self._baslangic_yapildi = False

        # ── QoS: görüntü ekibi BEST_EFFORT yayınlar (kamera pipeline) ────────
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── /yolo/class_id — UInt8, integer class index ───────────────────────
        self.create_subscription(UInt8, YOLO_CLASS_ID_TOPIC,
                                 self._on_class_id, qos)

        # ── /speed_limit — imu_guvenlik'ten hız sınırı ───────────────────────
        self.create_subscription(Float32, SPEED_LIMIT_TOPIC,
                                 self._on_speed_limit, 10)

        # ── Nav2 set_parameters servis client'ları ────────────────────────────
        # Her profilde hangi Nav2 node'ları hedef alınıyorsa client aç
        target_nodes = {
            node_name
            for profile in TERRAIN_PROFILES.values()
            for node_name in profile
        }
        self._param_clients: dict = {}
        for node_name in target_nodes:
            self._param_clients[node_name] = self.create_client(
                SetParameters,
                f'/{node_name}/set_parameters',
            )

        self.get_logger().info(
            f'TerrainAdapter v3.0 hazır | '
            f'topic={YOLO_CLASS_ID_TOPIC} (UInt8) | '
            f'filtre={self._consecutive_frames} ardışık frame'
        )

        # Nav2 servisleri hazır olunca başlangıç profilini uygula.
        # terrain_adapter 11.5s'de başlar, Nav2 8s'de — servisler genellikle hazırdır.
        self.create_timer(2.0, self._baslangic_profil)

    def _baslangic_profil(self) -> None:
        """Node başladıktan 2s sonra 'normal' profili Nav2'ye uygular (bir kez)."""
        if self._baslangic_yapildi:
            return
        self._baslangic_yapildi = True
        self.get_logger().info('Başlangıç profili uygulanıyor: normal')
        self._apply_profile('normal')

    def _load_consecutive_frames(self) -> int:
        """waypoints.yaml'dan ardisik_frame_sayisi okur, bulunamazsa 3 döner."""
        try:
            pkg = get_package_share_directory('teknofest_ika')
            wp_path = os.path.join(pkg, 'config', 'waypoints.yaml')
            with open(wp_path, 'r') as f:
                data = yaml.safe_load(f)
            val = int(data.get('parametreler', {}).get('ardisik_frame_sayisi', 3))
            self.get_logger().info(f'ardisik_frame_sayisi={val} (waypoints.yaml)')
            return val
        except Exception as e:
            self.get_logger().warn(f'waypoints.yaml okunamadı, varsayılan 3: {e}')
            return 3

    def _on_speed_limit(self, msg: Float32) -> None:
        """
        imu_guvenlik'ten gelen hız sınırını saklar.

        Önceden bu değer yalnızca _apply_profile() çağrıldığında (yani
        terrain DEĞİŞTİĞİNDE) Nav2'ye iletiliyordu — terrain aynı kalıp
        roll açısı/batarya durumu kötüleşirse (örn. devrilmeye giderken)
        yeni, daha düşük hız sınırı Nav2'ye HİÇ ulaşmıyordu. Artık her
        kısıtlama sıkılaşmasında (hız düşüşünde) anında yeniden uygulanır;
        gevşemelerde (iyileşme) RPC spam'ini önlemek için 0.5s rate-limit
        uygulanır — imu_guvenlik 10Hz yayın yapar, her örnekte Nav2'ye
        servis çağrısı yapmak gereksizdir.
        """
        yeni_limit = float(msg.data)
        onceki_limit = self._speed_limit
        self._speed_limit = yeni_limit

        onemli_dusus = yeni_limit < onceki_limit - 0.05
        now = self.get_clock().now().nanoseconds / 1e9
        rate_limit_gecti = (now - self._last_speed_limit_apply_t) > 0.5

        if onemli_dusus or (
            yeni_limit != self._last_applied_speed_limit and rate_limit_gecti
        ):
            self._last_applied_speed_limit = yeni_limit
            self._last_speed_limit_apply_t = now
            self._apply_profile(self._current_terrain)

    # ── Callback: YOLO class_id geldi ────────────────────────────────────────
    def _on_class_id(self, msg: UInt8) -> None:
        """
        Integer class_id alır, ardışık frame filtresi uygular,
        profil değişmişse Nav2'ye RPC atar.

        Ardışık frame filtresi:
            Aynı class_id CONSECUTIVE_FRAMES kez art arda gelmeden
            profil değiştirilmez. Yanlış pozitif tespitlere karşı koruma.
        """
        cid: int = msg.data

        # Bilinmeyen class_id → yoksay (görüntü ekibi hatalı yayın)
        if cid not in CLASS_TO_TERRAIN:
            self.get_logger().warn(
                f'Bilinmeyen class_id: {cid} — yoksayıldı',
                throttle_duration_sec=5.0,
            )
            return

        # Ardışık sayaç
        if cid == self._last_class_id:
            self._consecutive += 1
        else:
            self._consecutive  = 1
            self._last_class_id = cid

        # Eşiğe ulaşılmadıysa bekle
        if self._consecutive < self._consecutive_frames:
            return

        # Eşiğe ulaşıldı — profili kontrol et
        terrain = CLASS_TO_TERRAIN[cid]

        if terrain == self._current_terrain:
            return  # Zaten bu profildeyiz

        self.get_logger().info(
            f'[TerrainAdapter] class_id={cid} → {terrain.upper()} '
            f'(önceki: {self._current_terrain})'
        )

        self._current_terrain = terrain
        self._apply_profile(terrain)

    # ── Profili Nav2'ye uygula ────────────────────────────────────────────────
    def _apply_profile(self, terrain: str) -> None:
        profile = TERRAIN_PROFILES[terrain]

        for node_name, params in profile.items():
            client = self._param_clients.get(node_name)
            if client is None:
                continue

            if not client.wait_for_service(timeout_sec=0.05):
                self.get_logger().warn(
                    f'/{node_name}/set_parameters yok — Nav2 çalışıyor mu?',
                    throttle_duration_sec=5.0,
                )
                continue

            req = SetParameters.Request()
            for param_name, value in params.items():
                # desired_linear_vel imu_guvenlik hız sınırıyla kırpılır
                if param_name.endswith('desired_linear_vel'):
                    value = min(float(value), self._speed_limit)
                pv = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(value),
                )
                req.parameters.append(Parameter(name=param_name, value=pv))

            future = client.call_async(req)
            future.add_done_callback(
                lambda f, n=node_name, t=terrain: self._on_result(f, n, t)
            )

    def _on_result(self, future, node_name: str, terrain: str) -> None:
        try:
            result = future.result()
            failed = [r.reason for r in result.results if not r.successful]
            if failed:
                self.get_logger().warn(
                    f'{node_name} → bazı parametreler uygulanamadı: {failed}'
                )
            else:
                self.get_logger().info(f'{node_name} → {terrain} ✓')
        except Exception as e:
            self.get_logger().error(f'set_parameters hatası ({node_name}): {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
