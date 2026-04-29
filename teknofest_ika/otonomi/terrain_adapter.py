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

YOLO çıkışı zaten integer class_id üretir (0-8). Görüntü ekibi bunu
doğrudan /yolo/class_id topic'ine UInt8 olarak yayınlar.

─────────────────────────────────────────────────────────────────────────────
TOPIC ARAYÜZÜ
─────────────────────────────────────────────────────────────────────────────
  Giriş  : /yolo/class_id  (std_msgs/UInt8)
             0 = su_gecisi    → wet
             1 = tasli_yol    → gravel
             2 = yan_egim     → slope
             3 = dik_engel    → obstacle
             4 = trafik_koni  → normal (Nav2 halleder)
             5 = kayar_engel  → normal (Nav2 halleder)
             6 = dik_egim     → rough
             7 = atis         → slow
             8 = hizlanma     → fast
           255 = tespit yok   → normal (varsayılan profil)

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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt8
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

# ─────────────────────────────────────────────────────────────────────────────
# TOPIC SABİTİ
# topics.py import edilmez — terrain_adapter tek başına çalışabilsin diye
# Değiştirirsen topics.py'deki YOLO_CLASS_ID_TOPIC ile eşleştir.
# ─────────────────────────────────────────────────────────────────────────────
YOLO_CLASS_ID_TOPIC = '/yolo/class_id'   # std_msgs/UInt8

# Yok tespit sentinel değeri (255: UInt8 max → "boş")
NO_DETECTION = 255

# Ardışık frame eşiği — topics.py YOLO_CONSECUTIVE_FRAMES ile eşleşmeli
CONSECUTIVE_FRAMES = 3

# ─────────────────────────────────────────────────────────────────────────────
# INTEGER CLASS_ID → TERRAIN PROFİLİ
# Sıra topics.py YOLO_CLASSES listesiyle birebir eşleşmeli:
#   [su_gecisi, tasli_yol, yan_egim, dik_engel, trafik_koni,
#    kayar_engel, dik_egim, atis, hizlanma]
# ─────────────────────────────────────────────────────────────────────────────
CLASS_TO_TERRAIN = {
    0:   'wet',      # su_gecisi   — μ≈0.3, fren mesafesi artar
    1:   'gravel',   # tasli_yol   — lateral stabilite azalır
    2:   'slope',    # yan_egim    — F_lat = m·g·sin(θ)
    3:   'obstacle', # dik_engel   — lokal costmap kaçınır
    4:   'normal',   # trafik_koni — Nav2 local costmap yeterli
    5:   'normal',   # kayar_engel — Nav2 local costmap yeterli
    6:   'rough',    # dik_egim    — eğim + zemin: en kısıtlı profil
    7:   'slow',     # atis        — dur, nişan al
    8:   'fast',     # hizlanma    — düz zemin, tam gaz
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
        # Düz, kuru zemin — tam performans
        'controller_server': {
            'FollowPath.desired_linear_vel':            2.0,
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
            'FollowPath.desired_linear_vel':            0.8,
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
            'FollowPath.desired_linear_vel':            0.7,
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
            'FollowPath.desired_linear_vel':            0.5,
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
            'FollowPath.desired_linear_vel':            1.2,
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
            'FollowPath.desired_linear_vel':            0.4,
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
            'FollowPath.desired_linear_vel':            0.3,
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
            'FollowPath.desired_linear_vel':            3.0,
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

        self._current_terrain  = 'normal'
        self._last_class_id    = NO_DETECTION
        self._consecutive      = 0          # ardışık aynı class sayacı

        # ── QoS: görüntü ekibi BEST_EFFORT yayınlar (kamera pipeline) ────────
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── /yolo/class_id — UInt8, integer class index ───────────────────────
        self.create_subscription(UInt8, YOLO_CLASS_ID_TOPIC,
                                 self._on_class_id, qos)

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
            f'filtre={CONSECUTIVE_FRAMES} ardışık frame'
        )

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
        if self._consecutive < CONSECUTIVE_FRAMES:
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

            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(
                    f'/{node_name}/set_parameters yok — Nav2 çalışıyor mu?'
                )
                continue

            req = SetParameters.Request()
            for param_name, value in params.items():
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
