#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
ackermann_converter.py — cmd_vel → AckermannDriveStamped Dönüştürücü
LYDİA İKA Projesi | ROS2 Humble

─────────────────────────────────────────────────────────────────────────────
NEDEN GEREKLİ?
─────────────────────────────────────────────────────────────────────────────
Nav2, yerel kontrolcüsünün (RPP) ürettiği komutları geometry_msgs/Twist
formatında /cmd_vel topic'ine yayınlar. Bu format diferansiyel sürüş
modelini temel alır:

    Twist.linear.x  → ileri hız (m/s)
    Twist.angular.z → açısal hız (rad/s)

Ackermann kinematikli araçta ise seri köprü (seri_kopru.py) komutları
ackermann_msgs/AckermannDriveStamped formatında bekler:

    AckermannDrive.speed          → ileri hız (m/s)
    AckermannDrive.steering_angle → direksiyon açısı (rad)

Bu node ikisi arasındaki dönüşümü yapan köprüdür.

─────────────────────────────────────────────────────────────────────────────
MATEMATİKSEL TEMEL — BİSİKLET MODELİ KİNEMATİĞİ
─────────────────────────────────────────────────────────────────────────────
Ackermann aracı için basitleştirilmiş bisiklet modeli (single-track model)
şu kinematik ilişkiyi verir:

    v   = linear.x          [m/s]   — ileriye doğru hız
    ω   = angular.z         [rad/s] — dönüş açısal hızı

Dönüş yarıçapı:
    R = v / ω               [m]     (ω ≠ 0 koşuluyla)

Direksiyon açısı (ön tekerleğin orta noktası referansıyla):
    tan(δ) = L / R
    δ = arctan(L × ω / v)  [rad]

    Burada L = dingil arası (wheelbase) [m]

Fiziksel sınırlama:
    |δ| ≤ δ_max             — servonun mekanik limiti
    |v| ≤ v_max             — sürüş kartının hız tavanı

Özel durum (v ≈ 0, ω ≠ 0):
    Ackermann aracı yerinde dönemez. Nav2 bu komutu recovery
    davranışında üretebilir (spin behavior). Araç durur, direksiyon
    maksimuma alınır — Nav2 yeniden plan üretir.

─────────────────────────────────────────────────────────────────────────────
TF / TOPIC MİMARİSİ
─────────────────────────────────────────────────────────────────────────────
    Nav2 (RPP)
        │
        │ /cmd_vel (Twist)
        ▼
    [ackermann_converter]              ← bu node
        │
        │ /ackermann_cmd (AckermannDriveStamped)
        ▼
    [seri_kopru]
        │
        │ USB Seri (binary 8-byte)
        ▼
    Nucleo-F767ZI → BLDC sürücü + direksiyon step motoru

─────────────────────────────────────────────────────────────────────────────
PARAMETRELER (ros2 param set ile çalışma zamanında değiştirilebilir)
─────────────────────────────────────────────────────────────────────────────
    wheelbase          : Dingil arası [m]     — 1.44, mezürle ölçüldü
    max_steering_angle : Max direksiyon açısı [rad] — 30° = 0.5236 rad
                         🔴 ÖLÇÜLMEDİ, yer tutucu
    max_speed          : sürüş kartının otonom hız tavanı [m/s]
    cmd_vel_timeout    : Bu süre içinde /cmd_vel gelmezse araç durdurulur [s]

─────────────────────────────────────────────────────────────────────────────
KURULUM:
    setup.py entry_points'e ekle:
        'ackermann_converter = teknofest_ika.ackermann_converter:main'

    colcon build --packages-select teknofest_ika --symlink-install
─────────────────────────────────────────────────────────────────────────────
"""

import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import UInt8, UInt16

from teknofest_ika.otonomi.topics import (
    MUX_CMD_VEL_TOPIC, ACKERMANN_CMD_TOPIC, E_STOP_TOPIC,
    ANTI_ROLLBACK_AKTIF_TOPIC, ANTI_ROLLBACK_CMD_TOPIC,
    YOKUS_KALKIS_AKTIF_TOPIC, YOKUS_KALKIS_FREN_TOPIC,
    YOKUS_BAYATLAMA_S, ROLLBACK_BAYATLAMA_S, ATIS_AZAMI_S,
    FREN_KOMUT_TOPIC, FREN_IVME_ESIK_MIN, FREN_IVME_ESIK_MAX, SHOOT_CMD_TOPIC,
    FREN_TAM_DUR_ORAN, FREN_RAMP_PER_S, FREN_GUVENLI_DUR_BINDE,
    KART_HIZ_TAVAN, MOD_AKTIF_TOPIC, MOD_MANUAL, MOD_BAYATLAMA_S,
)
from teknofest_ika.otonomi.pure_logic import (
    ackermann_komut, fren_hedef_hesapla, fren_yumusat,
)


class AckermannConverter(Node):
    """
    /cmd_vel (Twist) → /ackermann_cmd (AckermannDriveStamped) dönüştürücü.

    Bisiklet modeli kinematik dönüşümü kullanır:
        steering_angle = arctan(wheelbase × angular_z / linear_x)
    """

    def __init__(self):
        super().__init__('ackermann_converter')

        # ── Parametreler ──────────────────────────────────────────────────────
        # wheelbase: ön aks ↔ arka aks, mezürle ölçüldü (kart ekibiyle teyitli).
        # Ackermann kinematiğinin tek girdisi bu: yanlışsa üretilen HER
        # direksiyon açısı yanlış olur, ve hata sahada ancak aracın virajı
        # geniş ya da dar almasıyla görülür.
        # urdf/arac.urdf teker joint'leri (x=±0.72) ve nav2_params.yaml
        # minimum_turning_radius (L/tan δ_max) ile TUTARLI tutulur; biri
        # değişirse diğerleri de değişmeli.
        # Çalışma zamanında değiştirmek için:
        #   ros2 param set /ackermann_converter wheelbase 1.44
        self.declare_parameter('wheelbase', 1.44)
        # 🔴 max_steering_angle ÖLÇÜLMEDİ. 30°, mekanik ucun altında kaldığı
        # varsayılan bir yer tutucu; gerçek uç 5°'lik adımlarla bulunup
        # birkaç derece altı yazılmalı (step motor uca dayanınca adım kaçırır
        # ve kartın konum sayacı gerçeği kaybeder).
        self.declare_parameter('max_steering_angle', 0.5236)   # 30° = π/6
        self.declare_parameter('max_speed', KART_HIZ_TAVAN)
        self.declare_parameter('cmd_vel_timeout', 0.5)         # [s]

        # R_min'den dar bir yay istendiğinde hız eğriliğin taştığı oranda
        # düşürülür; bu taban altına inilmez. Sahada ölçülen kalkış
        # sürtünmesi eşiği (nav2_params.yaml min_approach_linear_velocity ve
        # regulated_linear_scaling_min_speed ile aynı değer) — altında araç
        # viraj ortasında hareket edemez hale gelir.
        self.declare_parameter('viraj_taban_hizi', 0.45)       # [m/s]

        # Otomatik fren, hedef hızdaki düşüşten fren oranı üretir. Bu araçta
        # fren hattı kontrolcünün gaz kesme girişini de çektiği için, hız her
        # düştüğünde (örn. dar geçişte yavaşlama) gaz kesiliyor ve araç
        # ilerleyemiyor. Kapatıldığında /fren_komut sabit 0 yayınlanır.
        self.declare_parameter('otomatik_fren', True)

        self._L        = self.get_parameter('wheelbase').value
        self._delta_max = self.get_parameter('max_steering_angle').value
        self._otomatik_fren = bool(self.get_parameter('otomatik_fren').value)
        self._v_max    = self.get_parameter('max_speed').value
        self._timeout  = self.get_parameter('cmd_vel_timeout').value
        self._viraj_taban = self.get_parameter('viraj_taban_hizi').value

        # nav2_params.yaml minimum_turning_radius ile aynı büyüklük; orada
        # elle yazıldığı için burada δ_max'ten türetilir, ikisi ayrışamaz.
        self._r_min = self._L / math.tan(self._delta_max)

        self.get_logger().info(
            f'[AckermannConverter] Başlatıldı | '
            f'wheelbase={self._L:.3f}m | '
            f'δ_max={math.degrees(self._delta_max):.1f}° | '
            f'R_min={self._r_min:.2f}m | '
            f'v_max={self._v_max:.1f}m/s'
        )

        # ── QoS ───────────────────────────────────────────────────────────────
        # /cmd_vel: Nav2 RELIABLE yayınlar → RELIABLE abone
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── E-STOP durumu ────────────────────────────────────────────────────
        # E-stop aktifken anti_rollback dahil TÜM komutlar yoksayılır.
        self._e_stop_aktif = False

        # ── Override state (anti_rollback) ───────────────────────────────────
        self._override_lock   = threading.Lock()
        self._override_active = False
        self._override_twist  = Twist()
        self._override_zaman  = None   # son bayrak mesajının geliş anı

        # ── §6.10 yokuş kalkışı — fren SIKILIYKEN gaz ────────────────────────
        # Eğimdeki zorunlu duruştan kalkarken RampaState bu bayrağı kaldırır.
        # Bayrak açıkken fren, komut edilen hızın türevinden HESAPLANMAZ;
        # RampaState'in verdiği ‰ değerinde tutulur ve gaz normal geçer.
        # İkisinin normalde birbirini dışlaması fren_hedef_hesapla'nın
        # "hedef hız 0 ise en az tam_dur_oran" kuralından geliyor.
        self._yokus_aktif = False
        self._yokus_fren  = 0
        self._atis_aktif  = False   # /shoot_command — atış boyunca tam fren
        self._atis_zaman  = None    # isteğin BAŞLADIĞI an (kenar, nabız değil)
        self._mod         = None    # /mod/aktif — kip bilinmeden fren kısılmaz
        self._mod_zaman   = None
        self._yokus_zaman = None   # son bayrak mesajının geliş anı

        # ── Otomatik fren state'i ─────────────────────────────────────────────
        # Hedef hızdaki ani düşüşten fren oranı hesaplanır (bkz. pure_logic.
        # fren_hedef_hesapla) — ölçekler PLACEHOLDER, fiziksel testte kalibre
        # edilecek (topics.py: FREN_IVME_ESIK_MIN/MAX, FREN_TAM_DUR_ORAN).
        self._onceki_hiz   = 0.0
        self._onceki_zaman = None
        self._fren_orani   = 0.0   # [0-1], yumuşatılmış (rate-limited) çıkış

        # ── Subscriber: /mux/cmd_vel — mod_yoneticisi çıkışı ─────────────────
        self._sub = self.create_subscription(
            Twist,
            MUX_CMD_VEL_TOPIC,
            self._cmd_vel_callback,
            qos_reliable,
        )

        # ── Subscriber: anti_rollback override ───────────────────────────────
        from std_msgs.msg import Bool as BoolMsg
        self._override_sub = self.create_subscription(
            BoolMsg, ANTI_ROLLBACK_AKTIF_TOPIC,
            self._override_aktif_cb, 10
        )
        self._override_cmd_sub = self.create_subscription(
            Twist, ANTI_ROLLBACK_CMD_TOPIC,
            self._override_cmd_cb, qos_reliable
        )

        # ── Subscriber: yokuş kalkışı override ───────────────────────────────
        self._yokus_aktif_sub = self.create_subscription(
            BoolMsg, YOKUS_KALKIS_AKTIF_TOPIC, self._yokus_aktif_cb, 10)
        self._yokus_fren_sub = self.create_subscription(
            UInt16, YOKUS_KALKIS_FREN_TOPIC, self._yokus_fren_cb, qos_reliable)
        self.create_subscription(
            BoolMsg, E_STOP_TOPIC, self._e_stop_cb, 10
        )
        # Atış boyunca fren tam basılı tutulur (Şartname: atış sırasında
        # hareket cezalı). Kilit köprüde değil burada: /fren_komut'un sahibi
        # bu düğüm ve 20 Hz yayın yapıyor, başka bir yerden basılan fren
        # bir sonraki döngüde üzerine yazılırdı.
        self.create_subscription(
            BoolMsg, SHOOT_CMD_TOPIC, self._atis_cb, 10
        )
        # Tip UInt8 — yayıncı (mod_yoneticisi) ve diğer beş abone öyle
        # kullanıyor, topics.py de öyle diyor. DDS farklı tipleri
        # EŞLEŞTİRMEZ: yanlış tiple abone olmak hata vermez, sadece hiç
        # mesaj gelmez ve kip sonsuza kadar bilinmez kalır.
        self.create_subscription(UInt8, MOD_AKTIF_TOPIC, self._mod_cb, 10)

        # ── Publisher: /ackermann_cmd ─────────────────────────────────────────
        self._pub = self.create_publisher(
            AckermannDriveStamped,
            ACKERMANN_CMD_TOPIC,
            qos_reliable,
        )

        # ── Publisher: /fren_komut ─────────────────────────────────────────────
        self._fren_pub = self.create_publisher(UInt16, FREN_KOMUT_TOPIC, 10)

        # Önceden tahsis edilmiş mesajlar — hot path'de GC baskısını azaltır
        self._ackermann_msg = AckermannDriveStamped()
        self._ackermann_msg.header.frame_id = 'base_footprint'
        self._stop_msg = AckermannDriveStamped()
        self._stop_msg.header.frame_id = 'base_footprint'

        # ── Güvenlik: timeout watchdog ────────────────────────────────────────
        self._last_cmd_time = self.get_clock().now()
        self._watchdog_timer = self.create_timer(
            self._timeout / 2.0,
            self._watchdog_callback,
        )

    def _override_aktif_cb(self, msg) -> None:
        with self._override_lock:
            self._override_active = msg.data
            self._override_zaman  = self.get_clock().now()

    def _override_cmd_cb(self, twist: Twist) -> None:
        with self._override_lock:
            self._override_twist = twist

    def _yokus_aktif_cb(self, msg) -> None:
        with self._override_lock:
            self._yokus_aktif = msg.data
            self._yokus_zaman = self.get_clock().now()

    def _atis_cb(self, msg) -> None:
        with self._override_lock:
            istek = bool(msg.data)
            if istek and not self._atis_aktif:
                self._atis_zaman = self.get_clock().now()
            self._atis_aktif = istek

    def _mod_cb(self, msg) -> None:
        """
        Kip değişimini yakalar. Manuele GEÇİLDİĞİ an frenimiz bir kez sıfıra
        çekilir: susmak yetmez, çünkü kart bizim son gönderdiğimiz değeri
        tutuyor ve kaynakların büyüğünü alıyor — 1000 basılıyken susarsak o
        1000 orada kalır ve operatör onu çözemez.
        """
        yeni = int(msg.data)
        onceki = self._mod
        self._mod       = yeni
        self._mod_zaman = self.get_clock().now()
        if yeni == MOD_MANUAL and onceki != MOD_MANUAL:
            self._fren_orani = 0.0
            self._fren_pub.publish(UInt16(data=0))
            self.get_logger().info(
                'MANUEL kipe geçildi — fren isteğimiz sıfırlandı, '
                'araç kumandadan sürülüyor.')

    def _manuel_mi(self) -> bool:
        """
        Kesin ve TAZE olarak manuel kipte miyiz.

        Bilinmiyorsa False döner, yani fren serbest kalır. Ters kurmak cazip
        ama tehlikeli: mod_yoneticisi otonom koşunun ortasında ölürse ve biz
        "bilmiyorsam basmayayım" dersek araç %45 eğimde frensiz kalır. Yanlış
        bastırmanın bedeli operatörün anında gördüğü bir fren; yanlış
        bastırmamanın bedeli kontrolsüz araç.
        """
        if self._mod is None or self._mod_zaman is None:
            return False
        yas = (self.get_clock().now() - self._mod_zaman).nanoseconds / 1e9
        if yas > MOD_BAYATLAMA_S:
            return False
        return self._mod == MOD_MANUAL

    def _fren_yayinla(self, binde: int, estop: bool = False) -> None:
        """
        Frenin TEK çıkış kapısı.

        Manuelde aracı sürüş kartı doğrudan kumandadan sürüyor ve bizim sürüş
        komutumuzu yok sayıyor — ama freni yok SAYMIYOR: kaynakların büyüğünü
        alıyor ve bizimkini operatör çözemiyor. Yani manuelde hesapladığımız
        her fren, sürücünün gazı bıraktığı anda üstüne binen ve açamadığı bir
        frene dönüşür. O yüzden manuelde 0 basılır.

        E-STOP bunun dışında: acil durdurma kipten bağımsız olmalı. Mantar
        basılıyken 48 V zaten kesik, araç hareket etmiyor; buton çevrildiği an
        kaynak düşer ve fren serbest kalır.
        """
        if estop or not self._manuel_mi():
            self._fren_pub.publish(UInt16(data=binde))
            return
        self._fren_orani = 0.0
        self._fren_pub.publish(UInt16(data=0))

    def _yokus_fren_cb(self, msg) -> None:
        with self._override_lock:
            self._yokus_fren = int(msg.data)

    def _e_stop_cb(self, msg) -> None:
        self._e_stop_aktif = msg.data

    # ── Ana dönüşüm callback'i ────────────────────────────────────────────────
    def _cmd_vel_callback(self, twist: Twist) -> None:
        # E-STOP: anti_rollback dahil TÜM komutları yoksay
        if self._e_stop_aktif:
            self._stop_msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._stop_msg)
            self._onceki_hiz   = 0.0
            self._onceki_zaman = None
            self._fren_orani   = 0.0
            self._fren_yayinla(FREN_GUVENLI_DUR_BINDE, estop=True)
            return

        # anti_rollback aktifse Nav2 komutunu yoksay. Bayat bayrak override'ı
        # BIRAKIR: anti_rollback bayrağı 20 Hz nabız basıyor, susmuşsa ya
        # kayma bitmiştir ya düğüm ölmüştür. İkinci durumda bayrağa güvenmek
        # Nav2 komutunu sonsuza kadar kurtarma komutuyla değiştirir ve araç
        # sabit bir hızda sürülmeye devam eder. Bırakınca normal yola dönülür.
        with self._override_lock:
            override = self._override_active
            override_zaman = self._override_zaman
            override_twist = self._override_twist

        if override:
            if override_zaman is None:
                override = False
            else:
                yas = (self.get_clock().now() - override_zaman).nanoseconds / 1e9
                if yas > ROLLBACK_BAYATLAMA_S:
                    override = False
                    self.get_logger().warn(
                        f'Geri kayma bayrağı {yas:.1f}s bayat — override '
                        'bırakıldı, Nav2 komutuna dönüldü.',
                        throttle_duration_sec=2.0,
                    )
        if override:
            twist = override_twist

        self._last_cmd_time = self.get_clock().now()

        v = twist.linear.x      # İleri hız [m/s]
        ω = twist.angular.z     # Açısal hız [rad/s]

        # ── Direksiyon açısı ve eğrilik kırpması ──────────────────────────────
        # v ≈ 0 durumunda (Ackermann yerinde dönemez) direksiyon ω işaretine
        # göre maksimuma alınır (Nav2 recovery/spin davranışı için bilinçli
        # bir karar — "son değer korunur" DEĞİLDİR). İstenen yay R_min'den
        # darsa direksiyon doyar ve hız taşma oranında düşürülür, yoksa araç
        # çizemeyeceği virajı tam hızda dener. Klamplama dahil tüm mantık
        # pure_logic.ackermann_komut()'tadır; test_birim.py doğrudan test eder.
        hiz, steering, doydu = ackermann_komut(
            v, ω, self._L, self._delta_max, self._viraj_taban)

        if doydu:
            self.get_logger().warn(
                f'Eğrilik doydu: istenen R={abs(v / ω):.2f}m < '
                f'R_min={self._r_min:.2f}m → hız {v:.2f}→{hiz:.2f} m/s, '
                f'δ={math.degrees(steering):.1f}°',
                throttle_duration_sec=2.0,
            )

        # ── Hız sınırlaması (Karaşimşek/buja kontrolcü limiti) ────────────────
        speed = max(-self._v_max, min(self._v_max, hiz))

        # ── Otomatik fren — hedef hızdaki ani düşüşten oranı hesapla ─────────
        simdi = self.get_clock().now()
        with self._override_lock:
            yokus_aktif = self._yokus_aktif
            yokus_fren  = self._yokus_fren
            yokus_zaman = self._yokus_zaman
            atis_aktif  = self._atis_aktif
            atis_zaman  = self._atis_zaman

        # Bayat bayrak override'ı BIRAKIR. RampaState bayrağı nabız gibi
        # basıyor; susmuşsa ya aşama bitmiştir ya düğüm ölmüştür. İkinci
        # durumda bayrağa güvenmek freni sonsuza kadar override'da bırakır ve
        # otomatik fren tamamen ölür. Bırakınca normal hesaba dönülüyor:
        # /mux/cmd_vel de bayatlayıp sıfır hız bastığı için fren yine binerek
        # geliyor, yani güvenli tarafa düşülüyor.
        if yokus_aktif:
            if yokus_zaman is None:
                yokus_aktif = False
            else:
                yas = (simdi - yokus_zaman).nanoseconds / 1e9
                if yas > YOKUS_BAYATLAMA_S:
                    yokus_aktif = False
                    self.get_logger().warn(
                        f'Yokuş kalkışı bayrağı {yas:.1f}s bayat — override '
                        'bırakıldı, otomatik frene dönüldü.',
                        throttle_duration_sec=2.0,
                    )

        # Atış isteğinin süresi sınırlı. Yokuş bayrağından farklı olarak
        # ölçüt bayatlık DEĞİL toplam süre: /shoot_command nabız gibi
        # tekrarlanmıyor, True ile False ayrı birer kenar. Yayınlayan
        # düğüm ikisinin arasında ölürse fren kalıcı olarak basılı kalır
        # ve araç bir daha hiç hareket edemez — koşuyu bitiren, sebebi
        # hiçbir log satırında görünmeyen bir arıza. Sınır aşılınca
        # otomatik frene dönülüyor; gerçekten atış sürüyorsa /mux/cmd_vel
        # zaten sıfır hız bastığı için fren yine biniyor.
        if atis_aktif and atis_zaman is not None:
            yas = (simdi - atis_zaman).nanoseconds / 1e9
            if yas > ATIS_AZAMI_S:
                atis_aktif = False
                # Mandal da düşürülüyor: yalnız yerel değişkeni
                # bırakmak, düğüm geri gelip yeni bir atış isteği
                # yayınladığında yükselen kenarı yutardı ve o atışta
                # fren hiç basılmazdı.
                with self._override_lock:
                    self._atis_aktif = False
                    self._atis_zaman = None
                self.get_logger().warn(
                    f'Atış freni {yas:.1f}s sürdü (sınır '
                    f'{ATIS_AZAMI_S:.0f}s) — bırakıldı, otomatik frene '
                    'dönüldü. /shoot_command kapatma komutu gelmemiş.',
                    throttle_duration_sec=2.0,
                )

        if atis_aktif:
            # Atış sürerken fren hızdan türetilmez: araç aşamaya girerken hâlâ
            # yuvarlanıyor olabilir ve otomatik fren yavaşlama bittiğinde
            # sıfıra döner. Şartnamenin istediği şey aracın DURMASI değil,
            # atış boyunca HAREKET ETMEMESİ.
            self._fren_orani = FREN_GUVENLI_DUR_BINDE / 1000.0
            self._fren_yayinla(FREN_GUVENLI_DUR_BINDE)
        elif yokus_aktif:
            # Fren hızdan türetilmiyor: RampaState ne derse o basılıyor ve gaz
            # yukarıdan aynen geçiyor. `_fren_orani` senkron tutuluyor, yoksa
            # override bittiğinde fren_yumusat bayat bir değerden rampalar.
            self._fren_orani = max(0.0, min(1.0, yokus_fren / 1000.0))
            self._fren_yayinla(yokus_fren)
        else:
            if self._onceki_zaman is not None:
                dt = (simdi - self._onceki_zaman).nanoseconds / 1e9
                hedef_oran = fren_hedef_hesapla(
                    self._onceki_hiz, speed, dt,
                    FREN_IVME_ESIK_MIN, FREN_IVME_ESIK_MAX, FREN_TAM_DUR_ORAN,
                )
                self._fren_orani = fren_yumusat(
                    self._fren_orani, hedef_oran, FREN_RAMP_PER_S / 1000.0, dt,
                )
            self._fren_yayinla(
                int(self._fren_orani * 1000) if self._otomatik_fren else 0)

        self._onceki_hiz   = speed
        self._onceki_zaman = simdi

        # ── Mesaj güncelle ve yayınla ─────────────────────────────────────────
        self._ackermann_msg.header.stamp     = self.get_clock().now().to_msg()
        self._ackermann_msg.drive.speed          = float(speed)
        self._ackermann_msg.drive.steering_angle = float(steering)

        self._pub.publish(self._ackermann_msg)

        self.get_logger().debug(
            f'v={v:.3f} m/s | ω={ω:.3f} rad/s → '
            f'speed={speed:.3f} m/s | δ={math.degrees(steering):.2f}°'
        )

    # ── Güvenlik watchdog ─────────────────────────────────────────────────────
    def _watchdog_callback(self) -> None:
        """
        Belirli süre /cmd_vel gelmezse (Nav2 durdu, bağlantı kesildi vb.)
        araç durdurma komutu yayınlanır.

        Bu Nav2'nin built-in timeout mekanizmasına ek bir güvenlik katmanıdır.
        Şartname gereği araç komut kesilince durmalıdır.
        """
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9

        if elapsed > self._timeout:
            self._stop_msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(self._stop_msg)
            self._onceki_hiz   = 0.0
            self._onceki_zaman = None
            self._fren_orani   = 0.0
            # Tam fren — sıfır burada freni BIRAKIYORDU. /cmd_vel kesildiğinde
            # Jetson paket göndermeye devam ettiği için Mega'nın 700 ms heartbeat
            # failsafe'i de devreye girmiyor; §6.10'un zorunlu duruşunda araç
            # eğimde frensiz kalıyordu.
            self._fren_yayinla(FREN_GUVENLI_DUR_BINDE)

            self.get_logger().warn(
                f'[WATCHDOG] /cmd_vel {elapsed:.2f}s süredir gelmiyor → araç durduruldu.',
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
