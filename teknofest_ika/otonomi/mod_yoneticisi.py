#!/usr/bin/env python3
"""
mod_yoneticisi.py — Manuel / Tam Otonom Mod Yöneticisi
======================================================================
MOD TANIMI:
  MANUAL    (0): Sürüş kartı aracı doğrudan kumandadan sürer. Nav2 pasif.
  FULL_AUTO (2): Tam otonom — misyon_fsm + Nav2 kontrolü.

KİP KAYNAĞI:
  Kip anahtarını (SwC/CH9) sürüş kartı okuyor ve kararı kart veriyor; Jetson'ın
  oyu yok. Karar /kart/kip'ten geliyor, ham kanal eşiklenmiyor — anahtar üç
  konumlu ve kullanılmayan orta konum eski eşiğin üstüne düşüyordu, yani ham
  değeri bölen taraf o konumu otonom okurdu.

  Kart susarsa mod manuele döner: son bilinen kipte kalmak, gerçekte manuel
  sürülen bir araca otonom komut basmaya dönüşebilir.

YAZILIMSAL GEÇİŞ:
  ros2 topic pub /mod/komut std_msgs/msg/UInt8 "data: 2" --once

TOPIC MUX MİMARİSİ:
  Nav2 → /cmd_vel
                  ┐
  RC → /rc_input  ├→ [mod_yoneticisi] → /mux/cmd_vel → ackermann_converter
                  ┘

gercek_arac.launch.py'de ackermann_converter şu remapping ile başlatılır:
  remappings=[('/cmd_vel', '/mux/cmd_vel')]

GİRİŞLER:
  /kart/kip    (UInt8)  — kartın çözdüğü sürüş kipi, mod otoritesi
  /kart/durum  (UInt16) — DRM_* bayrakları; kesme anahtarı E-STOP kaynağı
  /rc_input    (Float32MultiArray) — MANUAL twist
  /mod/komut   (UInt8)
  /cmd_vel     (Twist) — Nav2 çıkışı

ÇIKIŞLAR:
  /mod/aktif      (UInt8)  — 0/1/2
  /mux/cmd_vel    (Twist)  — muxlanmış komut
  /mission_start  (Bool)   — FULL_AUTO'ya ilk geçişte FSM'i tetikler
"""
import threading
import time   # süre ölçümleri time.monotonic() ile: Jetson'ın RTC'si ölü ve
             # saat düzeltmesi sıçradığında time.time() aralıkları
             # milyonlarca saniye okunur (ayrıntı: misyon_fsm.py)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import UInt8, UInt16, Bool, Float32MultiArray, Float32
from geometry_msgs.msg import Twist

from teknofest_ika.otonomi.pure_logic import kip_modu, kesme_estop
from teknofest_ika.otonomi.topics import (
    RC_INPUT_TOPIC, MOD_KOMUT_TOPIC, CMD_VEL_TOPIC, E_STOP_TOPIC,
    MOD_AKTIF_TOPIC, MUX_CMD_VEL_TOPIC, MISSION_START_TOPIC,
    SPEED_LIMIT_TOPIC, E_STOP_FORCE_RC_TOPIC,
    NAV2_CMD_BAYATLAMA_S,
    KART_KIP_TOPIC, KART_DURUM_TOPIC, KART_KIP_OTONOM, DRM_KESME,
    MOD_MANUAL, MOD_FULL_AUTO,
)

# ─── Mod Sabitleri ─────────────────────────────────────────────────────────
# Değerler topics.py'de: ackermann_converter da aynı sayıya bakıyor.
_MOD_ISIMLER  = {MOD_MANUAL: 'MANUAL', MOD_FULL_AUTO: 'FULL_AUTO'}

# RC PWM eşikleri (µs — Flysky standart 1000–2000)
RC_NEUTRAL        = 1500
RC_MIN            = 1000
RC_MAX            = 2000
RC_DEADBAND       = 80       # ±80µs ölü bölge

# Hız sınırları (ackermann_converter da kırpar; burada güvenlik sınırı)
MANUAL_MAX_SPEED   = 2.0    # [m/s]
MANUAL_MAX_ANGULAR = 1.5    # [rad/s]

# RC zaman aşımı — sinyal kesilirse araç durdurulur
RC_TIMEOUT_S = 0.5          # [s]

# Kip bayatlama eşiği. Bu süre boyunca /kart/kip hiç gelmezse kartın ya da
# köprünün sustuğu varsayılır ve mod manuele düşer. Debounce'a gerek yok: kip
# eşiklenen bir pot değil, kartın verdiği ayrık bir karar.
#
# 🔴 Eşik, köprünün TEKRAR PERİYODUNA bağlıdır; kartın yayın hızına değil.
# seri_kopru /kart/kip'i yalnız değişimde ve DURUM_TAZELEME_S periyoduyla
# tekrarlıyor. Eşik o periyodun altına inerse kip her tekrar arasında bayat
# sayılır ve kart kip 2'de sabitken bile mod MANUAL↔FULL_AUTO zıplar; her
# FULL_AUTO geçişi /mission_start bastığı için misyon FSM'i hiç ilerleyemez
# (sahada ölçülen: 60 saniyede 122 geçiş, kart tek geçiş yaparken).
# 2.5 = tekrar periyodunun 2,5 katı; tek bir kayıp tekrar kipi düşürmemeli.
KIP_TEKRAR_PERIYOT_S = 1.0  # [s] seri_kopru.DURUM_TAZELEME_S ile aynı olmalı
KIP_TIMEOUT_S = 2.5         # [s]

# Kumanda kesmesinin (SwA) /e_stop/force/rc'ye tekrarlanma periyodu.
# 🔴 Yalnız DEĞİŞİMDE yayınlanan bir kesme açılışta kaybolur: kart açılışta
# zaten KESME'deyse yayın e_stop_node ayağa kalkmadan önce çıkar ve kimse
# duymaz; /e_stop false kalır ve SwA değişene kadar da düzelmez (sahada
# ölçülen fark: 3,7 s). seri_kopru buton durumunu aynı sebeple tekrarlıyor
# (_estop_yayinla).
#
# TRANSIENT_LOCAL seçilmedi: kaynak durumu sürekli, latch'lenmiş tek mesaj
# kaynak temizlendikten sonra doğan bir aboneye yanlış bilgi verir.
KESME_TEKRAR_S = 0.5        # [s]


class ModYoneticisi(Node):

    def __init__(self):
        super().__init__('mod_yoneticisi')

        self._mod      = MOD_MANUAL   # Kart otonom diyene kadar manuel
        self._lock     = threading.Lock()
        self._rc_son   = 0.0
        self._kip      = None    # kartın bildirdiği son kip
        self._kip_son  = 0.0
        self._drm      = 0       # kartın DRM_* bayrakları
        self._drm_geldi = False  # ilk 0x36'ya kadar kesme kaynağı susar

        self._ch1      = RC_NEUTRAL
        self._ch2      = RC_NEUTRAL

        self._nav2_twist = Twist()
        # Komutun GELDİĞİ an. Bu olmadan mux son Twist'i 20 Hz ile sonsuza
        # kadar tekrarlıyordu: Nav2 (ya da /cmd_vel'e doğrudan basan
        # RampaState/HizlanmaState) sıfırdan farklı bir komutla susarsa araç
        # o hızda gitmeye devam ederdi. Aşağı akıştaki hiçbir watchdog da
        # bunu yakalayamıyordu — ackermann_converter /mux/cmd_vel'i dinliyor
        # ve mux taze mesaj üretmeye devam ettiği için timeout'u hiç dolmuyor.
        self._nav2_son   = 0.0
        self._fsm_tetiklendi = False

        self._e_stop_aktif     = False
        self._kesme_aktif      = False   # kumandadan kesme (SwA) izleme
        # imu_guvenlik'ten gelen hız sınırı — FULL_AUTO'da Nav2 parametreleri
        # üzerinden de uygulanır ama MANUAL'de RC komutuna hiç yansımaz;
        # devrilme ya da düşük batarya gibi durumlarda manuel sürüşte de hız
        # kısıtlanmalı (Şartname §7.8 "araç en yüksek hızı güvenlik tehdidi
        # oluşturmayacak şekilde sınırlandırılmalı").
        self._speed_limit  = float('inf')

        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be  = QoSProfile(depth=5,  reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Float32MultiArray, RC_INPUT_TOPIC, self._rc_cb, qos_be)
        self.create_subscription(UInt8,  KART_KIP_TOPIC,   self._kip_cb,   qos_be)
        self.create_subscription(UInt16, KART_DURUM_TOPIC, self._durum_cb, qos_be)
        self.create_subscription(
            UInt8, MOD_KOMUT_TOPIC, self._komut_cb, 10)
        self.create_subscription(
            Twist, CMD_VEL_TOPIC, self._nav2_cb, qos_rel)
        self.create_subscription(
            Bool, E_STOP_TOPIC, self._e_stop_cb, 10)
        self.create_subscription(
            Float32, SPEED_LIMIT_TOPIC, self._speed_limit_cb, 10)

        self._mod_pub      = self.create_publisher(UInt8,  MOD_AKTIF_TOPIC,      10)
        self._mux_pub      = self.create_publisher(Twist,  MUX_CMD_VEL_TOPIC,    qos_rel)
        self._start_pub    = self.create_publisher(Bool,   MISSION_START_TOPIC,  10)
        self._kesme_pub    = self.create_publisher(Bool,   E_STOP_FORCE_RC_TOPIC, 10)

        self.create_timer(0.05, self._mux_dongusu)   # 20 Hz
        self.create_timer(1.0,  self._mod_yayinla)   # 1 Hz
        self.create_timer(KESME_TEKRAR_S, self._kesme_yayinla)

        self.get_logger().info(
            'ModYoneticisi hazır | başlangıç: MANUAL\n'
            '  kip kaynağı: /kart/kip (karar sürüş kartında)\n'
            '  /mod/komut 0=MANUAL 2=FULL_AUTO\n'
            '  /mux/cmd_vel → ackermann_converter'
        )

    # ── RC Callback — yalnız MANUAL twist ──────────────────────────────────
    def _rc_cb(self, msg: Float32MultiArray):
        # Dizinin mod alanı burada okunmuyor: kip kararı /kart/kip'ten geliyor.
        # Aux alanı da okunmuyor: sürüş kartı ham CH3'ü göndermiyor, dizide o
        # alan sabit duruyor. Lazer yetkisi /shoot_command'ın sahiplerinde —
        # misyon_fsm'in ShootState'i ve taret yazılımı.
        if len(msg.data) < 3:
            return
        with self._lock:
            self._ch1  = float(msg.data[0])
            self._ch2  = float(msg.data[1])
            self._rc_son = time.monotonic()

    # ── Kart kipi — mod otoritesi ───────────────────────────────────────────
    def _kip_cb(self, msg: UInt8):
        with self._lock:
            self._kip     = int(msg.data)
            self._kip_son = time.monotonic()

    # ── Kart durum bayrakları — kesme anahtarı E-STOP kaynağı ───────────────
    def _durum_cb(self, msg: UInt16):
        with self._lock:
            self._drm       = int(msg.data)
            self._drm_geldi = True
            drm, geldi = self._drm, self._drm_geldi

        kesme = kesme_estop(drm, geldi, DRM_KESME)
        if kesme != self._kesme_aktif:
            self._kesme_aktif = kesme
            if kesme:
                self.get_logger().error(
                    '!!! KUMANDA KESME (SwA) — E-STOP kaynağı aktif !!!')
            else:
                self.get_logger().warn('[E-STOP] kumanda kesmesi kaldırıldı.')
        # Yayın değişime bağlı DEĞİL: _kesme_yayinla periyodik basıyor.

    # ── Kesme kaynağının periyodik yayını ──────────────────────────────────
    def _kesme_yayinla(self):
        """
        Kumanda kesmesini KESME_TEKRAR_S'de bir tekrarlar.

        Tek seferlik bir yayın, geç doğan e_stop_node'a hiç ulaşmaz. Durum
        değişmese de basmak, kaynağın görünürlüğünü aboneliğin ne zaman
        kurulduğundan bağımsız kılar.
        """
        with self._lock:
            kesme = self._kesme_aktif
        self._kesme_pub.publish(Bool(data=kesme))

    # ── Yazılımsal Komut Callback ────────────────────────────────────────────
    def _komut_cb(self, msg: UInt8):
        if msg.data in (MOD_MANUAL, MOD_FULL_AUTO):
            self._mod_degistir(int(msg.data))
        else:
            self.get_logger().warn(
                f'Geçersiz mod komutu: {msg.data} — yalnız 0=MANUAL, 2=FULL_AUTO'
            )

    # ── Nav2 Twist Callback ──────────────────────────────────────────────────
    def _nav2_cb(self, msg: Twist):
        with self._lock:
            self._nav2_twist = msg
            self._nav2_son   = time.monotonic()

    # ── Hız Sınırı Callback (imu_guvenlik) ───────────────────────────────────
    def _speed_limit_cb(self, msg: Float32):
        with self._lock:
            self._speed_limit = float(msg.data)

    # ── E-STOP Callback ──────────────────────────────────────────────────────
    def _e_stop_cb(self, msg: Bool):
        with self._lock:
            self._e_stop_aktif = msg.data
        if msg.data:
            self.get_logger().error(
                '!!! E-STOP — /mux/cmd_vel sıfırlandı !!!',
                throttle_duration_sec=1.0
            )

    # ── Mod Değiştir ────────────────────────────────────────────────────────
    def _mod_degistir(self, yeni: int):
        onceki = self._mod
        self._mod = yeni
        self.get_logger().info(
            f'[MOD] {_MOD_ISIMLER[onceki]} → {_MOD_ISIMLER[yeni]}'
        )
        # FULL_AUTO'dan çıkınca bayrağı sıfırla — tekrar girilebilsin
        if onceki == MOD_FULL_AUTO and yeni != MOD_FULL_AUTO:
            self._fsm_tetiklendi = False
        # FULL_AUTO'ya geçişte FSM'i tetikle
        if yeni == MOD_FULL_AUTO and not self._fsm_tetiklendi:
            self._fsm_tetiklendi = True
            self._start_pub.publish(Bool(data=True))
            self.get_logger().info('[MOD] /mission_start yayınlandı — FSM başlıyor.')

    # ── Mux Döngüsü (20 Hz) ─────────────────────────────────────────────────
    def _mux_dongusu(self):
        with self._lock:
            e_stop      = self._e_stop_aktif
            mod         = self._mod
            ch1         = self._ch1
            ch2         = self._ch2
            rc_gecmis   = time.monotonic() - self._rc_son
            kip         = self._kip
            kip_gecmis  = time.monotonic() - self._kip_son
            kip_hic     = (self._kip_son == 0.0)
            nav2        = self._nav2_twist
            nav2_gecmis = time.monotonic() - self._nav2_son
            nav2_hic    = (self._nav2_son == 0.0)
            speed_limit = self._speed_limit

        # Kart kipini uygula. Kip hiç gelmediyse ya da bayatladıysa manuel:
        # kart susarken otonom komut basmak, gerçekte elle sürülen bir araca
        # komut göndermek olur.
        kip_bayat = kip_hic or kip_gecmis > KIP_TIMEOUT_S
        istenen   = kip_modu(kip if kip is not None else -1, kip_bayat,
                             KART_KIP_OTONOM, MOD_MANUAL, MOD_FULL_AUTO)
        if istenen != mod:
            self._mod_degistir(istenen)
            mod = istenen

        # E-STOP: tüm modlarda sıfır Twist yayınla
        if e_stop:
            self._mux_pub.publish(Twist())
            return

        out = Twist()

        if mod == MOD_MANUAL:
            # Manuel kipte aracı sürüş kartı doğrudan kumandadan sürüyor ve
            # buradan çıkan komutu yok sayıyor. Yayın yine de sürüyor çünkü
            # kart aldığı komutu 0x38 ile geri yolluyor: ölçek ve işaret
            # doğrulaması araç kımıldamadan burada yapılabiliyor.
            if rc_gecmis > RC_TIMEOUT_S:
                # Kanallar karttan geliyor; sessizlik kumandanın değil köprünün
                # ya da kartın sustuğu anlamına gelir.
                self.get_logger().warn(
                    'Kumanda kanalları gelmiyor — sıfır komut basılıyor.',
                    throttle_duration_sec=2.0
                )
            else:
                out = self._rc_twist(ch1, ch2)

        else:   # FULL_AUTO
            if nav2_hic or nav2_gecmis > NAV2_CMD_BAYATLAMA_S:
                # Bayat komut tekrarlanmaz, SIFIR basılır. Nav2'nin
                # velocity_smoother'ı araç durunca zaten susuyor (bilinen
                # davranış), o hâlde sıfır basmak doğru olanı yapıyor;
                # tehlikeli olan, hareket hâlindeyken susan bir yayıncının
                # son komutunun sonsuza kadar sürmesiydi.
                out = Twist()
                if not nav2_hic:
                    self.get_logger().warn(
                        f'[MUX] /cmd_vel {nav2_gecmis:.2f}s bayat — sıfır '
                        'komut basılıyor.',
                        throttle_duration_sec=2.0,
                    )
            else:
                out = nav2

        # imu_guvenlik hız sınırı (devrilme/düşük batarya vb.) — FULL_AUTO'da
        # zaten terrain_adapter→Nav2 yolundan da uygulanır, ama MANUAL'de bu
        # son savunma hattı olmadan RC komutu sınırsız geçerdi.
        if speed_limit < float('inf'):
            out.linear.x = max(-speed_limit, min(speed_limit, out.linear.x))

        self._mux_pub.publish(out)

    # ── RC PWM → Twist ──────────────────────────────────────────────────────
    def _rc_twist(self, ch1: float, ch2: float) -> Twist:
        def norm(pwm: float) -> float:
            d = pwm - RC_NEUTRAL
            if abs(d) < RC_DEADBAND:
                return 0.0
            rng  = (RC_MAX - RC_MIN) / 2.0 - RC_DEADBAND
            sign = 1.0 if d > 0 else -1.0
            return sign * max(0.0, min(1.0, (abs(d) - RC_DEADBAND) / rng))

        t = Twist()
        t.linear.x  = norm(ch1) * MANUAL_MAX_SPEED
        t.angular.z = norm(ch2) * MANUAL_MAX_ANGULAR
        return t

    def _mod_yayinla(self):
        self._mod_pub.publish(UInt8(data=self._mod))


def main(args=None):
    rclpy.init(args=args)
    node = ModYoneticisi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
