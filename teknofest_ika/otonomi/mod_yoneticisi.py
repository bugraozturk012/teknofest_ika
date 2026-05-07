#!/usr/bin/env python3
"""
mod_yoneticisi.py — Manuel / Yarı-Otonom / Tam Otonom Mod Yöneticisi
======================================================================
MOD TANIMI:
  MANUAL    (0): RC kumanda doğrudan VESC'i sürer. Nav2 pasif.
  SEMI_AUTO (1): RC hareket ettirirse override, değilse Nav2 sürer.
  FULL_AUTO (2): Tam otonom — misyon_fsm + Nav2 kontrolü.

RC GEÇİŞ LOJİĞİ (Flysky FS-i6X, ch5 mod anahtarı):
  ch5 < 1300 µs  → MANUAL
  1300 ≤ ch5 < 1700 µs → SEMI_AUTO
  ch5 ≥ 1700 µs  → FULL_AUTO

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
  /rc_input    (Float32MultiArray): [ch1_throttle_us, ch2_steering_us, ch5_switch_us]
  /mod/komut   (UInt8)
  /cmd_vel     (Twist) — Nav2 çıkışı

ÇIKIŞLAR:
  /mod/aktif      (UInt8)  — 0/1/2
  /mux/cmd_vel    (Twist)  — muxlanmış komut
  /mission_start  (Bool)   — FULL_AUTO'ya ilk geçişte FSM'i tetikler
"""
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import UInt8, Bool, Float32MultiArray
from geometry_msgs.msg import Twist

# ─── Mod Sabitleri ─────────────────────────────────────────────────────────
MOD_MANUAL    = 0
MOD_SEMI_AUTO = 1
MOD_FULL_AUTO = 2
_MOD_ISIMLER  = {0: 'MANUAL', 1: 'SEMI_AUTO', 2: 'FULL_AUTO'}

# RC PWM eşikleri (µs — Flysky standart 1000–2000)
RC_CH5_MANUAL_MAX = 1300
RC_CH5_SEMI_MAX   = 1700
RC_NEUTRAL        = 1500
RC_MIN            = 1000
RC_MAX            = 2000
RC_DEADBAND       = 80       # ±80µs ölü bölge

# ch3 aux — lazer tetikleme eşiği (MANUAL modda)
RC_CH3_LAZER_ON  = 1700   # > 1700µs → lazer aç
RC_CH3_LAZER_OFF = 1300   # < 1300µs → lazer kapat

# Hız sınırları (ackermann_converter da kırpar; burada güvenlik sınırı)
MANUAL_MAX_SPEED   = 2.0    # [m/s]
MANUAL_MAX_ANGULAR = 1.5    # [rad/s]

# Yarı-otonom override eşiği (normalize 0–1)
SEMI_OVERRIDE_THRESHOLD = 0.20

# RC zaman aşımı — sinyal kesilirse araç durdurulur
RC_TIMEOUT_S = 0.5          # [s]

# Mod geçiş debounce — switch geçici SEMI_AUTO'ya düşerse görmezden gel
MOD_DEBOUNCE_S = 0.25       # [s] — bu süre stabil kalmazsa mod değişmez


class ModYoneticisi(Node):

    def __init__(self):
        super().__init__('mod_yoneticisi')

        self._mod      = MOD_MANUAL   # Güvenli başlangıç: RC pozisyonuna göre geçiş
        self._lock     = threading.Lock()
        self._rc_son   = 0.0

        self._ch1      = RC_NEUTRAL
        self._ch2      = RC_NEUTRAL
        self._ch5      = RC_NEUTRAL
        self._ch3      = RC_MIN       # ch3 aux — varsayılan: lazer kapalı
        self._lazer_acik = False

        self._nav2_twist = Twist()
        self._fsm_tetiklendi = False

        self._mod_bekleyen      = None   # debounce: beklenen yeni mod
        self._mod_bekleyen_zaman = 0.0   # debounce: ne zaman değişmeye başladı

        self._e_stop_aktif = False

        qos_rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_be  = QoSProfile(depth=5,  reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Float32MultiArray, '/rc_input', self._rc_cb, qos_be)
        self.create_subscription(
            UInt8, '/mod/komut', self._komut_cb, 10)
        self.create_subscription(
            Twist, '/cmd_vel', self._nav2_cb, qos_rel)
        self.create_subscription(
            Bool, '/e_stop', self._e_stop_cb, 10)

        self._mod_pub   = self.create_publisher(UInt8,  '/mod/aktif',     10)
        self._mux_pub   = self.create_publisher(Twist,  '/mux/cmd_vel',   qos_rel)
        self._start_pub = self.create_publisher(Bool,   '/mission_start', 10)
        self._shoot_pub = self.create_publisher(Bool,   '/shoot_command', 10)

        self.create_timer(0.05, self._mux_dongusu)   # 20 Hz
        self.create_timer(1.0,  self._mod_yayinla)   # 1 Hz

        self.get_logger().info(
            'ModYoneticisi hazır | başlangıç: MANUAL\n'
            '  /mod/komut 0=MANUAL 1=SEMI_AUTO 2=FULL_AUTO\n'
            '  /mux/cmd_vel → ackermann_converter'
        )

    # ── RC Callback ─────────────────────────────────────────────────────────
    def _rc_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        with self._lock:
            self._ch1  = float(msg.data[0])
            self._ch2  = float(msg.data[1])
            self._ch5  = float(msg.data[2])
            self._ch3  = float(msg.data[3]) if len(msg.data) > 3 else RC_MIN
            self._rc_son = time.time()

        yeni = self._ch5_mod(self._ch5)
        with self._lock:
            if yeni != self._mod:
                if self._mod_bekleyen != yeni:
                    self._mod_bekleyen       = yeni
                    self._mod_bekleyen_zaman = time.time()
            else:
                self._mod_bekleyen = None

    def _ch5_mod(self, ch5: float) -> int:
        if ch5 < RC_CH5_MANUAL_MAX:
            return MOD_MANUAL
        if ch5 < RC_CH5_SEMI_MAX:
            return MOD_SEMI_AUTO
        return MOD_FULL_AUTO

    # ── Yazılımsal Komut Callback ────────────────────────────────────────────
    def _komut_cb(self, msg: UInt8):
        if msg.data in (MOD_MANUAL, MOD_SEMI_AUTO, MOD_FULL_AUTO):
            self._mod_degistir(int(msg.data))
        else:
            self.get_logger().warn(f'Geçersiz mod komutu: {msg.data}')

    # ── Nav2 Twist Callback ──────────────────────────────────────────────────
    def _nav2_cb(self, msg: Twist):
        with self._lock:
            self._nav2_twist = msg

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
        # FULL_AUTO'ya ilk geçişte FSM'i tetikle
        if yeni == MOD_FULL_AUTO and not self._fsm_tetiklendi:
            self._fsm_tetiklendi = True
            self._start_pub.publish(Bool(data=True))
            self.get_logger().info('[MOD] /mission_start yayınlandı — FSM başlıyor.')

    # ── Mux Döngüsü (20 Hz) ─────────────────────────────────────────────────
    def _mux_dongusu(self):
        # Debounce: lock dışında uygula (publish/log lock altında çağrılmamalı)
        mod_uygulanacak = None
        with self._lock:
            if (self._mod_bekleyen is not None and
                    time.time() - self._mod_bekleyen_zaman >= MOD_DEBOUNCE_S):
                mod_uygulanacak    = self._mod_bekleyen
                self._mod_bekleyen = None
        if mod_uygulanacak is not None:
            self._mod_degistir(mod_uygulanacak)

        with self._lock:
            e_stop    = self._e_stop_aktif
            mod       = self._mod
            ch1       = self._ch1
            ch2       = self._ch2
            ch3       = self._ch3
            rc_gecmis = time.time() - self._rc_son
            nav2      = self._nav2_twist

        # E-STOP: tüm modlarda sıfır Twist yayınla
        if e_stop:
            self._mux_pub.publish(Twist())
            return

        out = Twist()

        if mod == MOD_MANUAL:
            if rc_gecmis > RC_TIMEOUT_S:
                self.get_logger().warn(
                    'RC sinyal yok! Araç durduruluyor.',
                    throttle_duration_sec=2.0
                )
                # out = sıfır Twist → dur
            else:
                out = self._rc_twist(ch1, ch2)
                self._lazer_kontrol(ch3)

        elif mod == MOD_SEMI_AUTO:
            rc_cmd = self._rc_twist(ch1, ch2)
            rc_norm = (abs(rc_cmd.linear.x) / MANUAL_MAX_SPEED +
                       abs(rc_cmd.angular.z) / MANUAL_MAX_ANGULAR) / 2.0
            out = rc_cmd if rc_norm > SEMI_OVERRIDE_THRESHOLD else nav2

        else:   # FULL_AUTO
            out = nav2

        self._mux_pub.publish(out)

    # ── MANUAL Lazer Tetikleme (ch3 aux) ────────────────────────────────────
    def _lazer_kontrol(self, ch3: float):
        if ch3 > RC_CH3_LAZER_ON and not self._lazer_acik:
            self._lazer_acik = True
            self._shoot_pub.publish(Bool(data=True))
            self.get_logger().info('[MANUAL] Lazer AÇIK (ch3)')
        elif ch3 < RC_CH3_LAZER_OFF and self._lazer_acik:
            self._lazer_acik = False
            self._shoot_pub.publish(Bool(data=False))
            self.get_logger().info('[MANUAL] Lazer KAPALI (ch3)')

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
