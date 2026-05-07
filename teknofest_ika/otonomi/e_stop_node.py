#!/usr/bin/env python3
"""
e_stop_node.py — Schneider XB5AS84W3B5 E-STOP Buton Node'u
============================================================
Fiziksel acil durdurma butonunu izler, /e_stop (Bool) yayınlar.

BAĞLANTI (NC — fail-safe konfigürasyon):
  Schneider NC kontağı → Jetson GPIO pin (BOARD numarası, varsayılan: 7)
  Buton basılmadığında: NC kapalı → pin HIGH (3.3V pull-up)
  Buton basılınca    : NC açılır → pin LOW  → e_stop = True

  Fail-safe mantığı: kablo kopsa bile pin LOW → araç durur.

LATCHING DAVRANIŞ:
  Schneider XB5AS84W3B5 mandallamalı butondur.
  Basılınca kilitlenir → /e_stop True (sürekli)
  Çevirince açılır    → /e_stop False

YAZILIMSAL OVERRIDE (test için):
  ros2 topic pub /e_stop/force std_msgs/msg/Bool "data: true" --once
  ros2 topic pub /e_stop/force std_msgs/msg/Bool "data: false" --once

PARAMETRE:
  gpio_pin   : Jetson BOARD pin numarası (varsayılan: 7)
  gpio_mod   : False → GPIO kullanma, sadece yazılımsal override (varsayılan: True)
  publish_hz : Yayın frekansı Hz (varsayılan: 20)

NOT: Jetson.GPIO paketi kurulu değilse gpio_mod otomatik devre dışı kalır.
     sudo pip3 install Jetson.GPIO
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

_GPIO_MEVCUT = False
try:
    import Jetson.GPIO as GPIO
    _GPIO_MEVCUT = True
except ImportError:
    pass


class EStopNode(Node):

    def __init__(self):
        super().__init__('e_stop_node')

        self.declare_parameter('gpio_pin',   7)
        self.declare_parameter('gpio_mod',   True)
        self.declare_parameter('publish_hz', 20.0)

        self._gpio_pin = self.get_parameter('gpio_pin').value
        self._gpio_mod = self.get_parameter('gpio_mod').value and _GPIO_MEVCUT
        self._aktif    = False   # gerçek e-stop durumu

        self._gpio_kur()

        self._pub = self.create_publisher(Bool, '/e_stop', 10)
        self.create_subscription(Bool, '/e_stop/force', self._force_cb, 10)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._yayinla)

        self.get_logger().info(
            f'EStopNode hazır | GPIO: {"aktif pin=" + str(self._gpio_pin) if self._gpio_mod else "devre dışı"}'
        )

    # ── GPIO kurulumu ────────────────────────────────────────────────────────
    def _gpio_kur(self):
        if not self._gpio_mod:
            if not _GPIO_MEVCUT:
                self.get_logger().warn(
                    'Jetson.GPIO bulunamadı. Yükle: sudo pip3 install Jetson.GPIO\n'
                    'GPIO olmadan yalnızca /e_stop/force çalışır.'
                )
            return
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self._gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(
                self._gpio_pin, GPIO.BOTH,
                callback=self._gpio_cb,
                bouncetime=50
            )
            # Başlangıçta mevcut pin durumunu oku (güç gelirken buton basılıysa)
            self._aktif = (GPIO.input(self._gpio_pin) == GPIO.LOW)
            if self._aktif:
                self.get_logger().error('!!! BAŞLANGIÇTA E-STOP AKTIF — Butonu kontrol et !!!')
        except Exception as exc:
            self.get_logger().error(f'GPIO kur hatası: {exc} — GPIO devre dışı bırakıldı.')
            self._gpio_mod = False

    # ── GPIO edge callback ───────────────────────────────────────────────────
    def _gpio_cb(self, channel):
        try:
            low = (GPIO.input(self._gpio_pin) == GPIO.LOW)
        except Exception:
            return
        if low and not self._aktif:
            self._aktif = True
            self.get_logger().error(
                '!!! E-STOP BUTONUNA BASILDI — Tüm hareket durduruldu !!!'
            )
        elif not low and self._aktif:
            self._aktif = False
            self.get_logger().warn(
                '[E-STOP] Buton bırakıldı. /e_stop False yayınlanıyor.'
            )

    # ── Yazılımsal override ──────────────────────────────────────────────────
    def _force_cb(self, msg: Bool):
        self._aktif = msg.data
        if msg.data:
            self.get_logger().error('!!! YAZILIMSAL E-STOP AKTİF !!!')
        else:
            self.get_logger().warn('[E-STOP] Yazılımsal e-stop kaldırıldı.')

    # ── 20 Hz yayın ─────────────────────────────────────────────────────────
    def _yayinla(self):
        self._pub.publish(Bool(data=self._aktif))

    # ── Temizlik ─────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._gpio_mod:
            try:
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
