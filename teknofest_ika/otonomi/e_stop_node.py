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

E-STOP KAYNAK MİMARİSİ (OR mantığı):
  /e_stop/force'a birden fazla node yayın yapar:
    - imu_guvenlik  : devrilme tespiti (10 Hz sürekli)
    - seri_kopru    : fiziksel buton (olay bazlı)
    - lora_gcs      : GCS komutu (olay bazlı)

  Her kaynak ayrı takip edilir. Herhangi biri True → /e_stop True.
  Tümü aynı anda False göndermeden E-STOP temizlenmez.
  Bu sayede imu_guvenlik'in sürekli False yayını fiziksel butonu temizleyemez.
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from teknofest_ika.otonomi.topics import (
    E_STOP_TOPIC, E_STOP_FORCE_TOPIC, E_STOP_GPIO_FAULT_TOPIC,
)

_GPIO_MEVCUT = False
try:
    import Jetson.GPIO as GPIO
    _GPIO_MEVCUT = True
except ImportError:
    pass

# /e_stop/force yayıncıları — her biri ayrı takip edilir
_KAYNAK_IMU    = 'imu_guvenlik'
_KAYNAK_SERIAL = 'seri_kopru'
_KAYNAK_GCS    = 'lora_gcs'
_KAYNAK_GENEL  = 'genel'


class EStopNode(Node):

    def __init__(self):
        super().__init__('e_stop_node')

        self.declare_parameter('gpio_pin',   7)
        self.declare_parameter('gpio_mod',   True)
        self.declare_parameter('publish_hz', 20.0)

        self._gpio_pin = self.get_parameter('gpio_pin').value
        self._gpio_istenen = self.get_parameter('gpio_mod').value
        self._gpio_mod = self._gpio_istenen and _GPIO_MEVCUT

        # OR mantığı: her kaynak bağımsız takip edilir
        # Herhangi biri True → _aktif True
        self._lock = threading.Lock()
        self._gpio_aktif   = False   # fiziksel GPIO butonu
        self._force_aktif  = False   # /e_stop/force'tan gelen yazılımsal E-STOP

        self._gpio_fault = False
        self._gpio_fault_pub = self.create_publisher(Bool, E_STOP_GPIO_FAULT_TOPIC, 10)

        self._gpio_kur()

        self._pub = self.create_publisher(Bool, E_STOP_TOPIC, 10)
        self.create_subscription(Bool, E_STOP_FORCE_TOPIC, self._force_cb, 10)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._yayinla)

        self.get_logger().info(
            f'EStopNode hazır | GPIO: {"aktif pin=" + str(self._gpio_pin) if self._gpio_mod else "devre dışı"}'
        )

    @property
    def _aktif(self) -> bool:
        """GPIO VEYA yazılımsal kaynaklardan herhangi biri True → E-STOP aktif."""
        return self._gpio_aktif or self._force_aktif

    # ── GPIO kurulumu ────────────────────────────────────────────────────────
    def _gpio_kur(self):
        if not self._gpio_mod:
            if self._gpio_istenen and not _GPIO_MEVCUT:
                # Fiziksel buton istenmiş ama kütüphane yok — operatör bunu
                # bilmeli, sadece log'da kalırsa fiziksel buton sessizce
                # işlevsiz kalır (ayrılık ilkesi ihlali riski).
                self._gpio_fault = True
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
                bouncetime=100
            )
            # Başlangıçta mevcut pin durumunu oku (güç gelirken buton basılıysa)
            with self._lock:
                self._gpio_aktif = (GPIO.input(self._gpio_pin) == GPIO.LOW)
            if self._gpio_aktif:
                self.get_logger().error('!!! BAŞLANGIÇTA E-STOP AKTIF — Butonu kontrol et !!!')
        except Exception as exc:
            self.get_logger().error(f'GPIO kur hatası: {exc} — GPIO devre dışı bırakıldı.')
            self._gpio_mod = False
            self._gpio_fault = True

    # ── GPIO edge callback ───────────────────────────────────────────────────
    def _gpio_cb(self, channel):
        try:
            low = (GPIO.input(self._gpio_pin) == GPIO.LOW)
        except Exception:
            return
        with self._lock:
            onceki = self._gpio_aktif
            self._gpio_aktif = low
        if low and not onceki:
            self.get_logger().error(
                '!!! E-STOP BUTONUNA BASILDI — Tüm hareket durduruldu !!!'
            )
        elif not low and onceki:
            self.get_logger().warn(
                '[E-STOP] Fiziksel buton bırakıldı.'
            )

    # ── Yazılımsal override — OR mantığı ────────────────────────────────────
    def _force_cb(self, msg: Bool):
        """
        Birden fazla kaynak bu topic'e yayın yapar (imu_guvenlik, seri_kopru, lora_gcs).
        True → force_aktif bayrağını set et (OR mantığı).
        False → sadece force_aktif'i temizle; GPIO butonu hâlâ basılıysa E-STOP sürer.
        """
        with self._lock:
            onceki_force = self._force_aktif
            self._force_aktif = msg.data
            aktif = self._aktif   # OR sonucu

        if msg.data and not onceki_force:
            self.get_logger().error('!!! YAZILIMSAL E-STOP AKTİF !!!')
        elif not msg.data and onceki_force:
            if aktif:
                self.get_logger().warn(
                    '[E-STOP] Yazılımsal force kaldırıldı ama GPIO butonu hâlâ basılı.'
                )
            else:
                self.get_logger().warn('[E-STOP] Yazılımsal e-stop kaldırıldı.')

    # ── 20 Hz yayın ─────────────────────────────────────────────────────────
    def _yayinla(self):
        with self._lock:
            aktif = self._aktif
        self._pub.publish(Bool(data=aktif))
        self._gpio_fault_pub.publish(Bool(data=self._gpio_fault))

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
