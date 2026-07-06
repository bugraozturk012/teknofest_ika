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
    E_STOP_TOPIC, E_STOP_GPIO_FAULT_TOPIC,
    E_STOP_FORCE_IMU_TOPIC, E_STOP_FORCE_SERIAL_TOPIC, E_STOP_FORCE_GCS_TOPIC,
    E_STOP_FORCE_RC_TOPIC,
)

_GPIO_MEVCUT = False
try:
    import Jetson.GPIO as GPIO
    _GPIO_MEVCUT = True
except ImportError:
    pass

_KAYNAK_IMU    = 'imu'
_KAYNAK_SERIAL = 'serial'
_KAYNAK_GCS    = 'gcs'
_KAYNAK_RC     = 'rc'


class EStopNode(Node):

    def __init__(self):
        super().__init__('e_stop_node')

        self.declare_parameter('gpio_pin',   7)
        self.declare_parameter('gpio_mod',   True)
        self.declare_parameter('publish_hz', 20.0)

        self._gpio_pin = self.get_parameter('gpio_pin').value
        self._gpio_istenen = self.get_parameter('gpio_mod').value
        self._gpio_mod = self._gpio_istenen and _GPIO_MEVCUT

        self._lock = threading.Lock()
        self._gpio_aktif = False  # fiziksel GPIO butonu

        # Her kaynak bağımsız takip edilir — herhangi biri True → E-STOP aktif.
        # Tek bool kullanmak OR mantığını bozar: imu True gönderip False'a
        # dönünce seri_kopru'nun True'su silinirdi.
        self._force_sources = {
            _KAYNAK_IMU:    False,
            _KAYNAK_SERIAL: False,
            _KAYNAK_GCS:    False,
            _KAYNAK_RC:     False,
        }

        self._gpio_fault = False
        self._gpio_fault_pub = self.create_publisher(Bool, E_STOP_GPIO_FAULT_TOPIC, 10)

        self._gpio_kur()

        self._pub = self.create_publisher(Bool, E_STOP_TOPIC, 10)
        self.create_subscription(Bool, E_STOP_FORCE_IMU_TOPIC,
                                 lambda m: self._force_cb(m, _KAYNAK_IMU), 10)
        self.create_subscription(Bool, E_STOP_FORCE_SERIAL_TOPIC,
                                 lambda m: self._force_cb(m, _KAYNAK_SERIAL), 10)
        self.create_subscription(Bool, E_STOP_FORCE_GCS_TOPIC,
                                 lambda m: self._force_cb(m, _KAYNAK_GCS), 10)
        self.create_subscription(Bool, E_STOP_FORCE_RC_TOPIC,
                                 lambda m: self._force_cb(m, _KAYNAK_RC), 10)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._yayinla)

        self.get_logger().info(
            f'EStopNode hazır | GPIO: {"aktif pin=" + str(self._gpio_pin) if self._gpio_mod else "devre dışı"}'
        )

    @property
    def _aktif(self) -> bool:
        """GPIO VEYA herhangi bir yazılımsal kaynak True → E-STOP aktif."""
        return self._gpio_aktif or any(self._force_sources.values())

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

    # ── Yazılımsal override — per-kaynak OR mantığı ─────────────────────────
    def _force_cb(self, msg: Bool, kaynak: str):
        """
        Her kaynak kendi topic'ine yayın yapar; bu callback kaynağı bilir.
        Yalnızca o kaynağın bayrağı güncellenir — diğer kaynakların durumu
        değişmez. E-STOP, tüm kaynaklar False olmadan temizlenmez.
        """
        with self._lock:
            onceki = self._force_sources[kaynak]
            self._force_sources[kaynak] = msg.data
            aktif = self._aktif

        if msg.data and not onceki:
            self.get_logger().error(f'!!! YAZILIMSAL E-STOP [{kaynak}] AKTİF !!!')
        elif not msg.data and onceki:
            if aktif:
                self.get_logger().warn(
                    f'[E-STOP] [{kaynak}] kaldırıldı ama başka kaynak hâlâ aktif.'
                )
            else:
                self.get_logger().warn(f'[E-STOP] [{kaynak}] kaldırıldı — tüm kaynaklar temiz.')

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
