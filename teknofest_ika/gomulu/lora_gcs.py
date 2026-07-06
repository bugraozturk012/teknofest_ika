#!/usr/bin/env python3
"""
lora_gcs.py — LR02 433MHz LoRa GCS Telemetri Köprüsü
======================================================
Robot ↔ Yer Kontrol İstasyonu (GCS) çift yönlü iletişim.

DONANIM:
  LR02 433MHz 22dBm modülü → PL2303 USB-TTL → /dev/lora
  Varsayılan mod: şeffaf UART (transparent mode)
  Baud: 9600 (AT+BAUD=9600 ile ayarlanmış)

ROBOT → GCS (1 Hz, JSON):
  {"t":12,"m":2,"s":"NAVIGATE","v":15.8,"b":78,"e":0,"x":1.2,"y":0.5,"w":3}
  t: çalışma süresi [s mod 65536]
  m: mod (0=MANUAL 1=SEMI 2=FULL_AUTO)
  s: FSM durumu (IDLE/NAVIGATE/SHOOT_APPROACH/SHOOT/COMPLETE/ERROR)
  v: batarya voltajı [V]
  b: batarya yüzdesi [0-100]
  e: e-stop (0/1)
  x,y: pozisyon [m, odom frame]
  w: mevcut waypoint indeksi

GCS → ROBOT (JSON komut):
  {"cmd":"estop","val":1}    → e-stop aktifleştir
  {"cmd":"estop","val":0}    → e-stop kaldır
  {"cmd":"mode","val":0}     → modu değiştir (0/1/2)

GCS tarafında çalıştırılacak Python script'i:
  ~/ika_ws/scripts/gcs_terminal.py

PARAMETRE:
  port       : /dev/lora        (LR02 seri port)
  baud       : 9600
  publish_hz : 1.0              (telemetri gönderim sıklığı)
  at_init    : True             (başlangıçta AT komutu ile yapılandır)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import UInt8, Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

try:
    import serial
    _SERIAL_MEVCUT = True
except ImportError:
    _SERIAL_MEVCUT = False

from teknofest_ika.otonomi.topics import (
    E_STOP_FORCE_GCS_TOPIC, MOD_KOMUT_TOPIC, MOD_AKTIF_TOPIC,
    FSM_STATE_TOPIC, BATTERY_TOPIC, E_STOP_TOPIC, ODOM_TOPIC,
    MISYON_WP_INDEX_TOPIC,
)

_MOD_ISIM = {0: 'MANUAL', 1: 'SEMI', 2: 'AUTO'}


class LoraGCS(Node):

    def __init__(self):
        super().__init__('lora_gcs')

        self.declare_parameter('port',       '/dev/lora')
        self.declare_parameter('baud',       9600)
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('at_init',    True)

        self._port       = self.get_parameter('port').value
        self._baud       = self.get_parameter('baud').value
        self._hz         = self.get_parameter('publish_hz').value
        self._at_init    = self.get_parameter('at_init').value

        # Telemetri durumu
        self._lock      = threading.Lock()
        self._mod       = 0
        self._fsm       = 'IDLE'
        self._voltaj    = 0.0
        self._batarya   = 0
        self._e_stop    = False
        self._x         = 0.0
        self._y         = 0.0
        self._wp_index  = 0
        self._baslangic = time.time()

        # GCS komutlarından gelen publisher'lar
        self._e_stop_pub = self.create_publisher(Bool,  E_STOP_FORCE_GCS_TOPIC, 10)
        self._mod_pub    = self.create_publisher(UInt8, MOD_KOMUT_TOPIC,    10)

        # Abonelikler
        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(UInt8,        MOD_AKTIF_TOPIC,      self._mod_cb,    10)
        self.create_subscription(String,       FSM_STATE_TOPIC,      self._fsm_cb,    10)
        self.create_subscription(BatteryState, BATTERY_TOPIC,        self._bat_cb,    10)
        self.create_subscription(Bool,         E_STOP_TOPIC,         self._estop_cb,  10)
        self.create_subscription(Odometry,     ODOM_TOPIC,           self._odom_cb,   qos_be)
        self.create_subscription(UInt8,        MISYON_WP_INDEX_TOPIC, self._wp_cb,    10)

        # Seri port
        self._ser = None
        self._ser_kur()

        # Okuma thread
        if self._ser:
            threading.Thread(target=self._okuma_dongusu, daemon=True).start()

        self.create_timer(1.0 / self._hz, self._telemetri_gonder)
        self.get_logger().info(
            f'LoraGCS hazır | port={self._port} baud={self._baud} '
            f'{"[BAĞLI]" if self._ser else "[SİM]"}'
        )

    # ── Seri port kurulumu ───────────────────────────────────────────────────
    def _ser_kur(self):
        if not _SERIAL_MEVCUT:
            self.get_logger().error('pyserial yok. pip3 install pyserial')
            return
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=0.5)
            if self._at_init:
                self._at_yapilandir()
        except serial.SerialException as exc:
            self.get_logger().error(f'LR02 port açılamadı: {exc}')
            self._ser = None

    def _at_yapilandir(self):
        """LR02 AT komutları ile şeffaf moda geçiş."""
        komutlar = [
            b'AT+RST\r\n',           # Sıfırla
            b'AT+MODE=1\r\n',        # Şeffaf mod
            b'AT+FREQ=433000000\r\n', # 433 MHz
            b'AT+POWER=22\r\n',      # Maks güç (22 dBm)
            b'AT+BAUD=9600\r\n',     # 9600 baud
        ]
        try:
            for komut in komutlar:
                self._ser.write(komut)
                time.sleep(0.15)
            # Cevap oku (ignore)
            self._ser.read(self._ser.in_waiting or 1)
            self.get_logger().info('LR02 AT yapılandırması tamamlandı.')
        except Exception as exc:
            self.get_logger().warn(
                f'AT yapılandırma başarısız (zaten şeffaf modda olabilir): {exc}'
            )

    # ── Abonelik callback'leri ───────────────────────────────────────────────
    def _mod_cb(self, msg: UInt8):
        with self._lock:
            self._mod = int(msg.data)

    def _fsm_cb(self, msg: String):
        with self._lock:
            self._fsm = msg.data[:12]   # max 12 karakter — LoRa bant genişliği

    def _bat_cb(self, msg: BatteryState):
        with self._lock:
            self._voltaj  = round(msg.voltage, 1)
            self._batarya = int(msg.percentage * 100)

    def _estop_cb(self, msg: Bool):
        with self._lock:
            self._e_stop = msg.data

    def _odom_cb(self, msg: Odometry):
        with self._lock:
            self._x = round(msg.pose.pose.position.x, 1)
            self._y = round(msg.pose.pose.position.y, 1)

    def _wp_cb(self, msg: UInt8):
        with self._lock:
            self._wp_index = int(msg.data)

    # ── Telemetri gönder (1 Hz) ──────────────────────────────────────────────
    def _telemetri_gonder(self):
        with self._lock:
            paket = {
                't':  int(time.time() - self._baslangic) % 65536,
                'm':  self._mod,
                's':  self._fsm,
                'v':  self._voltaj,
                'b':  self._batarya,
                'e':  int(self._e_stop),
                'x':  self._x,
                'y':  self._y,
                'w':  self._wp_index,
            }

        ham = (json.dumps(paket, separators=(',', ':')) + '\n').encode()

        if self._ser:
            try:
                self._ser.write(ham)
            except serial.SerialException as exc:
                self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
        else:
            self.get_logger().debug(f'[SIM] LoRa TX: {ham.decode().strip()}')

    # ── GCS'den gelen komutları oku ──────────────────────────────────────────
    def _okuma_dongusu(self):
        tampon = b''
        while rclpy.ok():
            try:
                veri = self._ser.read(64)
                if not veri:
                    continue
                tampon += veri
                while b'\n' in tampon:
                    satir, tampon = tampon.split(b'\n', 1)
                    self._komut_isle(satir.strip())
            except serial.SerialException as exc:
                self.get_logger().error(str(exc), throttle_duration_sec=5.0)
                time.sleep(0.5)

    def _komut_isle(self, ham: bytes):
        if not ham:
            return
        try:
            veri = json.loads(ham.decode())
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.get_logger().warn(
                f'GCS geçersiz paket: {ham!r}', throttle_duration_sec=2.0
            )
            return

        komut = veri.get('cmd', '')
        deger = veri.get('val', 0)

        if komut == 'estop':
            self._e_stop_pub.publish(Bool(data=bool(deger)))
            self.get_logger().warn(
                f'GCS E-STOP komutu: {"AKTİF" if deger else "KALDIRILDI"}'
            )
        elif komut == 'mode':
            if deger in (0, 1, 2):
                self._mod_pub.publish(UInt8(data=int(deger)))
                self.get_logger().info(
                    f'GCS mod komutu: {_MOD_ISIM.get(deger, "?")}'
                )
            else:
                self.get_logger().warn(f'GCS geçersiz mod: {deger}')
        else:
            self.get_logger().warn(f'GCS bilinmeyen komut: {komut!r}')

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LoraGCS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
