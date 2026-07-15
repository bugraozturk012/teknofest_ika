#!/usr/bin/env python3
"""
taret_rc_koprusu.py  —  RC → Turret UNO Manuel Nişan Köprüsü
================================================================
Sağ stick'i (CH1/CH2) MANUEL modda ve SWB (taret aktif) açıkken pan/tilt
joystick'ine çevirip Turret UNO'ya (PCA9685 + BMI160, "P:{pan},T:{tilt}\n"
formatı) iletir. Kanal ataması ve seri protokol elektrikçi arkadaşın
turret_control.py + Arduino UNO koduyla birebir uyumlu — tek fark, kaynağın
ham pyserial yerine bu ROS2 node'u olması (seri_kopru zaten iBUS'ı çözüyor,
ikinci bir Mega/decode'a gerek yok).

Sağ stick MANUEL sürüşte direksiyon olarak da kullanıldığından (CH1), taret
aktifken main.cpp sürüşü zaten kilitler (bkz. rc_taret_aktif() / config.h) —
burada ayrıca mod/aktif doğrulaması, RC bağlantısı kesilirse veya taret
kapatılırsa güvenli şekilde merkeze (90/90) dönmek için yapılır.
"""

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32MultiArray, UInt8

from teknofest_ika.otonomi.topics import (
    RC_INPUT_TOPIC, MOD_AKTIF_TOPIC, SERIAL_TARET, SERIAL_BAUD,
)
from teknofest_ika.otonomi.mod_yoneticisi import MOD_MANUAL

RC_PWM_MIN  = 1000
RC_PWM_MAX  = 2000
RC_MOD_ESIK = 1500   # SWB > bu değer → taret aktif (main.cpp/config.h ile aynı eşik)
RC_STALE_S  = 0.3    # main.cpp RC_TIMEOUT_MS ile aynı eşik — bu süre mesaj gelmezse merkeze dön

PAN_ACI_MIN,  PAN_ACI_MAX  = 0, 180
TILT_ACI_MIN, TILT_ACI_MAX = 0, 180
MERKEZ_ACI = 90


def _us_to_aci(us: float, aci_min: int, aci_max: int) -> int:
    oran = (us - RC_PWM_MIN) / (RC_PWM_MAX - RC_PWM_MIN)
    oran = max(0.0, min(1.0, oran))
    return int(aci_min + oran * (aci_max - aci_min))


class TaretRcKoprusu(Node):

    def __init__(self):
        super().__init__('taret_rc_koprusu')

        self.declare_parameter('port',     SERIAL_TARET)
        self.declare_parameter('baud',     SERIAL_BAUD)
        self.declare_parameter('sim_mode', False)

        self._sim_mode = self.get_parameter('sim_mode').value
        self._ser = None
        if not self._sim_mode:
            try:
                self._ser = serial.Serial(
                    self.get_parameter('port').value,
                    self.get_parameter('baud').value,
                    timeout=0.1,
                )
                self.get_logger().info(f'Turret UNO portu açıldı: {self._ser.port}')
            except serial.SerialException as e:
                self.get_logger().error(f'Turret UNO portu açılamadı: {e}')
                self._sim_mode = True

        self._mod             = MOD_MANUAL
        self._pan_us          = (RC_PWM_MIN + RC_PWM_MAX) / 2
        self._tilt_us         = (RC_PWM_MIN + RC_PWM_MAX) / 2
        self._taret_aktif_us  = RC_PWM_MIN
        self._son_rc_zamani   = None
        self._son_pan, self._son_tilt = None, None

        qos_be = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(UInt8, MOD_AKTIF_TOPIC, self._mod_cb, 10)
        self.create_subscription(Float32MultiArray, RC_INPUT_TOPIC, self._rc_cb, qos_be)

        self.create_timer(0.05, self._komut_gonder)   # 20 Hz
        self.get_logger().info('TaretRcKoprusu hazır — sağ stick MANUEL+SWB ile taret nişanı.')

    def _mod_cb(self, msg: UInt8):
        self._mod = int(msg.data)

    def _rc_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            return
        self._pan_us          = float(msg.data[1])   # ch_direksiyon → taret aktifken pan
        self._tilt_us         = float(msg.data[4])
        self._taret_aktif_us  = float(msg.data[5])
        self._son_rc_zamani   = self.get_clock().now()

    def _rc_guncel_mi(self) -> bool:
        if self._son_rc_zamani is None:
            return False
        gecen_s = (self.get_clock().now() - self._son_rc_zamani).nanoseconds / 1e9
        return gecen_s < RC_STALE_S

    def _taret_aktif(self) -> bool:
        return (self._rc_guncel_mi() and self._mod == MOD_MANUAL
                and self._taret_aktif_us > RC_MOD_ESIK)

    def _komut_gonder(self):
        if self._taret_aktif():
            pan  = _us_to_aci(self._pan_us,  PAN_ACI_MIN,  PAN_ACI_MAX)
            tilt = _us_to_aci(self._tilt_us, TILT_ACI_MIN, TILT_ACI_MAX)
        else:
            pan, tilt = MERKEZ_ACI, MERKEZ_ACI

        # Değişmeyen açıyı tekrar tekrar yazmak servo/I2C hattını gereksiz
        # meşgul eder — sadece değişimde gönder.
        if (pan, tilt) == (self._son_pan, self._son_tilt):
            return
        self._son_pan, self._son_tilt = pan, tilt

        cmd = f'P:{pan},T:{tilt}\n'
        if self._sim_mode or self._ser is None:
            self.get_logger().debug(f'[SIM] {cmd.strip()}')
            return
        try:
            self._ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(f'P:{MERKEZ_ACI},T:{MERKEZ_ACI}\n'.encode())
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaretRcKoprusu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
