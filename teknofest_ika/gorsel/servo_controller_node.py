#!/usr/bin/env python3
"""
servo_controller_node.py — Hardware Turret Control (PCA9685 PWM)
Vector3.x = yaw_deg offset, .y = pitch_deg offset  (z yoksayılır)
Ateşleme: /shoot_command (Bool) → seri_kopru → Arduino → lazer
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16, Bool

try:
    from smbus2 import SMBus
    HAS_SMBUS = True
except ImportError:
    HAS_SMBUS = False

from teknofest_ika.otonomi.topics import (
    TURRET_CMD_TOPIC, TARET_PAN_TOPIC, TARET_TILT_TOPIC, SHOOT_CMD_TOPIC,
)


class PCA9685:
    """Minimal PCA9685 driver via smbus2."""
    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09
    RESTART = 0x80
    SLEEP = 0x10
    ALLCALL = 0x01

    def __init__(self, bus_num=1, address=0x40):
        if not HAS_SMBUS:
            raise RuntimeError("smbus2 not available")
        self.bus = SMBus(bus_num)
        self.address = address
        self._init()

    def _init(self):
        self.reset()
        # Set frequency to 50Hz (prescale = 121)
        prescale = int(round(25000000.0 / (4096.0 * 50.0)) - 1)
        old_mode = self.bus.read_byte_data(self.address, self.MODE1)
        new_mode = (old_mode & 0x7F) | self.SLEEP
        self.bus.write_byte_data(self.address, self.MODE1, new_mode)
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, old_mode | self.ALLCALL)

    def reset(self):
        self.bus.write_byte_data(self.address, self.MODE1, self.RESTART)
        time.sleep(0.01)

    def set_pwm(self, channel, on, off):
        reg = self.LED0_ON_L + 4 * channel
        data = [on & 0xFF, on >> 8, off & 0xFF, off >> 8]
        self.bus.write_i2c_block_data(self.address, reg, data)

    def set_servo_angle(self, channel, angle_deg, pwm_min=205, pwm_max=410):
        # Map 0-180 deg to PCA9685 counts (default 1ms-2ms: 205-410)
        # Override pwm_min/max for servos with different pulse ranges
        angle_deg = max(0.0, min(180.0, angle_deg))
        count = int(pwm_min + (angle_deg / 180.0) * (pwm_max - pwm_min))
        self.set_pwm(channel, 0, count)


class ServoControllerNode(Node):
    def __init__(self):
        super().__init__("servo_controller_node")

        self.declare_parameter("use_pca9685",     True)
        self.declare_parameter("pca9685_bus",      1)
        self.declare_parameter("pca9685_address",  0x40)
        self.declare_parameter("yaw_channel",      0)
        self.declare_parameter("pitch_channel",    1)
        self.declare_parameter("yaw_home_deg",     90.0)
        self.declare_parameter("pitch_home_deg",   90.0)
        self.declare_parameter("servo_pwm_min",    205)
        self.declare_parameter("servo_pwm_max",    410)

        use_pca        = self.get_parameter("use_pca9685").value
        self.yaw_ch    = self.get_parameter("yaw_channel").value
        self.pitch_ch  = self.get_parameter("pitch_channel").value
        self.yaw_home  = self.get_parameter("yaw_home_deg").value
        self.pitch_home = self.get_parameter("pitch_home_deg").value
        self.servo_pwm_min = self.get_parameter("servo_pwm_min").value
        self.servo_pwm_max = self.get_parameter("servo_pwm_max").value

        self._yaw   = self.yaw_home
        self._pitch = self.pitch_home
        self._pca   = None
        # Şartname §6.10: lazer aktifken araca/lazer yönlendiriciye (taret)
        # hareket verilemez. Bu bayrak, seri_kopru'daki kilidin yanında
        # ikinci (bağımsız) bir savunma katmanı olarak cb_cmd'yi bloklar —
        # tek hata noktasının (sadece Arduino/MCU tarafı) önüne geçer.
        self._lazer_aktif = False
        self.create_subscription(Bool, SHOOT_CMD_TOPIC, self._shoot_cb, 10)

        if use_pca and HAS_SMBUS:
            try:
                bus  = self.get_parameter("pca9685_bus").value
                addr = self.get_parameter("pca9685_address").value
                self._pca = PCA9685(bus, addr)
                self._pca.set_servo_angle(self.yaw_ch,   self.yaw_home)
                self._pca.set_servo_angle(self.pitch_ch, self.pitch_home)
                self.get_logger().info(f"PCA9685 initialized on bus {bus} addr {hex(addr)}")
            except Exception as e:
                self.get_logger().error(f"PCA9685 init failed: {e}")
                self._pca = None
        elif use_pca and not HAS_SMBUS:
            self.get_logger().warn("smbus2 not installed — dummy mode.")

        self.sub = self.create_subscription(
            Vector3, TURRET_CMD_TOPIC,
            self.cb_cmd, qos_profile_sensor_data)

        # seri_kopru'ya açı bildir → PKT_SERVO_PAN/TLT → Arduino
        self._pan_pub  = self.create_publisher(Int16, TARET_PAN_TOPIC,  10)
        self._tilt_pub = self.create_publisher(Int16, TARET_TILT_TOPIC, 10)

        self.get_logger().info("ServoControllerNode başlatıldı — ateşleme /shoot_command üzerinden yapılır.")

    def _shoot_cb(self, msg: Bool):
        self._lazer_aktif = msg.data

    def cb_cmd(self, msg: Vector3):
        if self._lazer_aktif:
            # Lazer aktif — taret hareketi tamamen yok sayılır (§6.10).
            self.get_logger().debug(
                "Lazer aktif — taret komutu yok sayıldı.", throttle_duration_sec=1.0)
            return

        target_yaw   = max(0.0, min(180.0, self.yaw_home   + float(msg.x)))
        target_pitch = max(0.0, min(180.0, self.pitch_home + float(msg.y)))

        self._yaw   = target_yaw
        self._pitch = target_pitch

        if self._pca is not None:
            try:
                self._pca.set_servo_angle(self.yaw_ch,   target_yaw,
                                          self.servo_pwm_min, self.servo_pwm_max)
                self._pca.set_servo_angle(self.pitch_ch, target_pitch,
                                          self.servo_pwm_min, self.servo_pwm_max)
            except Exception as e:
                self.get_logger().error(f"Servo write error: {e}")
        else:
            self.get_logger().debug(
                f"DUMMY servo YAW={target_yaw:.1f} PITCH={target_pitch:.1f}",
                throttle_duration_sec=2.0)

        # seri_kopru → PKT_SERVO_PAN/TLT → Arduino
        self._pan_pub.publish(Int16(data=int(target_yaw)))
        self._tilt_pub.publish(Int16(data=int(target_pitch)))

    def destroy_node(self):
        if self._pca is not None:
            try:
                self._pca.set_servo_angle(self.yaw_ch,   self.yaw_home)
                self._pca.set_servo_angle(self.pitch_ch, self.pitch_home)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
