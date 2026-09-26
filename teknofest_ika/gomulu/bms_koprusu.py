#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
bms_koprusu.py  —  JK BMS → /battery/status

Batarya gerilimi sürüş kartının seri protokolünde YOK ve olmayacak: araçta iki
BMS var, ikisi de gerilimi hücre bazında kendi ölçüyor ve doğru okuma yolu
BLE. Elektrik tarafındaki servis BLE'yi okuyup düz bir JSON olarak HTTP'den
yayınlıyor; bu düğüm o ucu ROS'a taşır.

TAZELİK
  Ölçüt `bagli` değil `yas`. BLE koptuğunda ölçüm alanları silinmiyor, son
  değerde DONUYOR — bağlantı bayrağına bakan bir tüketici donmuş bir gerilimi
  canlı sanır. Okuma bayatsa yayın yapılmaz; tüketiciye bayat sayı vermektense
  susmak doğru olanı yapıyor, çünkü /battery/status'u okuyan imu_guvenlik
  gelen son yüzdeyi kalıcı olarak tutuyor.

SAĞLIK ÖLÇÜTÜ
  Kesme kararı hücre dibinden (`bms_enaz`) okunur, BMS'in SOC tahmininden
  değil: LiFePO4'ün deşarj eğrisi düz olduğu için SOC yük altında zıplıyor.
  `percentage` alanına yine BMS'in kendi SOC'u yazılır — gösterge için o
  daha okunur — ama sağlık alanı ona bakmaz.

BİLİNEN SINIR
  BLE aynı anda tek istemci kabul ediyor. Sahada biri telefondan BMS
  uygulamasına bağlanırsa servis düşer ve bu düğüm bayat okuma görüp susar.
  Panoda "batarya yok" görünmesinin ilk şüphelisi budur.

  Uç yalnız JK'yı (ana traksiyon paketi) veriyor. Jetson'ı besleyen DALY
  paketi BLE taramasında görünmüyor ve şu an hiçbir kanaldan izlenmiyor.
"""

import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from teknofest_ika.otonomi.topics import (
    BATTERY_TOPIC, BMS_HTTP_URL, BMS_YAS_ESIK_S,
    BMS_HUCRE_DIP_MV, BMS_HUCRE_UYARI_MV, BMS_HUCRE_SAYISI,
)
from teknofest_ika.otonomi.pure_logic import bms_okuma_gecerli, bms_dip_olu


class BmsKoprusu(Node):

    def __init__(self):
        super().__init__('bms_koprusu')

        self.declare_parameter('url',        BMS_HTTP_URL)
        self.declare_parameter('periyot',    1.0)    # [s]
        self.declare_parameter('zaman_asimi', 2.0)   # [s] HTTP
        self.declare_parameter('yas_esigi',  BMS_YAS_ESIK_S)

        self._url      = str(self.get_parameter('url').value)
        self._zaman_a  = float(self.get_parameter('zaman_asimi').value)
        self._yas_esik = float(self.get_parameter('yas_esigi').value)
        periyot        = float(self.get_parameter('periyot').value)

        self._pub = self.create_publisher(BatteryState, BATTERY_TOPIC, 10)

        # Durum geçişlerini bir kez basmak için: uç sürekli erişilemezse
        # saniyede bir aynı hatayı yığmanın teşhis değeri yok.
        self._sessiz   = None    # None = henüz bilinmiyor
        self._son_dip  = None

        self.create_timer(periyot, self._oku)
        self.get_logger().info(
            f'BmsKoprusu hazır | {self._url} | {periyot:.1f} s | '
            f'bayatlık eşiği {self._yas_esik:.0f} s')

    # ── Uçtan okuma ────────────────────────────────────────────────────────
    def _oku(self):
        try:
            with urllib.request.urlopen(self._url, timeout=self._zaman_a) as y:
                veri = json.loads(y.read().decode('utf-8'))
        except (urllib.error.URLError, OSError, ValueError) as e:
            self._sustur(f'BMS ucu okunamadı ({self._url}): {e}')
            return

        gecerli, sebep = bms_okuma_gecerli(veri, self._yas_esik)
        if not gecerli:
            self._sustur(
                f'BMS okuması kullanılmadı ({sebep}) — yaş={veri.get("yas")} '
                f'bağlı={veri.get("bagli")} hata={veri.get("hata")}')
            return

        self._yayinla(veri)

    def _sustur(self, sebep: str):
        if self._sessiz is not True:
            self._sessiz = True
            self.get_logger().warn(f'{sebep} — /battery/status susturuldu.')

    # ── Yayın ──────────────────────────────────────────────────────────────
    def _yayinla(self, veri: dict):
        if self._sessiz is not False:
            self._sessiz = False
            self.get_logger().info('BMS okuması taze — /battery/status akıyor.')

        m = BatteryState()
        m.header.stamp    = self.get_clock().now().to_msg()
        m.header.frame_id = 'bms'
        m.present         = True

        m.voltage = float(veri['v48'])
        # ROS sözleşmesi: deşarjda akım NEGATİF. Uç zaten işaretli veriyor,
        # işareti çevirmiyoruz — yönü BMS belirliyor.
        if veri.get('bms_i') is not None:
            m.current = float(veri['bms_i'])
        if veri.get('bms_kalan') is not None:
            m.charge = float(veri['bms_kalan'])
        if veri.get('bms_tam') is not None:
            m.capacity = m.design_capacity = float(veri['bms_tam'])
        if veri.get('bms_soc') is not None:
            # Gösterge için BMS'in kendi tahmini; sağlık kararı buna BAKMAZ.
            m.percentage = max(0.0, min(1.0, float(veri['bms_soc']) / 100.0))

        hucreler = veri.get('hucre_mv') or []
        if isinstance(hucreler, list) and hucreler:
            m.cell_voltage = [float(h) / 1000.0 for h in hucreler]
            if len(hucreler) != BMS_HUCRE_SAYISI:
                self.get_logger().warn(
                    f'BMS {len(hucreler)} hücre bildiriyor, beklenen '
                    f'{BMS_HUCRE_SAYISI} — paket ya da BMS değişmiş olabilir.',
                    throttle_duration_sec=60.0)

        # En sıcak sonda raporlanır: tek alan var ve tehlikeyi en yüksek
        # ölçüm anlatıyor.
        sicakliklar = [veri[a] for a in ('bms_tmos', 'bms_t1', 'bms_t2')
                       if veri.get(a) is not None]
        if sicakliklar:
            m.temperature = float(max(sicakliklar))

        m.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        m.power_supply_health = self._saglik(float(veri['bms_enaz']))

        self._pub.publish(m)

    # ── Sağlık — hücre dibinden ────────────────────────────────────────────
    def _saglik(self, dip_mv: float) -> int:
        """
        Karar en düşük hücreden verilir. Paket gerilimi ya da SOC ortalamadır
        ve tek bir çökmüş hücreyi gizler; kesmesi gereken şey o hücredir.
        """
        olu = bms_dip_olu(dip_mv, BMS_HUCRE_DIP_MV)
        durum = (BatteryState.POWER_SUPPLY_HEALTH_DEAD if olu
                 else BatteryState.POWER_SUPPLY_HEALTH_GOOD)

        if self._son_dip is None or abs(dip_mv - self._son_dip) >= 10.0:
            self._son_dip = dip_mv
            if olu:
                self.get_logger().error(
                    f'BATARYA DİBİ: en düşük hücre {dip_mv:.0f} mV — '
                    f'{BMS_HUCRE_DIP_MV} mV sınırının altında.')
            elif dip_mv <= BMS_HUCRE_UYARI_MV:
                self.get_logger().warn(
                    f'Batarya düşüyor: en düşük hücre {dip_mv:.0f} mV.',
                    throttle_duration_sec=30.0)
        return durum


def main(args=None):
    rclpy.init(args=args)
    node = BmsKoprusu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
