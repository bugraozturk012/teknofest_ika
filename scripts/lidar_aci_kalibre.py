#!/usr/bin/env python3
"""
lidar_aci_kalibre.py — LiDAR Montaj Açısı Kalibrasyonu
=======================================================
LiDAR araca hangi yönde monteliyse, ham tarama açısı aracın önüyle
örtüşmez. Bu araç, aracın tam önüne konan bir referans cismi bulup
gereken açı düzeltmesini ÖLÇER — tahmin edilmez.

Kullanım:
  1. Aracın TAM ÖNÜNE, 0.8-1.5 m mesafeye bir huni/karton koy.
     Başka hiçbir cisim bu mesafede olmasın (yan ve arka en az 2 m boş).
  2. Çalıştır:
       ~/lydia_ortam/ortam.sh python3 scripts/lidar_aci_kalibre.py
  3. Çıkan iki değeri huni_reaktif'e parametre olarak ver.

Yöntem: 0.3-2.5 m bandındaki en yakın nokta kümesinin ağırlık merkezi
referans cismin ham açısıdır. Bu açının 0°'ye taşınması için gereken
işaret ve kayma hesaplanır; dashboard paneliyle aynı dönüşüm kullanılır
(önce isteğe bağlı aynalama, sonra sabit kayma).
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

ALT_MESAFE = 0.30    # bu mesafenin altı gövde yansıması sayılır [m]
UST_MESAFE = 2.50    # bu mesafenin üstü arka plan sayılır [m]
KARE_SAYISI = 25     # ortalama alınacak tarama sayısı


class Kalibre(Node):

    def __init__(self):
        super().__init__('lidar_aci_kalibre')
        self.acilar = []
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)

    def _on_scan(self, msg: LaserScan):
        if len(self.acilar) >= KARE_SAYISI:
            return
        a = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        r = np.asarray(msg.ranges, dtype=float)
        gecerli = np.isfinite(r) & (r > ALT_MESAFE) & (r < UST_MESAFE)
        if not gecerli.any():
            return

        # En yakın noktanın çevresindeki küme = referans cisim. Tek ışın
        # gürültüye açık; 15 cm bandındaki komşular birlikte değerlendirilir.
        en_yakin = r[gecerli].min()
        kume = gecerli & (r < en_yakin + 0.15)
        if kume.sum() < 2:
            return

        # Açısal ortalama birim vektörle alınır — ±180° sınırında sarma olmaz.
        self.acilar.append(math.atan2(np.sin(a[kume]).mean(),
                                      np.cos(a[kume]).mean()))


def main():
    rclpy.init()
    d = Kalibre()

    print('\n  Referans cisim aranıyor (0.30-2.50 m bandı)...')
    son = time.time() + 20
    while rclpy.ok() and time.time() < son and len(d.acilar) < KARE_SAYISI:
        rclpy.spin_once(d, timeout_sec=0.2)

    if len(d.acilar) < 5:
        print('  YETERSİZ VERİ — cismi 0.8-1.5 m mesafeye, tam öne koy.\n')
        d.destroy_node(); rclpy.shutdown(); return

    dizi = np.array(d.acilar)
    ham = math.atan2(np.sin(dizi).mean(), np.cos(dizi).mean())
    sapma = math.degrees(np.std(np.degrees(dizi)))

    print(f'  {len(d.acilar)} karede bulundu.')
    print(f'  Cismin ham açısı : {math.degrees(ham):+.1f}°  (sapma {sapma:.1f}°)')
    print()
    print('  ── ÖNERİLEN PARAMETRELER ──────────────────────────')
    print('  Cisim tam önde olduğuna göre bu açı 0°''a taşınmalı.')
    print()
    for ayna in (False, True):
        # g = (-ham if ayna else ham) + offset  ==>  0  olmalı
        off = -(-ham if ayna else ham)
        off_deg = math.degrees(math.atan2(math.sin(off), math.cos(off)))
        print(f'    -p aci_ayna:={str(ayna).lower():5s} -p aci_offset_deg:={off_deg:+.1f}')
    print()
    print('  Aynalama, cismi sağa aldığında hedefin sağa kayması')
    print('  gerektiği için seçilir: ikisini de deneyip cismi sağa')
    print('  koyduğunda "hedef" değeri negatif olanı kullan.')
    print()

    d.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
