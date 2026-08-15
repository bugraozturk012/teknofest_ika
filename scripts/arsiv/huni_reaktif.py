#!/usr/bin/env python3
"""
huni_reaktif.py — Haritasız Huni Koridoru Geçişi (bağımsız test aracı)
=======================================================================
Yarışma yığınından (Nav2 + slam_toolbox + misyon_fsm + waypoints.yaml)
tamamen bağımsız çalışır. Harita, odometri, TF ve lokalizasyon kullanmaz;
her lazer taramasında sıfırdan karar verir. Aracın nerede olduğunu bilmez,
yalnızca "şu anda nereye gidebilirim" sorusunu yanıtlar.

Yöntem — boşluk takibi + koridor dengesi:
  1. Öne bakan tarama dilimi alınır, geçersiz ışınlar elenir.
  2. En yakın engelin çevresine güvenlik balonu basılır: aracın yarı
     genişliği kadar açı geçilmez işaretlenir, huninin dibinden sıyırmaya
     çalışmaz.
  3. Kalan ışınlarda en uzun serbest dizi = geçilecek boşluk.
  4. Boşluğun ortası hedef yön olur.
  5. Koridorda kenara yanaşmayı önlemek için sol/sağ ortalama mesafe farkı
     küçük bir düzeltme terimi olarak eklenir. Düzensiz aralıklı huni
     dizilerinde saf ortalama şaşırdığı için ana karar boşluk takibindedir,
     denge yalnız ince ayar yapar.

Güvenlik:
  - Kumanda MANUEL'deyken hiçbir komut yazılmaz (mod_yoneticisi /mod/aktif).
  - /e_stop geldiğinde anında sıfır hız.
  - Önde durma mesafesinden yakın engel varsa tam duruş.
  - Tarama bayatlarsa (LiDAR susarsa) tam duruş.

Çalıştırma (Jetson):
    python3 ~/lydia_ws/src/teknofest_ika_yazilim/scripts/huni_reaktif.py

Parametre vererek:
    python3 scripts/huni_reaktif.py --ros-args -p max_hiz:=0.3 -p on_durma_m:=0.6

Yalnızca gözlem (tekerlek dönmez, /cmd_vel yazılmaz):
    python3 scripts/huni_reaktif.py --ros-args -p kuru_calisma:=true
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String, UInt8, UInt16

SCAN_TOPIC   = '/scan'
CMD_VEL      = '/cmd_vel'
E_STOP       = '/e_stop'
MOD_AKTIF    = '/mod/aktif'
DURUM_TOPIC  = '/huni/durum'
FREN_TOPIC   = '/fren_komut'

MOD_OTONOM = 2       # mod_yoneticisi: 0=MANUEL 1=YARI 2=TAM OTONOM


class HuniReaktif(Node):

    def __init__(self):
        super().__init__('huni_reaktif')

        # ── Hız ───────────────────────────────────────────────────────────
        self.declare_parameter('max_hiz',        0.40)   # açık koridorda [m/s]
        self.declare_parameter('min_hiz',        0.15)   # dar geçişte [m/s]
        self.declare_parameter('max_donus',      0.80)   # direksiyon tavanı [rad/s]

        # ── Geometri ──────────────────────────────────────────────────────
        # Tekerlek dışları arası ölçüldü: 1.18 m. Güvenlik balonunun yarıçapı
        # bunun yarısı + paydır; büyütmek huniden uzak tutar ama dar koridoru
        # kapalı gösterip aracı durdurabilir.
        self.declare_parameter('arac_genislik_m',  1.18)
        self.declare_parameter('yanal_pay_m',      0.08)
        self.declare_parameter('on_aci_deg',     140.0)  # dikkate alınan ön dilim
        # LiDAR araca 90° dönük monteli: ham taramada aracın önü 0° değil.
        # Sahada ölçüldü (scripts/lidar_aci_kalibre.py): tam öne konan
        # referans cisim ham taramada -93.3° görünüyor.
        # Işın açıları burada araç eksenine çevrilir (dashboard paneli de
        # aynı dönüşümü kullanır).
        self.declare_parameter('aci_ayna',       True)
        self.declare_parameter('aci_offset_deg', -93.3)
        self.declare_parameter('lidar_max_m',    5.00)   # bundan uzağı serbest say
        # LiDAR araca monteli; gövde kenarı, direk ve kablolar taramaya girip
        # sabit "çok yakın engel" üretiyor (sahada ölçüldü: solda 4 cm'de 45
        # nokta). Bu yarıçapın içi araç kabul edilip yok sayılır — araç 1.18 m
        # genişlikte olduğu için bu mesafede gerçek bir engel zaten çarpışmadır.
        self.declare_parameter('govde_yaricapi_m', 0.25)
        self.declare_parameter('balon_esik_m',   2.50)   # bu mesafeye kadar engellere balon
        # Gaz kesmek yetmiyor: araç ataletle kayıyor. Durma mesafesi fren
        # mesafesini kapsayacak kadar büyük olmalı (sahada 0.60 m ile engele
        # çarpma noktasına gelindi).
        self.declare_parameter('on_durma_m',     1.50)   # önde bu mesafede dur
        self.declare_parameter('fren_binde',     1000)   # duruşta fren gücü (0-1000)

        # ── Kontrol kazançları ────────────────────────────────────────────
        self.declare_parameter('yaw_kazanci',    1.20)   # hedef açı → dönüş
        self.declare_parameter('denge_kazanci',  0.35)   # sol/sağ farkı → dönüş
        # Sahada ölçüldü (2026-07-22): -1.0 ile araç boşluğun tersine, engele
        # doğru dönüyordu. Firmware ile ROS açı işareti aslında uyumlu.
        self.declare_parameter('yaw_yonu',       1.0)

        # ── Güvenlik / mod ────────────────────────────────────────────────
        self.declare_parameter('mod_takip',      True)   # MANUEL'de komut yazma
        self.declare_parameter('kuru_calisma',   False)  # /cmd_vel yazma, sadece raporla
        self.declare_parameter('tarama_timeout_s', 0.60)

        g = lambda ad: self.get_parameter(ad).value
        self.max_hiz    = float(g('max_hiz'))
        self.min_hiz    = float(g('min_hiz'))
        self.max_donus  = float(g('max_donus'))
        self.arac_gen   = float(g('arac_genislik_m'))
        self.guvenlik_r = self.arac_gen / 2.0 + float(g('yanal_pay_m'))
        self.on_aci     = math.radians(float(g('on_aci_deg')))
        self.aci_ayna   = bool(g('aci_ayna'))
        self.aci_off    = math.radians(float(g('aci_offset_deg')))
        self.lidar_max  = float(g('lidar_max_m'))
        self.govde_r    = float(g('govde_yaricapi_m'))
        self.balon_esik = float(g('balon_esik_m'))
        self.on_durma   = float(g('on_durma_m'))
        self.fren_binde = int(g('fren_binde'))
        self.k_yaw      = float(g('yaw_kazanci'))
        self.k_denge    = float(g('denge_kazanci'))
        self.yaw_yonu   = float(g('yaw_yonu'))
        self.mod_takip  = bool(g('mod_takip'))
        self.kuru       = bool(g('kuru_calisma'))
        self.timeout    = float(g('tarama_timeout_s'))

        self._estop     = False
        self._mod       = 0
        self._son_scan  = 0.0

        self._pub  = self.create_publisher(Twist, CMD_VEL, 10)
        self._drm  = self.create_publisher(String, DURUM_TOPIC, 10)
        self._fren = self.create_publisher(UInt16, FREN_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, E_STOP, self._on_estop, 10)
        self.create_subscription(UInt8, MOD_AKTIF, self._on_mod, 10)

        # Tarama kesilirse komut göndermeyi bırakmak için ayrı bekçi
        self.create_timer(0.1, self._bekci)

        self.get_logger().info(
            f'huni_reaktif hazır | araç {self.arac_gen:.2f} m | '
            f'hız {self.min_hiz:.2f}-{self.max_hiz:.2f} m/s | '
            f'balon {self.guvenlik_r:.2f} m | gövde {self.govde_r:.2f} m | '
            f'ön dilim {math.degrees(self.on_aci):.0f}° | '
            f'açı offset {math.degrees(self.aci_off):+.0f}° | '
            f'{"KURU ÇALIŞMA (tekerlek dönmez)" if self.kuru else "SÜRÜŞ AKTİF"}')

    # ── Abonelikler ───────────────────────────────────────────────────────
    def _on_estop(self, msg: Bool):
        if msg.data and not self._estop:
            self.get_logger().error('E-STOP — duruluyor.')
        self._estop = bool(msg.data)

    def _on_mod(self, msg: UInt8):
        self._mod = int(msg.data)

    # ── Ana döngü ─────────────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self._son_scan = time.time()

        if self._estop:
            self._dur('E_STOP')
            return

        if self.mod_takip and self._mod != MOD_OTONOM:
            self._dur('MANUEL')
            return

        ham = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        # Montaj düzeltmesi: ham tarama açısı → araç ekseni (0° = aracın önü)
        acilar = (-ham if self.aci_ayna else ham) + self.aci_off
        acilar = np.arctan2(np.sin(acilar), np.cos(acilar))   # [-pi, pi]
        mesafe = np.asarray(msg.ranges, dtype=np.float32)

        # Geçersiz ışınlar: NaN/inf, menzil dışı, sıfır. Uzak olanlar serbest
        # sayılır (lidar_max'a kırpılır), yakın gürültü elenir.
        gecersiz = (~np.isfinite(mesafe) | (mesafe <= msg.range_min)
                    | (mesafe <= self.govde_r))   # gövde yansımaları
        mesafe = np.where(gecersiz, self.lidar_max, mesafe)
        mesafe = np.clip(mesafe, 0.0, self.lidar_max)

        # Yalnız öne bakan dilim
        dilim = np.abs(acilar) <= (self.on_aci / 2.0)
        if not dilim.any():
            self._dur('TARAMA_BOS')
            return
        acilar, mesafe = acilar[dilim], mesafe[dilim]

        on_mesafe = self._on_mesafe(acilar, mesafe)
        if on_mesafe < self.on_durma:
            self._dur(f'ENGEL {on_mesafe:.2f}m')
            return

        serbest = self._balon_uygula(acilar, mesafe)
        bas, son = self._en_genis_bosluk(serbest)
        if bas is None:
            self._dur('BOSLUK_YOK')
            return

        hedef_aci = float(acilar[(bas + son) // 2])
        denge     = self._koridor_dengesi(acilar, mesafe)
        bosluk_genislik = self._bosluk_metre(acilar, mesafe, bas, son)

        donus = self.yaw_yonu * (self.k_yaw * hedef_aci + self.k_denge * denge)
        donus = float(np.clip(donus, -self.max_donus, self.max_donus))

        # Boşluk daraldıkça ve önde engel yaklaştıkça yavaşla. Balon zaten
        # araç genişliğini düştüğü için buradaki değer merkez hattının
        # oynayabildiği paydır; araç genişliğiyle kıyaslamak çifte sayım olur.
        dar_oran  = np.clip((bosluk_genislik - 0.40) / 1.20, 0.0, 1.0)
        yakin_oran = np.clip((on_mesafe - self.on_durma) / 1.5, 0.0, 1.0)
        hiz = self.min_hiz + (self.max_hiz - self.min_hiz) * min(dar_oran, yakin_oran)

        self._sur(hiz, donus)
        self._rapor(f'hedef {math.degrees(hedef_aci):+5.1f}° | boşluk {bosluk_genislik:.2f} m | '
                    f'ön {on_mesafe:.2f} m | v {hiz:.2f} | w {donus:+.2f}')

    # ── Yardımcılar ───────────────────────────────────────────────────────
    def _on_mesafe(self, acilar, mesafe):
        """Tam öndeki dar koninin en yakın ölçümü — acil duruş için."""
        on = np.abs(acilar) <= math.radians(20.0)
        return float(mesafe[on].min()) if on.any() else self.lidar_max

    def _balon_uygula(self, acilar, mesafe):
        """Yakın engellerin her birinin çevresini geçilmez işaretler.

        Araç bir nokta değil; huninin tam kenarını hedef seçerse yanıyla
        sürtünür. Her engel, kendi mesafesinde guvenlik_r'yi gören açı kadar
        ışını kapatır — uzaktaki dar, yakındaki geniş bölge kapatır.

        Koridorda yalnız en yakın engele balon basmak yetmez: sol ve sağ huni
        neredeyse eşit uzaklıktayken sadece biri kapatılır, karşı taraf
        serbest görünür ve araç ona yanaşır. Bu yüzden balon_esik_m içindeki
        tüm engeller birlikte işlenir.
        """
        yakin = mesafe < self.balon_esik
        if not yakin.any():
            return mesafe.copy()
        if float(mesafe.min()) <= 0.05:
            return np.zeros_like(mesafe)

        # Her engelin kapattığı açısal yarıçap; ışın i, engel j'nin balonuna
        # düşüyorsa kapanır.
        yari = np.arctan2(self.guvenlik_r, np.maximum(mesafe, 0.05))
        fark = np.abs(acilar[:, None] - acilar[None, :])
        kapali = (fark <= yari[None, :]) & yakin[None, :]
        return np.where(kapali.any(axis=1), 0.0, mesafe)

    def _en_genis_bosluk(self, serbest):
        """Sıfır olmayan en uzun ardışık diziyi bulur."""
        acik = serbest > 0.0
        if not acik.any():
            return None, None
        en_iyi = (0, -1, -1)     # (uzunluk, baş, son)
        bas = None
        for i, a in enumerate(acik):
            if a and bas is None:
                bas = i
            elif not a and bas is not None:
                if i - bas > en_iyi[0]:
                    en_iyi = (i - bas, bas, i - 1)
                bas = None
        if bas is not None and len(acik) - bas > en_iyi[0]:
            en_iyi = (len(acik) - bas, bas, len(acik) - 1)
        return en_iyi[1], en_iyi[2]

    def _bosluk_metre(self, acilar, mesafe, bas, son):
        """Boşluğun metre cinsinden yaklaşık genişliği (yay uzunluğu)."""
        d = float(np.median(mesafe[bas:son + 1])) if son >= bas else 0.0
        return abs(float(acilar[son] - acilar[bas])) * max(d, 0.1)

    def _koridor_dengesi(self, acilar, mesafe):
        """Sol ve sağ duvara olan ortalama mesafe farkı [-1, 1].

        Pozitif → solda daha çok yer var → sola doğru ince düzeltme.
        Yan dilimler (30°-80°) kullanılır; tam yan ölçümler koridor
        duvarını, ön ölçümler hedefi temsil ettiği için karıştırılmaz.
        """
        sol = (acilar > math.radians(30.0)) & (acilar < math.radians(80.0))
        sag = (acilar < -math.radians(30.0)) & (acilar > -math.radians(80.0))
        if not sol.any() or not sag.any():
            return 0.0
        ds, dg = float(np.median(mesafe[sol])), float(np.median(mesafe[sag]))
        toplam = ds + dg
        return 0.0 if toplam < 0.1 else float(np.clip((ds - dg) / toplam, -1.0, 1.0))

    # ── Çıkış ─────────────────────────────────────────────────────────────
    def _sur(self, hiz, donus):
        if self.kuru:
            return
        self._fren.publish(UInt16(data=0))      # sürerken fren serbest
        t = Twist()
        t.linear.x = float(hiz)
        t.angular.z = float(donus)
        self._pub.publish(t)

    def _dur(self, sebep):
        if not self.kuru:
            self._pub.publish(Twist())
            # Yalnız gaz kesmek yetmiyor — araç ataletle kayıyor, fren şart.
            self._fren.publish(UInt16(data=self.fren_binde))
        self._rapor(f'DUR — {sebep}')

    def _rapor(self, metin):
        self._drm.publish(String(data=metin))
        self.get_logger().info(metin, throttle_duration_sec=0.5)

    def _bekci(self):
        """LiDAR susarsa bayat veriyle sürmeyi engeller."""
        if self._son_scan and (time.time() - self._son_scan) > self.timeout:
            if not self.kuru:
                self._pub.publish(Twist())
            self.get_logger().warn('Tarama bayat — duruldu.',
                                   throttle_duration_sec=2.0)


def main():
    rclpy.init()
    dugum = HuniReaktif()
    try:
        rclpy.spin(dugum)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            dugum._pub.publish(Twist())     # çıkarken aracı durdur
            dugum._fren.publish(UInt16(data=1000))
        except Exception:
            pass
        dugum.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
