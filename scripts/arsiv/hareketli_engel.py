#!/usr/bin/env python3
"""
hareketli_engel.py — Kayar Engel İstasyonu (bağımsız test aracı)
=================================================================
Yarışma yığınından (Nav2 + slam_toolbox + misyon_fsm) bağımsız çalışır.
Harita, odometri, TF kullanmaz; yalnız /scan okur. Direksiyon hiç
kullanılmaz — istasyon düz bir koridor olduğu için her komutta dönüş 0
yazılır, böylece test edilen tek şey engel algısı olur.

Akış:
  1. Düz ilerle, ön koniyi izle.
  2. Koni içinde durma_mesafe_m'den yakın engel görülünce DUR: sıfır hız +
     tam fren. Yalnız gaz kesmek yetmiyor, araç ataletle kayıyor.
  3. Engel açıdan çıkıp ön temizlenince devam et.
  4. İlk engel geçildikten sonra ilerleme_m kadar yol alınca dur ve kilitlen.
  5. Kumanda MANUEL'e alınıp otonoma geri alınınca istasyon baştan başlar;
     kilidi çözmek için süreci yeniden başlatmak gerekmiyor.

Engel görülmeden önce yol sınırı işlemez; araç istasyona yaklaşırken
süresiz ilerler. Sayaç yalnız ilk engel temizlendikten sonra işlemeye
başlar ve duruşlarda durur, çünkü ölçülen şey süre değil alınan yoldur.

Gürültü bağışıklığı:
  Ön konideki ham minimum tek bir hatalı yansımayla eşiğin altına
  inebiliyor. Bu yüzden karar tek ışına değil, eşiğin altındaki ışın
  SAYISINA bakıyor: gerçek bir engel 2 m'de onlarca ışın kaplar, gürültü
  bir ikisini. min_isin bu ayrımı kuruyor.

Histerezis:
  Durma ve devam eşikleri ayrı (durma_mesafe_m < temiz_mesafe_m) ve
  temizlik temiz_sure_s boyunca sürmeden devam edilmiyor. Tek eşikle
  engel sınırda salınırken araç dur-kalk yapıyor.

Güvenlik:
  - Kumanda MANUEL'deyken hiçbir komut yazılmaz (mod_yoneticisi /mod/aktif).
  - /e_stop geldiğinde anında sıfır hız.
  - Tarama bayatlarsa (LiDAR susarsa) tam duruş.

Çalıştırma (Jetson):
    python3 ~/lydia_ws/src/teknofest_ika_yazilim/scripts/hareketli_engel.py

Parametre vererek:
    python3 scripts/hareketli_engel.py --ros-args -p durma_mesafe_m:=2.0 \
        -p ilerleme_m:=5.0

Yalnızca gözlem (tekerlek dönmez, /cmd_vel yazılmaz):
    python3 scripts/hareketli_engel.py --ros-args -p kuru_calisma:=true
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
DURUM_TOPIC  = '/engel/durum'
FREN_TOPIC   = '/fren_komut'

MOD_OTONOM = 2       # mod_yoneticisi: 0=MANUEL 1=YARI 2=TAM OTONOM


class HareketliEngel(Node):
    def __init__(self):
        super().__init__('hareketli_engel')

        # hiz: kalkış sürtünmesi eşiğinin üstünde kalmalı. Gaz voltajı
        # 0.90 + hız; kontrolcü duran aracı ~1.35 V altında kopartamıyor,
        # yani 0.45'in altına inilirse araç komut alır ama yerinden oynamaz.
        self.declare_parameter('hiz',              0.45)   # [m/s]
        self.declare_parameter('durma_mesafe_m',   2.00)   # bu mesafede dur
        self.declare_parameter('temiz_mesafe_m',   2.50)   # bu mesafeden sonra devam
        self.declare_parameter('temiz_sure_s',     0.40)   # temizlik bu kadar sürsün
        self.declare_parameter('koni_deg',        25.0)    # engel arama konisi (±)
        self.declare_parameter('min_isin',           3)    # eşik altı asgari ışın
        self.declare_parameter('ilerleme_m',       2.00)   # engel sonrası gidilecek yol
        self.declare_parameter('fren_binde',      1000)    # duruşta fren gücü (0-1000)

        # LiDAR montajı — huni_reaktif ile aynı takım kullanılmalı, tek
        # başına değiştirilirse ön koni yanlış yere bakar.
        self.declare_parameter('aci_ayna',        True)
        self.declare_parameter('aci_offset_deg', -93.3)
        self.declare_parameter('govde_yaricapi_m', 0.25)   # araç parçalarının yansımaları
        self.declare_parameter('lidar_max_m',      5.00)

        # Kumanda MANUEL'de bu kadar kalınca istasyon baştan alınır. Mod
        # anlık sekebildiği için doğrudan sıfırlanmıyor; kısa bir sekme
        # çıkış ortasında ilerlemeyi silmesin.
        self.declare_parameter('sifirla_sure_s',   1.00)
        self.declare_parameter('mod_takip',       True)    # MANUEL'de komut yazma
        self.declare_parameter('kuru_calisma',   False)    # /cmd_vel yazma, sadece raporla
        self.declare_parameter('tarama_timeout_s', 0.60)

        g = lambda ad: self.get_parameter(ad).value
        self.hiz          = float(g('hiz'))
        self.durma_m      = float(g('durma_mesafe_m'))
        self.temiz_m      = max(float(g('temiz_mesafe_m')), self.durma_m)
        self.temiz_sure   = float(g('temiz_sure_s'))
        self.koni         = math.radians(float(g('koni_deg')))
        self.min_isin     = int(g('min_isin'))
        self.ilerleme     = float(g('ilerleme_m'))
        self.fren_binde   = int(g('fren_binde'))
        self.aci_ayna     = bool(g('aci_ayna'))
        self.aci_off      = math.radians(float(g('aci_offset_deg')))
        self.govde_r      = float(g('govde_yaricapi_m'))
        self.lidar_max    = float(g('lidar_max_m'))
        self.sifirla_sure = float(g('sifirla_sure_s'))
        self.mod_takip    = bool(g('mod_takip'))
        self.kuru         = bool(g('kuru_calisma'))
        self.timeout      = float(g('tarama_timeout_s'))

        self._estop     = False
        self._mod       = 0
        self._son_scan  = 0.0
        self._engel_gordu   = False   # istasyonda hiç engel görüldü mü
        self._temiz_bas     = 0.0     # önün temizlendiği an
        self._yol           = 0.0     # engel sonrası alınan yol [m]
        self._yol_ts        = 0.0     # yol birikiminde son örnek anı
        self._bitti         = False
        self._manuel_bas    = 0.0     # MANUEL'e düşülen an

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
            f'hareketli_engel hazır | hız {self.hiz:.2f} m/s | '
            f'dur {self.durma_m:.2f} m / devam {self.temiz_m:.2f} m '
            f'({self.temiz_sure:.1f} s) | koni ±{math.degrees(self.koni):.0f}° | '
            f'asgari {self.min_isin} ışın | çıkış {self.ilerleme:.1f} m | '
            f'açı offset {math.degrees(self.aci_off):.0f}° | '
            f'{"KURU ÇALIŞMA" if self.kuru else "SÜRÜŞ AKTİF"}')

    def _on_estop(self, msg: Bool):
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
            simdi = time.time()
            if self._manuel_bas == 0.0:
                self._manuel_bas = simdi
            elif simdi - self._manuel_bas >= self.sifirla_sure:
                self._sifirla()
            self._dur('MANUEL')
            return
        self._manuel_bas = 0.0

        if self._bitti:
            self._dur('İSTASYON BİTTİ')
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

        koni = np.abs(acilar) <= self.koni
        if not koni.any():
            self._dur('TARAMA_BOŞ')
            return
        koni_mesafe = mesafe[koni]

        # Hem durma hem devam kararı ışın SAYISINA bakıyor. Devam tarafını ham
        # minimuma bağlamak, çıkış boyunca tek bir sapan ışının aracı tekrar
        # tekrar durdurmasına yol açıyor.
        en_yakin = float(koni_mesafe.min())
        yakin_isin = int((koni_mesafe < self.durma_m).sum())
        temiz_isin = int((koni_mesafe < self.temiz_m).sum())
        engel_var = yakin_isin >= self.min_isin
        yol_temiz = temiz_isin < self.min_isin

        if engel_var:
            self._engel_gordu = True
            self._temiz_bas   = 0.0
            self._yol_ts      = 0.0    # duruşta yol birikmez
            self._dur(f'ENGEL {en_yakin:.2f} m ({yakin_isin} ışın)')
            return

        # Engel eşiğin altında değil; devam için önün temiz_mesafe_m ötesinde
        # ve bunun temiz_sure_s sürmüş olması gerekiyor. Sınırda salınan bir
        # engelde araç dur-kalk yapmasın diye.
        if self._engel_gordu:
            if not yol_temiz:
                self._temiz_bas = 0.0
                self._yol_ts    = 0.0
                self._dur(f'ENGEL ÇEKİLİYOR {en_yakin:.2f} m '
                          f'({temiz_isin} ışın)')
                return
            simdi = time.time()
            if self._temiz_bas == 0.0:
                self._temiz_bas = simdi
            if simdi - self._temiz_bas < self.temiz_sure:
                self._yol_ts = 0.0
                self._dur(f'TEMİZ BEKLE {simdi - self._temiz_bas:.1f} s')
                return

            # Yol, odometri olmadığı için sürülen süreden çıkarılıyor.
            if self._yol_ts == 0.0:
                self._yol_ts = simdi
            self._yol += self.hiz * (simdi - self._yol_ts)
            self._yol_ts = simdi

            if self._yol >= self.ilerleme:
                self._bitti = True
                self._dur('İSTASYON BİTTİ')
                return

            self._sur(self.hiz, 0.0)
            self._rapor(f'ÇIKIŞ | ön {en_yakin:.2f} m | '
                        f'{self._yol:.1f}/{self.ilerleme:.1f} m')
            return

        # Henüz engel görülmedi — istasyona yaklaşılıyor, süresiz ilerle.
        self._sur(self.hiz, 0.0)
        self._rapor(f'YAKLAŞ | ön {en_yakin:.2f} m')

    def _sifirla(self):
        """İstasyonu baştan alır.

        Kumanda otonoma her alındığında yeniden denenebilsin diye durum
        MANUEL'de temizleniyor; bitiş kilidi için süreci yeniden başlatmak
        gerekmiyor.
        """
        if not (self._bitti or self._engel_gordu or self._yol > 0.0):
            return
        self._bitti       = False
        self._engel_gordu = False
        self._temiz_bas   = 0.0
        self._yol         = 0.0
        self._yol_ts      = 0.0
        self._rapor('İSTASYON SIFIRLANDI — otonoma alınca baştan')

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
    dugum = HareketliEngel()
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
