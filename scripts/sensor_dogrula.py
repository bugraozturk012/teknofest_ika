#!/usr/bin/env python3
"""
sensor_dogrula.py — Enkoder ve IMU Doğrulama
=============================================
Enkoder ve IMU takıldıktan sonra "doğru çalışıyor mu" sorusunu göz kararı
yerine ölçümle yanıtlar. Hiçbir şey yayınlamaz, yalnız dinler — araç bu betik
yüzünden hareket etmez.

Ölçtüğü şeyler kod sabitlerinin karşılığıdır: mesafe testi
seri_kopru.TEKERLEK_YARICI'yı, yön testi IMU montaj yönünü, hız testi de gaz
voltajı ↔ hız eşlemesini doğrular.

Modlar (-p test:=<mod>):

  ozet    (varsayılan)  Topic'ler geliyor mu, hangi frekansta, anlık değerler.
                        Tek parça aksta beklenen imzayı da kontrol eder.

  mesafe  Aracı bilinen bir mesafe kadar it/sür, ölçülenle karşılaştır.
          Sapma varsa düzeltilmiş TEKERLEK_YARICI'yı hesaplar.
              -p gercek_m:=10.0        gidilecek gerçek mesafe
              -p mevcut_yaricap:=0.200 koddaki değer

  yon     Aracı bilinen bir açı kadar çevir, IMU'nun ne saydığına bak.
          İşaret hatası ve ölçek hatası burada görünür.
              -p gercek_deg:=90.0

  hiz     Aracı MANUEL sür; gaz voltajı ile ölçülen hızı eşleştirip
          v = a·gaz + b doğrusunu uydurur. Gaz voltajı telemetriden
          okunur (/battery/status.current × 10).

  ham     AS5600'ün ham ADC'sine bakar: hangi analog pin gerçekten bağlı,
          tur başına kaç tick düşüyor, okuma ne kadar gürültülü. Enkoder
          değişse de tekrarlanır. seri_kopru -p ham_enkoder:=true ister.
              -p tur_sayisi:=10   elle çevrilecek tekerlek turu

Çalıştırma:
    ~/lydia_ortam/ortam.sh python3 scripts/sensor_dogrula.py \
        --ros-args -p test:=mesafe -p gercek_m:=10.0

Her modda ENTER ölçümü başlatır, ikinci ENTER bitirir.
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import UInt16MultiArray

from teknofest_ika.otonomi.pure_logic import encoder_delta

ODOM_TOPIC    = '/odom'
IMU_TOPIC     = '/imu/data'
BATTERY_TOPIC = '/battery/status'
HAM_TOPIC     = '/enkoder/ham'

# Mega telemetriyi batarya alanına bindiriyor: gaz hattının voltajı current
# alanında, onda bir ölçekli taşınır.
GAZ_V_OLCEK = 10.0

# AS5600 10-bit ADC. Modül 5 V ile beslendiğinde analog çıkış ADC tavanına
# ulaşır ve tur başına bu kadar tick okunur; 3.3 V regülatörlü modüllerde
# tepe değer kırpılır ve sayı ~675'e düşer.
TICKS_PER_REV = 1024


def _yaw(q) -> float:
    """Quaternion → yaw [rad]."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sarmala(a: float) -> float:
    """Açıyı (-π, π] aralığına indirger."""
    return math.atan2(math.sin(a), math.cos(a))


def _tick_delta(yeni: int, eski: int) -> int:
    """İki ADC okuması arasındaki tick farkı, 0↔1023 sarmasına karşı korumalı."""
    return encoder_delta(yeni, eski, TICKS_PER_REV)


class FrekansSayaci:
    """Mesaj arası süreden çalışan frekans kestirimi."""

    def __init__(self, pencere: int = 30):
        self._t = []
        self._pencere = pencere

    def tik(self):
        self._t.append(time.time())
        if len(self._t) > self._pencere:
            self._t.pop(0)

    @property
    def hz(self) -> float:
        if len(self._t) < 2:
            return 0.0
        aralik = self._t[-1] - self._t[0]
        return (len(self._t) - 1) / aralik if aralik > 0 else 0.0

    @property
    def sayi(self) -> int:
        return len(self._t)


class SensorDogrula(Node):

    def __init__(self):
        super().__init__('sensor_dogrula')

        self.declare_parameter('test',            'ozet')
        self.declare_parameter('gercek_m',        10.0)
        self.declare_parameter('mevcut_yaricap',  0.200)
        self.declare_parameter('gercek_deg',      90.0)
        self.declare_parameter('tur_sayisi',      10)

        self.test           = str(self.get_parameter('test').value)
        self.gercek_m       = float(self.get_parameter('gercek_m').value)
        self.mevcut_yaricap = float(self.get_parameter('mevcut_yaricap').value)
        self.gercek_deg     = float(self.get_parameter('gercek_deg').value)
        self.tur_sayisi     = int(self.get_parameter('tur_sayisi').value)

        self._kilit = threading.Lock()

        # Anlık durum
        self.odom_hz  = FrekansSayaci()
        self.imu_hz   = FrekansSayaci()
        self.bat_hz   = FrekansSayaci()
        self.vx       = 0.0
        self.vyaw     = 0.0
        self.pose     = None          # (x, y)
        self.odom_yaw = 0.0
        self.imu_yaw  = None
        self.roll     = 0.0
        self.pitch    = 0.0
        self.gaz_v    = None

        # Ölçüm birikimi
        self.kayit    = False
        self.yol_m    = 0.0
        self.onceki   = None
        self.yaw_bas  = None
        self.yaw_top  = 0.0
        self.yaw_once = None
        self.ciftler  = []            # (gaz_v, vx)
        self.vyaw_max = 0.0
        self.oyaw_max = 0.0

        # Ham enkoder — [sol, sag] kanalları ayrı izlenir
        self.ham_hz    = FrekansSayaci()
        self.ham       = [None, None]     # anlık ADC
        self.ham_once  = [None, None]     # sarma hesabı için önceki okuma
        self.ham_net   = [0, 0]           # işaretli toplam (yön korunur)
        self.ham_mutlak= [0, 0]           # |delta| toplamı (titreşim dahil)
        self.ham_ornek = [[], []]         # gürültü penceresi
        # Ölçüm boyunca görülen ADC uçları. Tur başına tick'i kümülatif
        # deltadan hesaplamak besleme sorununu GİZLER: 0↔1023 sarma düzeltmesi
        # kırpılmış tavanı (ör. 674→0 sıçraması) eksik turmuş gibi görüp
        # farkı kapatır ve sonuç her hâlükârda ~1024 çıkar. Kırpma ancak ham
        # okumanın tepe değerine bakılarak görülür.
        self.ham_min   = [None, None]
        self.ham_max   = [None, None]

        self.create_subscription(Odometry, ODOM_TOPIC, self._odom, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._imu, 10)
        self.create_subscription(BatteryState, BATTERY_TOPIC, self._bat, 10)
        self.create_subscription(UInt16MultiArray, HAM_TOPIC, self._ham, 10)

    # ── Abonelikler ─────────────────────────────────────────────────────────
    def _odom(self, msg: Odometry):
        with self._kilit:
            self.odom_hz.tik()
            self.vx   = msg.twist.twist.linear.x
            self.vyaw = msg.twist.twist.angular.z
            self.odom_yaw = _yaw(msg.pose.pose.orientation)
            self.vyaw_max = max(self.vyaw_max, abs(self.vyaw))
            self.oyaw_max = max(self.oyaw_max, abs(self.odom_yaw))

            p = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if self.kayit and self.onceki is not None:
                self.yol_m += math.hypot(p[0] - self.onceki[0],
                                         p[1] - self.onceki[1])
                if self.gaz_v is not None:
                    self.ciftler.append((self.gaz_v, self.vx))
            self.onceki = p
            self.pose   = p

    def _imu(self, msg: Imu):
        with self._kilit:
            self.imu_hz.tik()
            y = _yaw(msg.orientation)
            self.imu_yaw = y
            q = msg.orientation
            self.roll  = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                                    1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            self.pitch = math.asin(max(-1.0, min(1.0,
                                       2.0 * (q.w * q.y - q.z * q.x))))
            if self.kayit:
                if self.yaw_once is not None:
                    self.yaw_top += _sarmala(y - self.yaw_once)
                self.yaw_once = y

    def _ham(self, msg: UInt16MultiArray):
        if len(msg.data) < 2:
            return
        with self._kilit:
            self.ham_hz.tik()
            for i in range(2):
                deger = int(msg.data[i])
                if self.ham_once[i] is not None:
                    d = _tick_delta(deger, self.ham_once[i])
                    if self.kayit:
                        self.ham_net[i]    += d
                        self.ham_mutlak[i] += abs(d)
                if self.kayit:
                    self.ham_min[i] = (deger if self.ham_min[i] is None
                                       else min(self.ham_min[i], deger))
                    self.ham_max[i] = (deger if self.ham_max[i] is None
                                       else max(self.ham_max[i], deger))
                self.ham_once[i] = deger
                self.ham[i]      = deger
                self.ham_ornek[i].append(deger)
                if len(self.ham_ornek[i]) > 200:
                    self.ham_ornek[i].pop(0)

    def _bat(self, msg: BatteryState):
        with self._kilit:
            self.bat_hz.tik()
            self.gaz_v = msg.current * GAZ_V_OLCEK

    # ── Ölçüm kontrolü ──────────────────────────────────────────────────────
    def basla(self):
        with self._kilit:
            self.kayit    = True
            self.yol_m    = 0.0
            self.onceki   = self.pose
            self.yaw_top  = 0.0
            self.yaw_once = self.imu_yaw
            self.yaw_bas  = self.imu_yaw
            self.ciftler  = []
            self.ham_net    = [0, 0]
            self.ham_mutlak = [0, 0]
            self.ham_min    = [None, None]
            self.ham_max    = [None, None]

    def bitir(self):
        with self._kilit:
            self.kayit = False

    def veri_var(self) -> bool:
        with self._kilit:
            return (self.odom_hz.sayi > 1 or self.imu_hz.sayi > 1
                    or self.ham_hz.sayi > 1)


# ── Modlar ──────────────────────────────────────────────────────────────────
def mod_ozet(d: SensorDogrula):
    print('\n  Topic akışı izleniyor — çıkmak için Ctrl-C\n')
    print('  {:<14}{:>9}{:>9}{:>9}'.format('', 'ODOM', 'IMU', 'BATARYA'))
    try:
        while rclpy.ok():
            time.sleep(1.0)
            with d._kilit:
                oh, ih, bh = d.odom_hz.hz, d.imu_hz.hz, d.bat_hz.hz
                vx, vyaw   = d.vx, d.vyaw
                iy         = d.imu_yaw
                rl, pt     = d.roll, d.pitch
                gaz        = d.gaz_v
                oy_max     = d.oyaw_max
                vy_max     = d.vyaw_max
            print('  {:<14}{:>8.1f}H{:>8.1f}H{:>8.1f}H'.format('frekans', oh, ih, bh))
            print('    hız {:+.3f} m/s   dönüş {:+.3f} rad/s   gaz {}'.format(
                vx, vyaw, '{:.2f} V'.format(gaz) if gaz is not None else '—'))
            if iy is None:
                print('    IMU: veri yok')
            else:
                print('    IMU  yaw {:+7.1f}°  roll {:+6.1f}°  pitch {:+6.1f}°'.format(
                    math.degrees(iy), math.degrees(rl), math.degrees(pt)))
            print('    tek parça aks imzası: odom yaw |max| {:.3f}°  vyaw |max| {:.4f} rad/s'
                  .format(math.degrees(oy_max), vy_max))
            print()
    except KeyboardInterrupt:
        pass


def _olcum_al(d: SensorDogrula, mesaj: str):
    input('  {} — hazır olunca ENTER'.format(mesaj))
    d.basla()
    print('  ölçüm başladı… bitince ENTER')
    input()
    d.bitir()


def mod_mesafe(d: SensorDogrula):
    print('\n  MESAFE KALİBRASYONU')
    print('  Aracı düz bir hatta tam {:.2f} m ilerlet.'.format(d.gercek_m))
    print('  Lastik basıklığı sonuca girsin diye araç yüklü olmalı.\n')
    _olcum_al(d, 'Başlangıç çizgisine getir')

    with d._kilit:
        olculen = d.yol_m
    print('\n  gerçek   : {:.3f} m'.format(d.gercek_m))
    print('  ölçülen  : {:.3f} m'.format(olculen))

    if olculen < 1e-3:
        print('\n  Ölçüm sıfır — enkoderden veri gelmiyor.')
        print('  Kontrol: ros2 topic hz /odom, kablolama, PKT_ENC akışı.')
        return

    oran = d.gercek_m / olculen
    print('  oran     : {:.4f}  (gerçek / ölçülen)'.format(oran))
    print('  hata     : %{:+.1f}'.format((oran - 1.0) * 100.0))
    print('\n  Düzeltilmiş yarıçap:')
    print('    TEKERLEK_YARICI = {:.4f}   (şu an {:.4f})'.format(
        d.mevcut_yaricap * oran, d.mevcut_yaricap))
    print('    seri_kopru.py içinde güncellenir.')
    if abs(oran - 1.0) < 0.02:
        print('\n  %2 altında sapma — mevcut değer yeterli, dokunma.')


def mod_yon(d: SensorDogrula):
    print('\n  IMU YÖN DOĞRULAMASI')
    print('  Aracı yerinde tam {:.0f}° çevir (saat yönünün TERSİ = pozitif,'
          .format(d.gercek_deg))
    print('  ROS kuralı: yukarıdan bakınca sola dönüş pozitiftir).\n')
    _olcum_al(d, 'Araç başlangıç yönünde')

    with d._kilit:
        olculen = math.degrees(d.yaw_top)
        veri    = d.imu_hz.sayi

    if veri < 2:
        print('\n  IMU verisi gelmedi — /imu/data boş.')
        print('  Kontrol: BMI160 I2C adresi 0x68 mi, bmi_kur() başarılı mı.')
        return

    print('\n  gerçek   : {:+.1f}°'.format(d.gercek_deg))
    print('  ölçülen  : {:+.1f}°'.format(olculen))

    if abs(olculen) < 5.0:
        print('\n  IMU dönüşü görmüyor — eksen eşlemesi yanlış ya da gyro ölü.')
        return

    if olculen * d.gercek_deg < 0:
        print('\n  İŞARET TERS: IMU ters yönde sayıyor.')
        print('  main.cpp içindeki eksen eşlemesinde gz işareti çevrilmeli.')
    else:
        print('\n  İşaret doğru.')

    olcek = abs(olculen) / abs(d.gercek_deg)
    print('  ölçek    : {:.3f}  (hata %{:+.1f})'.format(olcek, (olcek - 1.0) * 100.0))
    if abs(olcek - 1.0) > 0.05:
        print('  %5 üstü ölçek hatası — IMU_GYRO_SCALE değeri gözden geçirilmeli.')


def mod_hiz(d: SensorDogrula):
    print('\n  GAZ ↔ HIZ EŞLEMESİ')
    print('  Aracı MANUEL modda sür; birkaç farklı sabit gazda birer süre tut.')
    print('  Ne kadar çok farklı hız görülürse uydurma o kadar güvenilir.\n')
    _olcum_al(d, 'Sürüşe hazır ol')

    with d._kilit:
        ciftler = list(d.ciftler)

    hareketli = [(g, v) for g, v in ciftler if abs(v) > 0.05]
    if len(hareketli) < 20:
        print('\n  Yeterli örnek yok ({} nokta).'.format(len(hareketli)))
        if not ciftler:
            print('  /battery/status gelmiyor olabilir — gaz voltajı okunamadı.')
        return

    gaz = np.array([g for g, _ in hareketli])
    hiz = np.array([v for _, v in hareketli])
    if gaz.max() - gaz.min() < 0.15:
        print('\n  Gaz hep aynı seviyede kalmış — farklı hızlar denenmeli.')
        return

    a, b = np.polyfit(gaz, hiz, 1)
    tahmin = a * gaz + b
    r2 = 1.0 - np.sum((hiz - tahmin) ** 2) / np.sum((hiz - hiz.mean()) ** 2)

    print('\n  örnek    : {} nokta'.format(len(hareketli)))
    print('  gaz      : {:.2f} – {:.2f} V'.format(gaz.min(), gaz.max()))
    print('  hız      : {:.2f} – {:.2f} m/s'.format(hiz.min(), hiz.max()))
    print('\n  uydurma  : hız = {:.3f} × gaz {:+.3f}     R² = {:.3f}'.format(a, b, r2))
    if abs(a) > 1e-6:
        print('  ters yön : gaz = {:.3f} + hız / {:.3f}'.format(-b / a, a))
        print('             (sahada ölçülen eşleme: gaz = 0.90 + hız)')
    if r2 < 0.8:
        print('\n  R² düşük — noktalar doğruya oturmuyor. Sabit gazda daha uzun')
        print('  süre tutmayı dene; hızlanma anları uydurmayı bozar.')


def mod_ham(d: SensorDogrula):
    print('\n  HAM ENKODER')
    print('  seri_kopru -p ham_enkoder:=true ile başlatılmış olmalı.\n')

    for _ in range(50):
        with d._kilit:
            if d.ham_hz.sayi > 1:
                break
        time.sleep(0.1)
    else:
        print('  {} sessiz.'.format(HAM_TOPIC))
        print('  seri_kopru ham_enkoder parametresi açık mı?\n')
        return

    # 1) Araç dururken gürültü — bağlı olmayan pin burada ele verir.
    print('  Araca DOKUNMA — 3 sn gürültü ölçülüyor…')
    with d._kilit:
        d.ham_ornek = [[], []]
    time.sleep(3.0)
    with d._kilit:
        ornek = [list(o) for o in d.ham_ornek]
        hz    = d.ham_hz.hz

    print('\n  paket akışı: {:.1f} Hz\n'.format(hz))
    print('  {:<8}{:>8}{:>8}{:>8}{:>10}'.format(
        'kanal', 'ort', 'min', 'max', 'std'))
    duruk = [0.0, 0.0]
    for i, ad in enumerate(('sol/A0', 'sag/A1')):
        if not ornek[i]:
            print('  {:<8}{:>8}'.format(ad, 'veri yok'))
            continue
        a = np.array(ornek[i], dtype=float)
        duruk[i] = float(a.std())
        print('  {:<8}{:>8.1f}{:>8.0f}{:>8.0f}{:>10.2f}'.format(
            ad, a.mean(), a.min(), a.max(), duruk[i]))

    print('\n  Dururken std birkaç LSB\'yi aşıyorsa o kanal ya boştadır ya da')
    print('  kablosu gürültü topluyordur; boş pin genelde geniş salınım gösterir.')

    # 2) Tekerleği elle çevirerek tur başına tick — ölçek buradan çıkar.
    print('\n  Şimdi tahrik tekerleğini elle tam {} tur çevir.'.format(d.tur_sayisi))
    print('  Yavaş ve tek yönde çevir; ileri yön pozitif sayılmalı.')
    _olcum_al(d, 'Tekerlek başlangıç işaretinde')

    with d._kilit:
        net    = list(d.ham_net)
        mutlak = list(d.ham_mutlak)
        hmin   = list(d.ham_min)
        hmax   = list(d.ham_max)

    turlar = max(1, d.tur_sayisi)
    print('\n  {:<8}{:>11}{:>10}{:>11}{:>10}'.format(
        'kanal', 'net tick', '|tick|', 'ADC aralık', 'tutarlı'))
    tick_tur = [0.0, 0.0]
    for i, ad in enumerate(('sol/A0', 'sag/A1')):
        tick_tur[i] = abs(net[i]) / turlar
        # Gerçek dönüş net sayımı biriktirir; gürültü ileri geri gider ve
        # |tick| şişerken net ~0 kalır. Oran kanalı ayırt eder.
        tutarli = abs(net[i]) / mutlak[i] if mutlak[i] else 0.0
        aralik = ('{}–{}'.format(hmin[i], hmax[i])
                  if hmin[i] is not None else '—')
        print('  {:<8}{:>11d}{:>10d}{:>11}{:>10.2f}'.format(
            ad, net[i], mutlak[i], aralik, tutarli))

    # Hareketle ilişkili kanal hangisiyse enkoder oraya bağlıdır.
    bagli = 0 if abs(net[0]) >= abs(net[1]) else 1
    ad    = ('sol', 'sag')[bagli]
    if abs(net[bagli]) < TICKS_PER_REV * turlar * 0.1:
        print('\n  Hiçbir kanal turla orantılı saymadı — enkoder okunmuyor.')
        print('  Kontrol: AS5600 OUT pini Mega A0/A1\'e mi gidiyor, mıknatıs')
        print('  sensöre paralel ve merkezde mi, besleme var mı.')
        return

    print('\n  Bağlı görünen kanal: {}  →  seri_kopru -p enkoder_kanali:={}'
          .format(ad.upper(), ad))
    if net[bagli] < 0:
        print('  Net sayım NEGATİF: ileri hareket geri okunuyor. Mıknatıs')
        print('  yönü ya da işaret ters — /odom mesafesi eksiye gider.')

    # Besleme teşhisi ADC tepe değerinden okunur, tick/tur'dan DEĞİL.
    tepe = hmax[bagli]
    print('\n  ADC tepe : {}   (5 V beslemede ~{} beklenir)'
          .format(tepe, TICKS_PER_REV - 1))
    if tepe < TICKS_PER_REV * 0.80:
        oran = (TICKS_PER_REV - 1) / max(1, tepe)
        print('\n  Tepe değer düşük — AS5600 büyük olasılıkla 3.3 V ile')
        print('  besleniyor ya da çıkışı bölünmüş. 5 V\'a al ve tekrar ölç.')
        if tepe < TICKS_PER_REV // 2:
            print('  DİKKAT: tepe yarım turun (512) altında. Bu durumda sarma')
            print('  düzeltmesi hiç devreye girmez ve net sayım sıfıra çöker —')
            print('  araç ilerlerken /odom mesafeyi hiç saymaz.')
        else:
            print('  Tepe 512\'nin üstünde olduğu için sarma düzeltmesi farkı')
            print('  kapatıyor ve mesafe şimdilik doğru çıkıyor (ölçüm {:.0f}'
                  .format(tick_tur[bagli]))
            print('  tick/tur); ama pay ~{:.0f}%\'e inmiş, titreşimde çöker.'
                  .format((oran - 1.0) * 100.0))
    else:
        print('  Besleme beklenen aralıkta — TICKS_PER_REV değişmesin.')

    print('\n  ölçülen tick/tur: {:.1f}'.format(tick_tur[bagli]))

    tutarli = abs(net[bagli]) / mutlak[bagli] if mutlak[bagli] else 0.0
    if tutarli < 0.95:
        kayip = mutlak[bagli] - abs(net[bagli])
        print('\n  Yön değiştiren sayım fazla ({} tick ileri-geri, tutarlılık'
              .format(kayip))
        print('  {:.2f}). Mekanik boşluk, zincir kaçırması ya da mıknatıs'
              .format(tutarli))
        print('  merkezden kaçmış olabilir; sayım gürültüsü mesafeye karışır.')


def main():
    rclpy.init()
    d = SensorDogrula()

    yurutucu = threading.Thread(target=rclpy.spin, args=(d,), daemon=True)
    yurutucu.start()

    print('\n' + '=' * 62)
    print('  SENSÖR DOĞRULAMA — mod: {}'.format(d.test))
    print('=' * 62)

    # Topic'lerin gelmesini bekle; hiç gelmiyorsa mod çalıştırmanın anlamı yok.
    for _ in range(30):
        if d.veri_var():
            break
        time.sleep(0.1)
    else:
        print('\n  {} ve {} sessiz.'.format(ODOM_TOPIC, IMU_TOPIC))
        print('  seri_kopru çalışıyor mu? ros2 node list ile bak.\n')

    modlar = {'ozet': mod_ozet, 'mesafe': mod_mesafe,
              'yon': mod_yon, 'hiz': mod_hiz, 'ham': mod_ham}
    calistir = modlar.get(d.test)
    if calistir is None:
        print('\n  Bilinmeyen mod: {}'.format(d.test))
        print('  Seçenekler: {}\n'.format(', '.join(modlar)))
    else:
        try:
            calistir(d)
        except (KeyboardInterrupt, EOFError):
            pass

    print()
    d.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
