#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

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
          v = a·gaz + b doğrusunu uydurur. Gaz voltajı kartın bildirdiği
          çıkıştan okunur (/kart/surus, 0x37).

  ham     Ham kuadratür sayımına bakar: duruşta oynuyor mu, hareketle
          tutarlı artıyor mu, yönü doğru mu. Ardından mezürle ölçülmüş bir
          mesafeden ÖLÇEK KATSAYISINI çıkarır — enkoder motor miline bağlı
          olduğu için dişli oranı ve tekerlek çevresi bu tek sayının içinde
          birlikte gelir, ayrı ölçülmeleri gerekmez.
          seri_kopru -p ham_enkoder:=true ister.

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
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int16MultiArray


ODOM_TOPIC    = '/odom'
IMU_TOPIC     = '/imu/data'
HAM_TOPIC     = '/enkoder/ham'
SURUS_TOPIC   = '/kart/surus'

# E6B2-CWZ6C artımlı kuadratür enkoder: 600 P/R, dört kenar sayıldığı için
# tur başına 2400 sayım. Tezgâhta doğrulandı. Enkoder tekerleğe değil
# traksiyon motorunun miline 1:1 kaplinle bağlı, yani bu sayı MOTOR turudur;
# tekerleğe çevirmek dişli oranını gerektirir ve o oran henüz ölçülmedi.
SAYIM_PER_TUR = 2400


def _yaw(q) -> float:
    """Quaternion → yaw [rad]."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sarmala(a: float) -> float:
    """Açıyı (-π, π] aralığına indirger."""
    return math.atan2(math.sin(a), math.cos(a))


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

        self.test           = str(self.get_parameter('test').value)
        self.gercek_m       = float(self.get_parameter('gercek_m').value)
        self.mevcut_yaricap = float(self.get_parameter('mevcut_yaricap').value)
        self.gercek_deg     = float(self.get_parameter('gercek_deg').value)

        self._kilit = threading.Lock()

        # Anlık durum
        self.odom_hz  = FrekansSayaci()
        self.imu_hz   = FrekansSayaci()
        self.surus_hz = FrekansSayaci()
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

        # Ham enkoder — kartın int32 kuadratür sayacı. Taşmayı kart çözüyor,
        # burada sarma düzeltmesi gerekmiyor.
        self.ham_hz    = FrekansSayaci()
        self.ham       = None             # anlık sayım
        self.ham_once  = None             # önceki okuma
        self.ham_net   = 0                # işaretli toplam (yön korunur)
        self.ham_mutlak= 0                # |delta| toplamı (titreşim dahil)
        self.ham_ornek = []               # gürültü penceresi

        self.create_subscription(Odometry, ODOM_TOPIC, self._odom, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._imu, 10)
        self.create_subscription(Int32, HAM_TOPIC, self._ham, 10)
        self.create_subscription(Int16MultiArray, SURUS_TOPIC, self._surus, 10)

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

    def _ham(self, msg: Int32):
        deger = int(msg.data)
        with self._kilit:
            self.ham_hz.tik()
            if self.ham_once is not None and self.kayit:
                d = deger - self.ham_once
                self.ham_net    += d
                self.ham_mutlak += abs(d)
            self.ham_once = deger
            self.ham      = deger
            self.ham_ornek.append(deger)
            if len(self.ham_ornek) > 200:
                self.ham_ornek.pop(0)

    def _surus(self, msg: Int16MultiArray):
        if len(msg.data) < 2:
            return
        with self._kilit:
            self.surus_hz.tik()
            # Kart ürettiği gaz çıkışını mV olarak bildiriyor.
            self.gaz_v = float(msg.data[0]) / 1000.0


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
            self.ham_net    = 0
            self.ham_mutlak = 0

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
    print('  {:<14}{:>9}{:>9}{:>9}'.format('', 'ODOM', 'IMU', 'SÜRÜŞ'))
    try:
        while rclpy.ok():
            time.sleep(1.0)
            with d._kilit:
                oh, ih, bh = d.odom_hz.hz, d.imu_hz.hz, d.surus_hz.hz
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
        print('  Kontrol: ros2 topic hz /odom, kablolama, 0x31 hız akışı.')
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
    """
    Ham kuadratür sayımını doğrular ve ölçek katsayısını çıkarır.

    Kart, tekerlek çevresi ve dişli oranı girilmediği sürece hız alanını
    bilerek 0 basıyor; ham sayım o sırada da akıyor ve ölçekten bağımsız.
    Bu mod önce sayımın gerçekten enkoderden geldiğini (duruşta oynamıyor,
    hareketle tutarlı artıyor) sonra da mesafeye çevrim katsayısını ölçer.

    Enkoder motor miline bağlı, yani sayım motor turunu gösteriyor. Katsayı
    yine de tek ölçümle çıkıyor: gerçek mesafeyi sayıma bölmek dişli oranını
    ve tekerlek çevresini birlikte içeren sayıyı doğrudan veriyor.
    """
    print('\n  HAM ENKODER SAYIMI VE ÖLÇEK')
    print('  seri_kopru -p ham_enkoder:=true ile başlatılmış olmalı.\n')

    for _ in range(50):
        with d._kilit:
            if d.ham_hz.sayi > 1:
                break
        time.sleep(0.1)
    else:
        print('  {} sessiz.'.format(HAM_TOPIC))
        print('  Sırayla bak: seri_kopru ham_enkoder parametresi açık mı,')
        print('  köprü portu açabildi mi, kart 0x30 gönderiyor mu.\n')
        return

    # 1) Araç dururken sayım oynuyorsa kaynak gürültüdür: kuadratür sayacı
    #    duran bir milde tek adım bile ilerlememeli.
    print('  Araca DOKUNMA — 3 sn duruşta sayım izleniyor…')
    with d._kilit:
        d.ham_ornek = []
    time.sleep(3.0)
    with d._kilit:
        ornek = list(d.ham_ornek)
        hz    = d.ham_hz.hz

    print('\n  yayın frekansı : {:.1f} Hz'.format(hz))
    if len(ornek) >= 2:
        oynama = max(ornek) - min(ornek)
        print('  duruşta oynama : {} sayım'.format(oynama))
        if oynama > 4:
            print('\n  Araç dururken sayım oynuyor. Kuadratür sayacında bu')
            print('  beklenmez: kablo gürültüsü, ekransız uzun hat ya da')
            print('  kaplinin boşta titremesi olabilir. Sayımın bu kadarı')
            print('  doğrudan mesafeye karışır.')
    else:
        print('  duruşta oynama : ölçülemedi (yeterli örnek yok)')

    # 2) Mezürle ölçülmüş bir mesafe — ölçek, yön ve tutarlılık burada çıkar.
    #
    # Enkoder motor miline bağlı olduğu için sayımı mesafeye çevirmek hem
    # dişli oranını hem tekerlek çevresini ister. İkisini ayrı ayrı ölçmeye
    # gerek yok: gerçek mesafeyi sayıma bölmek ikisini birden içeren tek
    # katsayıyı doğrudan veriyor. Dişli oranı sayının içinde kalıyor.
    print('\n  Aracı düz bir çizgide, mezürle ölçülmüş bir mesafe kadar İLERİ it.')
    print('  10 m yeterli; uzun mesafe ölçüm hatasını küçültür.')
    try:
        mesafe = float(input('  Kaç metre iteceksin? ').strip() or '0')
    except (ValueError, EOFError):
        mesafe = 0.0
    if mesafe <= 0:
        print('  Mesafe girilmedi — ölçek kalibrasyonu atlandı.\n')
        return

    print('  Ölçüm başlıyor. Aracı it, bitince Enter…')
    d.basla()
    try:
        input()
    except EOFError:
        time.sleep(5.0)
    d.bitir()

    with d._kilit:
        net    = d.ham_net
        mutlak = d.ham_mutlak

    if mutlak == 0:
        print('\n  Sayım hiç değişmedi. Enkoder mile bağlı değil, kaplin')
        print('  boşta dönüyor ya da kart sayacı okumuyor.\n')
        return

    print('\n  net sayım      : {:+d}'.format(net))
    print('  mutlak sayım   : {}'.format(mutlak))

    if abs(net) < 100:
        print('\n  Sayım mesafeye göre çok küçük ({} sayım / {:.2f} m).'
              .format(abs(net), mesafe))
        print('  Kaplin kayıyor ya da sayacın bir kanalı okunmuyor olabilir;')
        print('  katsayı bu ölçümden çıkarılmaz.\n')
        return

    sayim_per_m = abs(net) / mesafe
    mm_per_sayim = mesafe * 1000.0 / abs(net)
    print('\n  ÖLÇEK KATSAYISI')
    print('  metre başına sayım : {:.1f}'.format(sayim_per_m))
    print('  sayım başına mesafe: {:.4f} mm'.format(mm_per_sayim))
    print('\n  Bu tek sayı dişli oranını ve tekerlek çevresini birlikte içerir;')
    print('  ikisini ayrı ölçmeye gerek yok. Karta girecek değer budur.')

    # Ayrık iki sabit isteniyorsa dişli oranı çevreden çıkar; çevre bilinmiyorsa
    # bu adım atlanır, ölçek katsayısı tek başına yeterlidir.
    try:
        cevre = float(input('\n  Tekerlek çevresi biliniyorsa [m], yoksa boş geç: ')
                      .strip() or '0')
    except (ValueError, EOFError):
        cevre = 0.0
    if cevre > 0:
        tekerlek_turu = mesafe / cevre
        motor_turu    = abs(net) / SAYIM_PER_TUR
        print('  dişli oranı (motor turu / tekerlek turu): {:.3f}'
              .format(motor_turu / tekerlek_turu))
        print('  ENK_TEKER_CEVRE_MM = {:.0f}'.format(cevre * 1000.0))

    # 3) Yön. İleri itilirken sayım azalıyorsa işaret ters demektir; düzeltme
    #    kartta tek satırdır (ENK_TERS) ve elektrik tarafında yapılır.
    if net < 0:
        print('\n  İLERİ hareket sayımı AZALTIYOR — işaret ters.')
        print('  Düzeltme kartta: ENK_TERS. Elektrik ekibine bildir;')
        print('  düzeltilmezse /odom mesafeyi eksiye sayar ve geri kayma')
        print('  tespiti tam tersini okur.')
    else:
        print('\n  Yön doğru: ileri hareket sayımı artırıyor.')

    tutarli = abs(net) / mutlak if mutlak else 0.0
    print('  tutarlılık     : {:.2f}'.format(tutarli))
    if tutarli < 0.95:
        kayip = mutlak - abs(net)
        print('\n  Yön değiştiren sayım fazla ({} sayım ileri-geri).'
              .format(kayip))
        print('  Mekanik boşluk, zincir kaçırması ya da kaplin gevşekliği')
        print('  olabilir; bu sayım gürültüsü doğrudan mesafeye karışır.')

    print()



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
