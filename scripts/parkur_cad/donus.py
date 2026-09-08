"""Parkurun dönüş yarıçapı ve koni slalomu geçilebilirlik analizi.

    python3 donus.py            # rapor + donus_raporu.png

Aracın 1,90 × 1,16 m dikdörtgeni, eğriliği 1/R ile sınırlı ve GERİ VİTESSİZ
yollar boyunca süpürülüyor; engellerle tam dikdörtgen-dikdörtgen (ayırıcı
eksen) testi yapılıyor. Bariyerler `duzeltilmis.json`'dan yönelimleriyle
okunuyor — eksen hizalı sınır kutusu virajlarda koridoru sahte biçimde
daraltır (45° dönmüş 1,60 × 0,36'lık bariyerin kutusu 1,39 × 1,39'dur).

Şerit duvarının kesildiği yerler (yan eğim ve dik eğim yapılarının bulunduğu
aralıklar) §6.1'in 3 m şerit sınırında sanal duvarla kapatılıyor; açık
bırakılırsa araç koridorun dışına çıkıp anlamsız kısayol bulur.

Koniler §6.7'ye göre modelleniyor (40±10 cm kare taban), CAD'deki jenerik
52 cm'lik modellere göre değil. §6.7 koni yerleşimini yarışma günü hakem
heyetine bıraktığı için CAD'deki dizilim bağlayıcı değil; bu yüzden asıl
çıktı "şu dizilim geçilir mi" değil, geçilebilir dizilimlerin sınırı.
"""
import json, math, heapq, os
import numpy as np

BURASI = os.path.dirname(os.path.abspath(__file__))
JSON = os.path.join(BURASI, 'duzeltilmis.json')
BAR_UZUN, BAR_KALIN = 1.60, 0.36
SERIT = 3.0                       # §6.1


class Arac:
    """Bisiklet modeli; poz arka aks merkezinde."""

    def __init__(self, boy=1.90, gen=1.16, dingil=1.40, arka_tasma=0.20):
        self.boy, self.gen, self.dingil = boy, gen, dingil
        self.arka, self.on = arka_tasma, boy - arka_tasma
        self.yari_b, self.yari_g = boy / 2, gen / 2
        self.ofset = (self.on - self.arka) / 2      # dikdörtgen merkezi aksın önünde

    def dikdortgen(self, x, y, th):
        a = np.array([math.cos(th), math.sin(th)])
        b = np.array([-a[1], a[0]])
        return np.array([x, y]) + a * self.ofset, a, b, self.yari_b, self.yari_g


def _obb(c, u, h, k):
    u = np.asarray(u, float) / np.linalg.norm(u)
    return np.asarray(c, float), u, np.array([-u[1], u[0]]), h, k


def bariyerler():
    veri = json.load(open(JSON))
    out = []
    for p in veri:
        if 'bariyer' not in p['ad']:
            continue
        M = np.array(p['R']).reshape(3, 3)
        c = ((p['min'][0] + p['max'][0]) / 2000.0, -(p['min'][2] + p['max'][2]) / 2000.0)
        out.append(_obb(c, (M[0, 2], -M[2, 2]), BAR_UZUN / 2, BAR_KALIN / 2))
    return out


def sanal_duvarlar():
    """Yan eğim ve dik eğim yapılarının kestiği şerit duvarlarını kapatır."""
    yari = SERIT / 2
    return [_obb((-16.25, +yari), (1, 0), 4.45, 0.05),    # y=0 kuzey, x -20,7..-11,8
            _obb((-15.35, 20 - yari), (1, 0), 4.95, 0.05),  # y=20 güney, x -20,3..-10,4
            _obb((-15.70, 20 + yari), (1, 0), 4.60, 0.05)]  # y=20 kuzey, x -20,3..-11,1


def koni_dizilimi(n, aralik, yanal, merkez_y=10.0, bas_x=-12.6, taban=0.40):
    """Şaşırtmalı slalom: ardışık koniler ±yanal/2, x'te `aralik` arayla."""
    return [(bas_x + i * aralik, merkez_y + (yanal / 2 if i % 2 == 0 else -yanal / 2),
             taban * math.sqrt(2) / 2) for i in range(n)]


def cad_konileri():
    veri = json.load(open(JSON))
    return [((p['min'][0] + p['max'][0]) / 2000, -(p['min'][2] + p['max'][2]) / 2000,
             (p['max'][0] - p['min'][0]) / 2000)
            for p in veri if p['ad'] == 'Traffic Cone']


def _sat(c1, a1, b1, h1, k1, c2, a2, b2, h2, k2):
    d = c2 - c1
    for ax in (a1, b1, a2, b2):
        if abs(d @ ax) > (h1 * abs(a1 @ ax) + k1 * abs(b1 @ ax)
                          + h2 * abs(a2 @ ax) + k2 * abs(b2 @ ax)):
            return False
    return True


def carpisma(arac, x, y, th, kutular, koniler):
    c, a, b, h, k = arac.dikdortgen(x, y, th)
    for (bc, bu, bn, bh, bk) in kutular:
        if abs(bc[0] - c[0]) > 4.5 or abs(bc[1] - c[1]) > 4.5:
            continue
        if _sat(c, a, b, h, k, bc, bu, bn, bh, bk):
            return True
    for (kx, ky, kr) in koniler:
        d = np.array([kx - c[0], ky - c[1]])
        if abs(d[0]) > 3 or abs(d[1]) > 3:
            continue
        p = c + np.clip(d @ a, -h, h) * a + np.clip(d @ b, -k, k) * b
        if math.hypot(kx - p[0], ky - p[1]) < kr:
            return True
    return False


def yol_var_mi(arac, R, bas, hedef, kutu, engeller, koniler,
               adim=0.25, dth=math.radians(3), dxy=0.15,
               tol_xy=0.35, tol_th=math.radians(12), maks=600000):
    """Hybrid A*: eğriliği 1/R ile sınırlı, geri vitessiz yol var mı."""
    x0, y0, x1, y1 = kutu
    eng = [e for e in engeller if x0 - 3 < e[0][0] < x1 + 3 and y0 - 3 < e[0][1] < y1 + 3]
    kon = [c for c in koniler if x0 - 3 < c[0] < x1 + 3]
    if carpisma(arac, *bas, eng, kon) or carpisma(arac, *hedef, eng, kon):
        raise ValueError('başlangıç veya hedef pozu çarpışıyor')
    nt = int(round(2 * math.pi / dth))

    def anahtar(x, y, th):
        return int(x / dxy), int(y / dxy), int((th % (2 * math.pi)) / dth) % nt

    def kalan(x, y):
        return math.hypot(hedef[0] - x, hedef[1] - y)

    kuyruk = [(kalan(bas[0], bas[1]), 0.0, bas, None)]
    geldi = {anahtar(*bas): (0.0, None, bas)}
    kappa, genisletilen = 1.0 / R, 0
    while kuyruk:
        _, g, s, _ = heapq.heappop(kuyruk)
        genisletilen += 1
        if genisletilen > maks:
            return None, genisletilen
        x, y, th = s
        if (kalan(x, y) < tol_xy
                and abs((th - hedef[2] + math.pi) % (2 * math.pi) - math.pi) < tol_th):
            yol, a = [s], anahtar(*s)
            while geldi[a][1] is not None:
                a = geldi[a][1]
                yol.append(geldi[a][2])
            return yol[::-1], genisletilen
        a = anahtar(x, y, th)
        if geldi.get(a, (1e9,))[0] < g - 1e-9:
            continue
        for kp in (0.0, kappa, -kappa):
            if kp == 0.0:
                nx, ny, nth = x + adim * math.cos(th), y + adim * math.sin(th), th
            else:
                nth = th + kp * adim
                nx = x + (math.sin(nth) - math.sin(th)) / kp
                ny = y - (math.cos(nth) - math.cos(th)) / kp
            if not (x0 < nx < x1 and y0 < ny < y1):
                continue
            if carpisma(arac, nx, ny, nth, eng, kon):
                continue
            ng = g + adim * (1.0 if kp == 0.0 else 1.03)
            na = anahtar(nx, ny, nth)
            if na in geldi and geldi[na][0] <= ng:
                continue
            geldi[na] = (ng, a, (nx, ny, nth))
            heapq.heappush(kuyruk, (ng + kalan(nx, ny), ng, (nx, ny, nth), a))
    return None, genisletilen


def en_buyuk_R(arac, bas, hedef, kutu, engeller, koniler, alt=1.0, ust=9.0, tol=0.1):
    """Yolun hâlâ bulunabildiği en büyük R (dönüşün ne kadar geniş olduğunun ölçüsü)."""
    if yol_var_mi(arac, alt, bas, hedef, kutu, engeller, koniler)[0] is None:
        return None
    while ust - alt > tol:
        orta = (alt + ust) / 2
        if yol_var_mi(arac, orta, bas, hedef, kutu, engeller, koniler)[0] is not None:
            alt = orta
        else:
            ust = orta
    return alt


# ── rapor ────────────────────────────────────────────────────────────────────

U_DONUSLER = [
    ('sol U  (y=0 → y=10)',  (-19.0, 0.0, math.pi), (-19.0, 10.0, 0.0),
     (-28.0, -1.45, -18.0, 11.45)),
    ('sağ U  (y=10 → y=20)', (-1.0, 10.0, 0.0),     (-1.0, 20.0, math.pi),
     (-2.0, 8.45, 8.5, 21.55)),
]
KONI_PAY = 2.5          # koni dizisinin önündeki/arkasındaki düz yaklaşma [m]


def slalom_kur(s, yanal, taban=0.40):
    n = max(3, min(8, int(12.0 / s) + 1))
    x0 = -9.0 - (n - 1) * s / 2
    return (koni_dizilimi(n, s, yanal, bas_x=x0, taban=taban),
            (x0 - KONI_PAY, 10.0, 0.0), (x0 + (n - 1) * s + KONI_PAY, 10.0, 0.0))


def slalom_gecer(arac, R, s, yanal, engeller, taban=0.40):
    kon, bas, hedef = slalom_kur(s, yanal, taban)
    kutu = (bas[0] - 1.0, 8.45, hedef[0] + 1.0, 11.55)
    return yol_var_mi(arac, R, bas, hedef, kutu, engeller, kon)[0] is not None


def en_dar_aralik(arac, R, yanal, engeller, alt=0.8, ust=5.0, tol=0.05):
    """Bu yanal kaçıklıkta geçilebilen en küçük boyuna koni aralığı."""
    if slalom_gecer(arac, R, alt, yanal, engeller):
        return alt
    if not slalom_gecer(arac, R, ust, yanal, engeller):
        return None
    while ust - alt > tol:
        orta = (alt + ust) / 2
        if slalom_gecer(arac, R, orta, yanal, engeller):
            ust = orta
        else:
            alt = orta
    return ust


def rapor(R=2.42):
    arac, engeller = Arac(), bariyerler() + sanal_duvarlar()
    print(f'araç {arac.boy:.2f} × {arac.gen:.2f} m · dingil {arac.dingil:.2f} m · '
          f'R = {R:.2f} m · geri vites yok\n')

    print('U DÖNÜŞLERİ')
    for ad, bas, hedef, kutu in U_DONUSLER:
        yol, _ = yol_var_mi(arac, R, bas, hedef, kutu, engeller, [])
        buyuk = en_buyuk_R(arac, bas, hedef, kutu, engeller, [])
        delta = math.degrees(math.atan(arac.dingil / buyuk)) if buyuk else float('nan')
        print(f'  {ad:22s} R={R:.2f} m ile {"geçer" if yol else "GEÇMEZ"} · '
              f'sınır R = {buyuk:.2f} m · yeten δ_max = {delta:.1f}°')

    print('\nKONİ SLALOMU — gereken en küçük boyuna aralık [m]')
    print('  (§6.7 yerleşimi yarışma günü hakem heyeti belirliyor)')
    print('  >5,0 = şeride sığdırılabilen en geniş aralıkta bile bulunamadı;')
    print('         dar yanal kaçıklıkta araç şeridi baştan başa mekik dokumak')
    print('         zorunda kalıyor ve 3 m şerit buna yetmiyor')
    paylar = [('pay yok', 0.0), ('10 cm/yan', 0.20), ('15 cm/yan', 0.30)]
    print(f"    {'yanal':>7s}" + ''.join(f'{a:>12s}' for a, _ in paylar))
    for yanal in (0.75, 1.00, 1.25, 1.48, 1.75, 2.00, 2.25):
        hucre = []
        for _, p in paylar:
            s = en_dar_aralik(Arac(gen=arac.gen + p), R, yanal, engeller)
            hucre.append(f'{s:.2f}' if s else '>5,0')
        print(f'    {yanal:7.2f}' + ''.join(f'{h:>12s}' for h in hucre))


def cizim(dosya=None, R=2.42, slalom=(2.60, 1.48)):
    """Bulunan yolları araç dikdörtgenleriyle çizer — koridor dışına taşma gözle görülsün."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Circle

    arac, engeller = Arac(), bariyerler() + sanal_duvarlar()
    dosya = dosya or os.path.join(BURASI, 'donus_raporu.png')
    fig, eksenler = plt.subplots(1, 3, figsize=(21, 11))

    def ciz(ax, kutu, koniler, yol, baslik, her=4):
        for (c, u, n, h, k) in engeller:
            if not (kutu[0] - 3 < c[0] < kutu[2] + 3 and kutu[1] - 3 < c[1] < kutu[3] + 3):
                continue
            ax.add_patch(Polygon([c + su * u * h + sn * n * k
                                  for su, sn in ((1, 1), (1, -1), (-1, -1), (-1, 1))],
                                 fc='#c33', ec='k', lw=.3))
        for (x, y, r) in koniler:
            ax.add_patch(Circle((x, y), r, fc='orange', ec='k', lw=.4))
        if yol:
            p = np.array([(q[0], q[1]) for q in yol])
            ax.plot(p[:, 0], p[:, 1], '-', c='#06c', lw=1.4)
            for i, (x, y, th) in enumerate(yol):
                if i % her and i != len(yol) - 1:
                    continue
                c, a, b, h, k = arac.dikdortgen(x, y, th)
                ax.add_patch(Polygon([c + su * a * h + sn * b * k
                                      for su, sn in ((1, 1), (1, -1), (-1, -1), (-1, 1))],
                                     fc='none', ec='#093', lw=.8))
        ax.set_title(baslik)
        ax.set_aspect('equal')
        ax.grid(alpha=.35)

    for ax, (ad, bas, hedef, kutu) in zip(eksenler[:2], U_DONUSLER):
        yol, _ = yol_var_mi(arac, R, bas, hedef, kutu, engeller, [])
        ciz(ax, kutu, [], yol, f'{ad}  R={R:.2f} m')
        ax.set_xlim(kutu[0] - 1, kutu[2] + 1)
        ax.set_ylim(kutu[1] - 1, kutu[3] + 1)

    s, yanal = slalom
    kon, bas, hedef = slalom_kur(s, yanal)
    kutu = (bas[0] - 1, 8.45, hedef[0] + 1, 11.55)
    yol, _ = yol_var_mi(arac, R, bas, hedef, kutu, engeller, kon)
    ciz(eksenler[2], kutu, kon, yol, f'koni slalomu  d={yanal} m  s={s} m', her=3)
    eksenler[2].set_xlim(kutu[0] - 1, kutu[2] + 1)
    eksenler[2].set_ylim(7.5, 12.5)

    plt.savefig(dosya, dpi=80, bbox_inches='tight')
    print(f'\nçizim: {dosya}')


if __name__ == '__main__':
    rapor()
    cizim()
