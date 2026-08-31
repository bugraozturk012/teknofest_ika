"""Kayan hedef üreticisini parkur CAD'ine karşı doğrular — araç gerekmez.

    python3 koridor_dogrula.py

Gerçek bariyer geometrisinden (yönelimli kutular, `duzeltilmis.json`) her
poz için sanal bir LiDAR taraması üretir, `pure_logic.koridor_merkez_cizgisi`
+ `kayan_hedef`'i koşturur ve dört soruyu yanıtlar:

  1. Parkurun neresinde hedef üretilebiliyor (kapsama)?
  2. Üretilen hedef koridorun içinde mi, yoksa duvara mı düşüyor?
  3. Güven düzeyi öngörülen yerlerde düşüyor mu (yan eğimin tek duvarlı
     bölümü, rampanın iki duvarsız bölümü)?
  4. Koni bölgesinde koniler duvar sanılıyor mu?

Sanal tarama gerçek taramanın yerini tutmaz — gürültü yok, ışın yok sayılan
yüzeyden yansımıyor, bariyerler kusursuz dikdörtgen. Buradaki başarısızlık
gerçek bir hatadır; buradaki başarı sahada çalışacağının garantisi değildir.
"""
import json
import math
import os
import sys

import numpy as np

BURASI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(BURASI, '..', '..')))

from teknofest_ika.otonomi.pure_logic import (        # noqa: E402
    koridor_merkez_cizgisi, kayan_hedef, ic_duvar_hedefi,
)
import donus                                           # noqa: E402

ISIN_SAYISI = 480
MENZIL_MAKS = 12.0


def _obb_isin(p, d, kutu):
    """Işın × yönelimli dikdörtgen — en yakın giriş mesafesi ya da None."""
    c, u, n, h, k = kutu
    rel = p - c
    o = np.array([rel @ u, rel @ n])
    dd = np.array([d @ u, d @ n])
    t0, t1 = 0.0, MENZIL_MAKS
    for eksen, yari in ((0, h), (1, k)):
        if abs(dd[eksen]) < 1e-9:
            if abs(o[eksen]) > yari:
                return None
            continue
        a = (-yari - o[eksen]) / dd[eksen]
        b = (yari - o[eksen]) / dd[eksen]
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return None
    return t0 if t0 > 0.0 else None


def tarama_uret(poz, engeller, koniler=(), isin=ISIN_SAYISI):
    """(x, y, yaw) pozundan sanal 360° tarama. Dönüş: (ranges, angle_min, inc)."""
    x, y, yaw = poz
    p = np.array([x, y])
    yakin = [e for e in engeller
             if abs(e[0][0] - x) < MENZIL_MAKS + 2 and abs(e[0][1] - y) < MENZIL_MAKS + 2]
    kon = [c for c in koniler
           if abs(c[0] - x) < MENZIL_MAKS and abs(c[1] - y) < MENZIL_MAKS]
    inc = 2.0 * math.pi / isin
    amin = -math.pi
    ranges = []
    for i in range(isin):
        a = amin + i * inc + yaw          # dünya çerçevesinde ışın yönü
        d = np.array([math.cos(a), math.sin(a)])
        en = MENZIL_MAKS
        for kutu in yakin:
            t = _obb_isin(p, d, kutu)
            if t is not None and t < en:
                en = t
        for (kx, ky, kr) in kon:           # koni = daire
            f = np.array([x - kx, y - ky])
            b2 = f @ d
            cc = f @ f - kr * kr
            disc = b2 * b2 - cc
            if disc >= 0.0:
                t = -b2 - math.sqrt(disc)
                if 0.0 < t < en:
                    en = t
        ranges.append(en if en < MENZIL_MAKS else float('nan'))
    return ranges, amin, inc


def pozlar_uret(arac, engeller):
    """İstasyondan istasyona planlanmış gerçek yollardan poz örnekler."""
    wp = json.load(open(os.path.join(BURASI, 'waypoint_cad.json')))
    cikti = []
    for i in range(len(wp) - 1):
        a, b = wp[i], wp[i + 1]
        kutu = (min(a['x'], b['x']) - 14, min(a['y'], b['y']) - 4,
                max(a['x'], b['x']) + 4, max(a['y'], b['y']) + 4)
        try:
            yol, _ = donus.yol_var_mi(
                arac, 2.42, (a['x'], a['y'], a['yaw']), (b['x'], b['y'], b['yaw']),
                kutu, engeller, [], maks=900000)
        except ValueError:
            yol = None
        if not yol:
            continue
        for j in range(0, len(yol), 3):
            cikti.append((f"{a['no']}→{b['no']}", yol[j]))
    return cikti


# U dönüşlerinde iç duvarın aracın hangi yanında olduğu. Parkur geometrisinden:
# sol U (y=0 → y=10) sağa dönüştür, sağ U (y=10 → y=20) sola dönüştür.
VIRAJ_IC_DUVAR = {'3→4': 'sag', '6→7': 'sol'}


def viraj_dogrula(arac, engeller, koniler):
    """U dönüşlerinde iç duvar takibini PLANLANMIŞ YOLA karşı ölçer.

    Yer gerçeği planlayıcının ürettiği gerçek sürülebilir yoldur; hedefin o
    yolun ilerideki noktalarına uzaklığı raporlanır. "Hedef duvara düşmedi"
    ölçütü tek başına yetmez: 3-4 m'lik koridorda yanlış duvardan üretilen
    hedef de çarpışmasız çıkar, o yüzden ayırt etmez.
    """
    wp = json.load(open(os.path.join(BURASI, 'waypoint_cad.json')))
    print('\nU DÖNÜŞLERİ — iç duvar takibi (yer gerçeği: planlanmış yol)')
    print(f"{'dönüş':8s}{'iç duvar':>10s}{'poz':>6s}{'sapma ort':>11s}"
          f"{'maks':>8s}{'hedefsiz':>10s}{'duvara':>8s}")
    print('-' * 62)
    for i in range(len(wp) - 1):
        a, b = wp[i], wp[i + 1]
        etiket = f"{a['no']}→{b['no']}"
        if etiket not in VIRAJ_IC_DUVAR:
            continue
        kutu = (min(a['x'], b['x']) - 14, min(a['y'], b['y']) - 4,
                max(a['x'], b['x']) + 4, max(a['y'], b['y']) + 4)
        yol, _ = yol_var_mi_guvenli(arac, a, b, kutu, engeller)
        if not yol:
            print(f'{etiket:8s}  yol planlanamadı')
            continue
        taraf = VIRAJ_IC_DUVAR[etiket]
        sapmalar, yok, duvara = [], 0, 0
        for j in range(0, len(yol), 3):
            poz = yol[j]
            ranges, amin, inc = tarama_uret(poz, engeller, koniler)
            t = ic_duvar_hedefi(ranges, amin, inc, 0.0, taraf)
            if t is None:
                yok += 1
                continue
            x, y, yaw = poz
            hx = x + t[0] * math.cos(yaw) - t[1] * math.sin(yaw)
            hy = y + t[0] * math.sin(yaw) + t[1] * math.cos(yaw)
            sapmalar.append(min(math.hypot(hx - q[0], hy - q[1])
                                for q in yol[j:]))
            if donus.carpisma(arac, hx, hy, yaw + t[2], engeller, []):
                duvara += 1
        n = len(sapmalar) + yok
        ort = sum(sapmalar) / len(sapmalar) if sapmalar else float('nan')
        mak = max(sapmalar) if sapmalar else float('nan')
        print(f'{etiket:8s}{taraf:>10s}{n:6d}{ort:10.2f}m{mak:7.2f}m'
              f'{yok:10d}{duvara:8d}')


def yol_var_mi_guvenli(arac, a, b, kutu, engeller):
    try:
        return donus.yol_var_mi(
            arac, 2.42, (a['x'], a['y'], a['yaw']), (b['x'], b['y'], b['yaw']),
            kutu, engeller, [], maks=900000)
    except ValueError:
        return None, 0


def main():
    arac = donus.Arac()
    engeller = donus.bariyerler() + donus.sanal_duvarlar()
    koniler = donus.cad_konileri()
    pozlar = pozlar_uret(arac, engeller)
    print(f'{len(pozlar)} poz, {len(engeller)} engel, {len(koniler)} koni\n')

    ozet = {}
    duvara_dusen = []
    for etiket, poz in pozlar:
        ranges, amin, inc = tarama_uret(poz, engeller, koniler)
        cizgi = koridor_merkez_cizgisi(ranges, amin, inc, lidar_yaw=0.0)
        hedef = kayan_hedef(cizgi)
        d = ozet.setdefault(etiket, {'n': 0, 'hedef': 0, 'mesafe': [],
                                     'iki': 0, 'tek': 0, 'belirsiz': 0})
        d['n'] += 1
        for _, _, g in cizgi:
            d[{'iki_duvar': 'iki', 'tek_duvar': 'tek',
               'belirsiz': 'belirsiz'}[g]] += 1
        if hedef is None:
            continue
        d['hedef'] += 1
        d['mesafe'].append(hedef[0])
        # Hedef dünya çerçevesine taşınıp koridorda mı diye bakılır.
        x, y, yaw = poz
        hx = x + hedef[0] * math.cos(yaw) - hedef[1] * math.sin(yaw)
        hy = y + hedef[0] * math.sin(yaw) + hedef[1] * math.cos(yaw)
        if donus.carpisma(arac, hx, hy, yaw + hedef[2], engeller, []):
            duvara_dusen.append((etiket, round(hx, 2), round(hy, 2)))

    print(f"{'geçiş':10s}{'poz':>5s}{'hedef':>7s}{'kapsama':>9s}"
          f"{'ort mesafe':>12s}   güven dağılımı (iki/tek/belirsiz)")
    print('-' * 88)
    for etiket, d in ozet.items():
        kap = 100.0 * d['hedef'] / max(1, d['n'])
        ort = sum(d['mesafe']) / len(d['mesafe']) if d['mesafe'] else 0.0
        top = max(1, d['iki'] + d['tek'] + d['belirsiz'])
        print(f"{etiket:10s}{d['n']:5d}{d['hedef']:7d}{kap:8.0f}%{ort:11.2f}m   "
              f"{100*d['iki']//top:3d}% / {100*d['tek']//top:3d}% / "
              f"{100*d['belirsiz']//top:3d}%")
    print('-' * 88)
    n = sum(d['n'] for d in ozet.values())
    h = sum(d['hedef'] for d in ozet.values())
    print(f"TOPLAM kapsama: {100.0*h/max(1,n):.0f}%  ({h}/{n})")
    print(f"Duvara düşen hedef: {len(duvara_dusen)}"
          + (f"  → {duvara_dusen[:5]}" if duvara_dusen else "  ✓"))
    print('\nNOT: merkez çizgisi U dönüşlerinde çalışmıyor (3→4 ve 6→7\n'
          '     satırlarındaki düşük kapsama). Virajlar iç duvar takibiyle\n'
          '     sürülüyor — ölçümü aşağıda.')
    viraj_dogrula(arac, engeller, koniler)


if __name__ == '__main__':
    main()
