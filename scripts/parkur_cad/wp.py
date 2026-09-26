# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""Bariyerlerden şerit ortasını ve tabelalardan waypoint'leri çıkarır.
CAD çerçevesinde — saha dönüşümü uygulanmadan Nav2'de kullanılamaz.

Yol yönü parçanın dönüş matrisinden DEĞİL, aynı duvardaki komşu bariyerden
alınıyor: bariyerin yerel eksenleri uzunlukla hizalı değil (eksen 0 kalınlık),
komşu mesafesi ise §6.1'in sürekli bariyer dizisinden doğrudan geliyor.
"""
import json, math, collections

d = json.load(open('duzeltilmis.json'))
def m(v): return v / 1000.0
def ros_xy(i):   # CAD → ROS: x = X_cad, y = -Z_cad
    return (m((i['min'][0] + i['max'][0]) / 2), -m((i['min'][2] + i['max'][2]) / 2))

g = collections.defaultdict(list)
for x in d: g[x['ad']].append(x)

bar = [ros_xy(i) for ad, v in g.items() if 'bariyer' in ad for i in v]
print(f"bariyer: {len(bar)}")

def uzak(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])

# ── 1) her bariyerin yol yönü: aynı duvardaki en yakın komşu ────────────────
yon = []
for i, p in enumerate(bar):
    k = min((j for j in range(len(bar)) if j != i), key=lambda j: uzak(p, bar[j]))
    dx, dy = bar[k][0]-p[0], bar[k][1]-p[1]
    L = math.hypot(dx, dy) or 1.0
    yon.append((dx/L, dy/L))

# ── 2) karşı duvar eşi → şerit ortası ───────────────────────────────────────
YOL_GEN = 3.0                                   # §6.1
orta = []
for i, p in enumerate(bar):
    u = yon[i]
    en, ej = None, None
    for j, q in enumerate(bar):
        if i == j: continue
        dx, dy = q[0]-p[0], q[1]-p[1]
        L = math.hypot(dx, dy)
        if not (2.0 <= L <= 4.2): continue
        if abs((dx*u[0] + dy*u[1]) / L) > 0.35: continue   # yola DİK olmalı
        s = abs(L - YOL_GEN)
        if en is None or s < en: en, ej = s, j
    if ej is not None:
        q = bar[ej]
        orta.append(((p[0]+q[0])/2, (p[1]+q[1])/2, u[0], u[1]))
tekil = []
for p in orta:
    if not any(uzak(p, q) < 0.25 for q in tekil): tekil.append(p)
orta = tekil
print(f"şerit ortası noktası: {len(orta)}")
if orta:
    dd = [min(uzak(p, b) for b in bar) for p in orta]
    print(f"  ortadan duvara: {min(dd):.2f} .. {max(dd):.2f} m  (beklenen ~1,5)")

# ── 3) tabela → en yakın şerit ortası ───────────────────────────────────────
tab = {}
for ad, v in g.items():
    if not ad.startswith('İKA6-A-001-002-00'): continue
    et = ad.replace('İKA6-A-001-002-00', '').lstrip('_') or '1'
    tab.setdefault(et, []).append(ros_xy(v[0]))

ISTASYON = [('1','SULU_YOL'),('2','TASLI_YOL'),('3','YAN_EGIM'),('4','DIK_ENGEL'),
            ('5','KONİLİ_YOL'),('6','KAYAR_ENGEL'),('7','ENGEBELİ_ARAZİ'),
            ('8','DIK_EGIM_GIRIS'),('9','ATIS_BOLGESI'),('10','DIK_EGIM_CIKIS'),
            ('11','HIZLANMA_PARKURU')]
# Tabelayı şeridin ortasına DİK İZDÜŞÜMLE taşı. En yakın orta noktaya yapışmak
# yol boyunca ±0,8 m kuantize ediyor ve bariyer dizisinin kesildiği yerde
# (rampa) iki istasyonu aynı noktaya düşürüyor.
YARI = (YOL_GEN + 0.36) / 2          # duvar merkezinden şerit ortasına [m]
ham = []
for no, isim in ISTASYON:
    tp = tab[no][0]
    i = min(range(len(bar)), key=lambda j: uzak(tp, bar[j]))
    b, u = bar[i], yon[i]
    t = (tp[0]-b[0])*u[0] + (tp[1]-b[1])*u[1]          # yol boyunca izdüşüm
    px, py = b[0] + t*u[0], b[1] + t*u[1]
    n = (-u[1], u[0])
    aday = [(px + YARI*n[0], py + YARI*n[1]), (px - YARI*n[0], py - YARI*n[1])]
    q = max(aday, key=lambda c: uzak(c, tp))           # yol, tabelanın karşı tarafı
    ham.append({'no': no, 'isim': isim, 'x': q[0], 'y': q[1],
                'u': u[0], 'v': u[1], 'kayma': uzak(tp, q)})

for k, w in enumerate(ham):
    hedef = tab['11 STOP'][0] if w['no'] == '11' else \
            ((ham[k+1]['x'], ham[k+1]['y']) if k+1 < len(ham) else (ham[k-1]['x'], ham[k-1]['y']))
    ix, iy = hedef[0]-w['x'], hedef[1]-w['y']
    if w['no'] == '10': ix, iy = w['x']-ham[k-1]['x'], w['y']-ham[k-1]['y']
    if w['u']*ix + w['v']*iy < 0: w['u'], w['v'] = -w['u'], -w['v']
    w['yaw'] = math.atan2(w['v'], w['u'])

ROBOT_R = 0.50
print(f"\n{'no':4s}{'istasyon':17s}{'x':>9s}{'y':>9s}{'yaw°':>8s}{'tabeladan':>10s}{'duvara':>8s}")
print('-'*66)
for w in ham:
    w['duvar'] = min(uzak((w['x'], w['y']), b) for b in bar)
    print(f"{w['no']:4s}{w['isim']:17s}{w['x']:9.3f}{w['y']:9.3f}"
          f"{math.degrees(w['yaw']):8.1f}{w['kayma']:10.2f}{w['duvar']:8.2f}"
          f"  {'✓' if w['duvar'] >= ROBOT_R else '✗ DAR'}")
json.dump(ham, open('waypoint_cad.json','w'), ensure_ascii=False, indent=1)
json.dump([{'x':p[0],'y':p[1]} for p in orta], open('serit_ortasi.json','w'))
print("\nwaypoint_cad.json + serit_ortasi.json yazıldı")
