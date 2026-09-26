# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""STEP montajını çözer, her bileşenin dünya konumunu ve gerçek geometri sınır
kutusunu verir.

Dönüşüm yönü ÜRÜN AĞACINDAN belirlenir: CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'ın
bağladığı iki gösterimden hangisinin ebeveyne, hangisinin çocuğa ait olduğu
SHAPE_DEFINITION_REPRESENTATION üzerinden bulunur. Sabit sıra varsaymak,
yerleşimlerin bir kısmını ters çevirir (levha direğinin altına düşer).
"""
import re, sys, json, math
from collections import defaultdict

ENT   = re.compile(r'#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*)\)\s*;\s*$', re.S)
KARMA = re.compile(r'#(\d+)\s*=\s*\((.*)\)\s*;\s*$', re.S)   # bileşik varlık
REF   = re.compile(r'#(\d+)')
SAYI  = re.compile(r'-?\d+\.?\d*(?:[eE][-+]?\d+)?')

def oku(yol):
    E, buf = {}, ''
    with open(yol, encoding='latin-1') as f:
        for satir in f:
            buf += satir.rstrip('\n')
            if not buf.rstrip().endswith(';'):
                continue
            g = buf.strip(); buf = ''
            m = ENT.match(g)
            if m:
                E[int(m.group(1))] = (m.group(2), m.group(3))
            else:
                k = KARMA.match(g)
                if k:
                    E[int(k.group(1))] = ('__KARMA__', k.group(2))
    return E

def argbol(s):
    p, d, t, c = [], 0, False, ''
    for ch in s:
        if t:
            c += ch
            if ch == "'": t = False
            continue
        if ch == "'": t = True; c += ch
        elif ch == '(': d += 1; c += ch
        elif ch == ')': d -= 1; c += ch
        elif ch == ',' and d == 0: p.append(c.strip()); c = ''
        else: c += ch
    if c.strip(): p.append(c.strip())
    return p

def refler(s): return [int(x) for x in REF.findall(s)]

def metin(s):
    m = re.search(r"'((?:[^']|'')*)'", s)
    if not m: return ''
    def cz(mm):
        h = mm.group(1)
        return ''.join(chr(int(h[i:i+4], 16)) for i in range(0, len(h), 4))
    return re.sub(r'\\X2\\([0-9A-Fa-f]+)\\X0\\', cz, m.group(1))

YOL = sys.argv[1]
print('okunuyor...', file=sys.stderr)
E = oku(YOL)
print(f'{len(E)} varlık', file=sys.stderr)
def tip(i): return E[i][0] if i in E else None

# ── geometri ────────────────────────────────────────────────────────────────
def nokta(i):
    return [float(x) for x in SAYI.findall(argbol(E[i][1])[-1])]

def matris(i):
    a = argbol(E[i][1]); rf = refler(','.join(a[1:]))
    o = nokta(rf[0])
    z = nokta(rf[1]) if len(rf) > 1 else [0, 0, 1]
    x = nokta(rf[2]) if len(rf) > 2 else None
    def norm(v):
        n = math.sqrt(sum(c*c for c in v));  return [c/n for c in v] if n else [0, 0, 1]
    z = norm(z)
    if x is None:
        x = [1, 0, 0] if abs(z[0]) < 0.9 else [0, 1, 0]
    d = sum(a1*b for a1, b in zip(x, z))
    x = norm([a1 - d*b for a1, b in zip(x, z)])
    y = [z[1]*x[2]-z[2]*x[1], z[2]*x[0]-z[0]*x[2], z[0]*x[1]-z[1]*x[0]]
    return [[x[0], y[0], z[0], o[0]], [x[1], y[1], z[1], o[1]],
            [x[2], y[2], z[2], o[2]], [0, 0, 0, 1]]

def carp(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def ters(M):
    R = [[M[i][j] for j in range(3)] for i in range(3)]
    t = [M[i][3] for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    nt = [-sum(Rt[i][k]*t[k] for k in range(3)) for i in range(3)]
    return [Rt[0]+[nt[0]], Rt[1]+[nt[1]], Rt[2]+[nt[2]], [0, 0, 0, 1]]

BIRIM = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]

# ── ürün ağacı ──────────────────────────────────────────────────────────────
urun, form, pd_ad = {}, {}, {}
nauo, pds_hedef, idt = {}, {}, {}
sdr, cdsr, rrwt = {}, [], {}
for i, (t, a) in E.items():
    if   t == 'PRODUCT':                       urun[i] = metin(a)
    elif t.startswith('PRODUCT_DEFINITION_FORMATION'):
        r = refler(a);  form[i] = r[0] if r else None
    elif t == 'NEXT_ASSEMBLY_USAGE_OCCURRENCE':
        r = refler(a);  nauo[i] = (r[0], r[1])
    elif t == 'PRODUCT_DEFINITION_SHAPE':
        r = refler(a)
        if r: pds_hedef[i] = r[0]
    elif t == 'ITEM_DEFINED_TRANSFORMATION':
        r = refler(a);  idt[i] = (r[0], r[1])
    elif t == 'SHAPE_DEFINITION_REPRESENTATION':
        r = refler(a)
        if len(r) >= 2: sdr[r[0]] = r[1]
    elif t == 'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION':
        r = refler(a);  cdsr.append((r[0], r[1]))
for i, (t, a) in E.items():
    if t == 'PRODUCT_DEFINITION':
        r = refler(a)
        if r: pd_ad[i] = urun.get(form.get(r[0]), f'?{i}')
    if 'REPRESENTATION_RELATIONSHIP' in t or 'REPRESENTATION_RELATIONSHIP' in a[:400]:
        tr = [x for x in refler(a) if x in idt]
        if tr: rrwt[i] = (tr[0], [x for x in refler(a) if x in sdr.values() or tip(x)])

# gösterim ↔ ürün: parçanın SR'si + ona dönüşümsüz SRR ile bağlı geometri gösterimleri
rep_bag = defaultdict(set)
for i, (t, a) in E.items():
    if t == 'SHAPE_REPRESENTATION_RELATIONSHIP' and i not in rrwt:
        r = refler(a)
        if len(r) >= 2:
            rep_bag[r[0]].add(r[1]); rep_bag[r[1]].add(r[0])

rep_pd = {}
for pds, hedef in pds_hedef.items():
    if hedef in pd_ad and pds in sdr:
        kok = sdr[pds]
        yig, gor = [kok], set()
        while yig:
            r = yig.pop()
            if r in gor: continue
            gor.add(r); rep_pd[r] = hedef
            yig.extend(rep_bag.get(r, ()))

# ── yerleşimler: yön ürün ağacından ─────────────────────────────────────────
cocuk = defaultdict(list)
duz = ters_sayi = belirsiz = 0
for rr, pds in cdsr:
    n = pds_hedef.get(pds)
    if n not in nauo or rr not in rrwt:
        continue
    ebeveyn, cocuk_pd = nauo[n]
    t_id = rrwt[rr][0]
    a1, a2 = idt[t_id]
    r1, r2 = [x for x in refler(E[rr][1]) if x in rep_pd][:2] or [None, None]
    p1, p2 = rep_pd.get(r1), rep_pd.get(r2)
    if p1 == cocuk_pd and p2 == ebeveyn:
        T = carp(matris(a2), ters(matris(a1))); duz += 1
    elif p1 == ebeveyn and p2 == cocuk_pd:
        T = carp(matris(a1), ters(matris(a2))); ters_sayi += 1
    else:
        T = carp(matris(a2), ters(matris(a1))); belirsiz += 1
    cocuk[ebeveyn].append((cocuk_pd, T))

print(f'yerleşim: {duz} düz, {ters_sayi} TERS, {belirsiz} belirsiz', file=sys.stderr)

# ── geometri sınır kutusu ───────────────────────────────────────────────────
GEO = {'ADVANCED_BREP_SHAPE_REPRESENTATION', 'MANIFOLD_SURFACE_SHAPE_REPRESENTATION',
       'GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION'}
pd_rep = {}
for pds, hedef in pds_hedef.items():
    if hedef in pd_ad and pds in sdr:
        pd_rep.setdefault(hedef, sdr[pds])

def geo_repler(rep):
    gor, yig, out = set(), [rep], []
    while yig:
        r = yig.pop()
        if r in gor: continue
        gor.add(r)
        if tip(r) in GEO: out.append(r)
        yig.extend(rep_bag.get(r, ()))
    return out

def yerel_bbox(rep):
    gor, yig = set(), [rep]
    mn = [1e18]*3; mx = [-1e18]*3; n = 0
    while yig:
        i = yig.pop()
        if i in gor or i not in E: continue
        gor.add(i)
        t, a = E[i]
        if t == 'CARTESIAN_POINT':
            v = nokta(i)
            if len(v) == 3:
                n += 1
                for k in range(3):
                    mn[k] = min(mn[k], v[k]); mx[k] = max(mx[k], v[k])
            continue
        yig.extend(refler(a))
    return (mn, mx, n) if n else None

onb = {}
def bbox_of(pd):
    rep = pd_rep.get(pd)
    if rep is None: return None
    if rep not in onb:
        tot = None
        for g in geo_repler(rep):
            b = yerel_bbox(g)
            if not b: continue
            if tot is None: tot = [list(b[0]), list(b[1]), b[2]]
            else:
                for k in range(3):
                    tot[0][k] = min(tot[0][k], b[0][k]); tot[1][k] = max(tot[1][k], b[1][k])
                tot[2] += b[2]
        onb[rep] = tot
    return onb[rep]

# ── ağacı gez ───────────────────────────────────────────────────────────────
cocuk_olan = {c for v in cocuk.values() for c, _ in v}
kokler = [p for p in cocuk if p not in cocuk_olan]
print('kök:', [pd_ad.get(k) for k in kokler], file=sys.stderr)

def uygula(M, p): return [sum(M[i][j]*p[j] for j in range(3)) + M[i][3] for i in range(3)]

sonuc = []
def gez(pd, M, yol, derinlik=0):
    if derinlik > 20: return
    if pd not in cocuk:
        k = {'ad': pd_ad.get(pd, '?'), 'yol': yol,
             'orijin': [M[0][3], M[1][3], M[2][3]],
             # Dönüş bloğu: parçanın yerel eksenlerinin dünyadaki yönü. Bariyerin
             # uzun ekseni yolun yönü olduğu için yaw buradan okunuyor.
             'R': [M[i][j] for i in range(3) for j in range(3)]}
        bb = bbox_of(pd)
        if bb:
            mn, mx, n = bb
            kose = [[mn[0] if b & 1 else mx[0], mn[1] if b & 2 else mx[1],
                     mn[2] if b & 4 else mx[2]] for b in range(8)]
            dd = [uygula(M, c) for c in kose]
            k['min'] = [min(p[j] for p in dd) for j in range(3)]
            k['max'] = [max(p[j] for p in dd) for j in range(3)]
            k['nokta'] = n
        sonuc.append(k)
        return
    for c, T in cocuk[pd]:
        gez(c, carp(M, T), yol + '/' + pd_ad.get(c, '?'), derinlik + 1)

for k in kokler:
    gez(k, BIRIM, pd_ad.get(k, 'KOK'))
json.dump(sonuc, open(sys.argv[2], 'w'), ensure_ascii=False)
print(f'{len(sonuc)} bileşen → {sys.argv[2]}', file=sys.stderr)
