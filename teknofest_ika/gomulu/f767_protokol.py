#!/usr/bin/env python3
"""
f767_protokol.py — F767ZI sürüş + sensör kartının arayüz tanımları.

© 2026 dwifk0 — ahmetefenezli@gmail.com. Tüm hakları saklıdır.
LYDIA/MAGNESIA takımının Teknofest çalışmaları kapsamında, bu depo içinde
kullanılmak üzere paylaşılmıştır. Koşullar: donanim/TELIF.md

Bu dosya **yalnız arayüzdür**: paket kodları, alan birimleri ve bayrak
maskeleri. Çözme/gönderme mantığı `seri_kopru.py`de, kartın kendi mantığı ve
kalibrasyonu ise ayrı bir depoda tutulur ve yayınlanmaz — köprü tarafında o
sayılara ihtiyaç yoktur, çünkü komut gerçek fiziksel birimle verilir ve
çevrimi kart yapar.

Sözleşmenin tamamı: `donanim/PROTOKOL_F767.md`

Çerçeve (Mega dönemiyle birebir aynı):
    [0xAA][KOMUT][D0][D1][D2][D3][XOR][0x55]
    D0..D3 = big-endian int16 × 2 (v0, v1)
    XOR    = KOMUT ^ D0 ^ D1 ^ D2 ^ D3
"""

# ─── Çerçeve ──────────────────────────────────────────────────────────────
PKT_BASLA = 0xAA
PKT_BITIS = 0x55
PKT_BOYUT = 8

# ─── Jetson → kart ────────────────────────────────────────────────────────
PKT_SURUCU    = 0x01   # v0 = hız [mm/s] işaretli · v1 = direksiyon [1/100°]
PKT_DUR       = 0x02   # kilit; yalnız taze PKT_SURUCU çözer
PKT_HB        = 0x04   # heartbeat — 700 ms sessizlik güvenli tarafa düşürür
PKT_ESTOP_OUT = 0x07   # v0 = 1 ilan, 0 kaldır
PKT_FREN      = 0x08   # v0 = fren [‰, 0..1000]

# 0x03 lazer · 0x05 pan · 0x06 tilt: REZERVE.
# Taret ayrı Arduino'da; komutları /dev/turret'e metin olarak gider.
# Bu karta gönderilirse sessizce yok sayılır.

# ─── Kart → Jetson (0x30 bloğu) ───────────────────────────────────────────
# 🔴 Mega'nın 0x10–0x23 bloğuyla çakışmasın diye seçildi. Köprü bilmediği
# paketi sessizce attığı için, bu blok tanınmadan kartın telemetrisinin
# HİÇBİRİ Jetson'a ulaşmaz.
PKT_F7_ENK     = 0x30  # v0 = sayım üst 16 · v1 = sayım alt 16  (int32)
PKT_F7_HIZ     = 0x31  # v0 = ileri hız [mm/s]  ← odometrinin ana kaynağı
PKT_F7_IMU_ACI = 0x32  # v0 = yaw ×10 · v1 = roll ×10  [derece]
PKT_F7_IMU_PIT = 0x33  # v0 = pitch ×10 · v1 = BNO055 kalibrasyon baytı
PKT_F7_ESTOP   = 0x34  # v0 = basılı(0/1) · v1 = kanal uyuşmazlığı(0/1)
PKT_F7_SAGLIK  = 0x35  # v0 = arıza bayrakları · v1 = çalışma [sn]
PKT_F7_RC      = 0x36  # v0 = gaz [‰ işaretli] · v1 = sürüş durum baytı
PKT_F7_SURUS   = 0x37  # v0 = gaz çıkışı [mV] · v1 = fren [‰ işaretli]
PKT_F7_JETSON  = 0x38  # v0/v1 = kartın ANLADIĞI hız [mm/s] ve açı [1/100°]
PKT_F7_MOD     = 0x39  # v0 = kip (0/1/2) · v1 = JDR_* bayrakları

F767_ARALIK = range(0x30, 0x40)   # bu aralık F767'ye ayrılmıştır

# ─── Kip (PKT_F7_MOD v0) ──────────────────────────────────────────────────
# Kip anahtarı KUMANDADADIR (SwC/CH9), Jetson'da değil.
# Sıralama: güvenlik > kumanda > Jetson.
MOD_MANUEL = 0   # her şey kumandada
MOD_BOS    = 1   # 🔴 HER ŞEY KİLİTLİ — yumuşak E-STOP: gaz rölanti, fren
                 #    basar, direksiyon donar, Jetson komutu yok sayılır.
                 #    Manuel ile otonom arasında zorunlu DUR kademesi.
MOD_OTONOM = 2   # gaz + fren + direksiyon Jetson'da

MOD_AD = {MOD_MANUEL: "MANUEL", MOD_BOS: "BOS/DUR", MOD_OTONOM: "TAM OTONOM"}

# ─── Link durumu (PKT_F7_MOD v1) ──────────────────────────────────────────
JDR_LINK  = 0x01   # kart Jetson'ı canlı görüyor
JDR_ESTOP = 0x02   # Jetson E-STOP ilan etmişti, kart hâlâ öyle biliyor
JDR_DUR   = 0x04   # kart DUR kilidinde, taze sürüş komutu bekliyor
JDR_ELLE  = 0x08   # elle kip açık — direksiyon Jetson'ı dinlemiyor

# 🔴 JDR_LINK düşükken gönderilen sürüş komutları YOK SAYILIR.
# "Komut gönderiyorum ama araç dinlemiyor" durumunun tek görünür yeri budur.

# ─── Sürüş durum baytı (PKT_F7_RC v1) ─────────────────────────────────────
# "Motor gitmiyor" şikâyetinde hangi kilidin kapalı olduğu buradan okunur.
DRM_KESME = 0x01   # SwA kesme konumunda
DRM_TARET = 0x02   # SwB taret modunda — gaz kilitli
DRM_GERI  = 0x04   # geri vites rölesi çekili
DRM_SSR   = 0x08   # ana güç hattı açık
DRM_GECIS = 0x10   # yön değiştiriliyor, gaz kilitli
DRM_ISIK  = 0x20   # aydınlatma rölesi çekili

# ─── Zamanlama sözleşmesi ─────────────────────────────────────────────────
# Köprünün uyması gereken TEK zamanlama bu. Fren süresi, kalkış darbesi,
# rampa ve stall koruması kartın işidir; köprüde karşılıkları yoktur.
HB_TIMEOUT_MS = 700

# ─── Birimler — komut yönü ────────────────────────────────────────────────
# hız        : mm/s        (int16, işaretli; negatif = geri)
# direksiyon : 1/100 derece (int16; ROS/Ackermann işareti — pozitif SOLA)
# fren       : ‰ 0..1000   (0 serbest, 1000 tam fren)
#
# ⚠ Köprüde OLMAMASI gerekenler: gaz voltajı hesabı, fren zamanlama
#   sabitleri, tick→metre çevrimi, minimum kalkış hızı alt sınırı.
#   Hepsinin karşılığı karttadır.
