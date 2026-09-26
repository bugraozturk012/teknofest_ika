# Görev durum makinesi ve parkur

> LYDİA İKA otonom sürüş yazılımı — ayrıntılı belge.
> Genel bakış için [README](../README.md).

---

## Görev Durum Makinesi

**FSM Durum Açıklamaları:**

| Durum | Açıklama |
|---|---|
| IDLE | `/mission_start True` bekler |
| NAVIGATE | sıradaki aşamayı sürer (hedefleme yolu aşağıda) |
| SHOOT_APPROACH | ATIS waypoint'ine yaklaşır, hedef onayı bekler |
| SHOOT | `hedef_hata ≤ ±5px` ise `/shoot_command True` |
| HIZLANMA | Nav2 bypass — direkt /cmd_vel, 10m/s, 30m mesafe |
| ERROR_RECOVERY | navigasyon hatası → 2 deneme, sonra sonraki aşama |

**Hedefleme Yolu (`hedefleme_modu`)**

NAVIGATE aşamayı iki yoldan biriyle sürer. Seçim koşu başında bir kez yapılır
ve gerekçesiyle loglanır.

| mod | hedef nereden | aşama ne zaman biter | ön koşul |
|---|---|---|---|
| `harita` | `waypoints.yaml` → `waypoint:` (map çerçevesi) | Nav2 hedefe varınca | `parkur_cad.donusum` ölçülmüş ve koordinatlar doldurulmuş olmalı |
| `kayan` | her 1,5 s'de LiDAR taramasından üretilir | `mesafe_m` kadar yol kat edilince | odometri (enkoder) |
| `oto` | waypoint'ler doluysa `harita`, hepsi (0,0) ise `kayan` | — | — |

Kayan hedef haritaya da waypoint koordinatına da ihtiyaç duymaz: hedef araç
çerçevesinde doğar, `odom` çerçevesine taşınıp Nav2'ye verilir ve bir sonraki
döngüde yenisiyle değiştirilir. Düz şeritte koridorun merkez çizgisi, U
dönüşünde aşamanın `viraj` alanında yazan iç duvar takip edilir; merkez çizgisi
hedef veremediği anda iç duvara düşülür.

Sahada değiştirmek için:

```bash
HEDEFLEME_MODU=kayan NAV2_AKTIF=1 ENKODER_AKTIF=1 ./scripts/lydia_startup.sh
```

**Özel Durumlar:**
- `DIK_EGIM_GIRIS/CIKIS`: STOP tabelasında 2s dur (Şartname §6.10). STOP
  tabelası kaçırılırsa waypoint'e varışta konum-tabanlı yedek 2s bekleme
  otomatik devreye girer — tek tetikleyici görüntü tespitine bağımlı değildir.
- `KAYAR_ENGEL`: Yön tespiti için max 10s bekle (hız bileşeniyle öngörülür)
- `HIZLANMA`: Tabela_11 → max hız, Tabela_11_son → dur (§6.11)
- `SHOOT`: ateşten sonra hareket kilidi en az `LASER_FIRE_DURATION` (1s)
  yazılım seviyesinde garanti edilir, ardından kilit açılır

---

## Parkur Aşamaları

Aşama, waypoint koordinatına varışla değil **`mesafe_m` kadar yol kat edilmesiyle**
biter. Mesafeler parkur CAD'inden planlayıcıyla ölçüldü (kuş uçuşu değil, gerçek yol
uzunluğu); süreler o mesafelerden türetildi.

| Sıra | Aşama | Mesafe | Süre | Arazi | Özel |
|---|---|---|---|---|---|
| 1 | SULU_YOL | 10,0 m | 50 s | wet | — |
| 2 | TASLI_YOL | 7,2 m | 40 s | gravel | — |
| 3 | YAN_EGIM | 5,0 m | 25 s | slope | — |
| 4 | DIK_ENGEL | 22,0 m | 90 s | obstacle | Nav2 kaçınır |
| 5 | KONİLİ_YOL | 5,0 m | 25 s | normal | Nav2 costmap |
| 6 | KAYAR_ENGEL | 9,8 m | 65 s | normal | Yön beklemesi |
| 7 | ENGEBELİ_ARAZİ | 15,7 m | 65 s | rough | §6.9 koridor ortalaması |
| 8 | DIK_EGIM_GIRIS | 13,0 m | 65 s | rough | **2 s dur** · Nav2 baypas |
| 9 | ATIS_BOLGESI | 2,5 m | 25 s | slow | Nişan onayı → ateş |
| 10 | DIK_EGIM_CIKIS | 2,5 m | 25 s | normal | **2 s dur** · Nav2 baypas |
| 11 | HIZLANMA_PARKURU | 19,2 m | 65 s | fast | Nav2 baypas · 30 m puanlanan + 10 m durma payı |

Toplam 111,9 m yol, 540 s aşama bütçesi; koşu süresi tavanı 900 s.

> ⚠️ `waypoints.yaml`'daki `waypoint:` koordinatları 11/11 hâlâ (0,0). Kayan hedef
> modu koordinat kullanmadığı için bu bloker değil — aşamalar mesafeyle bitiyor.
> `waypoint_cad:` alanları ise parkur CAD'inden çıkarılmış durumda.

---
