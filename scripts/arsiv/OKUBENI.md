# Arşiv — reaktif sürüş düğümleri

Buradaki iki betik **haritasız, LiDAR-reaktif** sürüş yapar: global konum,
odometri, harita ya da waypoint istemezler; her taramada sıfırdan karar
verirler. Yarışma yığınının (Nav2 + `misyon_fsm` + `terrain_adapter`) hiçbir
parçasıyla bağlantıları yoktur.

| dosya | ne yapar | saha durumu |
|---|---|---|
| `huni_reaktif.py` | Huni koridorunda boşluk seçip geçer | 2026-07-28: 19 s / ~12 m tam tur tamamlandı |
| `hareketli_engel.py` | Kayar engel istasyonunda durup bekler | Bir kez sürüldü, duruş mesafesi kalibre edilmedi |

## Neden arşivde

Sürüş Nav2 yoluna geçirildi. Bu iki düğüm **boot betiğinde hiçbir zaman
yoktu**, sahada hep elle başlatılıyorlardı — ve tam olarak bu yüzden buraya
alındılar: ezberden çalıştırılan komut artık bir şey başlatmıyor.

**Çakışma sebebi:** ikisi de `/cmd_vel`'e yazar. Nav2 controller ve
`misyon_fsm`'in `HizlanmaState`'i de aynı topic'e yazar. `mod_yoneticisi` bu
topic'i dinleyip son gelen mesajı geçirdiği için, Nav2 çalışırken bunlardan
biri başlatılırsa araç iki karar arasında yalpalar. Her iki düğüm de logda
sağlıklı görünür; teşhisi zordur.

## Geri dönmek gerekirse

Nav2 sahada tökezlerse çalışan yola dönüş yolu açık. Önce Nav2 zincirini
kapat (boot betiğinde `NAV2_AKTIF=0`), sonra:

    ros2 run teknofest_ika ... # önce yığın ayakta olmalı
    python3 scripts/arsiv/huni_reaktif.py --ros-args \
        -p max_hiz:=0.65 -p min_hiz:=0.45

Sahada doğrulanmış iki ayar takımı:

- **Genel parkur:** `min_hiz 0.45  max_hiz 0.65` — 114 karar, %100 aday,
  0 kaçış ile tam tur.
- **Zikzak / 2 m koridor:** `max_hiz 0.50  min_hiz 0.45
  yeterli_bosluk_m 0.60  balon_esik_m 3.0  on_durma_m 1.0`

## Bilinen tuzaklar

- `bitis_kilit=True` **kalıcı bir latch**. Parkur bitti sayıldığında düğüm
  kilitlenir ve manuelde bu durum logda görünmez; "komut gitmiyor" sanılan
  tablonun ilk şüphelisidir.
- Parametreler çalışma anında değişmez — düğümü yeniden başlat.
- `huni_reaktif` LiDAR açı düzeltmesini **kendi içinde** yapar
  (`aci_offset_deg`), TF'e bakmaz. Nav2 ise yalnız TF'e bakar. Birinde yapılan
  açı düzeltmesi diğerini etkilemez.
- `hareketli_engel.py` duruşu ışın SAYISINA göre tetikliyor; ölçümde 2.80 m
  yerine 1.83 m'de durdu. Kullanılacaksa önce bu kalibre edilmeli.

Buradaki sürümler Jetson'da sahada koşan kopyalardır
(`~/jetson_yedek_20260804/`, 2026-08-04) — depo kökündeki eski sürümler değil.
