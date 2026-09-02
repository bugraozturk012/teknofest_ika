# F767ZI ↔ Jetson — arayüz sözleşmesi

> **© 2026 dwifk0 — ahmetefenezli@gmail.com. Tüm hakları saklıdır.**
> LYDIA/MAGNESIA takımının Teknofest çalışmaları kapsamında, bu depo içinde
> kullanılmak üzere paylaşılmıştır. Koşullar: [`TELIF.md`](TELIF.md)

**Sürüm:** 1.0 · **Tarih:** 2026-09-02
**Kapsam:** Nucleo-F767ZI sürüş + sensör kartı ile Jetson arasındaki seri
protokol. Köprü tarafı (`teknofest_ika/gomulu/seri_kopru.py`) bu belgeye
dayanır.

> Bu belge **arayüzü** tanımlar: hangi paket, hangi birim, hangi davranış.
> Kartın içindeki kalibrasyon sayıları, eşikler ve zamanlamalar buraya
> girmez — sahada ölçülerek oturtulur, araç değiştikçe değişir ve tek elden
> yönetilir. **Köprü tarafında bu sayılara ihtiyaç yoktur:** komut gerçek
> fiziksel birimle verilir, çevrimi kart yapar.

---

## 1. Fiziksel yol

| | |
|---|---|
| Bağlantı | Kart ↔ Jetson, **UART** (üç tel: TX, RX, GND) |
| Jetson portu | **`/dev/ttyTHS1`** — sabit ad, udev gerekmez, USB yemez |
| Hız | **115200** baud, 8N1 |

Kablolama donanım tarafındadır ve kurulu gelir; yazılım tarafının bilmesi
gereken tek şey port adı ve hızdır.

⚠ `/dev/ttyTHS1` fabrika çıkışında seri konsola ayrılmış olabilir; port
açılmıyorsa ilk bakılacak yer budur:
`sudo systemctl stop nvgetty && sudo systemctl disable nvgetty`

---

## 2. Çerçeve

```
[0xAA] [KOMUT] [D0] [D1] [D2] [D3] [XOR] [0x55]
```

- Sabit **8 bayt**
- `D0..D3` = **big-endian `int16` × 2** (`v0`, `v1`)
- `XOR` = `KOMUT ^ D0 ^ D1 ^ D2 ^ D3`
- Mega dönemiyle **birebir aynı** — mevcut `paket_olustur` / `paket_dogrula`
  değişmeden çalışır

**Çözücü kuralı:** `0xAA` beklenir, ardından 7 bayt daha okunur; bitiş baytı
veya XOR tutmazsa paket atılır ve senkron aranmaya devam edilir. Yarım kalan
çerçeve kısa bir süre sonra bırakılmalıdır — tek kayıp bayt aksi hâlde
senkronu kalıcı kaydırır.

---

## 3. Kart → Jetson (0x30 bloğu)

🔴 Numaralar Mega'nın `0x10–0x23` bloğuyla **çakışmasın diye** seçilmiştir.
Köprü bilmediği paketi sessizce attığı için, bu blok tanınmadan **kartın
telemetrisinin hiçbiri Jetson'a ulaşmaz.**

| Kod | `v0` | `v1` |
|---|---|---|
| `0x30` | enkoder sayımı (üst 16 bit) | enkoder sayımı (alt 16 bit) — birlikte `int32` |
| `0x31` | **ileri hız [mm/s]**, işaretli | (kullanılmıyor) |
| `0x32` | yaw ×10 [derece] | roll ×10 [derece] |
| `0x33` | pitch ×10 [derece] | BNO055 kalibrasyon baytı |
| `0x34` | E-STOP basılı (0/1) | kanal uyuşmazlığı (0/1) |
| `0x35` | arıza bayrakları | çalışma süresi [saniye] |
| `0x36` | gaz komutu [‰, işaretli] | sürüş durum baytı |
| `0x37` | gaz çıkışı [mV] | fren [‰, işaretli] |
| `0x38` | kartın **anladığı** hız [mm/s] | kartın **anladığı** açı [1/100°] |
| `0x39` | kip (0/1/2) | link durum bayrakları |

### Odometri nereden beslenir

`0x31` **hazır ileri hızı mm/s cinsinden** taşır. Tick → metre çevrimi,
kuadratür çarpanı ve dişli oranı **kart tarafındadır**; köprüde
`TICKS_PER_REV` / `METRE_PER_TICK` benzeri sabitlere karşılık yoktur.

⚠ **Tekerlek çevresi ölçülene kadar bu alan `0` gelir.** Sessizce sıfır
gelmesi arıza değildir, "henüz ölçülmedi" demektir; hata bayrağı çıkmaz.
Odometri boş kalırsa ilk bakılacak yer burasıdır.

### `0x39` — link durum bayrakları

| Bit | Anlam |
|---|---|
| `0x01` | kart Jetson'ı **canlı görüyor** |
| `0x02` | Jetson E-STOP ilan etmişti, kart hâlâ öyle biliyor |
| `0x04` | kart **DUR kilidinde**, taze sürüş komutu bekliyor |
| `0x08` | elle kip (tezgah) açık — direksiyon Jetson'ı dinlemiyor |

---

## 4. Jetson → Kart

| Kod | Ad | `v0` | `v1` |
|---|---|---|---|
| `0x01` | `PKT_SURUCU` | hız **[mm/s]**, işaretli | direksiyon **[1/100 derece]** |
| `0x02` | `PKT_DUR` | — | — |
| `0x04` | `PKT_HB` | — | — |
| `0x07` | `PKT_ESTOP_OUT` | 1 = ilan, 0 = kaldır | — |
| `0x08` | `PKT_FREN` | fren **[‰, 0–1000]** | — |

`0x03` (lazer), `0x05` (pan), `0x06` (tilt) **rezervedir**: taret ayrı bir
Arduino'dadır ve komutları `/dev/turret`e metin olarak gider. Bu kart onları
sessizce yok sayar.

---

## 5. Davranış kuralları

Köprü bu kurallara uymak zorundadır; kart bunları uygular.

1. **Heartbeat şart.** 700 ms boyunca hiçbir paket gelmezse kart güvenli
   tarafa düşer: gaz kesilir, fren basar. 10 Hz heartbeat yeterlidir.
2. **`PKT_DUR` bir kilittir.** Kilidi yalnız **taze bir `PKT_SURUCU`** çözer.
   Bayat komutla araç kalkmaz.
3. **`PKT_DUR` direksiyon açısını sıfırlamaz.** Hareket hâlindeki araçta
   tekerlekleri ortaya kırmak durdurmak değil, yön değiştirmektir. Açı korunur.
4. **Kip anahtarı kumandadadır (SwC/CH9), Jetson'da değil.**
   `0 manuel · 1 yarı (yalnız direksiyon Jetson'da) · 2 tam otonom`.
5. **Sıralama: güvenlik > kumanda > Jetson.** Kesme, sinyal kaybı ve E-STOP
   Jetson komutunun üstündedir ve her zaman kazanır.
6. **Fren:** otonom kipte kumanda her zaman **üstüne basabilir** — kartta
   ikisinin büyüğü alınır. İnsan fren ekleyebilir, kaldıramaz.

### Birim ve çevrim sorumluluğu

| İş | Nerede |
|---|---|
| m/s → gaz voltajı | **kart** |
| derece → adım | **kart** |
| ‰ → fren PWM | **kart** |
| tick → metre | **kart** |
| Nav2 / Ackermann → m/s + derece | köprü |

**Köprüde bulunmaması gerekenler:** gaz voltajı hesabı, fren zamanlama
sabitleri, tick→metre çevrimi, minimum kalkış hızı alt sınırı. Hepsinin
karşılığı karttadır; iki yerde tutulursa her kalibrasyonda iki yerde birden
düzeltme gerekir ve biri unutulur.

---

## 6. Teşhis

Sorun aramanın sırası:

1. **`0x39` link bayrağı düşük mü?** Düşükse kart Jetson'ı canlı görmüyordur
   ve **komutlar yok sayılıyordur.** "Komut gönderiyorum ama araç dinlemiyor"
   durumunun tek görünür yeri burasıdır.
2. **`0x38` ile gönderdiğin `0x01` aynı mı?** Aradaki fark tek başına
   teşhistir: ölçek hatası, işaret hatası ve kayıp paket burada görünür.
3. **`0x36` durum baytı.** "Motor gitmiyor" şikâyetinde hangi kilidin kapalı
   olduğu buradan okunur — tahmine gerek yoktur.
4. **`0x35` arıza bayrakları.**

---

## 7. Bilinen açıklar

- 🔴 **Direksiyon işareti doğrulanmadı.** ROS/Ackermann'da pozitif
  `steering_angle` **sola** dönüştür; çeviri kartta yapılır ama araçta henüz
  sınanmadı. İlk otonom denemede tekerlekler yerden kesik, küçük pozitif açı
  verilip yön gözlenecek.
- 🔴 **Direksiyonda referans yok.** Homing ve limit anahtarı yoktur; açılış
  konumu "düz" kabul edilir. Karta güç verilmeden önce tekerlekler elle düz
  konuma getirilmiş olmalıdır.
- 🔴 **Tekerlek çevresi ölçülmedi** → `0x31` sıfır gelir (§3).
- ⚠ **Alıcı, verici kapalıyken de yayın yapar.** "Çerçeve gelmiyorsa dur"
  mantığı bu araçta kopuk kabloya karşı çalışır, kapanan vericiye karşı
  çalışmaz. Sinyal kaybını yakalayan tek şey alıcının failsafe kaydıdır.
