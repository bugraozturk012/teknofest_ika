# Parkur CAD → waypoint çıkarımı

TEKNOFEST'in yayımladığı parkur CAD paketinden (`20260604_Parkur`) tabela ve
engel konumlarını, oradan da `config/waypoints.yaml`'ın `waypoint_cad`
alanlarını üreten betikler.

```
python3 parkur.py <parkur.STEP> duzeltilmis.json   # montajı çöz
python3 wp.py                                       # şerit ortası + waypoint
python3 donus.py                                    # dönüş yarıçapı + koni slalomu
```

`wp.py` ve `donus.py` bulunduğu dizindeki `duzeltilmis.json`'ı okur; o dosya
STEP ayrıştırmasının tek ürünü olduğu için repoda tutuluyor.

## donus.py — geçilebilirlik

Aracın dikdörtgenini eğriliği sınırlı, geri vitessiz yollar boyunca süpürüp
engellerle tam dikdörtgen-dikdörtgen testi yapar. Üç şeyi bilmek gerekir:

**Şerit ortası noktaları virajda kullanılamaz.** `wp.py`'nin "karşı duvarda
2–4,2 m mesafede dik eş ara" kuralı düz şeritte çalışır, virajda noktaları dış
duvara yapıştırır. `donus.py` koridoru bariyerlerden yeniden kurar.

**Duvar boşlukları kapatılmalı.** Yan eğim (x −20,0…−12,7) ve dik eğim
(x −19,9…−11,2) yapılarının bulunduğu yerde bariyer dizisi kesiliyor. Açık
bırakılırsa planlayıcı koridordan çıkıp anlamsız kısayol bulur ve sol U dönüşü
olduğundan geniş görünür. §6.1'in 3 m şerit sınırında sanal duvarla kapatılıyor.

**Koni ölçüsü CAD'den değil şartnameden alınır.** CAD'deki modeller 52 cm taban
/ 96 cm boy, §6.7 ise 40±10 cm kare taban / 75±5 cm — yani CAD konileri jenerik
yer tutucu. §6.7 yerleşimi de yarışma gününe bıraktığı için CAD'deki dizilim
bağlayıcı değil; `donus.py` bu yüzden tek bir dizilimi değil, geçilebilir
dizilimlerin sınırını çıkarır.

## Bilinmesi gerekenler

**Eksen.** CAD'de zemin X–Z düzlemi, düşey eksen **Y ve yukarı pozitif**;
birim mm. ROS'a çevrim: `x = X_cad`, `y = -Z_cad`, `z = Y_cad`.

**Dönüşüm yönü ürün ağacından belirlenir.** `CONTEXT_DEPENDENT_SHAPE_-`
`REPRESENTATION`'ın bağladığı iki gösterimden hangisinin ebeveyne ait olduğu
`SHAPE_DEFINITION_REPRESENTATION` üzerinden bulunuyor. Sabit sıra varsaymak
(a1=çocuk, a2=ebeveyn) bu dosyada 541 yerleşimin 541'ini de ters çeviriyor ve
tüm model orijinden aynalanıyor — aynalanmış bir oval hâlâ oval göründüğü için
gözle fark edilmiyor.

**Bileşik varlıklar.** Dönüşümler `#N =( TIP1(...) TIP2(...) )` biçiminde
saklanıyor. Yalnız `#N = TIP(...)` eşleyen bir regex bunları sessizce atlar ve
sonuç boş çıkar, hata vermez.

**Geometri ayrı gösterimde.** Parçanın `SHAPE_REPRESENTATION`'ı yalnız yerleşim
eksenlerini taşıyor; BREP `ADVANCED_BREP_SHAPE_REPRESENTATION`'da ve ikisini
dönüşümsüz bir `SHAPE_REPRESENTATION_RELATIONSHIP` bağlıyor. Bağ kurulmazsa her
parçanın sınır kutusu tek noktaya çöker.

**Bariyerin yerel eksen 0'ı uzunluk DEĞİL, kalınlık.** Yol yönü dönüş
matrisinden değil, aynı duvardaki komşu bariyerden alınmalı.

## Sağlama kuralı

Grup içi tutarlılık yetmez — katı bir dönüşüm hatası ondan sağ çıkar. Dış
referansa bakan sağlamalar kullanılmalı; bu dosyada hepsi tutuyor:

| sağlama | beklenen | ölçülen |
|---|---|---|
| levha direğin tepesinde | ~2,1 m | 1,83…2,17 |
| dik engel yüksekliği (§6.6) | 15 cm | 0,15 m |
| rampa eğimi (§6.10) | %45 | 1,86 / 4,13 |
| bariyer yüksekliği (§6.1) | 80±10 cm | 0,86 m |
| yol genişliği (§6.1) | 3 m | 3,26 (bariyer dahil) |
| tabela dış çapı (§6.2) | 60 cm | 600 mm |
| hızlanma bölümü (§6.11) | 30 m | 31,0 m |
| atış mesafesi (§6.10) | ≥10 m | 10,92 m |
