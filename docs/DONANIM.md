# Donanım

> LYDİA İKA otonom sürüş yazılımı — ayrıntılı belge.
> Genel bakış için [README](../README.md).

---

## Donanım

| Bileşen | Model / Değer |
|---|---|
| Ana işlemci | NVIDIA Jetson Orin Nano |
| Sürüş kartı | Nucleo-F767ZI — motor, fren, direksiyon, RC, IMU, enkoder |
| Motor sürücü | Pilmak 48 V 1200 W BLDC |
| Gaz DAC | MCP4725 (12 bit) |
| Fren | BTS7960B H-köprü, vidalı aktüatör |
| Direksiyon | Step motor + DM860H sürücü (açık çevrim, homing yok) |
| LiDAR | YDLidar Tmini Pro (2D, 360°, 10Hz) |
| Kamera | IMX258 OIS (×1 ön) + Microcase 720P (×3) |
| Derinlik | Orbbec Gemini 335L (karar verildi, sipariş edilmedi) |
| IMU | BNO055 (NDOF, kartta) |
| Enkoder | E6B2-CWZ6C 600 P/R kuadratür ×4 = 2400 sayım/tur, motor miline 1:1 |
| RC Kumanda | Flysky FS-i6X — CH1 direksiyon · CH2 gaz · CH3 fren · CH7 kesme · CH8 taret · CH9 mod · CH10 ışık |
| E-STOP | Schneider XB5AS84W3B5 — mekanik kontak 48 V'u keser, NC blok karta gider |
| Gövde | 1,90 × 1,16 × 0,76 m |
| Kinematik | Ackermann (otomobil tipi direksiyon) |

⚠ Jetson'ın GPIO'suna E-STOP hattı **bağlı değil**; durum karttan `0x34` ile
geliyor. Gaz voltajı, fren zamanlaması ve kalkış itişi kartın işi — köprüde
karşılıkları yok.

### Ackermann Parametreleri

```
Dingil arası (L)  : 1.44 m       mezürle ölçüldü
Maks. steer açısı : 30° (0.52 rad)  ⚠ mekanik uç ölçülmedi, muhafazakâr sınır
Min. dönüş yarıçapı: L / tan(δ_max) = 2.49 m
```

Direksiyon kolonu ile tekerlek arasındaki oran **13.091** olarak ölçüldü ve karta
`0x09` ayar paketiyle gönderiliyor. Bu oranla kolonun ±750° sınırı tekerde ±57°
demek; iki ölçümden biri tutarsız olduğu için `max_steering_angle` 30°'de
bırakıldı — kelepçenin rahat içinde kalıyor ve `minimum_turning_radius` geçerli.

---
