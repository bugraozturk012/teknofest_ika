# Ölçümler ve bekleyen kalibrasyonlar

> LYDİA İKA otonom sürüş yazılımı — ayrıntılı belge.
> Genel bakış için [README](../README.md).

---

## Ölçümler ve bekleyen kalibrasyonlar

### Ölçülenler

| Ölçüm | Değer | Nerede |
|---|---|---|
| Dingil arası | **1,44 m** | `ackermann_converter.py`, `urdf/arac.urdf`, `nav2_params.yaml`, `gercek_arac.launch.py` — dördü senkron |
| Min. dönüş yarıçapı | **2,49 m** | `nav2_params.yaml` — dingil arasından türer |
| Gövde ölçüleri | **1,83 × 1,22 m** | Nav2 footprint (yarım en 0,610 + 0,05 padding) |
| Direksiyon kolon/teker oranı | **13,091** | karta `0x09` ayarıyla gönderiliyor |
| LiDAR tarama düzlemi | zeminden **0,60 m** | `lydia_startup.sh` statik TF, `urdf/arac.urdf` ile tutarlı |
| LiDAR montaj açısı | **93,3°** | sahada huniyle çift yönlü kalibre edildi |
| Aşama mesafeleri | 11/11 dolu | parkur CAD'inden planlayıcıyla ölçüldü |

### Bekleyenler

| Görev | Nerede | Not |
|---|---|---|
| 🔴 **Odometri ölçeği** | kart `0x09`: `tekerlek_cevre_mm` + `gosterge_darbe_tur` | **En kritik madde.** Kart hız alanını (`0x31`) ölçek girilmeden bilerek `0` basıyor; `/odom` → EKF → aşama mesafesi zinciri buna bağlı. Çevre **yük altında** ölçülmeli: havalı lastik çöktüğü için geometrik `π·d` gerçekten %2–4 büyük. ⚠ Kart hızı iki kaynaktan alıyor (mil enkoderi ya da gösterge ucu) ve canlı olanı kendisi seçiyor; hangisinin bağlı olduğu aracı iterek `/enkoder/ham` sayımından anlaşılır. Sayım dönerken hız alanı 0 kalıyorsa köprü **"ÖLÇEK UYGULANMIYOR"** basar |
| 🔴 **BNO055 yaw doğrulaması** | — | EKF'in tek yön kaynağı. Son ölçümde yaw bütün örneklerde tam `0,000` ve kalibrasyon eşiğin altındaydı; araç durduğu için meşru da olabilir, **çevirmeden ayırt edilemez**. `imu_guvenlik` bu yüzden kapalı |
| 🔴 **Direksiyon işareti** | kart `0x09`: `direksiyon_isaret` | Yalnız `+1` / `−1` kabul edilir, ölçülmedi. Ters işarette düzeltme hatayı büyütür; ilk deneme **tekerlekler yerden kesik** yapılmalı |
| 🟠 Maks. steer açısı | `ackermann_converter.py` | 30° muhafazakâr sınır. Mekanik uç ölçülmedi; kolon ±750° ile 13,091 oranı tekerde ±57° veriyor ve iki ölçümden biri tutarsız |
| 🟠 Rampa hızları | `waypoints.yaml` | Tırmanış 0,60 / iniş 0,40 m/s — ikisi de ölçülmedi |
| 🟠 Fren kalibrasyonu | `topics.py` | Fren oranı sabitleri ve tork oturma penceresi muhafazakâr yer tutucu |
| ⚪ Parkur waypoint koordinatları | `waypoints.yaml` | 11 nokta (0,0). Kayan hedef modu koordinat kullanmadığı için bloker değil |
| ⚪ TensorRT engine | `models/best.engine` | `scripts/export_tensorrt.py` ile üretilir; depoda dağıtılmıyor |

---
