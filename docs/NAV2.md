# Nav2 ve sensör füzyonu

> LYDİA İKA otonom sürüş yazılımı — ayrıntılı belge.
> Genel bakış için [README](../README.md).

---

## Nav2 Konfigürasyonu

### Gerçek Araç (Ackermann)

```yaml
planner: SmacPlannerHybrid
  motion_model_for_search: REEDS_SHEPP   # İleri + geri yay (geri vites cezalı)
  allow_reverse_expansion: true          # REEDS_SHEPP'in geri yayı için gerekli
  minimum_turning_radius: 2.49           # L/tan(δ_max) = 1.44/tan(30°)

controller: RegulatedPurePursuitController
  use_rotate_to_heading: false   # Ackermann yerinde dönemez
  desired_linear_vel: 0.65       # terrain_adapter çalışma anında 0.45–0.90 yazar
```

### Davranış Ağacı — neden kendi ağacımız var

Nav2'nin stok ağaçları kurtarma dalında `<Spin>` kullanıyor. Ackermann araç
yerinde dönemediği için `nav2_spin_action_bt_node` yüklenmiyor; tanınmayan
düğüm ise ağacın ayrıştırılamamasına ve **bt_navigator'ın aktive olamamasına**
yol açıyor. Sonuç yalnız kurtarmanın değil Nav2'nin tamamının düşmesi:

```
[bt_navigator] Node not recognized: Spin
[lifecycle_manager] Failed to bring up all requested nodes. Aborting bringup.
```

Bu yüzden `config/bt/` altında Spin'siz iki ağaç var ve `nav2_params.yaml`
onlara **tam yolla** işaret ediyor. İkisi de gerekli: `navigators` yalnız
`navigate_to_pose` dese bile bt_navigator through_poses ağacını da yüklüyor.

Çalışma alanı taşınırsa güncellenecek yer: `nav2_params.yaml` içindeki iki
`default_nav_*_bt_xml` satırı.

### Kurtarma sırası ve geri gitme

`RoundRobin` her başarısızlıkta sıradaki davranışı dener:

1. **costmap temizle** — hayalet engel yüzünden plan üretilemiyorsa
2. **geri git** (`BackUp` 0,5 m @ 0,5 m/s) — poz planlanamaz hâle geldiyse.
   İleri-yönlü planlayıcıda R_min'den dar çıkış gerektiren bir pozdan hiçbir
   ileri yay çıkmaz; araç kımıldamadan tekrar denemek aynı sonucu verir.
3. **bekle** (3 s) — §6.8 kayar engel gibi geçici bir engel varsa

Geri gitme beklemeden önce geliyor: bekleme yalnız hareketli engelde işe yarar,
planlanamaz pozda geçen her saniye koşu saatinden gider.

Sayılar aracın ölçülen gerçeğinden: araç **0,45 m/s altında yerinden kalkmıyor**,
bu yüzden `backup_speed` 0,5 ve `velocity_smoother.min_velocity[0]` −0,50.
Stok değer (0,05 m/s) bu araçta iki kere ölüdür — smoother'ın deadband'i (0,05)
sıfırlar, kalkış sürtünmesi de geçirmez.

⚠️ **Otonom geri gitme YALNIZ bu kurtarmadır.** Planlayıcı geri vites kullanmaz
ve kullanmamalı: LiDAR gövdeye takıldığı için aracın arkası kör, §6.12 geri
kamera da takılı değil.

---

## Sensör Füzyonu (EKF)

```
/odom (kart 0x31 hızı)   ──▶ EKF ──▶ /odometry/filtered
/imu/data (BNO055, kart) ──┘          child_frame: base_footprint
```

---
