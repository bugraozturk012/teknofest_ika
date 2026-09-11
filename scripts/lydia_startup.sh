#!/bin/bash
# lydia_startup.sh — LYDİA açılış yığını (systemd: lydia_autostart.service)
#
# Ağ hazır olduğunda tüm algı, kontrol ve izleme düğümlerini sırayla başlatır.
# Sıra önemlidir: sensörler → algı → kontrol → izleme. Aradaki beklemeler USB
# cihazlarının numaralandırılması ve düğümlerin abonelik kurması içindir.
#
# DDS: paylaşılan bellek taşıması kapalı, yalnız UDP kullanılıyor. /dev/shm'de
# bayat fastrtps kilit dosyaları kaldığında düğümler birbirini bulamıyordu
# ("open_and_lock_file failed"); UDP-only profil bu sınıf hatayı tamamen
# ortadan kaldırıyor.

source /opt/ros/humble/setup.bash

# Derlenmemiş ya da yarım derlenmiş workspace sessizce geçilmemeli: overlay
# yoksa aşağıdaki her `ros2 run teknofest_ika ...` çağrısı "Package not found"
# ile kendi log dosyasına düşer, betik "başlatıldı" der ve dışarıdan görünen
# tablo boş topic listesi olur. Bir kez `rm -rf build install` + tek paket
# derlemesi bu duruma soktu ve DDS arızası sanıldı.
_OVERLAY=${OVERLAY:-/home/lydia/lydia_ws/install/setup.bash}   # kuru test için geçersiz kılınabilir
if [ ! -f "$_OVERLAY" ]; then
    echo "HATA: $_OVERLAY yok — workspace derlenmemiş." >&2
    echo "      cd ~/lydia_ws && colcon build --packages-select teknofest_ika" >&2
    exit 1
fi
source "$_OVERLAY"

# Dosyanın varlığı yetmez: --packages-select ile tek paket derlendiğinde
# overlay durur ama teknofest_ika içinde olmayabilir.
if ! ros2 pkg prefix teknofest_ika >/dev/null 2>&1; then
    echo "HATA: teknofest_ika paketi overlay'de yok — derleme eksik." >&2
    echo "      cd ~/lydia_ws && colcon build   (tam derleme)" >&2
    exit 1
fi

# Overlay VAR olmak yetmiyor, GÜNCEL de olmalı. Yukarıdaki iki denetim
# "derlenmiş mi" sorusunu cevaplıyor; "çekilen kod derlendi mi" sorusunu
# hiçbir şey sormuyordu. Araçtaki kurulum kopya tabanlıysa `git pull` sonrası
# .py değişiklikleri ETKİSİZDİR ve betik hiçbir uyarı basmadan ESKİ kodu
# başlatır — sahada "düzeltme araçta" sanılan en pahalı hata sınıfı.
#
# İki kurulum kipi ayrı davranıyor (ölçüldü):
#   symlink (`colcon build --symlink-install`) → build/<paket>/<paket> kaynağa
#     symlink olur, .py değişiklikleri anında geçerlidir; yalnız setup.py
#     (console_scripts) değişirse yeniden derleme gerekir çünkü stub'lar kopya.
#   kopya   (düz `colcon build`) → her .py değişikliği derleme ister.
# Zaman damgası ölçütü `colcon_build.rc`: başarılı her derlemenin sonunda
# yazılıyor ve iki kipte de var.
_WS_KOK=${WS%/src/*}
_YAPI_IZI="$_WS_KOK/build/teknofest_ika/colcon_build.rc"
if [ -f "$_YAPI_IZI" ]; then
    # Kip göstergesi EGG-LINK, build/ içindeki symlink DEĞİL. `--symlink-install`
    # ament_python'da site-packages'a bir `.egg-link` koyuyor ve modül ağacını
    # kurmuyor; düz derleme ise gerçek dosyaları kopyalıyor.
    # 🔴 build/<paket>/<paket> symlink'i ölçüt OLAMAZ: eski bir symlink-install
    # denemesinden geride kalıyor ve sonraki düz derlemeler onu silmiyor.
    # Araçta tam bu hâl ölçüldü — Ağustos tarihli symlink duruyor, kurulum
    # kopya. O symlink'e bakan bir denetim aracı "symlink kipi" sanıp .py
    # bayatlığını hiç uyarmaz, yani kontrolün en gerekli olduğu yerde susar.
    _EGG=$(find "$_WS_KOK/install/teknofest_ika" -name "*.egg-link" -print -quit 2>/dev/null)
    if [ -n "$_EGG" ]; then
        _KURULUM=symlink
        _YENI=$(find "$WS/setup.py" -newer "$_YAPI_IZI" -print -quit 2>/dev/null)
        _NE="setup.py (console_scripts stub'ları kopya)"
    else
        _KURULUM=kopya
        _YENI=$(find "$WS/teknofest_ika" "$WS/setup.py" -name '*.py' -newer "$_YAPI_IZI" \
                -print -quit 2>/dev/null)
        _NE="Python kaynağı"
    fi
    if [ -n "$_YENI" ]; then
        echo "🔴 UYARI: $_NE son derlemeden YENİ (kurulum kipi: $_KURULUM)."
        echo "   İlk yeni dosya: $_YENI"
        echo "   Çalışacak olan ESKİ koddur. Düzeltmek için:"
        echo "     cd $_WS_KOK && colcon build --packages-select teknofest_ika"
    else
        echo "Kurulum güncel (kip: $_KURULUM)."
    fi
fi

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/lydia/lydia_ortam/udp_only.xml
unset ROS_DISCOVERY_SERVER

# systemd servisi durdururken SIGTERM gönderir; alt süreçler yakalanmazsa
# hayatta kalıp servisi "deactivating" durumunda kilitliyorlar.
#
# Kapsam süreç GRUBU, doğrudan çocuklar değil: `jobs -p` yalnız betiğin kendi
# başlattığı ~18 süreci verir, oysa cgroup'ta 300'den fazla task var. Çoğu
# düğüm `ros2 run` sarmalayıcısının altında koşuyor ve sarmalayıcıyı öldürmek
# altındaki gerçek binary'yi öldürmüyor. Geride kalanları systemd
# TimeoutStopSec (90 s) dolunca SIGKILL ediyordu: durdurma 92 saniye sürüyor,
# servis her normal duruşta "Failed with result 'timeout'" damgası yiyor ve
# gerçek bir açılış arızasından ayırt edilemez hâle geliyordu. SIGKILL ayrıca
# düğümlerin kapanış işini de atlatıyor — seri_kopru Mega'ya PKT_DUR
# gönderemiyor, kayıt açıkken bag indekslenmeden kalıyor.
#
# Betik systemd altında grup lideri olduğu için (pid = pgid = sid) "-$$"
# tüm torunlara ulaşır. SIGTERM'i kendimiz yok sayarız ki grup sinyali betiği
# düşürmesin; SIGKILL yakalanamadığı için o adımda kendimizi listeden çıkarırız,
# yoksa systemd ana süreci SIGKILL'le ölmüş görüp yine "failed" yazardı.
# Kalanların listesi öldürmeden ÖNCE değişkene alınır: ps/awk/xargs boru hattı
# da bu grubun üyesi, doğrudan `| xargs kill` yazıldığında hat listeyi
# bitirmeden kendini öldürüyor. Süpürme adımına kalan iş SIGTERM'den sağ çıkan
# süreçler olduğu için bu çoğu zaman sonucu değiştirmez, ama listenin nerede
# kesileceği rastlantıya kalır.
temizle() {
    trap - TERM INT
    trap '' TERM
    kill -TERM -- "-$$" 2>/dev/null
    sleep 3
    _kalan=$(ps -eo pid=,pgid= | awk -v g=$$ '$2 == g && $1 != g {print $1}')
    [ -n "$_kalan" ] && kill -KILL $_kalan 2>/dev/null
    exit 0
}
trap temizle TERM INT

# ── Yığın anahtarları ───────────────────────────────────────────────────────
# ENKODER_AKTIF: odom→base_footprint dönüşümünü kimin yayınlayacağını belirler.
#   0 → enkoder yok sayılır; dönüşümü statik yayıncı basar (araç hep başlangıç
#       noktasındaymış gibi görünür, SLAM tarama eşlemeyle yürür).
#   1 → dönüşümü seri_kopru'nun odometrisi basar; statik yayıncı başlatılmaz.
# NAV2_AKTIF=1 iken sahip üçüncü bir adaya, EKF'e geçer: seri_kopru /odom'u
# yalnız ölçüm olarak yayınlar, TF'i EKF basar (bkz. _TF_SAHIBI aşağıda).
# Aynı halkayı iki kaynak basarsa TF ağacı iki konum arasında titrer ve SLAM
# haritayı bozuk kapatır — bu yüzden seçim üç yönlü ve karşılıklı dışlamalı.
: "${ENKODER_AKTIF:=0}"
# HAM_ENKODER: kartın ham enkoder sayımını /enkoder/ham'a yayınlar. Sayım
# ölçekten bağımsız olduğu için tekerlek çevresi ve dişli oranı ölçülmeden de
# hareketin var olup olmadığını gösterir; /odom hızı o sırada 0 gelir.
: "${HAM_ENKODER:=true}"
# NAV2_AKTIF burada erken okunuyor: TF sahibinin kim olduğu seri_kopru
# başlatılmadan önce bilinmeli, Nav2 bloğu ise betiğin çok sonrasında.
# Anahtarın ön koşulları o bloğun başında yazılı.
: "${NAV2_AKTIF:=0}"
# HEDEFLEME_MODU: misyon_fsm hedefleri nereden alsın. Boş bırakılırsa
# waypoints.yaml'daki `hedefleme_modu` geçerli olur. Değerler ve ne zaman
# hangisinin seçileceği Nav2 bloğunun içinde, düğüm başlatılan yerde yazılı.
: "${HEDEFLEME_MODU:=}"
# IMU_GUVENLIK_AKTIF: yatış açısına göre /speed_limit yayınlar ve 15°'de
# E-STOP zorlar. Kapalı çünkü eşiği IMU'nun montaj yönüne güveniyor: BMI160
# karta dönük lehimliyse araç düz dururken bile devrilmiş sanılır ve sürekli
# E-STOP basar. Açmadan önce araç düz dururken /imu/data'nın roll ve pitch'i
# ~0 okumalı (gerekirse IMU_ROLL/PITCH/YAW_RAD ile düzelt).
: "${IMU_GUVENLIK_AKTIF:=0}"

# BNO_TAKILI: BNO055 karta bağlı mı. 11 Eylül'de takıldı ve /imu/data 50 Hz
# akıyor, bu yüzden varsayılan 1. Watchdog'un IMU izlemesini bu açıyor.
# ⚠️ IMU_GUVENLIK_AKTIF'ten AYRI: o düğüm roll eşiğine göre aracı durduruyor
# ve eşiği IMU'nun MONTAJ YÖNÜNE güveniyor; yön sahada doğrulanana kadar
# kapalı kalmalı. Burada yalnız "veri akıyor mu" izleniyor.
: "${BNO_TAKILI:=1}"
# SLAM_SCAN_TOPIC: SLAM'in okuduğu tarama. Varsayılan filtrelenmiş tarama —
# ham /scan aracın arkasındaki gövde dönüşlerini de taşıyor ve o dönüşler araç
# çerçevesinde sabit durduğu için eşleştiriciyi yanıltıp haritaya leke basıyor.
# Sahada sorun çıkarsa SLAM_SCAN_TOPIC=/scan ile ham taramaya dönülür.
: "${SLAM_SCAN_TOPIC:=/scan/filtered}"
# TARET_AKTIF: nişan alma ve taret aktüasyonu. Kapalı çünkü /turret/cmd'yi
# tüketen İKİ aday var ve hangisinin araçta gerçek olduğu doğrulanmadı:
#   taret_rc_koprusu   → seri port → Turret UNO   (düğümün kendi belgesi
#                        servoların UNO'da olduğunu söylüyor)
#   servo_controller_node → Jetson I2C → PCA9685 → /taret/pan,/taret/tilt
#                        → seri_kopru → Mega
# İkisi birden koşarsa taret iki kaynaktan sürülür. TARET_YOLU ile seç.
: "${TARET_AKTIF:=0}"
: "${TARET_YOLU:=uno}"        # uno | pca9685
# KAYIT_AKTIF: veri_paketi rosbag kaydı. Kapalı — Jetson'ın diski dolarsa
# loglar ve harita da yazılamaz.
: "${KAYIT_AKTIF:=0}"
# DERINLIK_AKTIF: OS30A derinlik kamerası ve onun costmap TF'i. Kapalı, çünkü
# kaynağın Nav2'ye kattığı değer ölçülmemiş bir dönüşümün arkasında duruyor:
# base_link → dm_base_frame'in üç sayısı (OS30A_X_M/Y_M/Z_M) tahmin. Yükseklik
# yanlışsa nokta bulutu düşeyde kayar, zemin engel olarak işaretlenir ve araç
# kendini duvarla çevrili sanıp hiç rota üretmez.
# Kapatma iki yerde birden geçerli olmak zorunda: burası düğümü ve TF'i
# başlatmıyor, nav2_params.yaml da os30a_cloud'u observation_sources'a
# yazmıyor. Yalnız TF'i kapatmak kaynağı ayrı bir TF adasında bırakır ve
# costmap "Transform failure" basmaya başlar — sorun kalır, gürültü artar.
: "${DERINLIK_AKTIF:=0}"
# SLAM_AKTIF: slam_toolbox ve panonun harita görüntüsü (map_image_node).
# Kapalı, çünkü sürüş zincirinin hiçbir yeri haritayı okumuyor: hedef her
# döngüde taramadan doğuyor ve `odom` çerçevesinde gönderiliyor
# (topics.py KAYAN_HEDEF_FRAME), aşama da koordinatla değil kat edilen yolla
# bitiyor. Geriye kalan tek tüketici panonun harita paneliydi, o da ızgaradan
# çıkarıldı. Karşılığı: açılıştan ~17 s, YOLO ile paylaşılan CPU/RAM, ve
# `map→odom` düzeltmesinin koşu ortasında sıçrama ihtimali.
#
# 🔑 Nav2 bu anahtardan BAĞIMSIZ olarak `odom`'da çalışır (nav2_params.yaml:
# bt_navigator ve global_costmap global_frame). SLAM açılsa bile yalnız harita
# üretir, planlayıcı onu kullanmaz. Ayrım kasıtlı: yapılandırmanın anahtara
# göre iki farklı hâli olsaydı sahada hangi hâlde olunduğu görünmezdi ve
# `map` çerçevesi yokken Nav2 hiç ayağa kalkmazdı.
: "${SLAM_AKTIF:=0}"
# GOZCU_AKTIF: koşu sırasında ölen düğümleri izleyen gözcü döngüsü.
# Açık, çünkü kapattığı arıza teorik değil: `seri_kopru` kapanmış porta
# yazarken öldü, betik geri getirmedi ve boşalan seri portu elektrikçilerin
# telemetri servisi kaptı — `/kart/*`'ın tamamı sustu ve sebebi kart ekibi
# tarafından bulundu. Açılış doğrulaması bunu göremiyor: bir kez, açılışta
# bakıyor (satır ~792) ve sonra betik `wait`'te bekliyordu.
# Kapatmak için GOZCU_AKTIF=0 — o hâlde davranış eski hâline döner.
: "${GOZCU_AKTIF:=1}"
# Yoklama periyodu. `ros2 node list` bir keşif sorgusu ve ~1 s sürüyor;
# sık yoklamak yığından CPU çalar, seyrek yoklamak arızayı geç görür.
: "${GOZCU_PERIYOT_S:=15}"
# Aynı düğüm için azami yeniden başlatma. Belirleyici bir hata yüzünden ölen
# düğüm yeniden başlatılınca aynı çökme döngüsüne girer; sınır olmadan gözcü
# koşu boyunca CPU yakar ve log'u boğar. Sınıra ulaşınca adıyla vazgeçiliyor.
: "${GOZCU_AZAMI_DENEME:=3}"

# SERI_PORT: sürüş kartının komut portu (8 baytlık ikili çerçeve, seri_kopru).
# 3 Eylül'deki not "ST-LINK yalnız ASCII teşhis basar, komut kabul etmez,
# USART6/PG9-PG14'e ayrı kablo şart" diyordu. Kart→Jetson yönü için bu artık
# geçerli değil: 8 Eylül'de ST-LINK VCP'sinden (/dev/f767, 921600) XOR'u tutan
# ikili çerçeve okundu ve uzun süre tek bir bozuk paket çıkmadı. Kart ekibinin
# 031/033 belgeleri de tek USB kablosunu tarif ediyor.
# 🔴 Jetson→kart yönü HÂLÂ KANITLANMADI: komutlarımızın kabul edilip
#    edilmediği yalnız 0x39'un JDR_LINK bitinde görünüyor ve o bit eski
#    köprüde okunmuyordu. İlk otonom denemeden önce panodan doğrulanacak.
# Ayrı bir komut hattı çekilirse: SERI_PORT=/dev/f767_komut ver, gerisi aynı.
: "${SERI_PORT:=/dev/f767}"

# LIDAR_PORT: sürücünün açacağı cihaz. Ham düğüm (/dev/ttyUSB0) DEĞİL udev
# symlink'i kullanılıyor — ST-LINK ile LiDAR aynı hub'ın arkasında ve ttyUSB
# numarası açılışlar arasında kayabiliyor. Symlink yoksa kural oturmamıştır;
# ham düğüme elle bağlanmak yanlış cihazı açma riski taşır.
: "${LIDAR_PORT:=/dev/lidar}"

# F767 TELEMETRİ: ST-LINK USB'sinden akan ASCII teşhis satırını JSON'a çevirip
# panoya besler. ⚠ Komut yolu DEĞİL — tek yönlü, yalnız dinler.
#
# 🔴 VARSAYILAN 0. Aşağıdaki "aynı port" koruması çakışmayı zaten çözüyor ama
# koruma bir kural, varsayılan bir niyet: kartın tek okuyucusu seri_kopru.
# İki okuyucu denendiğinde iki ayrı arıza çıktı (10 Eylül, sahada):
#   · telemetri portu SONRA açıp hat hızını 115200'e çekti ve kendisi
#     kapandıktan sonra da öyle bıraktı — köprü portu tutuyor ama yalnız çöp
#     okuyor, tek belirti seyrek bir XOR hatası. Kart durumu açılışta dondu.
#   · baytlar bölüşüldü, köprü portu 141 kez yeniden açtı ve çöktü.
# Teşhis: stty -F /dev/ttyACM0 speed → 921600 olmalı.
# Teşhis servisi gerekiyorsa köprü durdurulup F767_TELEMETRI_AKTIF=1 verilir.
: "${F767_TELEMETRI_AKTIF:=0}"
: "${F767_TELEMETRI_PORT:=/dev/f767}"
: "${F767_TELEMETRI_HTTP:=8092}"

# BMS zinciri iki parçalı ve ikisi ayrı şeyler:
#   jk_servis.py   — elektrik tarafının betiği, BLE'yi okuyup :8091/bms'te düz
#                    JSON yayınlıyor. Bu betik onu aşağıda başlatıyor.
#   bms_koprusu    — bizim ROS düğümümüz, o HTTP ucunun İSTEMCİSİ; BLE'ye hiç
#                    dokunmuyor, veriyi /battery/status'a taşıyor.
# 🔴 :8091'i servis eden kalmazsa bms_koprusu bayat okuma görüp bilerek susar
# ve /battery/status hiç yayınlanmaz — panoda "batarya yok", imu_guvenlik
# gerilim körü. bms_koprusu'nun ayakta olması veri geldiği anlamına GELMEZ.
# ⚠ Aynı servisi elektrik tarafının hepsini_baslat betiği de açabiliyor; iki
# başlatıcının kavga etmemesi port kontrolüne bağlı (ikinci açılış :8091'i
# dolu bulup vazgeçmeli).
# ⚠ BLE aynı anda TEK istemci kabul eder: servis koşarken telefondaki JK
# uygulaması bağlanamaz, tersi de doğru. "Batarya yok" şikâyetinin ilk
# şüphelisi budur. BMS_MAC boşsa servis başlatılmaz.
: "${BMS_AKTIF:=1}"
: "${BMS_MAC:=28:D4:1E:12:C1:70}"   # 3 Eyl 2026'da canlı doğrulandı
: "${BMS_HTTP:=8091}"

# PANO_PORT: ROS panosu (web_dashboard.py). 8080 elektrik ekibinin statik
# panosunda — ikisi ayri ise bakiyor, biri kapatilmiyor, portlar ayriliyor.
: "${PANO_PORT:=8083}"

# TEKERLEK_CEVRE_MM: tekerleğin bir turda yerde kat ettiği mesafe [mm].
# Kart bu sayı girilmeden 0x31 hız alanını bilerek 0 basıyor; o hâlde /odom
# ilerlemiyor ve Nav2 her hedefi 20 saniyede "ilerleme yok" diye iptal ediyor.
# Kart ölçeği mm x 10 (1256.6 -> 12566, int16 tavanı 32767).
#
# 🔴 HENÜZ ÖLÇÜLMEDİ — 0 bırakıldı. Sıfır "ölçüm yok" demek ve GÖNDERİLMİYOR;
#   kart da hız alanını 0 basmaya devam ediyor. Uydurma bir sayı göndermek
#   susmaktan kötüdür: yanlış mesafeye dayanan rota sessizce yanlış yere gider.
#
#   Ölçüm: tekerleğe işaret koy, aracı düz bir çizgide it, işaret tam bir tur
#   dönünce yerdeki mesafeyi mezürle ölç. ⚠ YÜK ÜSTÜNDEYKEN ölç — havalı lastik
#   çöktüğü için yuvarlanma çevresi 2*pi*r'den küçüktür; hesaplanmış geometrik
#   çevre kullanılırsa araç aldığı yolu olduğundan fazla sanar.
#
#   Ölçüm gelince tek satır: aşağıdaki 0 yerine milimetre değerini yaz.
#   Yeniden derleme gerekmiyor, düğümü yeniden başlatmak yeter.
: "${TEKERLEK_CEVRE_MM:=0}"

# Kartın kalan 0x09 ayarları. Hepsi TEKERLEK_CEVRE_MM ile aynı kuralda:
# 0 = ÖLÇÜLMEDİ ve gönderilmez. Ortam değişkeni olmalarının sebebi sahada
# betiği düzenlemek zorunda kalmamak — ölçüm araç başında yapılıyor ve
# metin düzenleyiciyle girilen bir sayı yanlış satıra da yazılabiliyor.
#
# 🔴 ÇEVRE TEK BAŞINA YETMEZ: kart hız alanını doldurmak için çevreyi VE
#    darbe/tur'u birlikte istiyor. Yalnız biri girilirse hız 0 kalmaya devam
#    eder ve sahada "girdik ama olmadı" denir.
: "${GOSTERGE_DARBE_TUR:=0}"    # gösterge ucu darbe/tur
: "${ENKODER_DISLI_ORANI:=0}"   # enkoder mili turu : teker turu
: "${DIREKSIYON_ORANI:=0}"      # direksiyon kolon/teker oranı
# ⚠ Yalnız +1 ya da -1 kabul edilir; başka değer köprüde reddedilir.
#   İşaret yanlışsa Nav2 sola ister araç sağa gider ve sapma büyür — bu yüzden
#   ilk denemesi TEKERLEKLER YERDEN KESİK yapılır.
: "${DIREKSIYON_ISARET:=0}"

if [ "$ENKODER_AKTIF" != "1" ]; then
    _TF_SAHIBI=statik
elif [ "$NAV2_AKTIF" = "1" ]; then
    _TF_SAHIBI=ekf
else
    _TF_SAHIBI=seri_kopru
fi
echo "odom→base_footprint sahibi: $_TF_SAHIBI"
# ENKODER_AKTIF=0 iken odom→base_footprint STATİK basılır ve aracı TF'te
# ilerleten tek şey SLAM'in tarama eşlemesidir. İkisi birden kapalıysa araç
# TF'te çakılı durur: Nav2 hedefe hiç yaklaşmadığını görür ve her hedefi
# bütçesi dolunca iptal eder. Kombinasyon geçerli (manuel sürüş, teşhis) ama
# sessiz kalırsa donanım arızası gibi görünür.
if [ "$_TF_SAHIBI" = "statik" ] && [ "$SLAM_AKTIF" != "1" ]; then
    echo "UYARI: ENKODER_AKTIF=0 ve SLAM_AKTIF=0 — araç TF'te hiç hareket etmez;"
    echo "       otonom sürüş için ENKODER_AKTIF=1 gerekir."
fi

WS=${WS:-/home/lydia/lydia_ws/src/teknofest_ika_yazilim}
LOG=${LOG:-/home/lydia/lydia_log}
mkdir -p "$LOG"
cd "$WS" || exit 1

# Ağ arayüzü gelene kadar bekle (en fazla 60 s)
for _ in $(seq 1 30); do
    ip -4 addr show | grep -q '192\.168\.100\.' && break
    sleep 2
done

# Bayat DDS kilitleri temizlensin (önceki oturumdan kalmış olabilir)
rm -f /dev/shm/fastrtps* 2>/dev/null

# Önceki oturumdan artan düğümleri kapat. systemd yeniden başlatırken eski
# süreçler ölmezse port 8080 ve seri portlar meşgul kalıyor, yeni düğümler
# sessizce açılamıyordu.
#
# Liste bu betiğin başlattığı HER süreci kapsamalı. Temiz `systemctl stop`'ta
# SIGTERM trap'i çocukları zaten öldürüyor; buradaki tarama trap'in çalışmadığı
# hâller içindir — betiğin elle yeniden çalıştırılması, kill -9, ya da Jetson'ın
# kendiliğinden resetlenmesi. Eksik bırakılan bir ad yarı temizlenmiş bir yığın
# üretir: en tehlikelisi static_transform_publisher, çünkü ikinci bir kopya
# odom→base_footprint'i aynı anda basıp TF'i titretir.
for _p in web_dashboard.py yolo_detection_node preprocessing_node \
          usb_cam_node_exe apc_camera_node \
          static_transform_publisher \
          e_stop_node watchdog anti_rollback imu_guvenlik \
          targeting_node taret_rc_koprusu servo_controller_node veri_paketi \
          ekf_node yolo_adapter_node terrain_adapter cone_fusion_node \
          kayar_engel_kalman kayar_engel_costmap misyon_fsm \
          controller_server planner_server bt_navigator behavior_server \
          smoother_server velocity_smoother waypoint_follower lifecycle_manager \
          seri_kopru mod_yoneticisi ackermann_converter \
          async_slam_toolbox_node map_image_node foxglove_bridge; do
    pkill -f "$_p" 2>/dev/null
done

# 🔴 3 Eylül 2026: pano ve onu besleyen iki HTTP servisi bu listede YOKTU.
# Sonucu: `systemctl restart` sonrası eski süreçler 8080/8091/8092'yi tutmaya
# devam ediyor, yeni kopyalar "Address already in use" ile ölüyor ve pano
# ÖNCEKİ oturumun verisiyle çalışmaya devam ediyordu — yani restart'ın hiçbir
# etkisi görünmüyordu. Kalıbı dar tut: düz "http.server" başka bir işi vurabilir.
for _p in "f767_telemetri.py" "jk_servis.py" "http.server 8080"; do
    pkill -f "$_p" 2>/dev/null
done

# LiDAR ayrı ele alınıyor: sürücü kapanırken seri portu geç bırakıyor, hemen
# yeniden açılırsa "cannot bind to serial port" verip düşüyor.
pkill -f ydlidar_ros2_driver_node 2>/dev/null
pkill -f "ydlidar_ros2_driver ydlidar_launch.py" 2>/dev/null
sleep 8

# ── Sensörler ────────────────────────────────────────────────────────────────
# LiDAR: fixed_resolution=false şart. true iken sürücü 430 noktaya sabitlemeye
# çalışıyor, gerçek tarama 400-642 nokta geldiği için /scan hiç yayınlanmıyordu.
# 🔴 3 Eylül 2026: cihaz yokken bu döngü 3×22 sn boşa harcıyordu (LiDAR
# sökülü). Kontrol symlink üzerinden: udev kuralı 99-ika.rules'ta.
# _LIDAR_VAR: taramanın gerçekten aktığı aşağıdaki Nav2 kapısında da
# bilinmek zorunda. Cihazın yokluğu burada zaten yazılıyordu ama satır,
# Nav2 kararından dakikalarca önce ekrandan akıp gidiyor ve sürücünün üç
# denemede de açılamaması hiçbir iz bırakmıyordu.
_LIDAR_VAR=0
if [ ! -e /dev/lidar ]; then
    echo "UYARI: LiDAR (/dev/lidar) bulunamadı — 66 sn'lik açılış denemesi ATLANDI"
else
# 🔴 Sürücü portu KENDİ yaml'ından okuyor ve orada ham düğüm yazılı
# (`port: /dev/ttyUSB0`). Yukarıdaki kontrol symlink'e bakıyor, sürücü başka
# bir şeye: ikisi ayrışabilir. ST-LINK ile LiDAR aynı hub'a bağlandığından
# sıralama artık açılışlar arasında sabit değil, ve ttyUSB numarası kayarsa
# sürücü yanlış cihazı açar ya da hiç açamaz — belirti "Lidar has started"
# satırının gelmemesi olur, sebebi hiçbir yerde yazmaz.
# Çözüm SLAM parametrelerindeki desenin aynısı: çalışma anı kopyası üretilip
# port udev symlink'ine çevriliyor. Depodaki yaml değil Jetson'daki dosya
# kaynak olduğu için kopya log dizinine yazılıyor; dosya yoksa özgün yola
# düşülüyor (o hâlde davranış eskisi gibi).
_LIDAR_YAML=/home/lydia/lydia_ortam/tmini_pro.yaml
if [ -f "$_LIDAR_YAML" ]; then
    _LIDAR_PARAMS="$LOG/tmini_pro.runtime.yaml"
    sed "s|^\( *port: *\).*|\1$LIDAR_PORT|" "$_LIDAR_YAML" > "$_LIDAR_PARAMS"
    echo "[LiDAR] port → $LIDAR_PORT (çalışma anı kopyası)"
else
    _LIDAR_PARAMS="$_LIDAR_YAML"
    echo "UYARI: $_LIDAR_YAML yok — sürücü kendi varsayılanıyla açılacak"
fi
for _deneme in 1 2 3; do
    ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
        params_file:="$_LIDAR_PARAMS" \
        > "$LOG/lidar.log" 2>&1 &
    sleep 14
    grep -q "Lidar has started" "$LOG/lidar.log" && break
    echo "LiDAR açılmadı (deneme $_deneme), port serbest bırakılıp tekrar denenecek"
    # Sürücünün yanında launch sürecinin kendisi ve onun statik TF yayıncısı da
    # düşürülür. Yalnız sürücü öldürülürse launch ayakta kalıyor ve her deneme
    # geride bir base_link→laser_frame yayıncısı bırakıyor; üç denemeden sonra
    # aynı dönüşümü basan üç ayrı süreç oluyor.
    pkill -f ydlidar_ros2_driver_node 2>/dev/null
    pkill -f "ydlidar_ros2_driver ydlidar_launch.py" 2>/dev/null
    pkill -f static_tf_pub_laser 2>/dev/null
    sleep 8
done
# Son denemenin log'u bakılıyor: başarıda döngü break ile çıkıyor ve log o
# denemeye ait, başarısızlıkta da son denemeye.
grep -q "Lidar has started" "$LOG/lidar.log" && _LIDAR_VAR=1
[ "$_LIDAR_VAR" = "1" ] || echo "UYARI: LiDAR sürücüsü üç denemede de açılmadı"
fi

# ydlidar_launch.py kendi base_link→laser_frame dönüşümünü de basıyor:
# "0 0 0.02", montaj dönüşü yok. Betiğin aşağıda bastığı ölçülmüş dönüşümle
# (LIDAR_Z_M, LIDAR_YAW_RAD) aynı parent/child çiftini paylaşıyorlar. tf2'nin
# statik tamponunda bir çifti en son gelen mesaj ezdiği için hangisinin
# geçerli olacağı abonenin bağlanma anına bağlı kalıyor — costmap bir açılışta
# doğru, ötekinde 93° dönük ve 53 cm alçak bir LiDAR görür. Sürücü ayağa
# kalktıktan sonra launch'ın kopyası düşürülür, tek ve doğru yayıncı kalır.
pkill -f static_tf_pub_laser 2>/dev/null

# Derinlik kamerası (OS30A) — /apc/depth/image_raw renklendirilmiş derinlik,
# /apc/left/image_color stereo sol göz renkli görüntü. Açılışı 14 saniye
# bekletiyor ve YOLO ile aynı USB/CPU bütçesinden yiyor; tüketicisi
# kalmadığında başlatmamak bunların ikisini birden geri veriyor.
if [ "$DERINLIK_AKTIF" != "1" ]; then
    echo "Derinlik kamerası kapalı (DERINLIK_AKTIF=0) — açılışı atlandı"
elif [ ! -e /dev/kamera_stereo ]; then
    echo "UYARI: derinlik kamerası (/dev/kamera_stereo) bulunamadı — açılışı ATLANDI"
else
    ros2 launch ydlidar_os30a apc_camera_launch.py > "$LOG/apc.log" 2>&1 &
    sleep 14
fi

# Kameralar udev symlink'i üzerinden açılır; USB düğüm numaraları (video4/6…)
# takılma sırasına göre kayıyor, symlink cihaz kimliğine bağlı olduğu için
# sabit kalıyor.
#
# MJPEG şart: her iki kamera da YUYV modunda 640x480'i 1 Hz veriyor, usb_cam
# o modda veri gelmeden zaman aşımına düşüp çöküyor.

# Ön kamera — icSpring WebCamera (32e6:9211)
if [ -e /dev/kamera_on ]; then
    ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p camera_name:=on_kamera \
        -p video_device:=/dev/kamera_on \
        -p image_width:=640 -p image_height:=480 \
        -p pixel_format:=mjpeg2rgb -p framerate:=30.0 \
        -p qos_history_policy:=keep_last -p qos_history_depth:=1 \
        -r /image_raw:=/camera/image_raw \
        -r /camera_info:=/camera/camera_info \
        > "$LOG/on_cam.log" 2>&1 &
else
    echo "UYARI: ön kamera (/dev/kamera_on) bulunamadı"
fi
sleep 5

# Nişan kamerası — Microdia Hy-13M (0c45:6370)
if [ -e /dev/kamera_nisan ]; then
    ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p camera_name:=nisan_kamera \
        -p video_device:=/dev/kamera_nisan \
        -p image_width:=640 -p image_height:=480 \
        -p pixel_format:=mjpeg2rgb -p framerate:=30.0 \
        -p qos_history_policy:=keep_last -p qos_history_depth:=1 \
        -r /image_raw:=/camera/taret/image_raw \
        -r /camera_info:=/camera/taret/camera_info \
        > "$LOG/nisan_cam.log" 2>&1 &
else
    echo "UYARI: nişan kamerası (/dev/kamera_nisan) bulunamadı"
fi
sleep 6

# Geri sürüş kamerası — panoda ön ve nişanla birlikte üçlü sürüş görünümünü
# besliyor. Konu adı topics.py CAMERA_REAR_TOPIC ile aynı tutulmalı; pano o
# adı dinliyor ve cihaz yokken "sinyal yok" gösteriyor.
# 🔴 /dev/kamera_arka için udev kuralı HENÜZ YOK (Jetson'da
#    /etc/udev/rules.d/99-ika.rules). Kamera takıldığında ham video düğümüne
#    değil, kurala bağlanmalı: ham numara her açılışta kayabiliyor.
if [ -e /dev/kamera_arka ]; then
    ros2 run usb_cam usb_cam_node_exe --ros-args \
        -p camera_name:=geri_kamera \
        -p video_device:=/dev/kamera_arka \
        -p image_width:=640 -p image_height:=480 \
        -p pixel_format:=mjpeg2rgb -p framerate:=30.0 \
        -p qos_history_policy:=keep_last -p qos_history_depth:=1 \
        -r /image_raw:=/camera/arka/image_raw \
        -r /camera_info:=/camera/arka/camera_info \
        > "$LOG/arka_cam.log" 2>&1 &
    sleep 5
else
    echo "UYARI: arka kamera (/dev/kamera_arka) bulunamadı"
fi

# ── Gövde bağlantısı ve kontrol zinciri ──────────────────────────────────────
if [ "$_TF_SAHIBI" = "seri_kopru" ]; then
    _SERI_TF=true
else
    _SERI_TF=false
fi
# Sürüş kartı Nucleo-F767ZI: $SERI_PORT @ 921600. Bu iki değer düğümün
# varsayılanıyla aynı tutulmalı — betik launch dosyasını kullanmıyor, yani
# yalnız orada değiştirilen bir ayar sahaya hiç ulaşmaz.
# KART AYARLARI (0x09) — hepsi varsayılan 0 = ÖLÇÜLMEDİ, sıfır olan
# gönderilmez. Kart bu sayıları flash'a yazmıyor: köprü, kartı ilk gördüğünde
# ve kart her resetlendiğinde yeniden gönderiyor, yani kalıcılık burada.
# Beşi de ortam değişkeninden geliyor; değeri değiştirmek için ne betiği
# düzenlemek ne yeniden DERLEMEK gerekiyor, düğümü yeniden başlatmak yeter:
#     TEKERLEK_CEVRE_MM=1842.5   tekerlekte bir tam tur, YÜK ALTINDA yerde ölçülür
#     GOSTERGE_DARBE_TUR=6       gösterge ucu darbe/tur
#     ENKODER_DISLI_ORANI=3.25   enkoder mili turu : teker turu
#     DIREKSIYON_ORANI=12.4      kolon/teker oranı
#     DIREKSIYON_ISARET=-1       ⚠ ÖNCE tekerlekler yerden kesik denenir
# 🔴 Tekerlek çevresi girilmeden kart hız alanını 0 basar; o hâlde Nav2
#    aracı hareketsiz sanar ve her hedefi 20 saniyede iptal eder.
if [ ! -e "$SERI_PORT" ]; then
    echo "UYARI: sürüş kartı portu ($SERI_PORT) YOK — seri_kopru veri alamaz."
    echo "       udev kuralı oturmamış olabilir; ham ttyACM*'a ELLE bağlanma."
    echo "       Düğüm yine de başlatılıyor: portu 2 sn'de bir yeniden dener."
fi
ros2 run teknofest_ika seri_kopru --ros-args \
    -p port:="$SERI_PORT" -p baud:=921600 \
    -p tekerlek_cevre_mm:="$TEKERLEK_CEVRE_MM" \
    -p gosterge_darbe_tur:="$GOSTERGE_DARBE_TUR" \
    -p enkoder_disli_orani:="$ENKODER_DISLI_ORANI" \
    -p direksiyon_orani:="$DIREKSIYON_ORANI" \
    -p direksiyon_isaret:="$DIREKSIYON_ISARET" \
    -p publish_tf:="$_SERI_TF" \
    -p ham_enkoder:="$HAM_ENKODER" > "$LOG/seri_kopru.log" 2>&1 &
sleep 8
ros2 run teknofest_ika mod_yoneticisi      > "$LOG/mod_yoneticisi.log" 2>&1 &
sleep 3
ros2 run teknofest_ika ackermann_converter > "$LOG/ackermann.log" 2>&1 &
sleep 3
# E-STOP toplayıcı. Dört kaynağı (Arduino butonu, IMU devrilme, RC sinyal
# kaybı, panodan GCS komutu) tek /e_stop'ta birleştirir. Bu düğüm olmadan
# /e_stop'a HİÇ yayın yapılmaz; ona abone olan seri_kopru, mod_yoneticisi,
# ackermann_converter, misyon_fsm ve pano sürekli "E-STOP yok" okur ve
# panodaki durme düğmesi hiçbir şey yapmaz. Aracın kendi donanım kesmesi
# ayrı bir yolla çalışsa bile ROS tarafındaki bütün kilitler buna bağlı.
# Fiziksel buton Jetson'ın GPIO'suna değil sürüş kartına bağlı ve durumu
# 0x34 ile geliyor; düğümün işi kaynakları OR'lamak. gpio_mod düğüm
# varsayılanında kapalı, burada da açılmıyor — boştaki bir giriş pini 48 V'un
# gürültüsü altında rastgele E-STOP üretir.
ros2 run teknofest_ika e_stop_node         > "$LOG/e_stop.log" 2>&1 &
sleep 3
# Eğimde geri kaymayı yakalayıp karşı komut basar. /odom'a bağlı olduğu için
# ENKODER_AKTIF=0 iken ölçüm sabit kalır ve düğüm hiç tetiklenmez — zararsız,
# ama gerçek koruma ancak enkoderle gelir.
ros2 run teknofest_ika anti_rollback       > "$LOG/anti_rollback.log" 2>&1 &
sleep 3

# ── Algı zinciri ─────────────────────────────────────────────────────────────
# preprocessing ön kameradan (/camera/image_raw) besleniyor ve
# /camera/image_processed üretiyor; yolo_detection_node oradan okur.
# Nişan kamerası araca ters monte; görüntü burada çevriliyor ki hem panel hem
# targeting düz görsün. Ön kamera için -p flip_ana:=true eklenebilir.
#
# derinlik_isle kapalı: /depth/points/filtered'ın tek tüketicisi Nav2 costmap
# katmanıydı ve Nav2 kapalı. Açıkken kare başına KD-tree kurulup nokta bulutu
# Python'da geziliyor, çıktıyı kimse okumuyordu. Derinlik GÖRÜNTÜSÜ etkilenmez
# (/apc/depth/image_raw doğrudan OS30A'dan panele gider).
#
# /scan_lidar remap'i ZORUNLU: preprocessing_node tarama girişini bu adda
# bekliyor, ydlidar sürücüsü ise /scan basıyor. Remap olmadan cb_scan hiç
# tetiklenmez ve /scan/filtered ÜRETİLMEZ — Nav2'nin obstacle_layer scan
# kaynağı (iki costmap'te de), cone_fusion_node, kayar_engel_kalman ve
# kayar_engel_costmap hep birden susar, hiçbiri hata vermeden.
# (Launch dosyasındaki çözüm sürücüyü /scan_raw'a remap edip scan_relay'i
# araya koymaktı; relay sürücünün eski 0x202 zaman damgası hatası içindi,
# doğrudan remap aynı işi tek satırda yapıyor.)
ros2 run teknofest_ika preprocessing_node --ros-args \
    -r /scan_lidar:=/scan \
    -p flip_taret:=true \
    -p derinlik_isle:=false > "$LOG/preprocessing.log" 2>&1 &
sleep 5
ros2 run teknofest_ika yolo_detection_node > "$LOG/yolo.log" 2>&1 &
sleep 3

# ── SLAM ─────────────────────────────────────────────────────────────────────
# TF zincirinin odom→base_footprint halkasını yalnız _TF_SAHIBI basar.
if [ "$_TF_SAHIBI" = "statik" ]; then
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint \
        > "$LOG/tf_odom.log" 2>&1 &
fi
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link \
    > "$LOG/tf_base.log" 2>&1 &
# EKF /imu/data'yı imu_link'ten base_footprint'e çevirebilmek için bu halkaya
# muhtaç; bulamazsa ölçümü SESSİZCE düşürür (robot_localization'ın
# lookupTransformSafe uyarısı kaynakta yorum satırı). O durumda EKF yalnız vx
# ile koşar, yön hiç güncellenmez ve araç estimatörde hep düz gider.
#
# ⚠️ DÖNÜŞ AÇILARI DOĞRULANMADI — urdf/arac.urdf imu_joint'inden alındı
#    (rpy 0 0 0). Yön füzyonunda önemli olan öteleme değil dönüştür: BMI160
#    karta ters ya da 90° dönük lehimliyse roll/pitch/yaw da o kadar yanlış
#    gelir. Araç düz dururken /imu/data'nın roll ve pitch'i ~0 okumuyorsa
#    burayı düzelt.
: "${IMU_X_M:=0}"
: "${IMU_Y_M:=0}"
: "${IMU_Z_M:=0}"
: "${IMU_ROLL_RAD:=0}"
: "${IMU_PITCH_RAD:=0}"
: "${IMU_YAW_RAD:=0}"
ros2 run tf2_ros static_transform_publisher \
    "$IMU_X_M" "$IMU_Y_M" "$IMU_Z_M" \
    "$IMU_YAW_RAD" "$IMU_PITCH_RAD" "$IMU_ROLL_RAD" \
    base_link imu_link > "$LOG/tf_imu.log" 2>&1 &
# LiDAR gövdeye 93.3° dönük monte (sahada huniyle çift yönlü kalibre edildi).
# huni_reaktif bu düzeltmeyi kendi içinde (aci_offset_deg) ham /scan üzerinde
# yapar ve TF'ten etkilenmez; Nav2 costmap ise YALNIZ TF'e bakar — dönüş burada
# verilmezse tüm engelleri 93° kaydırarak yerleştirir.
#
# Konum ölçülmeyi bekliyor, iki eksen de şüpheli:
#   z — 0.55 ZEMİNDEN ölçüldü ama dönüşümün ebeveyni base_link ve base_joint
#       gövdeyi zeminden 0.28 m yukarı alıyor; base_link'e göre değer 0.27
#       civarı olmalı. Jetson çalışma kopyasındaki urdf 0.265 diyor.
#   x — burada 0 yazıyor, LiDAR ise burna yakın monte. Jetson kopyası 0.80.
#       Doğruysa her engel base_link'e 0.80 m daha yakın işaretleniyor,
#       raytrace temizliği de aynı kadar kayıyor.
# Mezürle ölçülüp (ön aks merkezi x=+0.70 referans) iki yer birlikte
# güncellenmeli: burası ve urdf'teki lidar_joint.
: "${LIDAR_YAW_RAD:=1.6284}"   # 93.3° — huni_reaktif aci_offset_deg ile aynı
: "${LIDAR_Z_M:=0.55}"         # zeminden tarama düzlemine, ölçüldü
ros2 run tf2_ros static_transform_publisher 0 0 "$LIDAR_Z_M" "$LIDAR_YAW_RAD" 0 0 \
    base_link laser_frame > "$LOG/tf_laser.log" 2>&1 &
# OS30A derinlik kamerası ayrı bir TF adasında duruyor: kendi launch'ı
# dm_base_frame → {points_frame, depth_frame, ...} dönüşümlerini basıyor ama
# dm_base_frame'i hiçbir şey base_link'e bağlamıyor. Bu halka olmadan Nav2'nin
# os30a_cloud kaynağı tek nokta bile dönüştüremez (costmap sürekli "Transform
# failure" basar) ve derinlik kamerası costmap'e hiçbir katkı yapmaz.
#
# Varsayılanı DERINLIK_AKTIF belirliyor: TF'i kameradan bağımsız açık bırakmak
# ikisinin sessizce ayrışmasına yol açar — kamera kapalıyken boşa bir yayıncı,
# kamera açıkken TF'siz bir TF adası. Ayrı ayarlanması gerekirse bu satır
# yine ezilebilir.
#
# ⚠️ AŞAĞIDAKİ KONUM ÖLÇÜLMEDİ — urdf/arac.urdf os30a_joint'inden alındı
#    (x=0.30 ileri, z=0.20 yukarı, base_link'e göre); urdf'te o da placeholder.
#    Ön kameranın kamera_joint'i (x=0.55) BAŞKA bir sensördür, karıştırma.
#    OS30A menzili 0.02–2.5 m olduğu için 10 cm'lik hata bile engelleri gözle
#    görülür kaydırır. Şerit metreyle ölç ve OS30A_X_M / OS30A_Z_M ile geç.
#    Bu üç sayı ölçülmeden derinlik kaynağını açmak, zemini engel işaretleyip
#    Nav2'yi hiç rota üretemez hâle getirebilir.
: "${OS30A_TF_AKTIF:=$DERINLIK_AKTIF}"
: "${OS30A_X_M:=0.30}"    # ⚠️ PLACEHOLDER — base_link'ten kamera gövdesine ileri
: "${OS30A_Y_M:=0.0}"     # ⚠️ PLACEHOLDER — yanal kaçıklık
: "${OS30A_Z_M:=0.20}"    # ⚠️ PLACEHOLDER — base_link'ten kamera gövdesine yukarı
if [ "$OS30A_TF_AKTIF" = "1" ]; then
    ros2 run tf2_ros static_transform_publisher \
        "$OS30A_X_M" "$OS30A_Y_M" "$OS30A_Z_M" 0 0 0 \
        base_link dm_base_frame > "$LOG/tf_os30a.log" 2>&1 &
fi
sleep 4
# SLAM parametre dosyası AÇIKÇA geçiliyor. Geçilmediğinde slam_toolbox kendi
# stok ayarlarıyla açılıyordu ve config/mapper_params_online_sync.yaml'ın
# tamamı — çözünürlük, çerçeveler, döngü kapama — hiç uygulanmıyordu; dosya
# duruyor ama hiçbir şey yapmıyor görünmüyordu.
#
# Tarama konusu çalışma anında eziliyor: launch dosyası tek tek parametre
# almıyor, yalnız dosya alıyor. Kopya log dizinine üretiliyor ki depodaki
# yaml sahada değişmesin.
if [ "$SLAM_AKTIF" != "1" ]; then
    echo "SLAM kapalı (SLAM_AKTIF=0) — harita üretilmiyor, Nav2 odom'da sürüyor"
else
    _SLAM_PARAMS="$LOG/mapper_params.runtime.yaml"
    sed "s|^\( *scan_topic: *\).*|\1$SLAM_SCAN_TOPIC|" \
        "$WS/config/mapper_params_online_sync.yaml" > "$_SLAM_PARAMS"
    echo "[SLAM] tarama konusu: $SLAM_SCAN_TOPIC"
    ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:="$_SLAM_PARAMS" > "$LOG/slam.log" 2>&1 &
    sleep 14
    # map_image_node'un tek girdisi /map — SLAM'siz hiç kare üretmez ve
    # açılış doğrulaması onu boşuna "ayağa kalkmadı" diye rapor eder.
    ros2 run teknofest_ika map_image_node > "$LOG/map_image.log" 2>&1 &
    sleep 3
fi

# ── Nav2 yolu (tabela → arazi profili → planlama) ────────────────────────────
# Zincir: ön kamera → yolo_detection → yolo_adapter → /yolo/class_id →
#         terrain_adapter → Nav2 controller parametreleri.
# Engel katmanları: /scan/filtered (preprocessing), /costmap/cone_cloud
# (cone_fusion), /moving_obs_cloud (kayar_engel_costmap), /apc/points/data_raw.
#
# NAV2_AKTIF=1 yapmadan önce ÜÇ ön koşul sağlanmalı, aksi halde Nav2 aracı
# yanlış konumlandırır:
#   1) ENKODER_AKTIF=1 ve enkoder ölçümü doğrulanmış olmalı — Nav2 konumu
#      /odometry/filtered'dan alır, o da enkoder odometrisinden türer.
#   2) LIDAR_YAW_RAD gerçek montaj açısına ayarlanmış olmalı (bkz. yukarısı).
#   3) LiDAR AKIYOR olmalı. Kayan hedef tek girdisini taramadan alıyor ve
#      /scan/filtered yoksa ilk döngüde `failed` dönüyor (olcum_bayat_mi
#      "hiç gelmedi"yi bayatla aynı sınıfa koyuyor), yani 11 aşama saniyeler
#      içinde tükenip koşu MISSION_ABORT'la biter. İki costmap'in tek duvar
#      kaynağı da `scan`.
#   4) YALNIZ `harita` modunda: config/waypoints.yaml'daki koordinatlar
#      doldurulmuş olmalı; hepsi 0.0 iken misyon_fsm her istasyonu aynı
#      noktaya gönderir. `kayan` modda bu dosyadan yalnız `mesafe_m`
#      okunuyor, koordinatlar hiç kullanılmıyor.
if [ "$NAV2_AKTIF" = "1" ]; then
    # Atış aşaması (waypoints.yaml type: shoot) targeting_node'a bağlı. Taret
    # kapalıyken misyon_fsm o istasyona girer, 15 s timeout'a düşer, üç deneme
    # yapar ve puan alamadan devam eder — sessizce, koşu süresinden yiyerek.
    if [ "$TARET_AKTIF" != "1" ]; then
        echo "UYARI: NAV2_AKTIF=1 ama TARET_AKTIF=0 — atış aşaması (§6.10)" \
             "boşa yanacak, koşu süresinden ~30-60 s gider."
    fi
    # LiDAR kararın verildiği YERDE söyleniyor. Cihazın yokluğu yukarıda da
    # yazılıyor ama o satır burayla arasındaki bekleme sürelerinde ekrandan
    # akıp gidiyor ve operatör otonomu başlatana kadar sorunu görmüyor.
    # Nav2 yine de başlatılıyor: tarama olmadan koşmak faydasız ama zararlı
    # değil ve yığını teşhis için ayağa kaldırmak gerekebiliyor.
    if [ "$_LIDAR_VAR" != "1" ]; then
        echo "🔴 UYARI: NAV2_AKTIF=1 ama LiDAR AKMIYOR."
        case "$HEDEFLEME_MODU" in
            kayan|"" |oto)
                echo "   Kayan hedef tek girdisini taramadan alıyor:" \
                     "her aşama ilk döngüde 'failed' döner ve koşu" \
                     "saniyeler içinde MISSION_ABORT'la biter."
                ;;
            *)
                echo "   Costmap'lerin tek duvar kaynağı 'scan':" \
                     "Nav2 bariyerleri hiç görmez."
                ;;
        esac
        echo "   LiDAR'ı takıp yığını yeniden başlatın."
    fi
    if [ "$ENKODER_AKTIF" != "1" ]; then
        echo "UYARI: NAV2_AKTIF=1 ama ENKODER_AKTIF=0 — Nav2 sabit odometriyle" \
             "aracı hareketsiz sanar. Nav2 başlatılmadı."
    else
        # EKF: /odom + /imu/data → /odometry/filtered (Nav2'nin odom_topic'i)
        # Düğüm adı açıkça veriliyor: ekf.yaml'daki parametre bloğu
        # `ekf_filter_node:` anahtarının altında ve ROS parametreleri düğüm
        # adına göre eşleşir. Ad tutmazsa blok hiç yüklenmez, EKF sensörsüz
        # varsayılanlarla açılır ve /odometry/filtered boş kalır — hata
        # vermeden. `ros2 run ... ekf_node` bu adı garanti etmiyor.
        ros2 run robot_localization ekf_node --ros-args \
            -r __node:=ekf_filter_node \
            --params-file "$WS/config/ekf.yaml" > "$LOG/ekf.log" 2>&1 &
        sleep 4
        ros2 launch nav2_bringup navigation_launch.py \
            use_sim_time:=false \
            params_file:="$WS/config/nav2_params.yaml" > "$LOG/nav2.log" 2>&1 &
        sleep 12
        # 🔴 lifecycle_manager `autostart: true` ile geçişleri SABİT bir zaman
        # aşımıyla sürüyor. Bir düğümün `configure`'ı o bütçeyi aşarsa manager
        # bringup'ın TAMAMINI iptal eder ve BİR DAHA DENEMEZ; log'da tek satır
        # kalır, düğümler `unconfigured`'da durur ve Nav2 hiç hedef kabul
        # etmez. Yakalanan düğüm her koşuda başkası olduğu için arıza
        # "aralıklı" görünüyor — sebebi Jetson'ın o anki yüküdür.
        #
        # Kontrol tek yönlü: iki çekirdek düğüm `active` değilse bringup bir
        # kez daha denenir. İkinci deneme de tutmazsa durum adıyla basılır;
        # kör bir döngü, sorunu gizlemekten başka bir şey yapmaz.
        _nav2_aktif_mi() {
            for _d in bt_navigator controller_server; do
                ros2 lifecycle get "/$_d" 2>/dev/null | grep -q '^active' || return 1
            done
            return 0
        }
        if ! _nav2_aktif_mi; then
            echo "UYARI: Nav2 lifecycle geçişi tamamlanmadı — bringup yeniden" \
                 "başlatılıyor (autostart sabit zaman aşımı)."
            pkill -f navigation_launch.py 2>/dev/null
            # Sabit bir bekleme YETMEZ. navigation_launch.py düğümleri ayrı
            # süreç olarak açıyor (use_composition varsayılanı False) ve
            # launch'ın kapanış merdiveni kademeli: SIGINT → sigterm_timeout
            # → SIGTERM → sigkill_timeout → SIGKILL, ikisinin de varsayılanı
            # 5 saniye (launch/actions/execute_local.py). Yani en kötü hâlde
            # kapanış 10 saniye sürüyor; altında kalan bir sleep, ikinci
            # bringup'ı eskiler hâlâ çıkarken başlatır ve aynı adda iki düğüm
            # doğar. Süre değil DURUM bekleniyor.
            # Tespit KOMUT SATIRINDAN değil SÜREÇ ADINDAN (comm) yapılıyor:
            # `pgrep -f` çağıranın kendi komut satırını da tarar ve desen o
            # satırda geçtiği an fonksiyon kendini bulur, yani hiç nav2 süreci
            # yokken bile 15 saniye bekler (denendi, tam bu oldu).
            # ⚠ comm 15 KARAKTERE KIRPILIYOR (ölçüldü): `controller_server`
            # 17 hane olduğu için çekirdekte `controller_serv` duruyor.
            # Adlar navigation_launch.py'deki executable'lardan alındı ve
            # kırpılmış hâlleriyle yazıldı — kısaltmalar yazım hatası değil.
            _nav2_ayakta() {
                pgrep "^(controller_serv|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smooth|waypoint_follow|lifecycle_manag)$" \
                    >/dev/null 2>&1
            }
            _bekle=0
            while _nav2_ayakta && [ "$_bekle" -lt 15 ]; do
                sleep 1
                _bekle=$((_bekle + 1))
            done
            if _nav2_ayakta; then
                echo "   Nav2 düğümleri ${_bekle}s'de kapanmadı — adıyla kapatılıyor."
                for _n in controller_server planner_server bt_navigator \
                          behavior_server smoother_server velocity_smoother \
                          waypoint_follower lifecycle_manager; do
                    pkill -f "$_n" 2>/dev/null
                done
                sleep 3
            fi
            ros2 launch nav2_bringup navigation_launch.py \
                use_sim_time:=false \
                params_file:="$WS/config/nav2_params.yaml" \
                > "$LOG/nav2_ikinci.log" 2>&1 &
            sleep 15
            if _nav2_aktif_mi; then
                echo "Nav2 ikinci denemede ayağa kalktı."
            else
                echo "🔴 Nav2 AYAĞA KALKMADI. Durumlar:"
                for _d in bt_navigator controller_server planner_server \
                          behavior_server smoother_server velocity_smoother; do
                    echo "   /$_d: $(ros2 lifecycle get "/$_d" 2>/dev/null || echo 'yok')"
                done
            fi
        fi
        # Tabela → arazi profili zinciri
        ros2 run teknofest_ika yolo_adapter_node  > "$LOG/yolo_adapter.log" 2>&1 &
        sleep 2
        ros2 run teknofest_ika terrain_adapter    > "$LOG/terrain_adapter.log" 2>&1 &
        sleep 2
        # Costmap besleyicileri
        ros2 run teknofest_ika cone_fusion_node       > "$LOG/cone_fusion.log" 2>&1 &
        sleep 2
        ros2 run teknofest_ika kayar_engel_kalman     > "$LOG/kayar_kalman.log" 2>&1 &
        sleep 2
        ros2 run teknofest_ika kayar_engel_costmap    > "$LOG/kayar_costmap.log" 2>&1 &
        sleep 2
        # HEDEFLEME_MODU: harita | kayan | oto (boş → waypoints.yaml'daki değer).
        #   harita  waypoint koordinatları map çerçevesinde Nav2'ye verilir;
        #           `parkur_cad.donusum` ölçülmüş ve `waypoint:` alanları
        #           doldurulmuş olmalı, yoksa koşu parkuru sürmeden biter.
        #   kayan   hedef her döngüde LiDAR taramasından üretilir; haritaya da
        #           waypoint koordinatına da ihtiyaç yok, aşamalar mesafe_m
        #           kadar yol kat edilince biter.
        #   oto     waypoint'ler doluysa harita, hepsi (0,0) ise kayan.
        if [ -n "$HEDEFLEME_MODU" ]; then
            ros2 run teknofest_ika misyon_fsm --ros-args \
                -p hedefleme_modu:="$HEDEFLEME_MODU" > "$LOG/misyon_fsm.log" 2>&1 &
        else
            ros2 run teknofest_ika misyon_fsm         > "$LOG/misyon_fsm.log" 2>&1 &
        fi
        sleep 2
        echo "Nav2 yolu başlatıldı (tabela → terrain_adapter → controller)"
    fi
fi

# ── Taret ────────────────────────────────────────────────────────────────────
if [ "$TARET_AKTIF" = "1" ]; then
    # HSV+Hough+PID hedef takibi → /turret/cmd, ve /targeting/status.
    # misyon_fsm'in ShootState'i nişan onayını YALNIZ bu topic'ten alır;
    # düğüm koşmazsa üç atış denemesi de zaman aşımına düşer.
    ros2 run teknofest_ika targeting_node  > "$LOG/targeting.log" 2>&1 &
    sleep 3
    case "$TARET_YOLU" in
        uno)
            ros2 run teknofest_ika taret_rc_koprusu \
                > "$LOG/taret_koprusu.log" 2>&1 &
            ;;
        pca9685)
            ros2 run teknofest_ika servo_controller_node \
                > "$LOG/servo_controller.log" 2>&1 &
            ;;
        *)
            echo "UYARI: TARET_YOLU='$TARET_YOLU' tanınmadı (uno|pca9685) —" \
                 "aktüasyon başlatılmadı, taret yalnız nişan alır."
            ;;
    esac
    sleep 2
    echo "Taret yolu başlatıldı: $TARET_YOLU"
fi

# ── İzleme ───────────────────────────────────────────────────────────────────
# Sensör canlılık bekçisi: /scan_lidar, /scan/filtered, /odom, /imu/data ve
# tespit akışını zaman aşımıyla izleyip /sensor/fault basar. Yığındaki sessiz
# kopuklukları ilk fark edecek düğüm budur — mesajları deserialize etmediği
# için maliyeti düşük.
# nav2_aktif: EKF ve yolo_adapter yalnız Nav2 yolunda ayağa kalkıyor; kapalıyken
# izlenirlerse watchdog kalıcı sahte arıza raporlar.
# ham_tarama_topic: bu betikte sürücü doğrudan /scan basıyor (launch yolunda
# scan_relay /scan_lidar basar). Yanlış ad = kalıcı "LiDAR veri gelmedi".
# batarya_izle: /battery/status'un yayıncısı bms_koprusu ve o aşağıda koşulsuz
# başlıyor. Sessizlik gerçek bir arızadır — BLE servisi düşmüş ya da hattı
# başka bir istemci kapmıştır — ve /sensor/fault yalnız panoya gidiyor, hiçbir
# kilit buna bağlı değil.
# imu_izle: BNO055 karta takılı ve /imu/data 50 Hz akıyor. Çip takılıyken
# sessizlik gerçek arızadır ve KENDİLİĞİNDEN DÜZELMEZ — kart BNO'yu bir kez
# bulduktan sonra kablo koparsa yeniden aramıyor, kartın yeniden başlatılması
# gerekiyor. Görülmezse koşu yön kaynağı olmadan sürer. Çip sökülürse
# BNO_TAKILI=0 ile kapatılır, yoksa kalıcı sahte alarm olur.
ros2 run teknofest_ika watchdog --ros-args \
    -p nav2_aktif:="$([ "$NAV2_AKTIF" = "1" ] && echo true || echo false)" \
    -p batarya_izle:=true \
    -p imu_izle:="$([ "$BNO_TAKILI" = "1" ] && echo true || echo false)" \
    -p ham_tarama_topic:=/scan          > "$LOG/watchdog.log" 2>&1 &
sleep 2
# Batarya: gerilim seri hattan gelmiyor, elektrik tarafının BLE servisi
# :8091'de düz JSON yayınlıyor. Köprü bayat okumayı bilerek yayınlamıyor;
# panoda "batarya yok" görünürse ilk şüpheli, BLE'nin tek istemci kabul etmesi
# (telefondan BMS uygulamasına bağlanılmış olması). Düğüm kipten bağımsız:
# gerilim izleme, IMU güvenlik dalı kapalıyken de gerekli.
ros2 run teknofest_ika bms_koprusu   > "$LOG/bms.log" 2>&1 &
sleep 2
if [ "$IMU_GUVENLIK_AKTIF" = "1" ]; then
    ros2 run teknofest_ika imu_guvenlik  > "$LOG/imu_guvenlik.log" 2>&1 &
    sleep 2
fi
if [ "$KAYIT_AKTIF" = "1" ]; then
    ros2 run teknofest_ika veri_paketi   > "$LOG/veri_paketi.log" 2>&1 &
    sleep 2
fi
ros2 run foxglove_bridge foxglove_bridge > "$LOG/foxglove.log" 2>&1 &
sleep 2
# Nişan paneli işlenmiş görüntüyü alsın (çevirme preprocessing'de yapılıyor).
# İKİ PANO BİRDEN — biri ötekinin yerine geçmiyor.
#
# 3 Eylül'de web_dashboard.py kapatılmıştı ve gerekçesi o gün DOĞRUYDU: köprü
# /dev/mega arıyordu, o cihaz yoktu, telemetride araç alanlarının tamamı null
# geliyordu ve /kare/ uçları 503 dönüyordu. Pano boş bir kabuktu.
#
# O gerekçe köprü 0x30 bloğunu konuşmaya başlayınca ortadan kalkıyor: /kart/*
# konuları ancak bu panoda görünüyor (link bayrakları, paket sayaçları,
# protokol/yapı, kip, ham CH1/CH9, enkoder sayımı, HATA_*/DRM_*, 0x3E ayar
# turnikesi). Otonom öncesi doğrulamaların hepsi buradan yapılıyor.
#
# Statik pano 8080'de kalıyor: kartın ASCII teşhis akışını gösteriyor ve o iş
# bizimkinde yok. İkisi aynı portu açamayacağı için ROS panosu PANO_PORT'a
# alındı. ⚠ seri_kopru portu exclusive açtığı için statik panoyu besleyen
# f767_telemetri (:8092) köprü çalışırken susar — beklenen davranış.
python3 -m http.server 8080 --bind 0.0.0.0 --directory /home/lydia/pano \
    > "$LOG/ika_pano.log" 2>&1 &
sleep 1
# Nişan paneli işlenmiş görüntüyü alsın (çevirme preprocessing'de yapılıyor).
PANO_PORT="$PANO_PORT" python3 "$WS/scripts/web_dashboard.py" --ros-args \
    -r /camera/taret/image_raw:=/camera/taret/image_processed \
    > "$LOG/web_dashboard.log" 2>&1 &
sleep 2

# Panoyu besleyen iki veri servisi. 🔴 3 Eylül 2026'ya kadar ikisi de ELLE
# başlatılıyordu: araç yeniden başladığında pano boş bir kabuk oluyordu ve
# bunu kimse fark etmiyordu. Artık açılışın parçası.
#
# ⚠ İkisi de ROS düğümü DEĞİL, düz HTTP servisi — aşağıdaki `_BEKLENEN`
# düğüm doğrulaması bunları görmez; kontrol port dinlemesiyle yapılıyor.
#
# 🔴 Telemetri ve seri_kopru AYNI seri portu açamaz. Denendiğinde çekirdek
# "multiple access on port" diyor ve iki okuyucu çerçeveleri paylaşıyor:
# 8 Eylül'de 41 port yeniden açılışı, 13 okuma hatası, 13 XOR hatası çıktı.
# Bölünmenin izi çerçevede görünüyordu — `aa 01 00 00 35 55 aa 36`, ortada
# `55 aa`, yani bir çerçevenin sonu artı sonrakinin başı.
#
# Kimin kazandığı açılış sırasına kalıyor ve bu bir YARIŞ: telemetri kazanırsa
# /kart/* konularının tamamı boş kalır, yani odometri, IMU, RC, E-STOP ve mod
# birden susar. Köprü hattın sahibi olduğu için çakışmada telemetri geri
# çekilir — teşhis servisi, sürüşün kendisinden önce gelemez.
if [ "$F767_TELEMETRI_AKTIF" = "1" ] && [ "$F767_TELEMETRI_PORT" = "$SERI_PORT" ]; then
    echo "F767 telemetrisi başlatılmadı: $SERI_PORT'u seri_kopru kullanıyor" \
         "(aynı portu iki okuyucu paylaşamaz)."
    F767_TELEMETRI_AKTIF=0
fi
if [ "$F767_TELEMETRI_AKTIF" = "1" ]; then
    if [ -e "$F767_TELEMETRI_PORT" ]; then
        python3 -u /home/lydia/f767_telemetri.py \
            --port "$F767_TELEMETRI_PORT" --http "$F767_TELEMETRI_HTTP" \
            > "$LOG/f767_telemetri.log" 2>&1 &
        sleep 2
    else
        echo "UYARI: F767 ($F767_TELEMETRI_PORT) bulunamadı — telemetri servisi başlatılmadı"
    fi
fi

# BMS: BLE tarama açılışta yavaş olabiliyor, bu yüzden en sona bırakıldı ve
# başlaması beklenmiyor. Koparsa servis kendi içinde yeniden bağlanıyor.
if [ "$BMS_AKTIF" = "1" ] && [ -n "$BMS_MAC" ]; then
    python3 -u /home/lydia/bms_ble/jk_servis.py \
        --mac "$BMS_MAC" --port "$BMS_HTTP" \
        > "$LOG/bms_jk.log" 2>&1 &
fi

# ── Açılış doğrulaması ───────────────────────────────────────────────────────
# Betik buraya kadar yirmiden fazla süreç başlatıp hiçbirinin ayağa kalkıp
# kalkmadığına bakmıyordu; bir düğüm seri portu açamayınca ya da modeli
# yükleyemeyince tek belirti kendi log dosyasındaki satır oluyor ve arıza ancak
# araç komuta cevap vermeyince fark ediliyordu. Aşağısı ucuz bir varlık
# kontrolü: beklenen adlar `ros2 node list` çıktısında aranır.
# Düğümlerin ÇALIŞTIĞINI değil AYAKTA olduğunu söyler — topic akışı için
# ros2 topic hz'e bakmak gerekir.
#
# Liste tek atışta okunmaz. Bir önceki oturum SIGKILL'le kapandıysa ölen DDS
# katılımcıları veda mesajı yayınlayamıyor ve yeni katılımcının keşif
# veritabanı bir süre kirli kalıyor; tek okumayla sekiz sağlam düğüm birden
# "ayağa kalkmadı" diye raporlanabiliyordu. Sahte uyarı, kontrolün kendisini
# değersizleştirdiği için gerçek arızayı kaçırmakla aynı sonucu veriyor.
_BEKLENEN="seri_kopru mod_yoneticisi ackermann_converter e_stop_node
           anti_rollback preprocessing_node yolo_detection_node
           watchdog web_dashboard bms_koprusu"
[ "$SLAM_AKTIF" = "1" ] && _BEKLENEN="$_BEKLENEN map_image_node"
if [ "$NAV2_AKTIF" = "1" ]; then
    _BEKLENEN="$_BEKLENEN ekf_filter_node controller_server yolo_adapter_node
               terrain_adapter cone_fusion_node kayar_engel_kalman
               kayar_engel_costmap misyon_fsm"
fi
[ "$TARET_AKTIF" = "1" ]        && _BEKLENEN="$_BEKLENEN targeting_node"
[ "$IMU_GUVENLIK_AKTIF" = "1" ] && _BEKLENEN="$_BEKLENEN imu_guvenlik"

sleep 10
for _dogrulama in 1 2 3; do
    _CANLI=$(ros2 node list 2>/dev/null)
    _EKSIK=""
    for _n in $_BEKLENEN; do
        echo "$_CANLI" | grep -qx "/$_n" || _EKSIK="$_EKSIK $_n"
    done
    [ -z "$_EKSIK" ] && break
    [ "$_dogrulama" != "3" ] && sleep 5
done
if [ -n "$_EKSIK" ]; then
    echo "UYARI: ayağa kalkmayan düğümler:$_EKSIK"
    echo "       Sebebi kendi log dosyasında: ls -t $LOG | head"
else
    echo "Açılış doğrulaması: beklenen tüm düğümler ayakta."
fi

# Pano ve onu besleyen servisler ROS düğümü değil, düz HTTP servisi: yukarıdaki
# `ros2 node list` kontrolü bunları göremez. Karşılığı port dinlemesidir.
# ⚠ Port dinliyor olmak VERİ AKTIĞI anlamına gelmez — telemetride bunun cevabı
# /telemetri çıktısındaki "bagli" ve "yas" alanlarıdır.
_PORTLAR="8080:statik_pano $PANO_PORT:ros_panosu"
[ "$F767_TELEMETRI_AKTIF" = "1" ] && [ -e "$F767_TELEMETRI_PORT" ] &&
    _PORTLAR="$_PORTLAR $F767_TELEMETRI_HTTP:f767_telemetri"
[ "$BMS_AKTIF" = "1" ] && [ -n "$BMS_MAC" ] &&
    _PORTLAR="$_PORTLAR $BMS_HTTP:bms_jk"

_DINLEYEN=$(ss -tln 2>/dev/null)
_PEKSIK=""
for _pa in $_PORTLAR; do
    _p=${_pa%%:*}; _a=${_pa##*:}
    echo "$_DINLEYEN" | grep -q ":$_p " || _PEKSIK="$_PEKSIK $_a($_p)"
done
if [ -n "$_PEKSIK" ]; then
    echo "UYARI: dinlemeyen web servisleri:$_PEKSIK"
else
    echo "Web servisleri dinliyor:$(echo " $_PORTLAR" | tr " " "\n" | sed "s/^\([0-9]*\):\(.*\)$/ \2:\1/" | tr -d "\n")"
fi

echo "LYDİA açılış yığını başlatıldı — loglar: $LOG"

# ── GÖZCÜ ────────────────────────────────────────────────────────────────────
# `ros2 run ... &` ile başlatılan bir düğüm ölürse GERİ GELMEZ. Yukarıdaki
# doğrulama bir kez, açılışta bakıyor; koşu sırasında hiçbir denetim yoktu ve
# betik `wait`'te bekliyordu.
#
# 🔑 SÜPERVİZYON BAYRAKLA DEĞİL LİSTEYLE GELİR. Her düğüme `Restart=always`
# takmak bazı arızaları arızanın kendisinden kötü hâle getirir; bu yüzden
# beklenen düğümler üç gruba ayrılmış durumda:
#
#   SERBEST  — durumsuz, yeniden başlaması yalnız kendi verisini geri getirir.
#   DİKKATLİ — yeniden başlatılabilir ama bir ÖN KOŞUL var ve o koşul burada
#              otomatikleştirilemez; gözcü yalnız adıyla ve reçetesiyle uyarır:
#                seri_kopru      portu yeniden açmak kartı resetleyebilir;
#                                ayarlar (0x09) ve enkoder temeli yeniden
#                                kurulmalı, boşalan portu telemetri kapmış
#                                olabilir (F767_TELEMETRI_AKTIF=0 ile başlat)
#                ekf_filter_node odom→base_footprint'in SAHİBİ; eskisinin
#                                gerçekten öldüğü doğrulanmadan ikinci kopya
#                                kalkarsa TF iki konum arasında titrer
#                nav2 düğümleri  lifecycle_manager bond'u tek düğümle onarılamaz;
#                                yeniden başlatılacaksa bringup'ın TAMAMI
#   ASLA     — otomatik yeniden başlatma kabul edilemez:
#                misyon_fsm      0. waypoint'ten başlar ve koşu saati sıfırlanır;
#                                parkurun yarısındaki araç baştan sürmeye kalkar
#                e_stop_node     kaynaklar False başlıyor, yeniden başlarken
#                                kısa süre "E-STOP yok" der
#
# Gözcü SERBEST grubu yeniden başlatır, diğer ikisini adıyla bildirir.
# SERBEST — durumsuz ya da durumunu her döngüde yeniden yayınlayan düğümler.
# Sondaki beşi ilk yazımda hiçbir gruba girmemişti; varsayılan "dokunma"
# olduğu için davranış güvenliydi ama gözcü onları kapsamıyordu. Beşinin de
# yeniden başlatılabilirliği koddan doğrulandı:
#   ackermann_converter  `_mod = None` ile açılır → `_manuel_mi()` False →
#                        fren komutları geçer. Kodun kendi belgesi bunun
#                        bilinçli güvenli yön olduğunu yazıyor ("bilmiyorsam
#                        basmayayım" dersek araç %45 eğimde frensiz kalır).
#                        Ölürse /cmd_vel çevrimi durur ve ARAÇ DURUR.
#   anti_rollback        durumu her kontrol döngüsünde KOŞULSUZ yayınlıyor ve
#                        `_aktif = False` ile açılıyor; yeniden başlatmak
#                        ackermann_converter'da takılı kalmış `True`
#                        override'ı bir döngüde temizler — yani güvenli
#                        olmakla kalmıyor, kayıtlı bir riski onarıyor.
#   watchdog             saf gözlemci, yalnız /sensor/fault yayınlıyor.
#   targeting_node       yalnız atış aşamasında etkin; durumsuz.
#   imu_guvenlik         /speed_limit yayıncısı; ölürse son limit takılı kalır.
_GOZCU_SERBEST="preprocessing_node yolo_detection_node yolo_adapter_node
                terrain_adapter cone_fusion_node kayar_engel_kalman
                kayar_engel_costmap web_dashboard bms_koprusu map_image_node
                mod_yoneticisi ackermann_converter anti_rollback watchdog
                targeting_node imu_guvenlik"
# DİKKATLİ — yeniden başlatılabilir ama ön koşulu otomatikleştirilemiyor.
# 🔑 nav2'nin öteki düğümleri (bt_navigator, planner_server, behavior_server,
# smoother_server, velocity_smoother, lifecycle_manager) bu listede DEĞİL,
# çünkü `_BEKLENEN`'de de yoklar — listeye yazmak hiç eşleşmeyen ölü girdi
# üretiyordu. Tespit yine kayıp değil: nav2'den biri ölünce lifecycle
# manager'ın bond'u tüm yığını indiriyor ve belirti `controller_server`'ın
# kaybolmasıyla burada görünüyor.
_GOZCU_DIKKATLI="seri_kopru ekf_filter_node controller_server"
# Listelerde adı geçmeyen her beklenen düğüm ASLA sayılır: yeni bir düğüm
# eklenip gruplandırılmayı unutursa varsayılan davranış "dokunma" olmalı.
# test_birim.py ayrıca `_BEKLENEN`'deki her adın üç gruptan birinde geçtiğini
# denetliyor — sessizce varsayılana düşen düğüm kalmasın.

# Yeniden başlatma komutu, düğümü İLK başlatan komutla aynı olmak zorunda.
# 🔴 Argümanlı iki düğümün komutu burada İKİNCİ kez yazılı: preprocessing_node
#    `-r /scan_lidar:=/scan` olmadan yanlış konuya abone olur ve HİÇBİR ŞEY
#    üretmez (hata da basmaz), web_dashboard ise nişan panelini köreltir.
#    test_birim.py iki kopyanın argümanlarını karşılaştırıyor.
_gozcu_baslat() {
    case "$1" in
        preprocessing_node)
            ros2 run teknofest_ika preprocessing_node --ros-args \
                -r /scan_lidar:=/scan \
                -p flip_taret:=true \
                -p derinlik_isle:=false >> "$LOG/preprocessing.log" 2>&1 &
            ;;
        web_dashboard)
            PANO_PORT="$PANO_PORT" python3 "$WS/scripts/web_dashboard.py" --ros-args \
                -r /camera/taret/image_raw:=/camera/taret/image_processed \
                >> "$LOG/web_dashboard.log" 2>&1 &
            ;;
        *)
            ros2 run teknofest_ika "$1" >> "$LOG/$1.yeniden.log" 2>&1 &
            ;;
    esac
}

if [ "$GOZCU_AKTIF" != "1" ]; then
    echo "Gözcü kapalı (GOZCU_AKTIF=0) — ölen düğüm geri gelmez"
    wait
else
    echo "Gözcü açık — ${GOZCU_PERIYOT_S}s'de bir yoklama, azami $GOZCU_AZAMI_DENEME deneme"
    rm -f "$LOG"/gozcu_*.kez 2>/dev/null
    _onceki_eksik=""
    while :; do
        sleep "$GOZCU_PERIYOT_S"
        _canli=$(ros2 node list 2>/dev/null)
        # Çıktı tamamen boşsa sorun bizde: ROS ortamı ya da keşif çökmüş,
        # 25 düğümün 25'ini ölmüş sayıp yeniden başlatmak felaket olur.
        [ -z "$_canli" ] && continue

        _simdi_eksik=""
        for _n in $_BEKLENEN; do
            echo "$_canli" | grep -qx "/$_n" || _simdi_eksik="$_simdi_eksik $_n"
        done

        for _n in $_simdi_eksik; do
            # İKİ TUR ÜST ÜSTE eksik olmadan işlem yapılmaz. `ros2 node list`
            # bayat DDS keşfinde sağlam düğümleri de eksik gösteriyor —
            # açılış doğrulamasının üç deneme yapmasının sebebi de bu. Tek
            # turluk bir sarsıntıya bakıp sağlam düğümü öldürmek, gözcünün
            # önlemeye çalıştığı arızanın kendisini üretir.
            case " $_onceki_eksik " in
                *" $_n "*) ;;
                *) continue ;;
            esac

            _grup=asla
            case " $_GOZCU_SERBEST "  in *" $_n "*) _grup=serbest  ;; esac
            case " $_GOZCU_DIKKATLI " in *" $_n "*) _grup=dikkatli ;; esac

            case "$_grup" in
                serbest)
                    _kez_dosya="$LOG/gozcu_$_n.kez"
                    _kez=$(cat "$_kez_dosya" 2>/dev/null || echo 0)
                    if [ "$_kez" -ge "$GOZCU_AZAMI_DENEME" ]; then
                        echo "GÖZCÜ: $_n $_kez kez başlatıldı ve yine öldü — VAZGEÇİLDİ. Sebebi: $LOG/$_n*.log"
                    else
                        _kez=$((_kez + 1))
                        echo "$_kez" > "$_kez_dosya"
                        echo "GÖZCÜ: $_n ölmüş — yeniden başlatılıyor ($_kez/$GOZCU_AZAMI_DENEME)"
                        _gozcu_baslat "$_n"
                    fi
                    ;;
                dikkatli)
                    echo "GÖZCÜ: ⚠ $_n ölmüş — OTOMATİK BAŞLATILMIYOR (ön koşul var, bkz. gözcü notu). Log: $LOG/"
                    ;;
                asla)
                    echo "GÖZCÜ: 🔴 $_n ölmüş — otomatik başlatılmaz, koşuyu bozar. Log: $LOG/"
                    ;;
            esac
        done
        _onceki_eksik="$_simdi_eksik"
    done
fi
