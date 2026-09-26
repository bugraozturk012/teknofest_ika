#!/usr/bin/env python3
"""
taret_rc_koprusu.py  —  RC + Otonom Nişan → Turret UNO Köprüsü
================================================================
Turret UNO'ya giden tek seri yazıcı. Kart 360° sürekli dönüş servolarını
"P:{hiz},T:{hiz}\n" ile HIZ modunda sürer — değerler açı değil hızdır,
90 durdurur, 0 ve 180 iki yönde tam hızdır; "S" acil durdurur.

Düşük hız komutları servo sürtünmesinde yutulduğu için stick ölü bölgeyi
geçer geçmez hız, servonun gerçekten döndüğü minimum değerden başlatılır
(MIN_HIZ_* sabitleri). 360° servoların nötrü tam ortada olmadığından ve
tilt yerçekimine karşı çalıştığından bu eşik eksen ve yön başına ayrıdır.

İki nişan kaynağını önceliklendirir:

1. RC manuel: sağ stick (CH1/CH2) MANUEL modda ve SWB (taret aktif) açıkken
   pan/tilt joystick'i. Kanal ataması ve seri protokol elektrikçi arkadaşın
   turret_control.py + Arduino UNO koduyla birebir uyumlu.
2. Otonom: targeting_node'un /turret/cmd (Vector3, merkezden derece ofseti)
   komutu — atış waypoint'inde HSV+Hough+PID hedef takibi. Servolar fiziksel
   olarak Turret UNO'da olduğundan komut PCA9685'e Jetson I2C'sinden değil,
   bu köprü üzerinden seri porttan iletilir.

Sağ stick MANUEL sürüşte direksiyon olarak da kullanıldığından (CH1), taret
aktifken main.cpp sürüşü zaten kilitler (bkz. rc_taret_aktif() / config.h) —
burada ayrıca mod/aktif doğrulaması, RC bağlantısı kesilirse veya taret
kapatılırsa taretin durması için yapılır — hız modunda komut kesilmesi
"aynı hızda dönmeye devam et" demek olduğundan bu susma kritik. Lazer
yanarken taret hareketi tamamen kilitlenir (şartname §6.10).

Atış (UNO'nun 7. pinindeki röle) üç kaynaktan tetiklenir; hepsi aynı süreli
darbeyi üretir:

* RC: FlySky CH4 (config.h RC_CH_AUX, /rc_input[3]) MANUEL modda yukarı
  alındığında — kolu yukarıda tutmak tek atış verir, tekrar atış için indirip
  yeniden kaldırmak gerekir.
* Klavye: node bir terminalde çalışıyorsa MANUEL modda SPACE tuşu. Terminal
  yoksa (launch/systemd altında stdin tty değil) bu yol kendiliğinden kapalı
  kalır.
* /shoot_command: otonom atış; kapatma komutu kaybolursa failsafe söndürür.
"""

import os
import select
import sys
import termios
import tty

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32MultiArray, UInt8

from teknofest_ika.otonomi.topics import (
    RC_INPUT_TOPIC, MOD_AKTIF_TOPIC, SHOOT_CMD_TOPIC, TURRET_CMD_TOPIC,
    SERIAL_TARET, SERIAL_BAUD_TARET,
)
from teknofest_ika.otonomi.mod_yoneticisi import MOD_MANUAL

RC_PWM_MIN  = 1000
RC_PWM_MAX  = 2000
RC_MOD_ESIK = 1500   # SWB > bu değer → taret aktif (main.cpp/config.h ile aynı eşik)
RC_STALE_S  = 0.3    # main.cpp RC_TIMEOUT_MS ile aynı eşik — bu süre mesaj gelmezse merkeze dön
OTO_STALE_S = 0.5    # /turret/cmd bu süre gelmezse (targeting kapandı/hedef yok) merkeze dön

# UNO firmware'inde otomatik kapatma yok — /shoot_command False kaybolursa
# lazer sonsuza dek yanık kalır. Şartname atış süresi 1s (LAZER_SURE_MS);
# bu emniyet payıyla zorla söndürme eşiği.
LAZER_FAILSAFE_S = 1.5

# RC/klavye atışı tek darbedir: şartname atış süresi (LAZER_SURE_MS) kadar
# yanar, kendiliğinden söner. Klavyede tuş bırakma olayı zaten okunamaz.
ATIS_SURE_S = 1.0
KLAVYE_ATIS_TUSU = b' '
RC_ATIS_ESIK = 1500   # CH4 bu değerin üzerine çıkınca atış (config.h RC_CH_AUX)

# Sabit pan/tilt komutu bu süre dolunca değişmese bile yeniden yazılır —
# kararsız seri hatta tek paket kaybını onarır (bkz. _komut_gonder).
YENIDEN_GONDER_S = 0.5

# Turret UNO 360° sürekli dönüş servolarını HIZ modunda sürer: "P:x,T:y"
# değerleri açı değil hızdır, 90 durdurur, 0 ve 180 iki yönde tam hızdır.
KOMUT_MIN, KOMUT_MAX = 0, 180
DUR = 90

RC_OLU_BOLGE = 20    # [µs] stick merkez toleransı — config.h RC_DEADBAND ile aynı

# Servo ölü bandı: düşük hız komutları sürtünmede yutulur, servo hiç dönmez;
# stick eşiği geçer geçmez buradan başlanır ki hareket anında başlasın.
# 360° servoların nötrü tam ortada olmadığı ve tilt yerçekimine karşı
# çalıştığı için eşik eksen ve yön başına ayrıdır. ART = ham stick değeri
# merkezden büyük olan yön, EKS = küçük olan yön.
MIN_HIZ_PAN_ART,  MIN_HIZ_PAN_EKS  = 12, 12
MIN_HIZ_TILT_ART, MIN_HIZ_TILT_EKS = 20, 20
MAX_HIZ = 80

PAN_YON, TILT_YON = -1, -1   # eksen ters dönüyorsa işareti çevir

# Stick nötr değerleri — kumandada ölçüldü, tam 1500 değil.
PAN_MERKEZ_US, TILT_MERKEZ_US = 1507.0, 1508.0

OTO_MAX_OFSET = 45.0   # targeting_node PID çıkışı bu aralıkta sınırlı
# Hız modunda en küçük hata bile MIN_HIZ kadar dönüş üretir; bu eşiğin
# altındaki ofsette taret durur, yoksa hedefin etrafında salınıp yerleşemez.
OTO_OLU_BOLGE_DEG = 2.0


def _hiz_komutu(oran: float, yon: int, min_art: int, min_eks: int,
                max_hiz: int = MAX_HIZ) -> int:
    """Yön işaretli -1..+1 talebini UNO'nun 0-180 hız komutuna çevirir.

    max_hiz manuel için MAX_HIZ (tam), otonom nişan için daha düşük bir tavan
    (yavaş, kontrollü yaklaşım) — nişanda hızlı dönüş hedefi aşıyordu.
    """
    if oran == 0.0:
        return DUR
    isaret = 1 if oran > 0 else -1
    min_hiz = min_art if oran > 0 else min_eks
    max_hiz = max(min_hiz + 1, max_hiz)   # taban tavanın üstünde kalmasın
    hiz = min_hiz + min(1.0, abs(oran)) * (max_hiz - min_hiz)
    return max(KOMUT_MIN, min(KOMUT_MAX, DUR + int(round(yon * isaret * hiz))))


def _oto_orani(ofset_deg: float) -> float:
    if abs(ofset_deg) < OTO_OLU_BOLGE_DEG:
        return 0.0
    return max(-1.0, min(1.0, ofset_deg / OTO_MAX_OFSET))


def _stick_orani(us: float, merkez: float) -> float:
    sapma = us - merkez
    if abs(sapma) < RC_OLU_BOLGE:
        return 0.0
    yayilim = (RC_PWM_MAX - RC_PWM_MIN) / 2.0 - RC_OLU_BOLGE
    isaret = 1.0 if sapma > 0 else -1.0
    return isaret * min(1.0, (abs(sapma) - RC_OLU_BOLGE) / yayilim)


class TaretRcKoprusu(Node):

    def __init__(self):
        super().__init__('taret_rc_koprusu')

        self.declare_parameter('port',        SERIAL_TARET)
        self.declare_parameter('baud',        SERIAL_BAUD_TARET)
        self.declare_parameter('sim_mode',    False)
        self.declare_parameter('klavye_atis', True)
        # Otonom nişan hız tavanı — manuel MAX_HIZ (80) çok hızlı, nişanda
        # yavaş/kontrollü dönüş için düşük tutulur. Manuel bundan etkilenmez.
        self.declare_parameter('oto_max_hiz', 25)

        self._sim_mode = self.get_parameter('sim_mode').value
        self._ser = None
        # Port açılışı DTR ile UNO'yu resetler; bootloader penceresinde (~2s)
        # yazılan komutlar yutulur, kartı tekrar resetleyebilir. UNO'dan ilk
        # veri (banner/IMU telemetrisi) gelene kadar gönderim kapalı kalır.
        self._link_hazir = self._sim_mode
        if not self._sim_mode:
            try:
                # exclusive: aynı porta ikinci bir node bağlanırsa burada
                # hata alsın. Kilitsiz açılışta ikisi de açabiliyor, UNO'nun
                # açılış banner'ını hangisi önce okursa diğerinin gönderim
                # kapısı hiç açılmıyor ve sessizce ölü kalıyor.
                self._ser = serial.Serial(
                    self.get_parameter('port').value,
                    self.get_parameter('baud').value,
                    timeout=0.1,
                    exclusive=True,
                )
                self.get_logger().info(f'Turret UNO portu açıldı: {self._ser.port}')
            except serial.SerialException as e:
                self.get_logger().error(f'Turret UNO portu açılamadı: {e}')
                self._sim_mode = True

        self._mod             = MOD_MANUAL
        self._pan_us          = (RC_PWM_MIN + RC_PWM_MAX) / 2
        self._tilt_us         = (RC_PWM_MIN + RC_PWM_MAX) / 2
        self._taret_aktif_us  = RC_PWM_MIN
        self._atis_tetik_us   = RC_PWM_MIN
        # None = tetik kolunun durumu henüz bilinmiyor; ilk RC mesajı yalnızca
        # durumu kaydeder, açılışta kol yukarıdaysa kendiliğinden atış olmaz.
        self._atis_tetik_onceki = None
        self._son_rc_zamani   = None
        self._son_pan, self._son_tilt = None, None
        self._son_yazma_zamani = None
        self._oto_pan, self._oto_tilt = DUR, DUR
        self._son_oto_zamani  = None

        qos_be = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(UInt8, MOD_AKTIF_TOPIC, self._mod_cb, 10)
        self.create_subscription(Float32MultiArray, RC_INPUT_TOPIC, self._rc_cb, qos_be)
        self.create_subscription(Vector3, TURRET_CMD_TOPIC, self._turret_cmd_cb, 10)

        # Lazer fiziksel olarak Turret UNO'nun 7. pininde ("L:1"/"L:0" komutu).
        # Bu yol araçta ARTIK KULLANILMIYOR: taret sürüş kartına taşındı ve
        # atış /shoot_command → seri_kopru → 0x03 üzerinden yapılıyor. Düğüm
        # yalnız UNO'lu eski kurulumlar için duruyor, boot betiği başlatmıyor.
        self._lazer_yanik  = False
        self._lazer_acilis = None
        self._lazer_sure   = LAZER_FAILSAFE_S
        self.create_subscription(Bool, SHOOT_CMD_TOPIC, self._lazer_cb, 10)

        self._oto_max_hiz = int(self.get_parameter('oto_max_hiz').value)
        self._klavye_fd, self._klavye_eski = None, None
        self._klavye_kur()

        self.create_timer(0.05, self._komut_gonder)   # 20 Hz
        self.get_logger().info('TaretRcKoprusu hazır — sağ stick MANUEL+SWB ile taret nişanı.')

    def _klavye_kur(self) -> None:
        # stdin tty değilse (launch, systemd, yönlendirilmiş çıktı) klavye
        # atışı sessizce kapalı kalır; terminal ayarına da dokunulmaz.
        if not self.get_parameter('klavye_atis').value:
            return
        if not sys.stdin.isatty():
            return
        try:
            fd = sys.stdin.fileno()
            self._klavye_eski = termios.tcgetattr(fd)
            tty.setcbreak(fd)      # ISIG korunur, Ctrl-C çalışmaya devam eder
            self._klavye_fd = fd
        except (termios.error, ValueError, OSError) as e:
            self._klavye_eski = None
            self.get_logger().warn(f'Klavye atışı açılamadı: {e}')
            return
        self.get_logger().info(
            f'Klavye atışı açık — MANUEL modda SPACE lazeri {ATIS_SURE_S:.1f} s yakar.')

    def _klavye_geri_al(self) -> None:
        if self._klavye_fd is None or self._klavye_eski is None:
            return
        try:
            termios.tcsetattr(self._klavye_fd, termios.TCSADRAIN, self._klavye_eski)
        except (termios.error, OSError):
            pass
        self._klavye_fd, self._klavye_eski = None, None

    def _klavye_oku(self) -> None:
        if self._klavye_fd is None:
            return
        try:
            hazir, _, _ = select.select([self._klavye_fd], [], [], 0)
            if not hazir:
                return
            veri = os.read(self._klavye_fd, 32)
        except (OSError, ValueError):
            self._klavye_fd = None
            return
        if KLAVYE_ATIS_TUSU not in veri:
            return
        self._atis_yap('SPACE')

    def _rc_atis_oku(self) -> None:
        onceki, self._atis_tetik_onceki = self._atis_tetik_onceki, self._atis_tetik_us
        if onceki is None or not self._rc_guncel_mi():
            return
        # Yalnızca yükselen kenar: kol yukarıda tutulurken atış tekrarlanmaz.
        if not (self._atis_tetik_us > RC_ATIS_ESIK >= onceki):
            return
        self._atis_yap('RC atış tetiği (CH4)')

    def _atis_yap(self, kaynak: str) -> None:
        if self._mod != MOD_MANUAL:
            self.get_logger().warn(f'{kaynak} yok sayıldı — atış yalnızca MANUEL modda.')
            return
        if self._lazer_yanik:
            return
        self._lazer_ac(ATIS_SURE_S)
        self.get_logger().info(f'{kaynak} → taret lazeri AÇIK.')

    def _mod_cb(self, msg: UInt8):
        self._mod = int(msg.data)

    def _rc_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            return
        self._pan_us          = float(msg.data[1])   # ch_direksiyon → taret aktifken pan
        self._atis_tetik_us   = float(msg.data[3])   # ch_aux → atış tetiği
        self._tilt_us         = float(msg.data[4])
        self._taret_aktif_us  = float(msg.data[5])
        self._son_rc_zamani   = self.get_clock().now()

    def _turret_cmd_cb(self, msg: Vector3):
        # targeting_node merkezden derece ofseti yayınlar (PID çıkışı, ±45°
        # sınırlı) — servo_controller_node ile aynı semantik: merkez + ofset.
        # PID konum ofseti üretir; hız modunda hata büyüdükçe hızlanan bir
        # orantısal sürüşe çevrilir, servo ölü bandı da burada telafi edilir.
        self._oto_pan  = _hiz_komutu(_oto_orani(msg.x), PAN_YON,
                                     MIN_HIZ_PAN_ART, MIN_HIZ_PAN_EKS,
                                     self._oto_max_hiz)
        self._oto_tilt = _hiz_komutu(_oto_orani(msg.y), TILT_YON,
                                     MIN_HIZ_TILT_ART, MIN_HIZ_TILT_EKS,
                                     self._oto_max_hiz)
        self._son_oto_zamani = self.get_clock().now()

    def _rc_guncel_mi(self) -> bool:
        if self._son_rc_zamani is None:
            return False
        gecen_s = (self.get_clock().now() - self._son_rc_zamani).nanoseconds / 1e9
        return gecen_s < RC_STALE_S

    def _oto_guncel_mi(self) -> bool:
        if self._son_oto_zamani is None:
            return False
        gecen_s = (self.get_clock().now() - self._son_oto_zamani).nanoseconds / 1e9
        return gecen_s < OTO_STALE_S

    def _taret_aktif(self) -> bool:
        return (self._rc_guncel_mi() and self._mod == MOD_MANUAL
                and self._taret_aktif_us > RC_MOD_ESIK)

    def _link_kontrol(self) -> bool:
        if self._link_hazir:
            return True
        if self._ser is None:
            return False
        try:
            if self._ser.in_waiting:
                self._ser.reset_input_buffer()
                self._link_hazir = True
                self.get_logger().info('Turret UNO bağlantısı doğrulandı — gönderim aktif.')
        except (serial.SerialException, OSError):
            pass
        return self._link_hazir

    def _seri_yaz(self, cmd: str) -> None:
        if self._sim_mode or self._ser is None:
            self.get_logger().debug(f'[SIM] {cmd.strip()}')
            return
        if not self._link_hazir:
            return
        try:
            self._ser.write(cmd.encode())
        except serial.SerialException as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)

    def _lazer_ac(self, sure_s: float) -> None:
        self._lazer_yanik  = True
        self._lazer_acilis = self.get_clock().now()
        self._lazer_sure   = sure_s
        self._seri_yaz('L:1\n')

    def _lazer_kapat(self) -> None:
        self._lazer_yanik, self._lazer_acilis = False, None
        self._seri_yaz('L:0\n')

    def _lazer_cb(self, msg: Bool):
        if msg.data:
            self._lazer_ac(LAZER_FAILSAFE_S)
            self.get_logger().info('Taret lazeri AÇIK.')
        else:
            self._lazer_kapat()
            self.get_logger().info('Taret lazeri kapalı.')

    def _komut_gonder(self):
        self._klavye_oku()
        self._rc_atis_oku()

        if not self._sim_mode and not self._link_kontrol():
            return

        # Süre dolunca söndür: klavye atışında normal bitiş, /shoot_command
        # yolunda kapatma komutu kaybolursa failsafe.
        if self._lazer_yanik and self._lazer_acilis is not None:
            gecen_s = (self.get_clock().now() - self._lazer_acilis).nanoseconds / 1e9
            if gecen_s > self._lazer_sure:
                self._lazer_kapat()
                self.get_logger().info('Taret lazeri kapalı — atış süresi doldu.')

        # Lazer yanarken taret hareketi kilitli (şartname §6.10) — atış
        # sırasında nişan bozulmaz, açı komutu hiç gönderilmez.
        if self._lazer_yanik:
            return

        if self._taret_aktif():
            pan  = _hiz_komutu(_stick_orani(self._pan_us, PAN_MERKEZ_US), PAN_YON,
                               MIN_HIZ_PAN_ART, MIN_HIZ_PAN_EKS)
            tilt = _hiz_komutu(_stick_orani(self._tilt_us, TILT_MERKEZ_US), TILT_YON,
                               MIN_HIZ_TILT_ART, MIN_HIZ_TILT_EKS)
        elif self._oto_guncel_mi():
            pan, tilt = self._oto_pan, self._oto_tilt
        else:
            pan, tilt = DUR, DUR

        # Değişmeyen komutu her 20 Hz'lik döngüde yeniden yazmak I2C/seri
        # hattını gereksiz meşgul eder — ama HİÇ tekrarlamamak da riskli:
        # bugünkü kararsız CH340/hub bağlantısında tek bir paket kaybolursa
        # (manuel stick'in doğal titremesi olmayan sabit otonom komutlarda
        # kanıtlandı — sahada gözlemlendi) servo süresiz sessiz kalıyordu.
        # Periyodik yeniden gönderim kayıp paketi kendiliğinden onarır.
        simdi = self.get_clock().now()
        degisti = (pan, tilt) != (self._son_pan, self._son_tilt)
        gecen_s = (
            (simdi - self._son_yazma_zamani).nanoseconds / 1e9
            if self._son_yazma_zamani is not None else YENIDEN_GONDER_S + 1.0
        )
        if not degisti and gecen_s < YENIDEN_GONDER_S:
            return
        self._son_pan, self._son_tilt = pan, tilt
        self._son_yazma_zamani = simdi
        kaynak = ('RC' if self._taret_aktif()
                  else 'OTO' if self._oto_guncel_mi() else 'DUR')
        if degisti:
            self.get_logger().info(
                f'{kaynak} → P:{pan} T:{tilt} (hızP:{pan-DUR:+d} hızT:{tilt-DUR:+d})',
                throttle_duration_sec=0.5)
        self._seri_yaz(f'P:{pan},T:{tilt}\n')

    def destroy_node(self):
        self._klavye_geri_al()
        if self._ser and self._ser.is_open:
            try:
                if self._link_hazir:
                    self._ser.write(b'L:0\n')
                    self._ser.write(b'S\n')
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TaretRcKoprusu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
