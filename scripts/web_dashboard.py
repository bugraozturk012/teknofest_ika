#!/usr/bin/env python3
"""
web_dashboard.py — LYDİA GCS Web Dashboard (Jetson tarafı sunucu)
=================================================================
Eski PyQt dashboard'ın (ika_dashboard.py) yerini alır. Fark mimaridedir:

  ESKİ:  Jetson → (DDS, makineler-arası) → WSL PyQt çizer
         DDS kablosuzda koptuğunda hiçbir şey gelmiyordu.

  YENİ:  Bu sunucu JETSON'DA çalışır, ROS topic'lerine LOKAL abone olur
         (Jetson-içi DDS her zaman sağlam), görüntüleri JPEG'e çevirip
         HTTP ile servis eder. İzleyici herhangi bir tarayıcıdan bakar:
             http://192.168.100.2:8080

  Taşıma katmanları (hepsi düz HTTP/TCP, DDS'ten bağımsız):
    /stream/<ad>  → MJPEG (kamera akışları, JPEG kalite düşük + 480p)
    /veri         → SSE   (sensör değerleri, hazır metin+renk, 5 Hz)
    /estop  POST  → tarayıcıdan gelen E-STOP'u ROS'a yayınlar

Kablosuzda ham görüntü asla taşınmaz — her akış Jetson'da sıkıştırılır,
yalnız o akışa bir tarayıcı bağlıyken kodlanır (CPU tasarrufu).

Çalıştırma (Jetson):  ros2 run yok — düz script:
    python3 ~/lydia_ws/src/teknofest_ika_yazilim/scripts/web_dashboard.py
(ROS ortamı .bashrc'den gelir: ROS_DOMAIN_ID=42 + discovery server.)
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool, UInt8, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, Image, CompressedImage, LaserScan
from cv_bridge import CvBridge

from teknofest_ika.otonomi.topics import (
    E_STOP_TOPIC, E_STOP_FORCE_GCS_TOPIC, MOD_AKTIF_TOPIC, FSM_STATE_TOPIC,
    BATTERY_TOPIC, IMU_TOPIC, EKF_ODOM_TOPIC, MISYON_WP_INDEX_TOPIC,
    TARGETING_STATUS_TOPIC, TARGETING_DEBUG_TOPIC, SHOOT_RESULT_TOPIC,
    CAMERA_IMAGE_TOPIC, CAMERA_TARET_TOPIC, YOLO_RAW_DEBUG_TOPIC,
    MAP_IMAGE_TOPIC, DEPTH_IMAGE_TOPIC, SCAN_TOPIC,
)

# ── Ayarlar ───────────────────────────────────────────────────────────────────
PORT        = 8080
JPEG_KALITE = 35     # düşük = küçük dosya, kablosuz için uygun
MAKS_GENIS  = 480    # kameralar bu genişliğe küçültülür (en-boy korunur)
AKIS_FPS    = 12     # MJPEG akış tavanı (izleme için yeterli)
PANELLER    = {'on', 'yolo', 'nisan', 'harita', 'derinlik', 'lidar'}
# Bu süreden eski kare "sinyal yok" sayılır — donmuş görüntü canlı sanılırsa
# yanlış karar verdirir. SLAM haritası seyrek yayınlandığı için (~5 s) kamera
# eşiğine tabi tutulursa panel sürekli boş görünür, ayrı eşiği var.
KARE_TAZE_S = 3.0
KARE_TAZE_OZEL = {'harita': 30.0}
LIDAR_PANEL_PX  = 400   # radar görüntüsü kenar uzunluğu [px]
LIDAR_MENZIL_M  = 5.0   # panelin kapsadığı yarıçap [m]
# LiDAR montaj yönünü ekran eksenine çevirir: X'te aynalayıp saat yönünde 90°
# döndürünce "yukarı = aracın önü" olur. Yalnız görselleştirme, /scan değişmez.
LIDAR_AYNA_X    = True
LIDAR_DONDUR    = -math.pi / 2.0   # saat yönünde 90°
NISAN_DEBUG_TAZE_S = 1.5   # işaretli nişan görüntüsü bu kadar tazeyse ham yerine o

# Eski dashboard renk paleti (birebir korundu)
BG, PANEL, CARD, LINE = '#181818', '#202020', '#242424', '#2e2e2e'
TEXT, DIM = '#d4d4d4', '#5a5a5a'
GREEN, RED, AMBER, CYAN = '#4caf50', '#ef5350', '#ffa726', '#26c6da'

MOD_ISIMLER = {0: 'MANUAL', 1: 'SEMI AUTO', 2: 'FULL AUTO'}
MOD_RENK    = {'MANUAL': AMBER, 'SEMI AUTO': CYAN, 'FULL AUTO': GREEN}
TARGETING_R = {'SEARCHING': AMBER, 'LOCKED': GREEN, 'ALIGNED': CYAN,
               'STANDBY': DIM, 'NO_IMAGE': RED, 'STALE_IMAGE': RED}


# ── Paylaşılan durum ──────────────────────────────────────────────────────────
class Ortak:
    def __init__(self):
        self.kilit = threading.Lock()
        # panel adı → (bgr numpy, zaman)
        self.kareler = {'on': (None, 0.0), 'yolo': (None, 0.0),
                        'harita': (None, 0.0), 'nisan_ham': (None, 0.0),
                        'nisan_dbg': (None, 0.0), 'derinlik': (None, 0.0),
                        'lidar': (None, 0.0)}
        self.sensor = {
            'e_stop': False, 'mod': 0, 'fsm': '—',
            'batarya': -1.0, 'voltaj': 0.0, 'akim': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'hiz': 0.0, 'wp': 0,
            'targeting': '—', 'shoot': False,
        }
        self.son_ros = 0.0   # en son herhangi bir ROS mesajı zamanı

    def kare_yaz(self, ad, bgr):
        with self.kilit:
            self.kareler[ad] = (bgr, time.time())

    def kare_oku(self, ad):
        with self.kilit:
            return self.kareler.get(ad, (None, 0.0))


ortak = Ortak()


# ── ROS düğümü ────────────────────────────────────────────────────────────────
class WebDashboardNode(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self._kopru = CvBridge()
        be = qos_profile_sensor_data

        self.create_subscription(Bool, E_STOP_TOPIC, self._estop, 10)
        self.create_subscription(UInt8, MOD_AKTIF_TOPIC, self._mod, 10)
        self.create_subscription(String, FSM_STATE_TOPIC, self._fsm, 10)
        self.create_subscription(BatteryState, BATTERY_TOPIC, self._bat, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._imu, be)
        self.create_subscription(Odometry, EKF_ODOM_TOPIC, self._odom, be)
        self.create_subscription(UInt8, MISYON_WP_INDEX_TOPIC, self._wp, 10)
        self.create_subscription(String, TARGETING_STATUS_TOPIC, self._targeting, 10)
        self.create_subscription(Bool, SHOOT_RESULT_TOPIC, self._shoot, 10)

        self.create_subscription(Image, CAMERA_IMAGE_TOPIC,
                                 lambda m: self._ham_kare('on', m), be)
        self.create_subscription(Image, YOLO_RAW_DEBUG_TOPIC,
                                 lambda m: self._ham_kare('yolo', m), be)
        self.create_subscription(Image, MAP_IMAGE_TOPIC,
                                 lambda m: self._ham_kare('harita', m), be)
        self.create_subscription(Image, CAMERA_TARET_TOPIC,
                                 lambda m: self._ham_kare('nisan_ham', m), be)
        # Derinlik kamerası — sürücü rgb8 yayınlar (renklendirme kendisinde),
        # cv_bridge bgr8'e çevirir, ek işlem gerekmez.
        self.create_subscription(Image, DEPTH_IMAGE_TOPIC,
                                 lambda m: self._ham_kare('derinlik', m), be)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._tarama, be)
        # Nişan işaretli görüntü — targeting_node JPEG yayını (kablosuz dostu).
        self.create_subscription(CompressedImage, TARGETING_DEBUG_TOPIC + '/compressed',
                                 self._sik_nisan, be)

        self._estop_pub = self.create_publisher(Bool, E_STOP_FORCE_GCS_TOPIC, 10)

    # — görüntü —
    def _ham_kare(self, ad, msg):
        try:
            ortak.kare_yaz(ad, self._kopru.imgmsg_to_cv2(msg, 'bgr8'))
            ortak.son_ros = time.time()
        except Exception:
            pass

    def _tarama(self, msg: LaserScan):
        """Ham LaserScan'i tepeden bakışlı radar görüntüsüne çizer.

        SLAM haritası birikimlidir ve gecikmelidir; bu panel o anki lazer
        dönüşünü gösterir, engel aniden girdiğinde ilk oradan görülür.
        """
        try:
            boy = LIDAR_PANEL_PX
            im = np.zeros((boy, boy, 3), np.uint8)
            mrk = boy // 2
            olcek = (boy / 2 - 6) / max(LIDAR_MENZIL_M, 0.1)

            # menzil halkaları (1 m aralık) + eksen çizgileri
            for r in range(1, int(LIDAR_MENZIL_M) + 1):
                cv2.circle(im, (mrk, mrk), int(r * olcek), (38, 38, 38), 1)
            cv2.line(im, (mrk, 0), (mrk, boy), (30, 30, 30), 1)
            cv2.line(im, (0, mrk), (boy, mrk), (30, 30, 30), 1)

            aci = msg.angle_min
            for mesafe in msg.ranges:
                if 0.02 < mesafe < LIDAR_MENZIL_M:
                    # ROS: +x ileri, +y sola. Ekran: yukarı = ileri.
                    g = (-aci if LIDAR_AYNA_X else aci) + LIDAR_DONDUR
                    x = int(mrk + mesafe * math.sin(g) * olcek)
                    y = int(mrk - mesafe * math.cos(g) * olcek)
                    if 0 <= x < boy and 0 <= y < boy:
                        # tek piksel JPEG sıkıştırmasında kayboluyor
                        cv2.circle(im, (x, y), 1, (90, 255, 90), -1)
                aci += msg.angle_increment

            cv2.circle(im, (mrk, mrk), 2, (60, 160, 255), -1)   # araç
            cv2.putText(im, f'{LIDAR_MENZIL_M:.0f}m', (6, boy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
            ortak.kare_yaz('lidar', im)
            ortak.son_ros = time.time()
        except Exception:
            pass

    def _sik_nisan(self, msg):
        try:
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                ortak.kare_yaz('nisan_dbg', img)
                ortak.son_ros = time.time()
        except Exception:
            pass

    # — sensör —
    def _dokun(self):
        ortak.son_ros = time.time()

    def _estop(self, m):     ortak.sensor['e_stop'] = m.data; self._dokun()
    def _mod(self, m):       ortak.sensor['mod'] = int(m.data); self._dokun()
    def _fsm(self, m):       ortak.sensor['fsm'] = m.data; self._dokun()
    def _wp(self, m):        ortak.sensor['wp'] = int(m.data); self._dokun()
    def _targeting(self, m): ortak.sensor['targeting'] = m.data; self._dokun()
    def _shoot(self, m):     ortak.sensor['shoot'] = m.data; self._dokun()

    def _bat(self, m):
        ortak.sensor['batarya'] = m.percentage * 100 if m.percentage >= 0 else -1.0
        ortak.sensor['voltaj'] = m.voltage
        ortak.sensor['akim'] = m.current
        self._dokun()

    def _imu(self, m):
        q = m.orientation
        ortak.sensor['roll'] = math.degrees(
            math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2)))
        sinp = 2 * (q.w * q.y - q.z * q.x)
        ortak.sensor['pitch'] = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        self._dokun()

    def _odom(self, m):
        ortak.sensor['hiz'] = m.twist.twist.linear.x
        self._dokun()

    def gcs_estop(self, aktif):
        self._estop_pub.publish(Bool(data=bool(aktif)))


# ── Sensör → hazır metin + renk (renk mantığı eski dashboard ile birebir) ─────
def sensor_json():
    s = ortak.sensor
    bagli = (time.time() - ortak.son_ros) < 3.0

    mod = MOD_ISIMLER.get(s['mod'], '?')
    fsm = s['fsm'] or '—'
    fsm_renk = (RED if any(k in fsm for k in ('EMERGENCY', 'ERROR')) else
                AMBER if 'STOP' in fsm else
                DIM if fsm in ('IDLE', '—') else TEXT)
    r, p, h, b, v, a = (s['roll'], s['pitch'], s['hiz'],
                        s['batarya'], s['voltaj'], s['akim'])
    tgt = s['targeting']

    def hc(txt, renk):
        return {'t': txt, 'c': renk}

    return {
        'bagli': bagli,
        'estop': s['e_stop'],
        'mod': hc(mod, MOD_RENK.get(mod, TEXT)),
        'fsm': hc(fsm, fsm_renk),
        'roll': hc(f'{r:+.1f}°', RED if abs(r) > 15 else AMBER if abs(r) > 8 else TEXT),
        'pitch': hc(f'{p:+.1f}°', RED if abs(p) > 15 else AMBER if abs(p) > 8 else TEXT),
        'hiz': hc(f'{h:.2f} m/s', RED if abs(h) > 3.0 else TEXT),
        'wp': hc(str(s['wp']), TEXT),
        'bat': hc(f'{b:.0f}%' if b >= 0 else '—',
                  GREEN if b > 50 else AMBER if b > 20 else RED if b >= 0 else DIM),
        'voltaj': hc(f'{v:.1f} V' if v > 0 else '—',
                     GREEN if v > 29.6 else AMBER if v > 28.0 else RED if v > 0 else DIM),
        'akim': hc(f'{a:.1f} A' if b >= 0 else '—',
                   RED if a > 30 else AMBER if a > 20 else (TEXT if b >= 0 else DIM)),
        'target': hc(tgt, TARGETING_R.get(tgt, TEXT)),
        'lazer': hc('Ateş !' if s['shoot'] else 'Beklemede', RED if s['shoot'] else DIM),
    }


# ── Görüntü kodlama ───────────────────────────────────────────────────────────
def kucult_kodla(bgr):
    h, w = bgr.shape[:2]
    if w > MAKS_GENIS:
        s = MAKS_GENIS / w
        bgr = cv2.resize(bgr, (MAKS_GENIS, max(1, int(h * s))))
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_KALITE])
    return buf.tobytes() if ok else None


def panel_kare(ad):
    """Panel adına göre gösterilecek ham BGR kareyi seçer.

    Bayat kare döndürülmez: kaynak sustuğunda panel donmuş görüntü yerine
    "sinyal yok" göstersin.
    """
    if ad == 'nisan':
        dbg, tdbg = ortak.kare_oku('nisan_dbg')
        if dbg is not None and (time.time() - tdbg) < NISAN_DEBUG_TAZE_S:
            return dbg
        ham, tham = ortak.kare_oku('nisan_ham')
        return ham if (ham is not None and time.time() - tham < KARE_TAZE_S) else None
    kare, t = ortak.kare_oku(ad)
    esik = KARE_TAZE_OZEL.get(ad, KARE_TAZE_S)
    return kare if (kare is not None and time.time() - t < esik) else None


# ── HTML sayfası ──────────────────────────────────────────────────────────────
SAYFA = """<!doctype html><html lang=tr><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>LYDİA İKA — GCS</title><style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:%(BG)s;color:%(TEXT)s;font:13px 'Segoe UI',system-ui,sans-serif}
#bar{height:34px;background:%(PANEL)s;border:1px solid %(LINE)s;border-radius:6px;
 margin:6px;display:flex;align-items:center;padding:0 12px;gap:10px}
#bar b{font-size:14px}#bar .s{color:%(DIM)s;font-size:12px}
#bar .sag{margin-left:auto;display:flex;gap:12px;align-items:center}
#bag{font-size:12px;color:%(DIM)s}#saat{font-size:13px;color:%(DIM)s;font-family:monospace}
#ana{display:flex;gap:6px;margin:0 6px 6px}
#sol{width:178px;display:flex;flex-direction:column;gap:6px}
/* minmax(0,1fr): hücre tabanı içerikten bağımsız olsun. Düz 1fr min-content'i
   taban alır, SLAM haritası büyüdükçe panel boyu da oynar. */
#orta{flex:1;display:grid;grid-template-columns:repeat(3,minmax(0,1fr));grid-template-rows:repeat(2,minmax(0,1fr));gap:6px;min-height:0}
.vid{min-width:0;min-height:0;overflow:hidden}
#sag{width:200px;display:flex;flex-direction:column;gap:6px}
.kart{background:%(CARD)s;border:1px solid %(LINE)s;border-radius:6px;padding:8px 10px}
.kart .b{font-size:11px;color:%(DIM)s;margin-bottom:3px}
.kart .v{font-size:15px;font-weight:bold;word-wrap:break-word}
.vid{background:#111;border:1px solid %(LINE)s;border-radius:6px;overflow:hidden;
 display:flex;flex-direction:column;min-height:0}
.vid .h{height:20px;background:%(LINE)s;display:flex;align-items:center;padding:0 8px;
 font-size:11px;color:%(DIM)s}
.vid .g{flex:1;min-height:0;min-width:0;position:relative;overflow:hidden}
/* Mutlak konum: kare boyutu değiştiğinde (SLAM haritası büyürken olduğu gibi)
   hücreyi itip paneli büyütmez. */
.vid img{position:absolute;top:0;left:0;width:100%%;height:100%%;object-fit:contain}
/* SLAM haritası OccupancyGrid çözünürlüğünde gelir (~230px) — panele
   büyütülürken hücre sınırları keskin kalsın, bulanıklaşmasın. */
#cam_harita{image-rendering:pixelated}
.vid .yok{position:absolute;top:50%%;left:50%%;transform:translate(-50%%,-50%%);color:%(DIM)s;font-size:12px}
button{border:none;border-radius:4px;cursor:pointer;font-family:inherit}
#estop{background:%(RED)s;color:#fff;font-weight:bold;font-size:14px;padding:8px;width:100%%}
#estop:hover{background:#d32f2f}
#kaldir{background:%(LINE)s;color:%(TEXT)s;font-size:11px;padding:4px;width:100%%;margin-top:4px}
#kaldir:hover{background:#383838}
.srow{display:flex;justify-content:space-between;align-items:center;
 padding:6px 10px;border-bottom:1px solid %(LINE)s}
.srow:last-child{border-bottom:none}
.srow .k{color:%(DIM)s;font-size:12px}.srow .d{font-weight:bold;font-family:monospace}
#surekart .v{font-family:monospace;font-size:20px}
.sbtn{background:%(LINE)s;color:%(TEXT)s;font-size:11px;padding:4px 8px;margin-top:4px}
</style></head><body>
<div id=bar><b>LYDİA İKA</b><span class=s>|</span>
 <span class=s>MAGNESIA · TEKNOFEST 2026</span>
 <div class=sag><span id=bag>bağlantı yok</span><span id=saat>--:--:--</span></div></div>
<div id=ana>
 <div id=sol>
  <div class=kart><div class=b>ACİL DURDURMA</div>
   <div class=v id=estopv>—</div>
   <button id=estop onclick=estopBas()>ACİL DURDUR</button>
   <button id=kaldir onclick=estopKaldir()>E-STOP Kaldır (GCS)</button></div>
  <div class=kart><div class=b>Mod</div><div class=v id=modv>—</div></div>
  <div class=kart><div class=b>Parkur Aşaması</div><div class=v id=fsmv>—</div></div>
  <div class=kart id=surekart><div class=b>Süre</div><div class=v id=surev>00:00.0</div>
   <button class=sbtn onclick=sureTog()>Başlat/Durdur</button>
   <button class=sbtn onclick=sureSif()>Sıfırla</button></div>
 </div>
 <div id=orta>
  <div class=vid><div class=h>Ön Kamera</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_on></div></div>
  <div class=vid><div class=h>YOLO Debug</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_yolo></div></div>
  <div class=vid><div class=h>Nişan Kamera</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_nisan></div></div>
  <div class=vid><div class=h>SLAM Haritası</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_harita></div></div>
  <div class=vid><div class=h>Derinlik Kamerası</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_derinlik></div></div>
  <div class=vid><div class=h>LiDAR (anlık)</div><div class=g>
   <span class=yok>sinyal yok</span><img id=cam_lidar></div></div>
 </div>
 <div id=sag>
  <div class=kart style=padding:0>
   <div class=srow><span class=k>Batarya</span><span class=d id=s_bat>—</span></div>
   <div class=srow><span class=k>Voltaj</span><span class=d id=s_voltaj>—</span></div>
   <div class=srow><span class=k>Akım</span><span class=d id=s_akim>—</span></div>
   <div class=srow><span class=k>Roll</span><span class=d id=s_roll>—</span></div>
   <div class=srow><span class=k>Pitch</span><span class=d id=s_pitch>—</span></div>
   <div class=srow><span class=k>Hız</span><span class=d id=s_hiz>—</span></div>
   <div class=srow><span class=k>Waypoint</span><span class=d id=s_wp>—</span></div>
   <div class=srow><span class=k>Nişan</span><span class=d id=s_target>—</span></div>
   <div class=srow><span class=k>Lazer</span><span class=d id=s_lazer>—</span></div>
  </div>
 </div>
</div>
<script>
// Kamera panelleri — her kare ayrı istek. MJPEG bağlantıyı süresiz açık tutar,
// 6 panel + SSE tarayıcının 6 eşzamanlı bağlantı sınırını doldurur.
for(const [id,ad] of [['cam_on','on'],['cam_yolo','yolo'],
                      ['cam_nisan','nisan'],['cam_harita','harita'],
                      ['cam_derinlik','derinlik'],['cam_lidar','lidar']]){
  const im=document.getElementById(id), yok=im.previousElementSibling;
  let bekliyor=false;
  const cek=()=>{
    if(bekliyor) return;              // önceki kare inmeden yenisini isteme
    bekliyor=true;
    const y=new Image();
    y.onload=()=>{im.src=y.src; yok.style.display='none'; bekliyor=false};
    y.onerror=()=>{yok.style.display='block'; bekliyor=false};
    y.src='/kare/'+ad+'?t='+Date.now();
  };
  cek();
  setInterval(cek, 100);              // ~10 fps tavan
}
// Sensör akışı — SSE
const es=new EventSource('/veri');
es.onmessage=e=>{
  const d=JSON.parse(e.data);
  const bag=document.getElementById('bag');
  bag.textContent=d.bagli?'bağlı':'bağlantı kesildi';
  bag.style.color=d.bagli?'%(GREEN)s':'%(RED)s';
  document.getElementById('estopv').textContent=d.estop?'AKTİF':'Normal';
  document.getElementById('estopv').style.color=d.estop?'%(RED)s':'%(GREEN)s';
  set('modv',d.mod);set('fsmv',d.fsm);
  set('s_bat',d.bat);set('s_voltaj',d.voltaj);set('s_akim',d.akim);
  set('s_roll',d.roll);set('s_pitch',d.pitch);set('s_hiz',d.hiz);
  set('s_wp',d.wp);set('s_target',d.target);set('s_lazer',d.lazer);
};
function set(id,o){const el=document.getElementById(id);if(!el||!o)return;
  el.textContent=o.t;el.style.color=o.c;}
// E-STOP
function estopBas(){fetch('/estop',{method:'POST',body:JSON.stringify({aktif:true})});}
function estopKaldir(){if(confirm('E-STOP GCS kaynağı kaldırılsın mı?\\n(Diğer kaynaklar hâlâ aktifse durdurma sürer.)'))
  fetch('/estop',{method:'POST',body:JSON.stringify({aktif:false})});}
// Saat
setInterval(()=>{document.getElementById('saat').textContent=
  new Date().toLocaleTimeString('tr-TR');},1000);
// Süre kronometresi (tarayıcı tarafı)
let sT=0,sBas=0,sInt=null;
function sureTog(){if(sInt){clearInterval(sInt);sInt=null;sT+=Date.now()-sBas;}
  else{sBas=Date.now();sInt=setInterval(sureCiz,100);}}
function sureSif(){clearInterval(sInt);sInt=null;sT=0;sureCiz();}
function sureCiz(){let ms=sT+(sInt?Date.now()-sBas:0);
  let s=Math.floor(ms/1000),m=Math.floor(s/60);
  document.getElementById('surev').textContent=
    String(m).padStart(2,'0')+':'+String(s%%60).padStart(2,'0')+'.'+Math.floor(ms%%1000/100);}
</script></body></html>""" % dict(
    BG=BG, PANEL=PANEL, CARD=CARD, LINE=LINE, TEXT=TEXT, DIM=DIM,
    GREEN=GREEN, RED=RED)


# ── HTTP işleyici ─────────────────────────────────────────────────────────────
class Isleyici(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            govde = SAYFA.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(govde)))
            self.end_headers()
            self.wfile.write(govde)
        elif self.path.startswith('/kare/'):
            self._tek_kare(self.path[len('/kare/'):].split('?')[0])
        elif self.path.startswith('/stream/'):
            self._mjpeg(self.path[len('/stream/'):])
        elif self.path == '/veri':
            self._sse()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/estop':
            try:
                n = int(self.headers.get('Content-Length', 0))
                d = json.loads(self.rfile.read(n) or b'{}')
                DUGUM.gcs_estop(bool(d.get('aktif', False)))
                self.send_response(200)
                self.end_headers()
            except Exception:
                self.send_error(400)
        else:
            self.send_error(404)

    def _tek_kare(self, ad):
        """Tek JPEG döner ve bağlantıyı kapatır.

        MJPEG akışı bağlantıyı süresiz açık tutar; tarayıcılar aynı sunucuya
        en fazla 6 eşzamanlı bağlantı açtığı için 6 panel + SSE havuzu
        doldurur ve sayfa kilitlenir. Kare bazlı çekimde her istek
        milisaniyeler içinde kapanır, havuz serbest kalır.
        """
        if ad not in PANELLER:
            self.send_error(404)
            return
        bgr = panel_kare(ad)
        jpg = kucult_kodla(bgr) if bgr is not None else None
        if jpg is None:
            self.send_error(503)
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpg)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            self.wfile.write(jpg)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _mjpeg(self, ad):
        gecerli = PANELLER
        if ad not in gecerli:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=kare')
        self.end_headers()
        try:
            self.wfile.write(b'\r\n')
            son = 0.0
            while True:
                bgr = panel_kare(ad)
                if bgr is None:
                    time.sleep(0.2)
                    continue
                # fps sınırı
                dt = time.time() - son
                if dt < 1.0 / AKIS_FPS:
                    time.sleep(1.0 / AKIS_FPS - dt)
                son = time.time()
                jpg = kucult_kodla(bgr)
                if jpg is None:
                    continue
                self.wfile.write(b'--kare\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(('Content-Length: %d\r\n\r\n' % len(jpg)).encode())
                self.wfile.write(jpg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _sse(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            while True:
                veri = json.dumps(sensor_json())
                self.wfile.write(('data: %s\n\n' % veri).encode('utf-8'))
                self.wfile.flush()
                time.sleep(0.2)   # 5 Hz
        except (BrokenPipeError, ConnectionResetError):
            pass


DUGUM = None


def main():
    global DUGUM
    rclpy.init()
    DUGUM = WebDashboardNode()
    threading.Thread(target=rclpy.spin, args=(DUGUM,), daemon=True).start()
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Isleyici)
    DUGUM.get_logger().info(f'Web dashboard hazır: http://<jetson-ip>:{PORT}')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        DUGUM.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
