#!/usr/bin/env python3
"""
LYDİA İKA — GCS Dashboard
MAGNESIA · TEKNOFEST 2026
"""

import sys, threading, math, time
import numpy as np
import cv2
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from std_msgs.msg import Bool, UInt8, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, Image

from teknofest_ika.otonomi.topics import (
    E_STOP_TOPIC, E_STOP_FORCE_GCS_TOPIC, MOD_AKTIF_TOPIC, FSM_STATE_TOPIC,
    BATTERY_TOPIC, IMU_TOPIC, EKF_ODOM_TOPIC,
    MISYON_WP_INDEX_TOPIC,
    TARGETING_STATUS_TOPIC, SHOOT_RESULT_TOPIC,
    CAMERA_IMAGE_TOPIC, CAMERA_REAR_TOPIC, CAMERA_TARET_TOPIC,
    YOLO_RAW_DEBUG_TOPIC, MAP_IMAGE_TOPIC,
)

from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QGridLayout, QPushButton, QSizePolicy, QMessageBox,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QPixmap, QImage

# ── Renkler ───────────────────────────────────────────────────────────────────
BG    = '#181818'
PANEL = '#202020'
CARD  = '#242424'
LINE  = '#2e2e2e'
TEXT  = '#d4d4d4'
DIM   = '#5a5a5a'
GREEN = '#4caf50'
RED   = '#ef5350'
AMBER = '#ffa726'
CYAN  = '#26c6da'

MOD_ISIMLER = {0: 'MANUAL', 1: 'SEMI AUTO', 2: 'FULL AUTO'}
TARGETING_R = {
    'SEARCHING': AMBER, 'LOCKED': GREEN, 'ALIGNED': CYAN,
    'STANDBY': DIM,     'NO_IMAGE': RED, 'STALE_IMAGE': RED,
}

# ── Sinyal köprüsü ────────────────────────────────────────────────────────────
class Sinyaller(QObject):
    guncelle = pyqtSignal()
    kamera   = pyqtSignal()
    harita   = pyqtSignal()

_sig = Sinyaller()

durum = {
    'e_stop': False, 'mod': 0,       'fsm': '—',
    'batarya': -1.0, 'voltaj': 0.0,  'akim': 0.0,
    'roll': 0.0,     'pitch': 0.0,   'hiz': 0.0,
    'wp_index': 0,   'bag_ok': False,
    'targeting': '—','shoot': False,
    'cam_on': None, 'cam_arka': None, 'cam_taret': None,
    'cam_yolo': None, 'cam_harita': None,
}

# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class DashboardNode(Node):
    def __init__(self):
        super().__init__('ika_dashboard')
        self._bridge = CvBridge()
        be  = qos_profile_sensor_data
        rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool,          E_STOP_TOPIC,           self._estop,     10)
        self.create_subscription(UInt8,         MOD_AKTIF_TOPIC,        self._mod,       10)
        self.create_subscription(String,        FSM_STATE_TOPIC,        self._fsm,       10)
        self.create_subscription(BatteryState,  BATTERY_TOPIC,          self._bat,       10)
        self.create_subscription(Imu,           IMU_TOPIC,              self._imu,       be)
        self.create_subscription(Odometry,      EKF_ODOM_TOPIC,         self._odom,      be)
        self.create_subscription(UInt8,         MISYON_WP_INDEX_TOPIC,  self._wp,        10)
        self.create_subscription(String,        TARGETING_STATUS_TOPIC, self._targeting, 10)
        self.create_subscription(Bool,          SHOOT_RESULT_TOPIC,     self._shoot,     10)
        self.create_subscription(Image,         CAMERA_IMAGE_TOPIC,     self._c0,        be)
        self.create_subscription(Image,         CAMERA_REAR_TOPIC,      self._c1,        be)
        self.create_subscription(Image,         CAMERA_TARET_TOPIC,     self._c2,        be)
        self.create_subscription(Image,         YOLO_RAW_DEBUG_TOPIC,   self._c3,        be)
        self.create_subscription(Image,         MAP_IMAGE_TOPIC,        self._c4,        be)

        # GCS'den (bu dashboard) uzaktan E-STOP — WiFi/ROS2 üzerinden, LoRa'nın
        # yerini alıyor (donanımda hiç LoRa modülü yok). Diğer 3 kaynaktan
        # (imu/serial/rc) bağımsız, e_stop_node'da OR mantığıyla birleşiyor.
        self._estop_gcs_pub = self.create_publisher(Bool, E_STOP_FORCE_GCS_TOPIC, 10)

    def gcs_estop(self, aktif: bool):
        self._estop_gcs_pub.publish(Bool(data=aktif))

    def _estop(self, m):     durum['e_stop']=m.data; durum['bag_ok']=True; _sig.guncelle.emit()
    def _mod(self, m):       durum['mod']=m.data; _sig.guncelle.emit()
    def _fsm(self, m):       durum['fsm']=m.data; _sig.guncelle.emit()
    def _wp(self, m):        durum['wp_index']=m.data; _sig.guncelle.emit()
    def _targeting(self, m): durum['targeting']=m.data; _sig.guncelle.emit()
    def _shoot(self, m):     durum['shoot']=m.data; _sig.guncelle.emit()

    def _bat(self, m):
        durum['batarya'] = m.percentage*100 if m.percentage>=0 else -1.0
        durum['voltaj']  = m.voltage; durum['akim'] = m.current
        _sig.guncelle.emit()

    def _imu(self, m):
        q = m.orientation
        durum['roll']  = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x**2+q.y**2)))
        sinp = 2*(q.w*q.y-q.z*q.x)
        durum['pitch'] = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        _sig.guncelle.emit()

    def _odom(self, m):
        durum['hiz'] = m.twist.twist.linear.x
        _sig.guncelle.emit()

    def _img(self, msg):
        try:    return self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except: return None

    def _c0(self, m): durum['cam_on']     = self._img(m); _sig.kamera.emit()
    def _c1(self, m): durum['cam_arka']   = self._img(m); _sig.kamera.emit()
    def _c2(self, m): durum['cam_taret']  = self._img(m); _sig.kamera.emit()
    def _c3(self, m): durum['cam_yolo']   = self._img(m); _sig.kamera.emit()
    def _c4(self, m): durum['cam_harita'] = self._img(m); _sig.kamera.emit()


# ── Yardımcılar ───────────────────────────────────────────────────────────────
def _l(text='', pt=8, bold=False, mono=False, color=None):
    w = QLabel(text)
    f = QFont('Consolas' if mono else 'Segoe UI', pt); f.setBold(bold)
    w.setFont(f)
    w.setStyleSheet(f'color:{color or TEXT};background:transparent;')
    return w

def _hline():
    f = QFrame(); f.setFrameShape(QFrame.HLine)
    f.setFixedHeight(1); f.setStyleSheet(f'background:{LINE};border:none;')
    return f

SS = f'background:{CARD};border:1px solid {LINE};border-radius:6px;'


# ── Kart ─────────────────────────────────────────────────────────────────────
class Kart(QFrame):
    def __init__(self, baslik, pt=13, parent=None):
        super().__init__(parent)
        self.setStyleSheet(SS)
        lay = QVBoxLayout(self); lay.setContentsMargins(10, 8, 10, 8); lay.setSpacing(3)
        lay.addWidget(_l(baslik, 8, color=DIM))
        self._v = _l('—', pt, bold=True); self._v.setWordWrap(True)
        lay.addWidget(self._v)

    def set(self, txt, renk=None):
        self._v.setText(txt)
        self._v.setStyleSheet(f'color:{renk or TEXT};background:transparent;')


# ── E-STOP ────────────────────────────────────────────────────────────────────
class EStopKart(QFrame):
    def __init__(self, node: 'DashboardNode', parent=None):
        super().__init__(parent)
        self._node = node
        lay = QVBoxLayout(self); lay.setContentsMargins(10, 8, 10, 8); lay.setSpacing(3)
        lay.addWidget(_l('ACİL DURDURMA', 8, color=DIM))
        self._v = _l('—', 15, bold=True); self._v.setWordWrap(True)
        lay.addWidget(self._v)

        self._btn_dur = QPushButton('ACİL DURDUR')
        self._btn_dur.setStyleSheet(
            f'QPushButton{{background:{RED};color:white;border:none;'
            f'border-radius:4px;padding:6px;font-weight:bold;font-size:10pt;}}'
            f'QPushButton:hover{{background:#d32f2f;}}')
        self._btn_dur.clicked.connect(self._durdur)
        lay.addWidget(self._btn_dur)

        self._btn_kaldir = QPushButton('E-STOP Kaldır (GCS)')
        self._btn_kaldir.setStyleSheet(
            f'QPushButton{{background:{LINE};color:{TEXT};border:none;'
            f'border-radius:3px;padding:3px;font-size:8pt;}}'
            f'QPushButton:hover{{background:#383838;}}')
        self._btn_kaldir.clicked.connect(self._kaldir)
        lay.addWidget(self._btn_kaldir)

        self.set(False)

    def _durdur(self):
        # Tetikleme onay istemez — acil durumda hız önemli.
        self._node.gcs_estop(True)

    def _kaldir(self):
        # Kaldırma yanlışlıkla basmaya karşı onay ister; sadece GCS
        # kaynağını temizler, diğer kaynaklardan (imu/serial/rc) biri hâlâ
        # aktifse /e_stop True kalmaya devam eder (OR mantığı, e_stop_node).
        cevap = QMessageBox.question(
            self, 'E-STOP Kaldır',
            'GCS kaynaklı E-STOP kaldırılsın mı?\n'
            '(Fiziksel buton/IMU/RC aktifse araç yine duracaktır.)',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if cevap == QMessageBox.Yes:
            self._node.gcs_estop(False)

    def set(self, aktif):
        c = RED if aktif else GREEN
        self.setStyleSheet(
            f'background:{CARD};border-left:3px solid {c};'
            f'border-top:1px solid {LINE};border-right:1px solid {LINE};'
            f'border-bottom:1px solid {LINE};border-radius:6px;')
        self._v.setText('DURDURULDU' if aktif else 'SİSTEM HAZIR')
        self._v.setStyleSheet(f'color:{c};background:transparent;')


# ── Sayaç ─────────────────────────────────────────────────────────────────────
class SureKarti(QFrame):
    SURE = 15 * 60
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(SS)
        self._kalan = self.SURE; self._calisiyor = False

        lay = QVBoxLayout(self); lay.setContentsMargins(10, 8, 10, 8); lay.setSpacing(4)
        lay.addWidget(_l('KOŞu SÜRESİ', 8, color=DIM))
        self._s = _l('15:00', 22, bold=True, mono=True)
        lay.addWidget(self._s)

        BTN = (f'QPushButton{{background:{LINE};color:{TEXT};border:none;'
               f'border-radius:3px;padding:2px 8px;font-size:8pt;}}'
               f'QPushButton:hover{{background:#383838;}}')
        row = QHBoxLayout(); row.setSpacing(4); row.setContentsMargins(0,0,0,0)
        self._btn = QPushButton('Başlat'); self._btn.setStyleSheet(BTN)
        self._btn.clicked.connect(self._toggle)
        btn_r = QPushButton('Sıfırla'); btn_r.setStyleSheet(BTN)
        btn_r.clicked.connect(self._sifirla)
        row.addWidget(self._btn); row.addWidget(btn_r)
        lay.addLayout(row)

        self._t = QTimer(); self._t.timeout.connect(self._tik)

    def _toggle(self):
        if self._calisiyor:
            self._calisiyor=False; self._t.stop(); self._btn.setText('Devam')
        else:
            if self._kalan<=0: self._kalan=self.SURE
            self._calisiyor=True; self._t.start(1000); self._btn.setText('Durdur')

    def _sifirla(self):
        self._calisiyor=False; self._t.stop(); self._kalan=self.SURE
        self._btn.setText('Başlat'); self._s.setText('15:00')
        self._s.setStyleSheet(f'color:{TEXT};background:transparent;')

    def _tik(self):
        self._kalan = max(0, self._kalan-1)
        dk, sn = self._kalan//60, self._kalan%60
        c = RED if self._kalan<=60 else AMBER if self._kalan<=180 else TEXT
        self._s.setText(f'{dk:02d}:{sn:02d}')
        self._s.setStyleSheet(f'color:{c};background:transparent;')
        if self._kalan==0:
            self._calisiyor=False; self._t.stop(); self._btn.setText('Başlat')


# ── Video panel ───────────────────────────────────────────────────────────────
class VideoPanel(QFrame):
    def __init__(self, baslik, parent=None):
        super().__init__(parent)
        self.setStyleSheet(SS)
        lay = QVBoxLayout(self); lay.setContentsMargins(0,0,0,0); lay.setSpacing(0)

        hdr = QWidget(); hdr.setFixedHeight(20)
        hdr.setStyleSheet(f'background:{LINE};border-radius:6px 6px 0 0;')
        h = QHBoxLayout(hdr); h.setContentsMargins(8,0,8,0)
        h.addWidget(_l(baslik, 7, color=DIM)); h.addStretch()
        lay.addWidget(hdr)

        self._img = QLabel('sinyal yok')
        self._img.setAlignment(Qt.AlignCenter)
        self._img.setFont(QFont('Segoe UI', 8))
        self._img.setStyleSheet(f'color:{DIM};background:#111;border-radius:0 0 6px 6px;')
        self._img.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self._img.setMinimumSize(1, 1)
        lay.addWidget(self._img, stretch=1)

    def goster(self, frame):
        if frame is None:
            self._img.setPixmap(QPixmap()); self._img.setText('sinyal yok'); return
        pw, ph = max(self._img.width(),1), max(self._img.height(),1)
        h, w = frame.shape[:2]
        s = min(pw/w, ph/h)
        nw, nh = max(1,int(w*s)), max(1,int(h*s))
        rgb = np.ascontiguousarray(cv2.cvtColor(cv2.resize(frame,(nw,nh)), cv2.COLOR_BGR2RGB))
        self._img.setPixmap(QPixmap.fromImage(QImage(rgb.data,nw,nh,3*nw,QImage.Format_RGB888)))
        self._img.setText('')


HaritaPanel = lambda: VideoPanel('SLAM Haritası')


# ── Sağ sensör paneli ─────────────────────────────────────────────────────────
class Satir(QWidget):
    def __init__(self, lbl, parent=None):
        super().__init__(parent)
        self.setFixedHeight(38)
        self.setStyleSheet('background:transparent;')
        col = QVBoxLayout(self); col.setContentsMargins(10,4,10,4); col.setSpacing(1)
        self._lbl = _l(lbl, 7, color='#888888')
        self._v   = _l('—', 12, bold=True, mono=True)
        col.addWidget(self._lbl)
        col.addWidget(self._v)

    def set(self, txt, renk=None):
        self._v.setText(txt)
        self._v.setStyleSheet(f'color:{renk or TEXT};background:transparent;')


def _grup_hdr(txt):
    w = QWidget(); w.setFixedHeight(22)
    w.setStyleSheet(f'background:{LINE};')
    row = QHBoxLayout(w); row.setContentsMargins(10,0,10,0)
    row.addWidget(_l(txt, 8, bold=True, color='#888888')); row.addStretch()
    return w


class SensorPanel(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(SS)
        self.setFixedWidth(178)

        lay = QVBoxLayout(self); lay.setContentsMargins(0,0,0,0); lay.setSpacing(0)

        lay.addWidget(_grup_hdr('IMU'))
        self.roll  = Satir('Roll');  lay.addWidget(self.roll)
        self.pitch = Satir('Pitch'); lay.addWidget(self.pitch)

        lay.addWidget(_hline())
        lay.addWidget(_grup_hdr('HAREKET'))
        self.hiz = Satir('Hız');      lay.addWidget(self.hiz)
        self.wp  = Satir('Waypoint'); lay.addWidget(self.wp)

        lay.addWidget(_hline())
        lay.addWidget(_grup_hdr('GÜÇ'))
        self.bat    = Satir('Batarya');     lay.addWidget(self.bat)
        self.voltaj = Satir('Voltaj');      lay.addWidget(self.voltaj)
        self.akim   = Satir('Motor Akımı'); lay.addWidget(self.akim)

        lay.addWidget(_hline())
        lay.addWidget(_grup_hdr('ATIŞ'))
        self.target = Satir('Hedefleme'); lay.addWidget(self.target)
        self.lazer  = Satir('Lazer');     lay.addWidget(self.lazer)

        lay.addStretch()


# ── Ana pencere ───────────────────────────────────────────────────────────────
class Dashboard(QWidget):
    def __init__(self, node: 'DashboardNode'):
        super().__init__()
        self._node = node
        self.setWindowTitle('LYDİA İKA — GCS')
        self.resize(1500, 880)
        self.setStyleSheet(f'QWidget{{background:{BG};}}')
        self._son = 0.0

        root = QVBoxLayout(self); root.setContentsMargins(8,8,8,8); root.setSpacing(6)
        root.addWidget(self._hdr_bar())

        body = QHBoxLayout(); body.setSpacing(6)
        body.addWidget(self._sol())
        body.addLayout(self._grid(), stretch=1)
        self._sp = SensorPanel()
        body.addWidget(self._sp)
        root.addLayout(body, stretch=1)

        t1 = QTimer(); t1.timeout.connect(self._chk_bag); t1.start(500)
        t2 = QTimer(); t2.timeout.connect(self._tick_saat); t2.start(1000)
        self._t1, self._t2 = t1, t2

        _sig.guncelle.connect(self._up)
        _sig.kamera.connect(self._up_cam)

    def _hdr_bar(self):
        hdr = QFrame(); hdr.setFixedHeight(34)
        hdr.setStyleSheet(f'background:{PANEL};border:1px solid {LINE};border-radius:6px;')
        row = QHBoxLayout(hdr); row.setContentsMargins(12,0,12,0); row.setSpacing(10)
        row.addWidget(_l('LYDİA İKA', 10, bold=True))
        sep = QFrame(); sep.setFrameShape(QFrame.VLine)
        sep.setFixedHeight(14); sep.setStyleSheet(f'color:{LINE};')
        row.addWidget(sep)
        row.addWidget(_l('MAGNESIA · TEKNOFEST 2026', 8, color=DIM))
        row.addStretch()
        self._bag = _l('bağlantı yok', 7, color=DIM)
        self._saat = _l('--:--:--', 8, mono=True, color=DIM)
        row.addWidget(self._bag); row.addSpacing(12); row.addWidget(self._saat)
        return hdr

    def _sol(self):
        f = QFrame(); f.setFixedWidth(178)
        f.setStyleSheet('QFrame{background:transparent;border:none;}')
        lay = QVBoxLayout(f); lay.setContentsMargins(0,0,0,0); lay.setSpacing(6)
        self.estop = EStopKart(self._node)
        self.kmod  = Kart('Mod', 14)
        self.kfsm  = Kart('Parkur Aşaması', 13)
        self.sure  = SureKarti()
        lay.addWidget(self.estop)
        lay.addWidget(self.kmod)
        lay.addWidget(self.kfsm)
        lay.addWidget(self.sure)
        lay.addStretch()
        return f

    def _grid(self):
        g = QGridLayout(); g.setSpacing(6)
        self.p0 = VideoPanel('Ön Kamera')
        self.p1 = VideoPanel('YOLO Debug')
        self.p2 = VideoPanel('Nişan Kamera')
        self.p3 = HaritaPanel()
        g.addWidget(self.p0, 0, 0); g.addWidget(self.p1, 0, 1)
        g.addWidget(self.p2, 1, 0); g.addWidget(self.p3, 1, 1)
        return g

    def _up(self):
        self._son = time.time()
        self.estop.set(durum['e_stop'])

        mod = MOD_ISIMLER.get(durum['mod'], '?')
        self.kmod.set(mod, {'MANUAL': AMBER, 'SEMI AUTO': CYAN, 'FULL AUTO': GREEN}.get(mod, TEXT))

        fsm = durum['fsm'] or '—'
        self.kfsm.set(fsm,
            RED   if any(k in fsm for k in ('EMERGENCY','ERROR')) else
            AMBER if 'STOP' in fsm else
            DIM   if fsm in ('IDLE','—') else TEXT)

        sp = self._sp
        r, p = durum['roll'], durum['pitch']
        sp.roll.set(f'{r:+.1f}°',  RED if abs(r)>15 else AMBER if abs(r)>8  else TEXT)
        sp.pitch.set(f'{p:+.1f}°', RED if abs(p)>15 else AMBER if abs(p)>8  else TEXT)

        h = durum['hiz']
        sp.hiz.set(f'{h:.2f} m/s', RED if abs(h)>3.0 else TEXT)
        sp.wp.set(str(durum['wp_index']))

        b = durum['batarya']
        sp.bat.set(f'{b:.0f}%' if b>=0 else '—',
            GREEN if b>50 else AMBER if b>20 else RED if b>=0 else DIM)

        v = durum['voltaj']
        sp.voltaj.set(f'{v:.1f} V' if v>0 else '—',
            GREEN if v>29.6 else AMBER if v>28.0 else RED if v>0 else DIM)

        a = durum['akim']
        sp.akim.set(f'{a:.1f} A' if b>=0 else '—',
            RED if a>30 else AMBER if a>20 else (TEXT if b>=0 else DIM))

        tgt = durum['targeting']
        sp.target.set(tgt, TARGETING_R.get(tgt, TEXT))
        sp.lazer.set('Ateş !' if durum['shoot'] else 'Beklemede',
            RED if durum['shoot'] else DIM)

        self._bag.setText('bağlı')
        self._bag.setStyleSheet(f'color:{GREEN};background:transparent;')

    def _up_cam(self):
        self.p0.goster(durum['cam_on'])
        self.p1.goster(durum['cam_yolo'])
        self.p2.goster(durum['cam_taret'])
        self.p3.goster(durum['cam_harita'])

    def _tick_saat(self):
        self._saat.setText(datetime.now().strftime('%H:%M:%S'))

    def _chk_bag(self):
        if durum['bag_ok'] and time.time()-self._son > 3.0:
            self._bag.setText('bağlantı kesildi')
            self._bag.setStyleSheet(f'color:{RED};background:transparent;')



# ── Giriş ─────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = DashboardNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QApplication(sys.argv)
    win = Dashboard(node)
    win.show()
    try:    sys.exit(app.exec_())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
