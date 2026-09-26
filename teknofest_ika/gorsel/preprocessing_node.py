#!/usr/bin/env python3
"""
preprocessing_node.py — Sensor Pre-processing for MAGNESIA LYDIA
- Gaussian denoising
- Temporal Low-Pass (EMA) for vibration suppression
- Rain/Water drop inpainting
- LiDAR filtering (angle clipping + moving average + range filter)
- Depth PointCloud2 Radius Outlier Removal (ROR)
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from cv_bridge import CvBridge
import struct

from teknofest_ika.otonomi.pure_logic import tarama_kirpma_penceresi
from teknofest_ika.otonomi.pure_logic import yagmur_lekeleri
from teknofest_ika.otonomi.topics import (
    SCAN_LIDAR_TOPIC, LIDAR_MONTAJ_YAW_RAD,
    SCAN_FILTERED_TOPIC,
    CAMERA_PROCESSED_TOPIC,
    CAMERA_IMAGE_TOPIC,
    CAMERA_TARET_TOPIC,
    CAMERA_TARET_PROCESSED_TOPIC,
)


class PreprocessingNode(Node):
    def __init__(self):
        super().__init__("preprocessing_node")

        # Parameters
        # Kameralar araca ters monte edilebiliyor; görüntü kaynakta çevrilir ki
        # hem YOLO hem panel düz görsün (model düz görüntüyle eğitildi).
        self.declare_parameter("flip_ana", False)
        self.declare_parameter("flip_taret", False)
        self.declare_parameter("gaussian_kernel", 5)
        self.declare_parameter("temporal_alpha", 0.3)
        self.declare_parameter("enable_rain_inpaint", True)
        self.declare_parameter("rain_inpaint_radius", 3)
        self.declare_parameter("rain_blob_max_area_px", 120)
        # Bir karede kaç küçük parlak lekeye kadar "yağmur" sayılacağı.
        # Gerçek damla sayısı onlarla ölçülür; yüzlerce leke sahnenin
        # kendi dokusudur ve onarmaya çalışmak hem yanlış hem pahalıdır.
        self.declare_parameter("rain_max_blob_count", 80)
        # ⚠️ Bu pencere ARAÇ çerçevesinde tanımlıdır (0° = ileri), tarama
        # çerçevesinde değil. LiDAR gövdeye 93,3° dönük monte olduğu için ikisi
        # aynı şey değil: pencere tarama açılarına doğrudan uygulandığında
        # aracın SAĞ yanı (−131,7°…−41,7°) komple `inf` oluyordu ve bunu
        # /scan/filtered'ı okuyan herkes miras alıyordu — Nav2'nin iki
        # costmap'i, kayar engel, koni füzyonu, §6.9 koridor ölçümü. Hiçbiri
        # hata basmıyordu, yalnız sağdaki bariyer yok sayılıyordu.
        self.declare_parameter("lidar_angle_min_deg", -135.0)
        self.declare_parameter("lidar_angle_max_deg", 135.0)
        self.declare_parameter("lidar_montaj_yaw_rad", LIDAR_MONTAJ_YAW_RAD)
        self.declare_parameter("lidar_ma_window", 3)
        self.declare_parameter("depth_ror_nb_points", 6)
        self.declare_parameter("depth_ror_radius", 0.05)
        # Derinlik nokta bulutu filtresi. Çıktısı (/depth/points/filtered)
        # yalnız Nav2 costmap katmanına besleniyordu; Nav2 kapalı olduğu
        # sürece kare başına KD-tree kurup Python'da gezmek boşa CPU demek.
        # Nav2 geri açıldığında bu parametre true yapılır.
        self.declare_parameter("derinlik_isle", False)

        self._flip_ana = bool(self.get_parameter("flip_ana").value)
        self._flip_taret = bool(self.get_parameter("flip_taret").value)

        self.gaussian_kernel = self.get_parameter("gaussian_kernel").value
        self.temporal_alpha = self.get_parameter("temporal_alpha").value
        self.enable_rain = self.get_parameter("enable_rain_inpaint").value
        self.rain_radius = self.get_parameter("rain_inpaint_radius").value
        self.rain_blob_max_area = self.get_parameter("rain_blob_max_area_px").value
        self.rain_max_blob_count = self.get_parameter("rain_max_blob_count").value
        self.lidar_angle_min = math.radians(self.get_parameter("lidar_angle_min_deg").value)
        self.lidar_angle_max = math.radians(self.get_parameter("lidar_angle_max_deg").value)
        self.lidar_ma_window = self.get_parameter("lidar_ma_window").value
        self._lidar_yaw = float(self.get_parameter("lidar_montaj_yaw_rad").value)
        # Pencere bir kez tarama çerçevesine taşınıyor; her taramada 600+ açıyı
        # çevirmeye gerek yok. alt > ust çıkabilir — pencere ±180°'yi aşıyor
        # demektir, aşağıda VEYA ile uygulanır.
        self._kirp_alt, self._kirp_ust = tarama_kirpma_penceresi(
            self.lidar_angle_min, self.lidar_angle_max, self._lidar_yaw)
        self._kirp_sarmali = self._kirp_alt > self._kirp_ust
        self.ror_nb_points = self.get_parameter("depth_ror_nb_points").value
        self.ror_radius = self.get_parameter("depth_ror_radius").value
        self._derinlik_isle = bool(self.get_parameter("derinlik_isle").value)

        self.bridge = CvBridge()
        self.ema_main = None
        self.ema_taret = None
        self.lidar_buffer = []

        # Publishers
        self.pub_main = self.create_publisher(Image, CAMERA_PROCESSED_TOPIC, 10)
        # Nişan kamerası (taret üzeri) — şartname §6.10/§6.14: atış/veri paketi
        # için ayrı bir nişan kamerası gerekli; targeting_node bu çıkışı okur.
        self.pub_taret = self.create_publisher(Image, CAMERA_TARET_PROCESSED_TOPIC, 10)
        self.pub_scan = self.create_publisher(LaserScan, SCAN_FILTERED_TOPIC, 10)
        self.pub_depth = (self.create_publisher(
            PointCloud2, "/depth/points/filtered", 10)
            if self._derinlik_isle else None)

        # Subscribers — gerçek topic adları doğrudan topics.py'den alınır,
        # böylece launch dosyasında unutulabilecek bir remap'e bağımlı kalınmaz
        # (nişan kamerasının önceden hiç işlenmemesi tam olarak bu yüzden olmuştu).
        self.sub_main = self.create_subscription(
            Image, CAMERA_IMAGE_TOPIC,
            self.cb_main_camera, qos_profile_sensor_data)
        self.sub_taret = self.create_subscription(
            Image, CAMERA_TARET_TOPIC,
            self.cb_taret_camera, qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(
            LaserScan, SCAN_LIDAR_TOPIC,
            self.cb_scan, qos_profile_sensor_data)
        # Aboneliğin kendisi koşullu: yayıncı susmasa da işlenmemiş bulut
        # DDS'ten çekilmez, serileştirme ve KD-tree maliyeti hiç doğmaz.
        self.sub_depth = (self.create_subscription(
            PointCloud2, "/depth/points",
            self.cb_depth, qos_profile_sensor_data)
            if self._derinlik_isle else None)

        self.get_logger().info(
            "PreprocessingNode started (derinlik_isle={}).".format(
                self._derinlik_isle))

    # ------------------------------------------------------------------
    # Camera callbacks
    # ------------------------------------------------------------------
    def cb_main_camera(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self._flip_ana:
            cv_img = cv2.flip(cv_img, -1)      # 180°
        out = self._process_image(cv_img, self.ema_main)
        self.ema_main = out.astype(np.float32)
        self.pub_main.publish(self._msg_yap(out, msg))

    def cb_taret_camera(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self._flip_taret:
            cv_img = cv2.flip(cv_img, -1)
        out = self._process_image(cv_img, self.ema_taret)
        self.ema_taret = out.astype(np.float32)
        self.pub_taret.publish(self._msg_yap(out, msg))

    def _msg_yap(self, out: np.ndarray, kaynak: Image) -> Image:
        # Kaynak header'ı (stamp + frame_id) korunur — tüketiciler (targeting_node
        # kare tekrarı ayıklama, TF eşleme) stamp'in gerçek olmasına dayanır;
        # boş header her karenin stamp=0 görünüp elenmesine yol açıyordu.
        cikti = self.bridge.cv2_to_imgmsg(out, "bgr8")
        cikti.header = kaynak.header
        return cikti

    def _process_image(self, img: np.ndarray, prev_ema) -> np.ndarray:
        # 1) Gaussian denoising
        k = self.gaussian_kernel
        if k > 1:
            k = k if k % 2 == 1 else k + 1
            img = cv2.GaussianBlur(img, (k, k), 0)

        # 2) Temporal Low-Pass (EMA) — vibration suppression
        if prev_ema is not None:
            img = (self.temporal_alpha * img.astype(np.float32) +
                   (1.0 - self.temporal_alpha) * prev_ema)
            img = img.astype(np.uint8)

        # 3) Rain / water drop inpainting
        if self.enable_rain:
            img = self._remove_rain(img)
        return img

    def _remove_rain(self, img: np.ndarray) -> np.ndarray:
        """
        Lens üzerindeki yağmur damlalarını doldurur.

        🔴 BU YOL AÇIK ARAZİDE CPU'YU DOLDURABİLİR ve iki kapı onu sınırlıyor;
        ikisi de kaldırılırsa düğüm gerçek zamanın gerisine düşer, /scan
        filtresi ve nişan görüntüsü birlikte gecikir.
        """
        # Detect small bright spots (rain drops) using threshold + morphology
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, bright_mask = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, kernel)

        # Boyut filtresi: yağmur damlaları küçük/izole lekelerdir. Şerit
        # çizgisi, koni beyaz şeridi veya atış hedefinin beyaz halkaları
        # gibi GERÇEK nesneler de parlaklık eşiğini (240) geçebilir ama
        # bunlar büyük/bitişik alanlar oluşturur — sadece küçük bağlı
        # bileşenler (rain_blob_max_area_px altı) inpaint maskesine alınır,
        # böylece gerçek nesneler yanlışlıkla bozulmaz.
        #
        # 🔑 Maske TEK SEFERDE kuruluyor. Bileşen başına `labels == id`
        # yazmak her bileşen için TAM GÖRÜNTÜ taraması demek: 640×480'de
        # 376 bileşenli parlak bir sahnede kare başına ~16 ms, yani iki
        # kamerada 30 Hz'de tek başına bir çekirdek. np.isin aynı maskeyi
        # ~0,7 ms'de kuruyor (çıktı birebir aynı, ölçüldü).
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            bright_mask, connectivity=8)
        kucuk, sebep = yagmur_lekeleri(
            stats[:, cv2.CC_STAT_AREA].tolist(),
            self.rain_blob_max_area, self.rain_max_blob_count)
        if sebep != 'onar':
            if sebep == 'cok_leke':
                self.get_logger().debug(
                    'Yağmur onarımı atlandı: parlak leke sayısı sınırın '
                    f'({self.rain_max_blob_count}) üstünde — sahne dokusu, '
                    'yağmur değil.', throttle_duration_sec=10.0)
            return img

        filtered_mask = np.where(
            np.isin(labels, np.asarray(kucuk)), 255, 0).astype(np.uint8)
        return cv2.inpaint(img, filtered_mask, self.rain_radius, cv2.INPAINT_TELEA)

    # ------------------------------------------------------------------
    # LiDAR callback
    # ------------------------------------------------------------------
    def cb_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Açılar dizinin KENDİ uzunluğundan türetiliyor: angle_max'tan üretmek
        # yuvarlama yüzünden bir eleman eksik/fazla dizi verebiliyor ve maske
        # boyutu tutmayınca callback her taramada patlıyordu.
        tarama_acilari = (msg.angle_min
                          + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment)

        # 1) Angle clipping — pencere ARAÇ çerçevesinde tanımlı, açılışta
        #    tarama çerçevesine taşındı (bkz. __init__).
        if self._kirp_sarmali:
            mask = ((tarama_acilari >= self._kirp_alt)
                    | (tarama_acilari <= self._kirp_ust))
        else:
            mask = ((tarama_acilari >= self._kirp_alt)
                    & (tarama_acilari <= self._kirp_ust))
        # We keep full array but set out-of-range to inf so Nav2 ignores them
        filtered = ranges.copy()
        filtered[~mask] = float('inf')

        # 2) Range validity filter
        valid = np.isfinite(filtered) & (filtered > msg.range_min) & (filtered < msg.range_max)
        invalid = ~valid
        filtered[invalid] = float('inf')

        # 3) Komşu ışınlar arasında MEDYAN — tek ışınlık gürültüyü siler,
        #    kenarı korur. Ortalama burada yanlış araçtı: koni ve direk gibi
        #    ince engeller iki üç ışın kaplıyor ve arka planla ortalanınca
        #    kayboluyorlar; dahası yakın engelle uzak arka plan arasındaki
        #    sınırda gerçekte var olmayan ara mesafeler üretiliyordu.
        #    /scan/filtered'ı Nav2 costmap'i, cone_fusion ve kayar_engel
        #    birlikte okuyor, yani o uydurma mesafeler üçüne birden gidiyordu.
        #
        #    Pencere genişliği medyanın koruyabildiği en dar engeli belirler:
        #    3'lük pencere iki ışınlık bir engeli gerçek mesafesinde tutar,
        #    5'lik pencere onu arka plana gömer. Bedeli tek ışınlık engellerin
        #    de silinmesi — ama tek ışın zaten gürültüden ayırt edilemiyor.
        if self.lidar_ma_window > 1:
            temp = filtered.copy()
            temp[invalid] = np.nan
            w = self.lidar_ma_window
            padded = np.pad(temp, (w // 2, w // 2), mode='edge')
            # Pencereler kaydırılmış dilimlerden yığılıyor. sliding_window_view
            # daha derli toplu olurdu ama numpy 1.20 istiyor ve araçtaki sürüm
            # bilinmiyor; burada patlayan bir düğüm /scan/filtered'ı susturur,
            # yani Nav2 ve SLAM taramayı birden kaybeder.
            pencereler = np.stack([padded[i:i + len(temp)] for i in range(w)])
            with np.errstate(invalid='ignore'):
                # Tümü geçersiz olan pencerede nanmedian uyarı basar; sonucu
                # zaten nan ve aşağıda inf'e çevriliyor.
                smoothed = np.nanmedian(pencereler, axis=0)
            filtered = np.where(np.isfinite(smoothed), smoothed,
                                float('inf')).astype(np.float32)

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = filtered.tolist()
        out.intensities = msg.intensities
        self.pub_scan.publish(out)

    # ------------------------------------------------------------------
    # Depth PointCloud2 callback — simple ROR in Python
    # ------------------------------------------------------------------
    def cb_depth(self, msg: PointCloud2):
        points = self._read_points(msg)
        if len(points) == 0:
            self.pub_depth.publish(msg)
            return

        filtered = self._radius_outlier_removal(points,
                                                self.ror_nb_points,
                                                self.ror_radius)
        out_msg = self._create_cloud(msg.header, filtered)
        self.pub_depth.publish(out_msg)

    def _read_points(self, cloud: PointCloud2):
        """Read xyz from PointCloud2 using sensor_msgs_py (field-agnostic)."""
        try:
            from sensor_msgs_py.point_cloud2 import read_points
            pts = list(read_points(cloud, field_names=("x", "y", "z"), skip_nans=True))
            return np.array(pts, dtype=np.float32)
        except Exception as e:
            self.get_logger().warn(f"sensor_msgs_py read_points failed ({e}), falling back to manual parser")
            return self._read_points_manual(cloud)

    def _read_points_manual(self, cloud: PointCloud2):
        """Manual fallback parser that respects field offsets."""
        # Map PointField datatype to struct format char and size
        type_map = {
            PointField.INT8: ('b', 1),
            PointField.UINT8: ('B', 1),
            PointField.INT16: ('h', 2),
            PointField.UINT16: ('H', 2),
            PointField.INT32: ('i', 4),
            PointField.UINT32: ('I', 4),
            PointField.FLOAT32: ('f', 4),
            PointField.FLOAT64: ('d', 8),
        }

        # Find x, y, z offsets
        fields = {f.name: f for f in cloud.fields}
        if not all(k in fields for k in ('x', 'y', 'z')):
            self.get_logger().error("PointCloud2 missing x/y/z fields")
            return np.array([], dtype=np.float32)

        fmt_parts = ['<' if not cloud.is_bigendian else '>']
        offsets = []
        current_offset = 0
        for name in ('x', 'y', 'z'):
            f = fields[name]
            fmt_char, size = type_map.get(f.datatype, ('f', 4))
            # Add padding if needed
            if f.offset > current_offset:
                fmt_parts.append(f"{f.offset - current_offset}x")
            fmt_parts.append(fmt_char)
            offsets.append(f.offset)
            current_offset = f.offset + size

        fmt = ''.join(fmt_parts)
        width, height = cloud.width, cloud.height
        point_step = cloud.point_step
        data = np.frombuffer(cloud.data, dtype=np.uint8)
        points = []
        for v in range(height):
            for u in range(width):
                offset = (v * width + u) * point_step
                if offset + struct.calcsize(fmt) > len(data):
                    continue
                unpacked = struct.unpack_from(fmt, data, offset)
                x, y, z = unpacked[0], unpacked[1], unpacked[2]
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append([x, y, z])
        return np.array(points, dtype=np.float32)

    def _radius_outlier_removal(self, points: np.ndarray, nb_points: int, radius: float):
        if len(points) == 0:
            return points
        # Use scipy cKDTree if available (much faster than OpenCV flann on Jetson)
        try:
            from scipy.spatial import cKDTree
            tree = cKDTree(points)
            filtered = []
            for pt in points:
                dists, _ = tree.query(pt, k=nb_points, distance_upper_bound=radius)
                if isinstance(dists, np.ndarray):
                    if dists[-1] <= radius:
                        filtered.append(pt)
                else:
                    filtered.append(pt)
            return np.array(filtered, dtype=np.float32)
        except ImportError:
            pass

        # Fallback to OpenCV flann
        flann = cv2.flann_Index()
        params = dict(algorithm=1, trees=4)  # KDTree
        search_params = dict(checks=32)
        flann.build(points, params)

        filtered = []
        for pt in points:
            indices, dists = flann.knnSearch(pt.reshape(1, -1), nb_points, search_params)
            if dists[0][-1] <= radius * radius:
                filtered.append(pt)
        return np.array(filtered, dtype=np.float32)

    def _create_cloud(self, header, points: np.ndarray):
        try:
            from sensor_msgs_py.point_cloud2 import create_cloud
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            return create_cloud(header, fields, points.tolist())
        except Exception as e:
            self.get_logger().warn(f"sensor_msgs_py create_cloud failed ({e}), using manual")

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.data = points.tobytes() if len(points) > 0 else b''
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = PreprocessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
