import os
from glob import glob

from setuptools import find_packages, setup

PAKET = 'teknofest_ika'

# Kurulacak veri dosyaları. Kaynak ağacı ile kurulu ağacın AYRIŞMASI bu
# listeden doğuyor: burada olmayan bir yapılandırma dosyası araçta eski
# hâliyle kalır ve düğüm sessizce onu okur. Yeni bir config/ alt dizini
# eklendiğinde buraya da eklenmesi gerekir.
VERI = [
    ('share/ament_index/resource_index/packages', ['resource/' + PAKET]),
    ('share/' + PAKET, ['package.xml']),
    (os.path.join('share', PAKET, 'launch'), glob('launch/*.py')),
    (os.path.join('share', PAKET, 'config'), glob('config/*.yaml')),
    # Davranış ağaçları: bt_navigator bunları nav2_params.yaml'daki TAM
    # YOLDAN, kaynak ağacından okuyor. Buradaki kopya kurulum yolunu
    # kullanan çağrılar için duruyor.
    (os.path.join('share', PAKET, 'config', 'bt'), glob('config/bt/*.xml')),
    (os.path.join('share', PAKET, 'urdf'), glob('urdf/*.urdf')),
    # Model ağırlıkları depoda dağıtılmıyor (bkz. .gitignore) ama araçta
    # kaynak ağacında duruyorlar. yolo_detection_node model yolunu üç yerde
    # arıyor: çalışma dizini, PAKET share'i, kaynak ağacı — bu girdi
    # ortadakini besliyor. Dizin yoksa liste boş kalır, kurulum etkilenmez.
    (os.path.join('share', PAKET, 'models'),
     glob('models/**/*.pt', recursive=True)
     + glob('models/**/*.engine', recursive=True)),
]

setup(
    name=PAKET,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=VERI,
    install_requires=[
        'setuptools',
        'pyserial',
        'numpy',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Bugra Ozturk',
    maintainer_email='ozturkbugra684@gmail.com',
    description="LYDİA — TEKNOFEST İnsansız Kara Aracı otonom sürüş yazılımı",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── Gömülü ─────────────────────────────────────────────────
            'seri_kopru            = teknofest_ika.gomulu.seri_kopru:main',
            'bms_koprusu           = teknofest_ika.gomulu.bms_koprusu:main',
            'taret_rc_koprusu      = teknofest_ika.gomulu.taret_rc_koprusu:main',
            # ── Otonomi ────────────────────────────────────────────────
            'misyon_fsm            = teknofest_ika.otonomi.misyon_fsm:main',
            'terrain_adapter       = teknofest_ika.otonomi.terrain_adapter:main',
            'veri_paketi           = teknofest_ika.otonomi.veri_paketi:main',
            'ackermann_converter   = teknofest_ika.otonomi.ackermann_converter:main',
            'anti_rollback         = teknofest_ika.otonomi.anti_rollback:main',
            'imu_guvenlik          = teknofest_ika.otonomi.imu_guvenlik:main',
            'mod_yoneticisi        = teknofest_ika.otonomi.mod_yoneticisi:main',
            'e_stop_node           = teknofest_ika.otonomi.e_stop_node:main',
            'watchdog              = teknofest_ika.otonomi.watchdog:main',
            # ── Görsel ─────────────────────────────────────────────────
            'kayar_engel_kalman    = teknofest_ika.gorsel.kayar_engel_kalman:main',
            'kayar_engel_costmap   = teknofest_ika.gorsel.kayar_engel_costmap:main',
            'preprocessing_node    = teknofest_ika.gorsel.preprocessing_node:main',
            'yolo_detection_node   = teknofest_ika.gorsel.yolo_detection_node:main',
            'cone_fusion_node      = teknofest_ika.gorsel.cone_fusion_node:main',
            'lane_detection_node   = teknofest_ika.gorsel.lane_detection_node:main',
            'targeting_node        = teknofest_ika.gorsel.targeting_node:main',
            'servo_controller_node = teknofest_ika.gorsel.servo_controller_node:main',
            'yolo_adapter_node     = teknofest_ika.gorsel.yolo_adapter_node:main',
            'scan_relay            = teknofest_ika.gorsel.scan_relay:main',
            'map_image_node        = teknofest_ika.gorsel.map_image_node:main',
        ],
    },
)
