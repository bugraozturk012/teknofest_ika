#!/usr/bin/env python3
# Copyright 2026 Buğra Öztürk
# SPDX-License-Identifier: Apache-2.0

"""
taret_otonom.launch.py — Kumanda Anahtarlı Otonom Taret Zinciri
================================================================
Tam görev yığını (Nav2 + misyon_fsm) olmadan, FlySky'ın mod anahtarına
(SWA) bağlı otonom hedef takibi ve atış:

    Sürüş kartı → seri_kopru → /rc_input → mod_yoneticisi → /mod/aktif
                                                          │
    webcam_taret → preprocessing → targeting_node ────────┘
                       (OTONOM'da arar/kilitler/atar, MANUEL'de durur)
                              │ /turret/cmd  +  /shoot_command
                              ▼
                       taret_rc_koprusu → Turret UNO (servo hız + lazer)

Kumanda MANUEL'deyken: nişan kapalı, SWB açıksa sağ stick tareti sürer.
Kumanda OTONOM'a alınınca: targeting hedefi arar, kilitlenince lazer
darbesini kendisi basar (auto_fire). MANUEL'e dönüş nişanı anında durdurur.

Yarışma yolundan farkı mod_takip+auto_fire parametreleridir;
gercek_arac.launch.py'de ikisi de kapalıdır, enable/atış misyon_fsm'dedir.

HSV parametreleri 2026-07-19 gece kalibrasyonudur (kapalı alan, temiz
lens). Yarışma günü gün ışığında yeniden kalibre edilmelidir: işlenmiş
kareyi PNG kaydet → halka bölgesi H/S/V yüzdeliklerini ölç → S eşiğini
30. yüzdeliğe koy.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Sürüş kartı köprüsü — /rc_input kaynağı (kumanda kanalları buradan gelir).
    seri_kopru = Node(
        package='teknofest_ika', executable='seri_kopru',
        name='seri_kopru', output='screen',
        parameters=[{'use_sim_time': False,
                     'port': '/dev/f767',
                     'baud': 921600}]
    )

    # /rc_input CH5 → /mod/aktif (MANUEL/OTONOM anahtarı).
    mod_yoneticisi = Node(
        package='teknofest_ika', executable='mod_yoneticisi',
        name='mod_yoneticisi', output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Nişan kamerası — taret üstünde, /dev/kamera_nisan (udev, ID ile tanınır).
    # camera_info_url hiç verilmez: boş string usb_cam'de hata üretiyor
    # (sahada doğrulandı, 2026-07-18).
    webcam_taret = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam_taret',
        output='screen',
        parameters=[{
            'video_device':       '/dev/kamera_nisan',
            'image_width':        1280,
            'image_height':       720,
            'framerate':          30.0,
            'pixel_format':       'mjpeg2rgb',
            'camera_name':        'taret',
            'auto_white_balance': True,
            'autoexposure':       True,
        }],
        remappings=[
            ('image_raw',   '/camera/taret/image_raw'),
            ('camera_info', '/camera/taret/camera_info'),
        ]
    )

    preprocessing = Node(
        package='teknofest_ika', executable='preprocessing_node',
        name='preprocessing_node', output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('/depth/points', '/apc/points/data_raw'),
        ]
    )

    targeting = Node(
        package='teknofest_ika', executable='targeting_node',
        name='targeting_node', output='screen',
        parameters=[{
            'use_sim_time':           False,
            'mod_takip':              True,
            'auto_fire':              True,
            'flip_180':               True,   # nişan kamerası ters monte
            # Hedefi YOLO ile bul (hedef_tahtasi kutusu merkezine nişan).
            # Ana kamerayla aynı model; .engine yoksa .pt'ye düşer.
            'detector':               'yolo',
            'yolo_model_path':        'models/best.engine',
            'yolo_conf':              0.45,
            # Hız modu PID (manuel stick ölçümüne göre, 2026-07-22): servo
            # ±13'ün altında dönmüyor (köprü MIN_HIZ=12/20 = manuel taban),
            # bu yüzden otonom eşik yakınında en az ~22 hızda kalıp merkezi
            # aşıyordu → kilitlenemiyordu. D=0 (gürültü), P=0.20 (yumuşak
            # yaklaşım, uzakta bile taşmaz), align 30 px + histerezis: minimum
            # servo hızının bir karelik aşımı bu bandın içinde kalır, kilit
            # sabitlenir. Hizalanınca targeting zaten tam DUR gönderiyor.
            'pid_yaw':                [0.20, 0.0, 0.0],
            'pid_pitch':              [0.20, 0.0, 0.0],
            'align_threshold_px':     30.0,
            'unlock_factor':          1.8,
            'target_ema_alpha':       0.5,
            # Kontrol yönü (sahada doğrulandı 2026-07-22): pan +1, tilt -1
            # (iki eksen bağımsız — tilt -1'de kilitlenirken pan tersti).
            'yaw_sign':               1.0,
            'pitch_sign':             -1.0,
            # Besleme iki servoyu birden döndüremiyor (BEC eksik) → sıralı
            # eksen: önce pan ortalanır, sonra tilt. BEC takılınca False yap.
            'tek_eksen':              True,
            # Nişan kamerası HSV kalibrasyonu (2026-07-19): tabela halkası
            # bu kamerada H=165-169 (magenta tarafı), turuncu bant (H 5-15)
            # sahte tespit kaynağıydı ve aralık dışında bırakıldı; V>=70
            # karanlık sahte daireleri, S>=40 halkanın soluk kısmını tutuyor.
            'hsv_lower':              [0, 40, 70],
            'hsv_upper':              [4, 255, 255],
            'hsv_lower2':             [150, 40, 70],
            'hsv_upper2':             [179, 255, 255],
            # Yakın hedefte halka yarıçapı 200 px'i aşıyor (203 ölçüldü).
            'hough_max_radius':       250,
            # 3 kamera USB2'yi paylaşınca işlenmiş akış ~7 Hz'e düşüyor;
            # daha sıkı eşik her kareyi STALE_IMAGE yapıyordu.
            'image_timeout_sec':      1.0,
            'fire_lock_duration_sec': 0.5,
            'fire_cooldown_sec':      3.0,
            'publish_debug':          True,
        }]
    )

    taret_koprusu = Node(
        package='teknofest_ika', executable='taret_rc_koprusu',
        name='taret_rc_koprusu', output='screen',
        # Otonom nişan hız tavanı — 25 ve 50 mevcut mekanik direnci
        # asamadi (2026-07-23 sahada dogrulandi: sadece tam hizda (-100)
        # donuyor). 60 ile denendi.
        parameters=[{'oto_max_hiz': 60}],
    )

    return LaunchDescription([
        seri_kopru,
        mod_yoneticisi,
        webcam_taret,
        preprocessing,
        targeting,
        taret_koprusu,
    ])
