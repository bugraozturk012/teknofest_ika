# Kurulum ve çalıştırma

> LYDİA İKA otonom sürüş yazılımı — ayrıntılı belge.
> Genel bakış için [README](../README.md).

---

## Kurulum

```bash
# ROS2 Humble bağımlılıkları
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-ackermann-msgs \
  ros-humble-ydlidar-ros2-driver

pip3 install pyserial smbus2 ultralytics --break-system-packages

# OS30A derinlik kamerası (eYs3D BMVM0S30A) — AYRI bir workspace'te kurulur,
# rosdep ile çözümlenemeyen üçüncü parti bir paket olduğu için package.xml'e
# <depend> olarak eklenmemiştir (eklenirse rosdep install hata verir):
git clone https://github.com/eYs3D/HD-DM-ROS2-SDK-Release.git ~/eys3d_ws/src/dm_preview
cd ~/eys3d_ws && rosdep install -i --from-path src -y
colcon build --symlink-install
echo "source ~/eys3d_ws/install/setup.bash" >> ~/.bashrc

# Workspace build
cd ~/lydia_ws
# --symlink-install kullanilmaz: aractaki install/ kopya tabanli, bayrakla
# derlemek maps/teknofest_harita.pgm uzerinde [Errno 2] verip yarida keser.
colcon build --packages-select teknofest_ika
source install/setup.bash
echo "source ~/lydia_ws/install/setup.bash" >> ~/.bashrc
```

---

## Çalıştırma

### Gerçek Araç

```bash
# Harita alma (sahada, ilk çalıştırma)
ros2 launch teknofest_ika gercek_harita.launch.py
ros2 run nav2_map_server map_saver_cli -f ~/lydia_ws/maps/gercek_harita

# Yarışma
ros2 launch teknofest_ika gercek_arac.launch.py
ros2 topic pub /mission_start std_msgs/msg/Bool "data: true" --once
```

### Faydalı Komutlar

```bash
# TF zinciri kontrolü
ros2 run tf2_ros tf2_echo map odom

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Lidar veri kalitesi
ros2 topic hz /scan_lidar

# Nav2 manuel hedef
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
```

---
