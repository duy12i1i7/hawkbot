# 🤖 HawkBot ROS2 — Portable Deployment Guide

## Tổng quan

Dự án điều khiển robot HawkBot qua Wi-Fi, bao gồm:
- **HBSDK** — Module kết nối phần cứng (UDP socket ↔ robot)
- **ROS2 Nodes** — Điều khiển, camera, IMU, odometry, LiDAR
- **AI Vision** — MediaPipe (hand/face/pose detection)
- **SLAM & Navigation** — Cartographer, Nav2, robot_localization
- **GUI Desktop** — Ứng dụng Avalonia/.NET (chỉ x86-64 Linux)

---

## ⚡ Cài đặt nhanh

### Bước 1: Cài đặt ROS2 Humble

```bash
# Ubuntu 22.04
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Bước 2: Cài đặt dependencies

```bash
# ROS2 packages cần thiết
sudo apt install -y \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher \
  ros-humble-imu-complementary-filter \
  ros-humble-robot-localization \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport

# Python dependencies
pip3 install opencv-python requests mediapipe numpy
```

### Bước 3: Copy dự án & Build

```bash
# Copy thư mục HawkBot_Portable vào máy mới
# Ví dụ: scp -r HawkBot_Portable/ user@newmachine:~/

# Di chuyển vào thư mục
cd ~/HawkBot_Portable

# Source ROS2
source /opt/ros/humble/setup.bash

# Build tất cả packages
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Bước 4: Cài YDLidar driver (nếu dùng LiDAR)

```bash
cd ~/
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ydlidar_ros2_driver
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Chạy robot

### Kết nối cơ bản (thay `<ROBOT_IP>` bằng IP robot)

```bash
# Terminal 1: Source workspace
source /opt/ros/humble/setup.bash
source ~/HawkBot_Portable/install/setup.bash

# Khởi động tất cả nodes
ros2 launch hawkbot bringup_launch.py ip:=<ROBOT_IP>
```

### Điều khiển bằng bàn phím

```bash
# Terminal 2:
source ~/HawkBot_Portable/install/setup.bash
ros2 run hawkbot teleop_keyboard
```

### Chỉ chạy từng module

```bash
# Chỉ kết nối điều khiển (UDP + LiDAR):
ros2 run hawkbot hawkbot_node <ROBOT_IP> 1 7 ""

# Chỉ camera stream:
ros2 run hawkbot hawkbot_node <ROBOT_IP> 2 7 ""

# Chạy laser warning:
ros2 run hawkbotcar_laser laser_Warning
```

---

## 📁 Cấu trúc dự án

```
HawkBot_Portable/
├── README.md                          # File này
├── src/
│   ├── hawkbot/                       # ⭐ Package chính
│   │   ├── hawkbot/
│   │   │   ├── hawkbot_node.py        # Entry point → gọi HBSDK.run()
│   │   │   ├── teleop_key.py          # Điều khiển bàn phím
│   │   │   ├── sound.py               # Âm thanh/buzzer
│   │   │   ├── HBSDK.so               # Module gốc (x86-64 Linux + Python 3.10 ONLY)
│   │   │   └── HBSDK_decompiled/
│   │   │       └── HBSDK.py           # ⭐ Source code dịch ngược (chạy mọi platform)
│   │   ├── launch/
│   │   │   ├── bringup_launch.py      # Launch cho HB-01S
│   │   │   ├── bringup03_launch.py    # Launch cho HB-03S
│   │   │   └── bringup05_launch.py    # Launch cho HB-05S
│   │   ├── urdf/                      # Mô hình 3D robot
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   ├── hawkbotcar_laser/              # Laser warning/avoidance
│   ├── hawkbotcar_ai/                 # AI vision (OpenCV scripts)
│   ├── hawkbotcar_msgs/               # Custom ROS2 messages
│   ├── hawkbot_mediapipe/             # MediaPipe AI nodes
│   ├── hawkbot_cartographer/          # SLAM mapping
│   ├── hawkbot_navigation2/           # Autonomous navigation
│   ├── robot_localization/            # EKF sensor fusion
│   ├── slam_gmapping/                 # GMapping SLAM
│   └── openslam_gmapping/             # GMapping core
│
└── Hawkbot_ROS_Control/               # GUI Desktop app
    ├── ROS_Control                     # Binary (x86-64 Linux only)
    ├── libSkiaSharp.so
    ├── libHarfBuzzSharp.so
    └── ClearCache.sh
```

---

## 🔧 Chạy trên nền tảng khác (ARM / Python ≠ 3.10)

File `HBSDK.so` gốc **chỉ chạy trên x86-64 Linux + Python 3.10**. 
Nếu máy bạn dùng **ARM** (Raspberry Pi, Jetson) hoặc **Python khác 3.10**, hãy dùng bản dịch ngược:

```bash
# Sửa file hawkbot_node.py để import từ bản dịch ngược
cd ~/HawkBot_Portable/src/hawkbot/hawkbot/

# Backup bản gốc
cp hawkbot_node.py hawkbot_node.py.bak

# Sửa import: thay "from HBSDK import run" thành:
sed -i 's|from HBSDK import run|from HBSDK_decompiled.HBSDK import run|' hawkbot_node.py

# Rebuild
cd ~/HawkBot_Portable
colcon build --packages-select hawkbot --symlink-install
source install/setup.bash
```

---

## 🌐 Kiến trúc mạng

```
┌─────────────────────┐         WiFi          ┌──────────────────────┐
│   PC (ROS2 nodes)   │◄──────────────────────►│   HawkBot Robot     │
│                     │                        │                     │
│ UDP:29083 ◄─────────│── sensor data (XOR) ──│──► UDP:29084        │
│ UDP:29083 ──────────│── cmd_vel (XOR)  ─────│──► UDP:29084        │
│ UDP:18902 ◄─────────│── LiDAR data ─────────│──► UDP:18903        │
│ HTTP ◄──────────────│── camera MJPEG ───────│──► :3982/stream     │
│                     │                        │                     │
│ camera config ──────│── HTTP GET ────────────│──► /control?var=... │
└─────────────────────┘                        └──────────────────────┘

Encryption: XOR cipher, key = 29

Sensor data format (CSV, encrypted):
  odom_x, odom_y, odom_th, odom_vth, odom_vxy,
  accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
  battery, motor_current_Speed, motor_target_Speed
```

---

## 📋 ROS2 Topics

| Topic | Type | Mô tả |
|-------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Điều khiển tốc độ |
| `/odom` | `nav_msgs/Odometry` | Odometry từ encoder |
| `/imu/data_raw` | `sensor_msgs/Imu` | Dữ liệu IMU (accel + gyro) |
| `/battery` | `sensor_msgs/BatteryState` | Điện áp pin |
| `/motor_speed` | `std_msgs/Float32MultiArray` | Tốc độ motor |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | Camera stream |
| `/sound` | `std_msgs/String` | Điều khiển buzzer |
| `/servo` | `std_msgs/String` | Điều khiển servo |
| `/pid` | `std_msgs/String` | Cập nhật PID |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan (qua ydlidar driver) |

---

## ⚠️ Lưu ý quan trọng

1. **Mật khẩu WiFi robot**: Kết nối vào WiFi hotspot của robot trước khi chạy
2. **IP mặc định**: Robot thường ở `192.168.100.53` hoặc `192.168.4.1`
3. **Camera port**: `3982` (không phải 81 như ESP32 thông thường)
4. **HBSDK.so vs HBSDK.py**: 
   - `.so` = bản gốc Cython (nhanh hơn, nhưng chỉ x86-64 + Python 3.10)
   - `.py` = bản dịch ngược (chạy mọi nơi, chậm hơn không đáng kể)
5. **GUI ROS_Control**: Chỉ chạy trên x86-64 Linux (NativeAOT compiled)

---

## 🔌 Model robot hỗ trợ

| Model | Launch file | Mô tả |
|-------|-------------|--------|
| HB-01S | `bringup_launch.py` | Mẫu cơ bản |
| HB-03S | `bringup03_launch.py` | Mẫu trung cấp |
| HB-05S | `bringup05_launch.py` | Mẫu cao cấp |
