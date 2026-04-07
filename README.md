# 🤖 HawkBot ROS2 — Open Source Robot Control

> Dự án điều khiển robot HawkBot qua Wi-Fi.  
> Source code đã được reverse-engineer từ binary gốc, chạy được trên **mọi nền tảng** (x86, ARM, Raspberry Pi, Jetson...).

---

## 📋 Tính năng

| Module | Mô tả |
|--------|--------|
| **HBSDK** | Kết nối phần cứng robot qua UDP  |
| **Camera** | Stream MJPEG từ ESP32-CAM, publish qua ROS2 |
| **LiDAR** | Relay dữ liệu LiDAR qua UDP → pseudo-terminal → ydlidar driver |
| **IMU** | Đọc accelerometer + gyroscope, publish `/imu/data_raw` |
| **Odometry** | Encoder → odometry + TF broadcast |
| **AI Vision** | MediaPipe: hand/face/pose detection, gesture control |
| **SLAM** | Cartographer + GMapping |
| **Navigation** | Nav2 autonomous navigation |
| **Teleop** | Điều khiển bàn phím |

---

## ⚡ Cài đặt

### Yêu cầu hệ thống

- **OS**: Ubuntu 22.04 (hoặc bất kỳ Linux nào hỗ trợ ROS2)
- **ROS2**: Humble Hawksbill
- **Python**: 3.8+ (bất kỳ phiên bản nào)
- **Kiến trúc CPU**: x86-64 hoặc ARM (Raspberry Pi, Jetson)

### Bước 1: Cài đặt ROS2 Humble

```bash
# Ubuntu 22.04
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Bước 2: Cài dependencies

```bash
# ROS2 packages
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

# Python packages
pip3 install opencv-python requests mediapipe numpy
```

### Bước 3: Clone & Build

```bash
# Clone dự án
git clone https://github.com/duy12i1i7/hawkbot.git
cd hawkbot

# Source ROS2
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Bước 4: Cài YDLidar driver (tùy chọn — chỉ cần khi dùng LiDAR)

```bash
cd ~/
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ydlidar_ros2_driver
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Sử dụng

### Kết nối robot

1. Kết nối WiFi vào chung hotspot chung robot

### Khởi động đầy đủ

```bash
# Terminal 1: Khởi động tất cả nodes
source /opt/ros/humble/setup.bash
source ~/hawkbot/install/setup.bash
ros2 launch hawkbot bringup_launch.py ip:=<IP_robot>
```

```bash
# Terminal 2: Điều khiển bàn phím
source ~/hawkbot/install/setup.bash
ros2 run hawkbot teleop_keyboard
```

### Chạy từng module riêng

```bash
# Chỉ kết nối điều khiển (UDP + LiDAR)
ros2 run hawkbot hawkbot_node <IP_robot> 1 7 ""

# Chỉ camera stream
ros2 run hawkbot hawkbot_node <IP_robot> 2 7 ""

# Laser warning
ros2 run hawkbotcar_laser laser_Warning

# Hand gesture control
ros2 run hawkbot_mediapipe HandCtrl
```

### Chọn model robot

| Model | Lệnh launch |
|-------|-------------|
| HB-01S | `ros2 launch hawkbot bringup_launch.py ip:=<IP>` |
| HB-03S | `ros2 launch hawkbot bringup03_launch.py ip:=<IP>` |
| HB-05S | `ros2 launch hawkbot bringup05_launch.py ip:=<IP>` |

---

## 📁 Cấu trúc dự án

```
hawkbot/
├── README.md
├── .gitignore
├── src/
│   ├── hawkbot/                       # ⭐ Package chính
│   │   ├── hawkbot/
│   │   │   ├── hawkbot_node.py        # Entry point → gọi HBSDK.run()
│   │   │   ├── teleop_key.py          # Điều khiển bàn phím
│   │   │   ├── sound.py               # Âm thanh/buzzer
│   │   │   └── HBSDK_decompiled/
│   │   │       └── HBSDK.py           # ⭐ SDK 
│   │   ├── launch/
│   │   │   ├── bringup_launch.py      # Launch HB-01S
│   │   │   ├── bringup03_launch.py    # Launch HB-03S
│   │   │   └── bringup05_launch.py    # Launch HB-05S
│   │   └── urdf/                      # Mô hình 3D (URDF/xacro + STL)
│   │
│   ├── hawkbotcar_laser/              # Laser avoidance/tracking/warning
│   ├── hawkbotcar_ai/                 # AI vision (OpenCV, QR tracking)
│   ├── hawkbotcar_msgs/               # Custom ROS2 messages
│   ├── hawkbot_mediapipe/             # MediaPipe nodes (hand/face/pose)
│   ├── hawkbot_cartographer/          # SLAM mapping (Cartographer)
│   ├── hawkbot_navigation2/           # Autonomous navigation (Nav2)
│   ├── robot_localization/            # EKF sensor fusion
│   ├── slam_gmapping/                 # GMapping SLAM
│   └── openslam_gmapping/             # GMapping core library
│
└── Hawkbot_ROS_Control/               # GUI (chỉ script, binary tải riêng)
    └── ClearCache.sh
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
```

**Mã hóa**: XOR cipher, key = `29`

**Format dữ liệu sensor** (CSV, encrypted):
```
odom_x, odom_y, odom_th, odom_vth, odom_vxy,
accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
battery_voltage, motor_current_speed, motor_target_speed
```

---

## 📡 ROS2 Topics

| Topic | Type | Mô tả |
|-------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Điều khiển tốc độ di chuyển |
| `/odom` | `nav_msgs/Odometry` | Odometry từ encoder |
| `/imu/data_raw` | `sensor_msgs/Imu` | IMU (accelerometer + gyroscope) |
| `/battery` | `sensor_msgs/BatteryState` | Điện áp pin |
| `/motor_speed` | `std_msgs/Float32MultiArray` | Tốc độ motor [current, target] |
| `/image_raw/compressed` | `sensor_msgs/CompressedImage` | Camera stream (MJPEG) |
| `/sound` | `std_msgs/String` | Điều khiển buzzer |
| `/servo` | `std_msgs/String` | Điều khiển servo |
| `/pid` | `std_msgs/String` | Cập nhật tham số PID |
| `/robotParam` | `std_msgs/String` | Cập nhật tham số robot |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan (qua ydlidar driver) |

---

## 📦 File lớn (tải riêng khi cần)

Một số file binary/model lớn không có trong repo. Tải về khi cần:

### Face Detection Model (cho `FaceEyeDetection.py`)

```bash
cd src/hawkbot_mediapipe/hawkbot_mediapipe/file/
wget http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2
bzip2 -d shape_predictor_68_face_landmarks.dat.bz2
```

---

## ⚠️ Lưu ý

1. **WiFi**: Kết nối vào WiFi hotspot của robot trước khi chạy
2. **IP mặc định**: `192.168.100.53` hoặc `192.168.4.1`
3. **Camera port**: `3982` (MJPEG stream)
4. **Camera config**: `http://<CAMERA_IP>/control?var=framesize&val=<0-13>`
5. **Encryption**: Tất cả dữ liệu UDP đều mã hóa XOR (key=29)

---

## 📄 License

Dự án phục vụ mục đích học tập và nghiên cứu phát triển.
