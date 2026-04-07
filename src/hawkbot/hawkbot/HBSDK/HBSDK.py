#!/usr/bin/env python3
# ======================================================================
# HBSDK.py — HawkBot robot SDK bridge for ROS2
# ======================================================================

import os
import sys
import io
import time
import json
import math
import signal
import socket
import ctypes
import pty
import tempfile
import subprocess
import ipaddress
import requests
import cv2
from math import sin, cos
from multiprocessing import Process
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from geometry_msgs.msg import Twist, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CompressedImage, CameraInfo, BatteryState
from std_msgs.msg import String, Float32MultiArray
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

# ─── Module Constants ────────────────────────────────────────────────
control_data_dpt = 29084       # UDP destination port for control data
control_data_spt = 29083       # UDP source port for control data
lidar_data_dpt = 18903         # UDP destination port for LiDAR data
lidar_data_spt = 18902         # UDP source port for LiDAR data
robot_frame_id = 'base_footprint'
gyro_frame_id = 'gyro_link'
is_stop = False


# ═════════════════════════════════════════════════════════════════════
# CLASS: API (lines 44-88)
# ═════════════════════════════════════════════════════════════════════
class API:
    """HTTP API client for configuring ESP32 camera"""

    def __init__(self):  # line 44
        pass

    def set_cam_framesize(self, val, camera_ip):  # line 47
        """Set camera frame size via HTTP request to ESP32-CAM
        Format: http://<camera_ip>/control?var=framesize&val=<val>
        """
        try:
            url = "http://%s/control?var=framesize&val=%d" % (camera_ip, int(val))
            resp = requests.get(url, timeout=5)
        except Exception as err:
            print(f"set_cam_framesize error: {err}")


# ═════════════════════════════════════════════════════════════════════
# CLASS: BaseController (lines 89-230)
# ═════════════════════════════════════════════════════════════════════
class BaseController(Node):
    """Main robot controller. Bridges ROS2 ↔ UDP hardware communication."""

    def __init__(self, robot_id='', robot_ip='', udp_socket=None):  # line 89
        super().__init__('base_control')
        self.robot_id = robot_id
        self.robot_ip = robot_ip
        self.udp_socket = udp_socket

        # ── Subscribers ──
        if robot_id and robot_id != '':
            self.sub_cmd_vel = self.create_subscription(
                Twist, f'/{robot_id}/cmd_vel', self.cmdVelCallback, 10)
            self.sub_sound = self.create_subscription(
                String, f'/{robot_id}/sound', self.soundCallback, 10)
            self.sub_pid = self.create_subscription(
                String, f'/{robot_id}/pid', self.pidCallback, 10)
            self.sub_servo = self.create_subscription(
                String, f'/{robot_id}/servo', self.servoCallback, 10)
            self.sub_param = self.create_subscription(
                String, f'/{robot_id}/robotParam', self.robotParamCallback, 10)
        else:
            self.sub_cmd_vel = self.create_subscription(
                Twist, '/cmd_vel', self.cmdVelCallback, 10)
            self.sub_sound = self.create_subscription(
                String, '/sound', self.soundCallback, 10)
            self.sub_pid = self.create_subscription(
                String, '/pid', self.pidCallback, 10)
            self.sub_servo = self.create_subscription(
                String, '/servo', self.servoCallback, 10)
            self.sub_param = self.create_subscription(
                String, '/robotParam', self.robotParamCallback, 10)

        # ── Publishers ──
        self.odomPub = self.create_publisher(Odometry, '/odom', 10)
        self.odomBroadcaster = TransformBroadcaster(self)

    def cmdVelCallback(self, req):  # line 147
        """Receive Twist → encrypt → send via UDP to robot hardware"""
        try:
            vx = req.linear.x
            vz = req.angular.z
            cmd_str = f"CMD_VEL:{vx:.4f},{vz:.4f}"
            enc_data = enrypt_str(cmd_str)
            self.udp_socket.sendto(enc_data, (self.robot_ip, control_data_dpt))
        except Exception as err:
            pass

    def soundCallback(self, req):  # line 161
        """Send sound/buzzer command to robot"""
        try:
            cmd_str = f"SOUND:{req.data}"
            enc_data = enrypt_str(cmd_str)
            self.udp_socket.sendto(enc_data, (self.robot_ip, control_data_dpt))
        except Exception as err:
            pass

    def pidCallback(self, req):  # line 169
        """Update PID parameters on robot"""
        try:
            cmd_str = f"PID:{req.data}"
            enc_data = enrypt_str(cmd_str)
            self.udp_socket.sendto(enc_data, (self.robot_ip, control_data_dpt))
        except Exception as err:
            pass

    def robotParamCallback(self, req):  # line 176
        """Update robot parameters"""
        try:
            cmd_str = f"PARAM:{req.data}"
            enc_data = enrypt_str(cmd_str)
            self.udp_socket.sendto(enc_data, (self.robot_ip, control_data_dpt))
        except Exception as err:
            pass

    def servoCallback(self, req):  # line 184
        """Control servo motors"""
        try:
            cmd_str = f"SERVO:{req.data}"
            enc_data = enrypt_str(cmd_str)
            self.udp_socket.sendto(enc_data, (self.robot_ip, control_data_dpt))
        except Exception as err:
            pass

    def pubOdom(self, odom_x, odom_y, odom_th, odom_vth, odom_vxy):  # line 190
        """Publish odometry from hardware data + broadcast TF"""
        from geometry_msgs.msg import TransformStamped

        current_time = self.get_clock().now().to_msg()

        # Create quaternion from yaw
        odom_quat = Quaternion()
        odom_quat.x = 0.0
        odom_quat.y = 0.0
        odom_quat.z = sin(odom_th / 2.0)
        odom_quat.w = cos(odom_th / 2.0)

        # Broadcast TF: odom -> base_footprint
        tfile = TransformStamped()
        tfile.header.stamp = current_time
        tfile.header.frame_id = 'odom'
        tfile.child_frame_id = robot_frame_id
        tfile.transform.translation.x = odom_x
        tfile.transform.translation.y = odom_y
        tfile.transform.translation.z = 0.0
        tfile.transform.rotation = odom_quat
        self.odomBroadcaster.sendTransform(tfile)

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = robot_frame_id

        odom_msg.pose.pose.position.x = odom_x
        odom_msg.pose.pose.position.y = odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_quat

        odom_msg.twist.twist.linear.x = odom_vxy
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = odom_vth

        self.odomPub.publish(odom_msg)


# ═════════════════════════════════════════════════════════════════════
# CLASS: BATTERY (lines 233-255)
# ═════════════════════════════════════════════════════════════════════
class BATTERY(Node):
    """Battery state publisher node"""

    def __init__(self, robot_id=''):  # line 233
        super().__init__('battery')
        if robot_id and robot_id != '':
            self.pub_battery = self.create_publisher(BatteryState, f'/{robot_id}/battery', 10)
        else:
            self.pub_battery = self.create_publisher(BatteryState, '/battery', 10)

    def bat_update(self, battery_voltage):  # line 242
        """Publish battery voltage as BatteryState message"""
        bat_msg = BatteryState()
        bat_msg.voltage = float(battery_voltage)
        self.pub_battery.publish(bat_msg)


# ═════════════════════════════════════════════════════════════════════
# CLASS: MotorSpeed (lines 256-272)
# ═════════════════════════════════════════════════════════════════════
class MotorSpeed(Node):
    """Motor speed feedback publisher"""

    def __init__(self, robot_id=''):  # line 256
        super().__init__('motor_speed')
        if robot_id and robot_id != '':
            self.motorSpeedPub = self.create_publisher(
                Float32MultiArray, f'/{robot_id}/motor_speed', 10)
        else:
            self.motorSpeedPub = self.create_publisher(
                Float32MultiArray, '/motor_speed', 10)

    def motor_speed_update(self, motor_current_Speed, motor_target_Speed):  # line 265
        """Publish current and target motor speeds"""
        motor_speed_data = Float32MultiArray()
        motor_speed_data.data = [float(motor_current_Speed), float(motor_target_Speed)]
        self.motorSpeedPub.publish(motor_speed_data)


# ═════════════════════════════════════════════════════════════════════
# CLASS: MPU (lines 273-302)
# ═════════════════════════════════════════════════════════════════════
class MPU(Node):
    """IMU (accelerometer + gyroscope) data publisher"""

    def __init__(self, robot_id):  # line 273
        super().__init__('imu')
        if robot_id and robot_id != '':
            self.pub_imu = self.create_publisher(Imu, f'/{robot_id}/imu/data_raw', 10)
        else:
            self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Initialize covariance matrices
        self.angular_velocity_covariance = [0.0] * 9
        self.linear_acceleration_covariance = [0.0] * 9
        for i in range(0, 9, 4):  # diagonal elements
            self.angular_velocity_covariance[i] = 0.01
            self.linear_acceleration_covariance[i] = 0.01

    def imu_update(self, ax, ay, az, gx, gy, gz):  # line 284
        """Publish IMU data: accel (ax,ay,az) + gyro (gx,gy,gz)"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = gyro_frame_id

        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        imu_msg.angular_velocity_covariance = self.angular_velocity_covariance

        self.pub_imu.publish(imu_msg)


# ═════════════════════════════════════════════════════════════════════
# CLASS: VIDEO (lines 303-400)
# ═════════════════════════════════════════════════════════════════════
class VIDEO(Node):
    """Video stream receiver from ESP32-CAM"""

    def __init__(self, robot_id, camera_ip):  # line 303
        super().__init__('HawkBotVideo')
        self.camera_ip = camera_ip
        self.bridge = CvBridge()

        if robot_id and robot_id != '':
            self.VideoPub = self.create_publisher(
                CompressedImage, f'/{robot_id}/image_raw/compressed', 1)
            self.VideoCompressPub = self.create_publisher(
                Image, f'/{robot_id}/image/image_raw', 1)
        else:
            self.VideoPub = self.create_publisher(
                CompressedImage, '/image_raw/compressed', 1)
            self.VideoCompressPub = self.create_publisher(
                Image, '/image/image_raw', 1)

        # Connect to ESP32-CAM MJPEG stream
        stream_url = "http://" + camera_ip + ":3982/stream"
        try:
            self.stream = cv2.VideoCapture(stream_url)
        except Exception as err:
            self.get_logger().error(f"VideoCapture error: {err}")
            self.stream = None

        # Timer for video update (~30fps)
        self.timer = self.create_timer(0.033, self.video_update)

    @contextmanager
    def stderr_redirector(self, stream):  # line ~340
        """Redirect stderr to suppress OpenCV warnings during read"""
        def _redirect_stderr():
            old_stderr = os.dup(2)
            os.dup2(stream.fileno(), 2)
            try:
                yield
            finally:
                os.dup2(old_stderr, 2)
                os.close(old_stderr)
        return _redirect_stderr()

    def video_update(self):  # line 343
        """Timer callback: read frame from camera → publish"""
        if self.stream is None or not self.stream.isOpened():
            return
        try:
            f = io.BytesIO()
            with self.stderr_redirector(f):
                ret, frame = self.stream.read()
            if ret and frame is not None:
                # Publish compressed image
                msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                self.VideoPub.publish(msg)
        except Exception as err:
            pass


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: udp_client_process (lines 404-494)
# Main UDP communication process — the CORE hardware bridge
# ═════════════════════════════════════════════════════════════════════
def udp_client_process(host, robot_id):
    """
    Runs in separate Process. Creates ROS2 nodes and handles
    bidirectional UDP communication with robot hardware.
    
    Receives: odom, IMU, battery, motor speed data (encrypted)
    Sends: cmd_vel, sound, servo, PID, param commands (encrypted)
    """
    rclpy.init()

    # Create UDP socket
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', control_data_spt))
    socket.setdefaulttimeout(5)

    # Create ROS2 nodes
    base_control = BaseController(robot_id, host, udp_socket)
    imu = MPU(robot_id)
    bat = BATTERY(robot_id)
    motor_speed = MotorSpeed(robot_id)

    # Use MultiThreadedExecutor for concurrent processing
    executor = MultiThreadedExecutor()
    executor.add_node(base_control)
    executor.add_node(imu)
    executor.add_node(bat)
    executor.add_node(motor_speed)

    while not is_stop:
        try:
            # Receive data from robot
            data, addr = udp_socket.recvfrom(4096)
            rec_str = decrypt_controller_data(data)
            sig_list = rec_str.split(',')

            if len(sig_list) < 5:
                continue

            sig = sig_list[0] if len(sig_list) > 0 else ''
            other_list = sig_list[1:] if len(sig_list) > 1 else []

            # Parse odometry data
            odom_x = float(sig_list[0]) if len(sig_list) > 0 else 0.0
            odom_y = float(sig_list[1]) if len(sig_list) > 1 else 0.0
            odom_th = float(sig_list[2]) if len(sig_list) > 2 else 0.0
            odom_vth = float(sig_list[3]) if len(sig_list) > 3 else 0.0
            odom_vxy = float(sig_list[4]) if len(sig_list) > 4 else 0.0

            # Parse IMU data
            accel_x = float(sig_list[5]) if len(sig_list) > 5 else 0.0
            accel_y = float(sig_list[6]) if len(sig_list) > 6 else 0.0
            accel_z = float(sig_list[7]) if len(sig_list) > 7 else 0.0
            gyro_x = float(sig_list[8]) if len(sig_list) > 8 else 0.0
            gyro_y = float(sig_list[9]) if len(sig_list) > 9 else 0.0
            gyro_z = float(sig_list[10]) if len(sig_list) > 10 else 0.0

            # Parse battery and motor data
            battery = float(sig_list[11]) if len(sig_list) > 11 else 0.0
            motor_current_Speed = float(sig_list[12]) if len(sig_list) > 12 else 0.0
            motor_target_Speed = float(sig_list[13]) if len(sig_list) > 13 else 0.0

            # Update all ROS2 nodes
            base_control.pubOdom(odom_x, odom_y, odom_th, odom_vth, odom_vxy)
            imu.imu_update(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
            bat.bat_update(battery)
            motor_speed.motor_speed_update(motor_current_Speed, motor_target_Speed)

            # Spin executor briefly to process callbacks (cmd_vel, sound, etc.)
            executor.spin_once(timeout_sec=0.001)

        except socket.timeout:
            executor.spin_once(timeout_sec=0.001)
        except ValueError:
            # Invalid data format, skip
            pass
        except Exception as err:
            executor.spin_once(timeout_sec=0.001)

    # Cleanup
    base_control.destroy_node()
    imu.destroy_node()
    bat.destroy_node()
    motor_speed.destroy_node()
    rclpy.shutdown()


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: video_client_process (lines 495-510)
# ═════════════════════════════════════════════════════════════════════
def video_client_process(robot_ip):
    """Video streaming process (runs in separate Process)"""
    rclpy.init()
    try:
        argDic = get_param_from_json(robot_ip)
        video = VIDEO(
            argDic.get('robot_id', ''),
            argDic.get('camera_ip', robot_ip)
        )
        rclpy.spin(video)
    except ExternalShutdownException:
        pass
    except Exception as err:
        print(f"video_client_process error: {err}")
    finally:
        rclpy.shutdown()


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: lidar_trans (lines 511-548)
# LiDAR UDP→PTY relay
# ═════════════════════════════════════════════════════════════════════
def lidar_trans(host, pty_master):
    """
    Receive LiDAR data via UDP from robot, write to PTY.
    This allows ydlidar_ros2_driver to read from a virtual serial port.
    """
    lidar_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lidar_udp_socket.bind(('0.0.0.0', lidar_data_spt))

    while not is_stop:
        try:
            lidar_data, _ = lidar_udp_socket.recvfrom(4096)
            os.write(pty_master, lidar_data)
        except Exception as err:
            pass

    lidar_udp_socket.close()


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: signal_handler (lines 540-548)
# ═════════════════════════════════════════════════════════════════════
def signal_handler(sig, frame):
    """Handle SIGINT for graceful shutdown"""
    global is_stop
    is_stop = True
    sys.exit(0)


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: mkpty (lines 549-560)
# ═════════════════════════════════════════════════════════════════════
def mkpty():
    """Create pseudo-terminal pair for LiDAR data relay"""
    pty_master, slave = pty.openpty()
    pty_port = os.ttyname(slave)
    return pty_master, pty_port


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: save_param_to_json (lines 561-567)
# ═════════════════════════════════════════════════════════════════════
def save_param_to_json(robot_ip, lidar_port, robot_id, cam_quality, camera_ip):
    """Save robot connection parameters to temp JSON file"""
    resultfile = os.path.join(tempfile.gettempdir(), 'hawkbot_params.json')
    param_dic = {
        'robot_ip': robot_ip,
        'lidar_port': lidar_port,
        'robot_id': robot_id,
        'cam_quality': cam_quality,
        'camera_ip': camera_ip,
    }
    with open(resultfile, 'w') as f:
        json.dump(param_dic, f)


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: get_param_from_json (lines 568-576)
# ═════════════════════════════════════════════════════════════════════
def get_param_from_json(robot_ip):
    """Load robot connection parameters from temp JSON file"""
    resultfile = os.path.join(tempfile.gettempdir(), 'hawkbot_params.json')
    try:
        with open(resultfile, 'r') as f:
            content = f.read()
        paramDic = json.loads(content)
        return paramDic
    except Exception:
        return {
            'robot_ip': robot_ip,
            'lidar_port': '',
            'robot_id': '',
            'cam_quality': '7',
            'camera_ip': robot_ip,
        }


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: decrypt_controller_data (lines 577-598)
# ═════════════════════════════════════════════════════════════════════
def decrypt_controller_data(raw_data, dec_num=29):
    """
    Decrypt data from robot using XOR cipher.
    Scans for data start marker, then decrypts each byte.
    """
    dec_data = ''
    findStart = False
    for d in raw_data:
        decrypted = d ^ dec_num
        if not findStart:
            # Look for start of valid data
            ch = chr(decrypted)
            if ch == '-' or ch == '.' or ch.isdigit():
                findStart = True
                dec_data += ch
        else:
            dec_data += chr(decrypted)
    return dec_data


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: enrypt_str (lines 599-607) — [sic: typo in original]
# ═════════════════════════════════════════════════════════════════════
def enrypt_str(raw_str, dec_num=29):
    """Encrypt string using XOR cipher before sending to robot"""
    dec_data = bytearray()
    for d in raw_str:
        dec_data.append(ord(d) ^ dec_num)
    return bytes(dec_data)


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: is_valid_ip_address (lines 608-616)
# ═════════════════════════════════════════════════════════════════════
def is_valid_ip_address(ip_str):
    """Validate IP address string"""
    try:
        ipaddress.ip_address(ip_str)
        return True
    except ValueError:
        return False


# ═════════════════════════════════════════════════════════════════════
# FUNCTION: run (lines 617-690) — MAIN ENTRY POINT
# Called from hawkbot_node.py: run(robot_ip, node_type, cam_quality, robot_param)
# ═════════════════════════════════════════════════════════════════════
def run(robot_ip, node_type='2', cam_quality='7', robot_param=''):
    """
    Main entry point for HawkBot SDK.

    Args:
        robot_ip: IP address of the robot
        node_type: "1" = control node (UDP + LiDAR), "2" = video node
        cam_quality: Camera quality level (0-13, default 7)
        robot_param: Additional robot parameters

    Flow:
        1. Connect to robot via UDP to discover camera_ip and robot_id
        2. Save params to JSON for inter-process sharing
        3. Launch appropriate processes based on node_type
    """
    signal.signal(signal.SIGINT, signal_handler)

    if not is_valid_ip_address(robot_ip):
        print(f"Invalid IP address: {robot_ip}")
        return

    try:
        # Initial discovery: send handshake to robot
        tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tmp_socket.settimeout(10)

        # Send initial connection request
        init_msg = enrypt_str("INIT")
        tmp_socket.sendto(init_msg, (robot_ip, control_data_dpt))

        # Receive response with robot config
        data, addr = tmp_socket.recvfrom(4096)
        rec_str = decrypt_controller_data(data)
        sig_list = rec_str.split(',')

        # Parse robot info from response
        robot_id = ''
        camera_ip = robot_ip
        ip_sig_list = []

        for sig in sig_list:
            if is_valid_ip_address(sig.strip()):
                ip_sig_list.append(sig.strip())
            elif sig.strip() != '':
                other_list = sig.strip()

        if len(ip_sig_list) > 0:
            camera_ip = ip_sig_list[0]

        if len(sig_list) > 0:
            for sig in sig_list:
                sig = sig.strip()
                if sig != '' and not is_valid_ip_address(sig):
                    robot_id = sig
                    break

        tmp_socket.close()

    except socket.timeout:
        print(f"Timeout connecting to robot at {robot_ip}")
        camera_ip = robot_ip
        robot_id = ''
    except Exception as err:
        print(f"Connection error: {err}")
        camera_ip = robot_ip
        robot_id = ''

    # Save params for inter-process sharing
    lidar_port = ''
    save_param_to_json(robot_ip, lidar_port, robot_id, cam_quality, camera_ip)

    if node_type == '1':
        # ── Control Node ──
        # Create PTY for LiDAR data relay
        pty_master, pty_port = mkpty()

        # Start LiDAR relay in background process
        lidar_process = Process(target=lidar_trans, args=(robot_ip, pty_master))
        lidar_process.daemon = True
        lidar_process.start()

        # Run main UDP control loop (blocking)
        udp_client_process(robot_ip, robot_id)

    elif node_type == '2':
        # ── Video Node ──
        # Set camera quality
        api = API()
        api.set_cam_framesize(int(cam_quality), camera_ip)

        # Start video streaming (blocking)
        video_client_process(robot_ip)

    else:
        print(f"Unknown node_type: {node_type}")
