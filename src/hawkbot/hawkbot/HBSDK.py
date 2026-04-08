#!/usr/bin/python3
# encoding:utf-8
# Reverse-engineered from HBSDK.so (Cython 3.0.9 compiled)

import sys
import os
import io
import json
import signal
import socket
import subprocess
import tempfile
import time
import ctypes
import pty
import ipaddress
from math import sin, cos
from contextlib import contextmanager
from multiprocessing import Process

import cv2
import requests
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
try:
    from rclpy.exceptions import ExternalShutdownException
except ImportError:
    ExternalShutdownException = RuntimeError

from geometry_msgs.msg import Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState, CameraInfo, CompressedImage, Image
from std_msgs.msg import String, Float32MultiArray
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge

# Module-level constants
control_data_dpt = 29084
control_data_spt = 29083
lidar_data_dpt = 18903
lidar_data_spt = 18902
gyro_frame_id = 'gyro_link'
robot_frame_id = 'base_footprint'
is_stop = False


def signal_handler(signal, frame):
    global is_stop
    is_stop = True


def is_valid_ip_address(ip_str):
    try:
        ipaddress.ip_address(ip_str)
        return True
    except ValueError:
        return False


def enrypt_str(raw_str, dec_num=29):
    dec_data = b''
    for d in raw_str:
        dec_data += bytes([ord(d) - dec_num])
    return dec_data


def decrypt_controller_data(raw_data, dec_num=29):
    dec_data = ''
    findStart = False
    for d in raw_data:
        if d + dec_num == ord('#'):
            findStart = True
            continue
        if findStart:
            dec_data += chr(d + dec_num)
    return dec_data


def mkpty():
    pty_master, slave = pty.openpty()
    pty_port = os.ttyname(slave)
    return pty_master, slave, pty_port


def save_param_to_json(robot_ip, lidar_port, robot_id, cam_quality, camera_ip):
    resultfile = '/tmp/' + robot_ip + '.json'
    param_dic = {
        'robot_ip': robot_ip,
        'lidar_port': lidar_port,
        'robot_id': robot_id,
        'cam_quality': cam_quality,
        'camera_ip': camera_ip
    }
    with open(resultfile, 'w') as f:
        json.dumps(param_dic)
        f.write(json.dumps(param_dic))


def get_param_from_json(robot_ip):
    resultfile = '/tmp/' + robot_ip + '.json'
    try:
        with open(resultfile, 'r') as f:
            content = f.read()
        paramDic = json.loads(content)
        return paramDic
    except:
        return None


class API:
    def __init__(self):
        pass

    def set_cam_framesize(self, val, camera_ip):
        try:
            resp = requests.get("http://%s/control?var=framesize&val=%d" % (camera_ip, val))
        except Exception as err:
            pass


class BaseController(Node):
    def __init__(self, robot_id='', robot_ip='', udp_socket=None):
        super().__init__('HawkBotBase' + robot_id)
        self.udp_socket = udp_socket
        self.robot_ip = robot_ip
        self.robot_id = robot_id

        self.odomPub = self.create_publisher(Odometry, '/odom', 1)
        self.odomBroadcaster = TransformBroadcaster(self)

        if robot_id == '':
            self.cmd_vel_subscription = self.create_subscription(
                Twist, '/cmd_vel', self.cmdVelCallback, 1)
            self.sound_subscription = self.create_subscription(
                String, '/sound', self.soundCallback, 1)
            self.servo_subscription = self.create_subscription(
                String, '/servo', self.servoCallback, 1)
            self.pid_subscription = self.create_subscription(
                String, '/pid', self.pidCallback, 1)
            self.robot_param_subscription = self.create_subscription(
                String, '/robot_param', self.robotParamCallback, 1)
        else:
            self.cmd_vel_id_subscription = self.create_subscription(
                Twist, '/cmd_vel' + robot_id, self.cmdVelCallback, 1)
            self.sound_id_subscription = self.create_subscription(
                String, '/sound' + robot_id, self.soundCallback, 1)
            self.servo_id_subscription = self.create_subscription(
                String, '/servo' + robot_id, self.servoCallback, 1)
            self.pid_id_subscription = self.create_subscription(
                String, '/pid' + robot_id, self.pidCallback, 1)
            self.robot_param_subscription = self.create_subscription(
                String, '/robot_param' + robot_id, self.robotParamCallback, 1)

    def cmdVelCallback(self, req):
        try:
            vx = req.linear.x
            vz = req.angular.z
            self.udp_socket.sendto(
                enrypt_str("#A%f:%f*" % (vx, vz)),
                (self.robot_ip, control_data_spt))
        except Exception as err:
            pass

    def soundCallback(self, req):
        try:
            self.udp_socket.sendto(
                enrypt_str("#B" + req.data + "*"),
                (self.robot_ip, control_data_spt))
        except Exception as err:
            pass

    def pidCallback(self, req):
        try:
            self.udp_socket.sendto(
                enrypt_str("#D" + req.data + "*"),
                (self.robot_ip, control_data_spt))
        except Exception as err:
            pass

    def servoCallback(self, req):
        try:
            self.udp_socket.sendto(
                enrypt_str("#E" + req.data + "*"),
                (self.robot_ip, control_data_spt))
        except Exception as err:
            pass

    def robotParamCallback(self, req):
        try:
            self.udp_socket.sendto(
                enrypt_str("#G" + req.data + "*"),
                (self.robot_ip, control_data_spt))
        except Exception as err:
            pass

    def pubOdom(self, odom_x, odom_y, odom_th, odom_vth, odom_vxy):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(odom_th / 2.0)
        quaternion.w = cos(odom_th / 2.0)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = robot_frame_id
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.pose.covariance = [0.0] * 36
        odom.twist.twist.linear.x = odom_vxy
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = odom_vth
        odom.twist.covariance = [0.0] * 36
        self.odomPub.publish(odom)


class MPU(Node):
    def __init__(self, robot_id):
        super().__init__('imu' + robot_id)
        self.imuPub = self.create_publisher(Imu, '/imu/data_raw', 1)
        self.imuMsg = Imu()
        self.imuMsg.header.frame_id = gyro_frame_id
        for i in range(9):
            self.imuMsg.orientation_covariance[i] = 0.0
            self.imuMsg.angular_velocity_covariance[i] = 0.0
            self.imuMsg.linear_acceleration_covariance[i] = 0.0

    def imu_update(self, ax, ay, az, gx, gy, gz):
        self.imuMsg.header.stamp = self.get_clock().now().to_msg()
        self.imuMsg.linear_acceleration.x = ax
        self.imuMsg.linear_acceleration.y = ay
        self.imuMsg.linear_acceleration.z = az
        self.imuMsg.angular_velocity.x = gx
        self.imuMsg.angular_velocity.y = gy
        self.imuMsg.angular_velocity.z = gz
        self.imuPub.publish(self.imuMsg)


class BATTERY(Node):
    def __init__(self, robot_id=''):
        super().__init__('battery' + robot_id)
        self.batPub = self.create_publisher(BatteryState, '/battery', 1)
        self.batMsg = BatteryState()

    def bat_update(self, battery_voltage):
        self.batMsg.voltage = battery_voltage
        self.batPub.publish(self.batMsg)


class MotorSpeed(Node):
    def __init__(self, robot_id=''):
        super().__init__('motor_speed' + robot_id)
        self.motorSpeedPub = self.create_publisher(Float32MultiArray, '/motor_speed', 1)

    def motor_speed_update(self, motor_current_Speed, motor_target_Speed):
        motor_speed_data = Float32MultiArray()
        motor_speed_data.data = [motor_current_Speed, motor_target_Speed]
        self.motorSpeedPub.publish(motor_speed_data)


class VIDEO(Node):
    def __init__(self, robot_id, camera_ip):
        super().__init__('HawkBotVideo' + robot_id)
        self.br = CvBridge()
        try:
            self.stream_url = 'http://' + camera_ip + ':3982/stream'
            with self._redirect_stderr():
                self.cap = cv2.VideoCapture(self.stream_url)
            self.VideoPub = self.create_publisher(Image, '/image/image_raw', 1)
            self.VideoCompressPub = self.create_publisher(CompressedImage, '/image_raw/compressed', 1)
            self.create_timer(0.03, self.video_update)
        except Exception as err:
            self.get_logger().info(str(err))

    @contextmanager
    def _redirect_stderr(self):
        original_stderr_fd = sys.stderr.fileno()
        saved_stderr_fd = os.dup(original_stderr_fd)
        tfile = tempfile.TemporaryFile(mode='w+b')
        os.dup2(tfile.fileno(), original_stderr_fd)
        try:
            yield
        finally:
            os.dup2(saved_stderr_fd, original_stderr_fd)
            tfile.seek(0, os.SEEK_SET)
            tfile.close()
            os.close(saved_stderr_fd)

    # Alias for the contextmanager
    stderr_redirector = _redirect_stderr

    def video_update(self):
        try:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    f = self.br.cv2_to_imgmsg(frame, 'bgr8')
                    self.VideoPub.publish(f)
                    f = self.br.cv2_to_compressed_imgmsg(frame, 'jpg')
                    self.VideoCompressPub.publish(f)
        except Exception as err:
            self.get_logger().info(str(err))


def lidar_trans(host, pty_master):
    lidar_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lidar_udp_socket.bind(('0.0.0.0', lidar_data_dpt))
    while not is_stop:
        try:
            lidar_data = lidar_udp_socket.recv(999)
            os.write(pty_master, lidar_data)
        except Exception as err:
            pass


def udp_client_process(host, robot_id):
    rclpy.init()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind(('0.0.0.0', control_data_dpt))
    base_control = BaseController(robot_id=robot_id, robot_ip=host, udp_socket=udp_socket)
    imu = MPU(robot_id)
    bat = BATTERY(robot_id)
    motor_speed = MotorSpeed(robot_id)

    executor = MultiThreadedExecutor()
    executor.add_node(base_control)
    executor.add_node(imu)
    executor.add_node(bat)
    executor.add_node(motor_speed)

    while not is_stop:
        try:
            data, addr = udp_socket.recvfrom(999)
            rec_str = decrypt_controller_data(data)
            if len(rec_str) == 0:
                continue
            sig_list = rec_str.split('*')
            for sig in sig_list:
                if len(sig) == 0:
                    continue
                other_list = sig[1:].split(':')
                sig = sig[0]
                if sig == 'B':
                    odom_x = float(other_list[0])
                    odom_y = float(other_list[1])
                    odom_th = float(other_list[2])
                    odom_vth = float(other_list[3])
                    odom_vxy = float(other_list[4])
                    base_control.pubOdom(odom_x, odom_y, odom_th, odom_vth, odom_vxy)
                elif sig == 'D':
                    accel_x = float(other_list[0])
                    accel_y = float(other_list[1])
                    accel_z = float(other_list[2])
                    gyro_x = float(other_list[3])
                    gyro_y = float(other_list[4])
                    gyro_z = float(other_list[5])
                    imu.imu_update(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
                elif sig == 'E':
                    battery = float(other_list[0])
                    bat.bat_update(battery)
                elif sig == 'G':
                    motor_current_Speed = float(other_list[0])
                    motor_target_Speed = float(other_list[1])
                    motor_speed.motor_speed_update(motor_current_Speed, motor_target_Speed)
            executor.spin_once(timeout_sec=0.01)
        except Exception as err:
            pass

    base_control.destroy_node()
    imu.destroy_node()
    bat.destroy_node()
    motor_speed.destroy_node()
    rclpy.try_shutdown()


def video_client_process(robot_ip):
    try:
        argDic = get_param_from_json(robot_ip)
        rclpy.init()
        video = VIDEO(argDic.get('robot_id', ''), argDic.get('camera_ip', ''))
        rclpy.spin(video)
    except Exception as err:
        pass


def run(robot_ip, node_type='2', cam_quality='7', robot_param=''):
    global is_stop
    signal.signal(signal.SIGINT, signal_handler)
    socket.setdefaulttimeout(8)

    print("##########输入参数###################")
    print("机器人IP：%s" % robot_ip)
    print("图像质量：%s" % cam_quality)
    print("#####################################")
    print("关闭方法：一直按Ctrl+C，直到完全退出")
    print("#####################################")

    try:
        tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tmp_socket.bind(('0.0.0.0', control_data_dpt))
        tmp_socket.sendto(enrypt_str('#' + 'INFO_0000000000_000000' + '*'), (robot_ip, control_data_spt))
        data, addr = tmp_socket.recvfrom(999)
        rec_str = decrypt_controller_data(data)
        tmp_socket.close()
    except Exception as err:
        print("IP:%s %s Ctrl+C" % (robot_ip, str(err)))
        return

    sig_list = rec_str.split('*')
    for sig in sig_list:
        if len(sig) == 0:
            continue
        other_list = sig[1:].split(':')
        sig = sig[0]
        if sig == 'B':
            ip_sig_list = other_list
            camera_ip = ip_sig_list[0] if len(ip_sig_list) > 0 else '192.168.100.53'
            robot_id = ip_sig_list[1] if len(ip_sig_list) > 1 else ''

    save_param_to_json(robot_ip, str(lidar_data_spt), robot_id, cam_quality, camera_ip)

    if node_type == '1':
        pty_master, slave, pty_port = mkpty()
        p = Process(target=lidar_trans, args=(robot_ip, pty_master))
        p.start()
        udp_client_process(robot_ip, robot_id)
    elif node_type == '2':
        video_client_process(robot_ip)
    else:
        pty_master, slave, pty_port = mkpty()
        p = Process(target=lidar_trans, args=(robot_ip, pty_master))
        p.start()
        udp_client_process(robot_ip, robot_id)
