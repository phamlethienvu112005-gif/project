#!/usr/bin/env python3
import glob
import math
import serial
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class ESP32ROS2Driver(Node):
    def __init__(self):
        super().__init__('esp32_ros2_driver')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_diameter', 0.065) # Bánh 65mm
        self.declare_parameter('wheel_base', 0.473)     # Khoảng cách 2 bánh 473mm
        self.declare_parameter('counts_per_rev', 3960.0)

        self.port_param = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.d = self.get_parameter('wheel_diameter').value
        self.w = self.get_parameter('wheel_base').value
        self.cpr = self.get_parameter('counts_per_rev').value

        self.MAX_RPM = 110.0

        # Biến Odometry
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_ticks_l = None
        self.last_ticks_r = None
        self.last_time = self.get_clock().now()

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # Subscribers
        self.create_subscription(String, '/esp32_cmd', self.raw_cmd_cb, 10)
        self.create_subscription(Twist, '/diff_drive/cmd_vel', self.drive_cb, 10)

        # Mở Serial
        ports = glob.glob('/dev/ttyUSB*')
        selected_port = self.port_param if self.port_param in ports else (ports[0] if ports else self.port_param)

        try:
            self.ser = serial.Serial(selected_port, self.baudrate, timeout=0.1)
            self.ser.dtr = False
            self.ser.rts = False
            self.get_logger().info(f"Kết nối thành công ESP32: {selected_port}")
            
            # Luồng đọc Serial chạy song song
            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            self.get_logger().error(f"Không thể mở cổng Serial: {e}")

    def raw_cmd_cb(self, msg: String):
        cmd = msg.data.strip() + "\n"
        self.send_ser(cmd)

    def drive_cb(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z
        
        v_l = vx - (wz * self.w / 2.0)
        v_r = vx + (wz * self.w / 2.0)
        
        rpm_l = (v_l / (math.pi * self.d)) * 60.0
        rpm_r = (v_r / (math.pi * self.d)) * 60.0
        
        rpm_l = max(min(rpm_l, self.MAX_RPM), -self.MAX_RPM)
        rpm_r = max(min(rpm_r, self.MAX_RPM), -self.MAX_RPM)
        
        cmd = f"L{rpm_l:.1f}\nR{rpm_r:.1f}\n"
        self.send_ser(cmd)

    def send_ser(self, cmd_str: str):
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.write(cmd_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Lỗi gửi Serial: {e}")

    def read_serial_loop(self):
        """Đọc giải mã chuỗi PKT gửi từ ESP32"""
        while self.running and hasattr(self, 'ser') and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("PKT,"):
                    parts = line.split(",")
                    # Format: PKT, ticksL, ticksR, ticks3, gyro_z, timestamp
                    if len(parts) >= 6:
                        ticks_l = int(parts[1])
                        ticks_r = int(parts[2])
                        gyro_z = float(parts[4])
                        
                        self.process_sensor_data(ticks_l, ticks_r, gyro_z)
            except Exception:
                pass

    def process_sensor_data(self, ticks_l, ticks_r, gyro_z):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Khởi tạo tick lần đầu
        if self.last_ticks_l is None:
            self.last_ticks_l = ticks_l
            self.last_ticks_r = ticks_r
            return

        # Tính khoảng cách di chuyển từng bánh (mét)
        delta_l = ticks_l - self.last_ticks_l
        delta_r = ticks_r - self.last_ticks_r
        self.last_ticks_l = ticks_l
        self.last_ticks_r = ticks_r

        dist_l = (delta_l / self.cpr) * (math.pi * self.d)
        dist_r = (delta_r / self.cpr) * (math.pi * self.d)

        dist_c = (dist_r + dist_l) / 2.0
        d_th = (dist_r - dist_l) / self.w

        # Cập nhật vị trí Odometry
        self.x += dist_c * math.cos(self.th + d_th / 2.0)
        self.y += dist_c * math.sin(self.th + d_th / 2.0)
        self.th += d_th

        # Giới hạn góc quay trong khoảng [-pi, pi]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        v_x = dist_c / dt
        w_z = d_th / dt

        # 1. Publish Topic /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Tọa độ vị trí & Góc quay (2D Plane)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = euler_to_quaternion(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Hiệp phương sai Vị trí (Pose Covariance - 6x6 Matrix)
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.01   # Sai số X
        pose_cov[7] = 0.01   # Sai số Y
        pose_cov[14] = 1e-9  # Khóa Z 2D
        pose_cov[21] = 1e-9  # Khóa Roll
        pose_cov[28] = 1e-9  # Khóa Pitch
        pose_cov[35] = 0.05  # Sai số Yaw
        odom_msg.pose.covariance = pose_cov

        # Vận tốc tuyến tính & Vận tốc góc
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = w_z

        # Hiệp phương sai Vận tốc (Twist Covariance - 6x6 Matrix)
        twist_cov = [0.0] * 36
        twist_cov[0] = 0.02   # Sai số v_x
        twist_cov[7] = 1e-9   # Khóa v_y
        twist_cov[14] = 1e-9  # Khóa v_z
        twist_cov[21] = 1e-9  # Khóa w_x
        twist_cov[28] = 1e-9  # Khóa w_y
        twist_cov[35] = 0.05  # Sai số w_z
        odom_msg.twist.covariance = twist_cov

        self.pub_odom.publish(odom_msg)

        # 2. Publish Topic /imu/data
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Dữ liệu Vận tốc góc (Gyroscope Z)
        imu_msg.angular_velocity.z = gyro_z

        # Khai báo Covariance để EKF nhận diện chỉ dùng Gyro Z
        imu_msg.angular_velocity_covariance[8] = 0.02
        imu_msg.orientation_covariance[0] = -1.0         # Báo hiệu không có Orientation
        imu_msg.linear_acceleration_covariance[0] = -1.0 # Báo hiệu không có Accel

        self.pub_imu.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32ROS2Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
