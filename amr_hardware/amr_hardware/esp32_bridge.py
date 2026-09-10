#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial, threading, math, time, glob
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_diameter', 0.13)
        self.declare_parameter('wheelbase', 0.30)
        self.declare_parameter('counts_per_rev', 3960.0)
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_d = self.get_parameter('wheel_diameter').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.cpr = self.get_parameter('counts_per_rev').value
        self.wheel_circ = math.pi * self.wheel_d
        self.ser = None
        self.connect_serial()
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_raw', 10)
        self.sub_drive = self.create_subscription(Twist, '/diff_drive/cmd_vel', self.drive_cb, 10)
        self.sub_lift = self.create_subscription(Twist, '/lift/cmd_vel', self.lift_cb, 10)
        self.sub_raw = self.create_subscription(String, '/esp32_cmd', self.raw_cb, 10)
        self.tf = tf2_ros.TransformBroadcaster(self)
        self.running = True
        self.x = self.y = self.theta = 0.0
        self.last_el = self.last_er = 0
        threading.Thread(target=self.read_loop, daemon=True).start()
        self.get_logger().info('ESP32 Bridge started')

    def connect_serial(self):
        try:
            if self.port == '/dev/ttyUSB0':
                ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
                if ports: self.port = ports[0]
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2)
        except Exception as e:
            self.get_logger().error(f'Serial: {e}')

    def drive_cb(self, msg: Twist):
        if not self.ser or not self.ser.is_open: return
        vx, wz = msg.linear.x, msg.angular.z
        vl = vx - wz * self.wheelbase / 2.0
        vr = vx + wz * self.wheelbase / 2.0
        self.ser.write(f'L{vl/self.wheel_circ*60:.1f}\nR{vr/self.wheel_circ*60:.1f}\n'.encode())

    def lift_cb(self, msg: Twist):
        if not self.ser or not self.ser.is_open: return
        if abs(msg.angular.z) > 0.01:
            self.ser.write(f'M{msg.angular.z:.1f}\n'.encode())
        elif abs(msg.linear.z) > 0.001:
            self.ser.write(f'H{msg.linear.z*100:.1f}\n'.encode())

    def raw_cb(self, msg: String):
        if self.ser and self.ser.is_open:
            self.ser.write((msg.data.strip() + '\n').encode())

    def read_loop(self):
        buf = ""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        buf += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        while '\n' in buf:
                            line, buf = buf.split('\n', 1)
                            self.process(line.strip())
                except: pass
            time.sleep(0.001)

    def process(self, line):
        if line.startswith('PKT'):
            p = line.split(',')
            if len(p) < 6: return
            try:
                el, er, em3, gz = int(p[1]), int(p[2]), int(p[3]), float(p[4])
                now = self.get_clock().now().to_msg()
                imu = Imu()
                imu.header.stamp = now
                imu.header.frame_id = 'imu_link'
                imu.angular_velocity.z = gz
                self.pub_imu.publish(imu)
                js = JointState()
                js.header.stamp = now
                js.name = ['left_wheel_joint', 'right_wheel_joint', 'lift_screw_joint']
                js.position = [(el/self.cpr)*2*math.pi, (er/self.cpr)*2*math.pi, em3/(self.cpr/0.004)]
                self.pub_js.publish(js)
                dl = (el-self.last_el)/self.cpr*self.wheel_circ
                dr = (er-self.last_er)/self.cpr*self.wheel_circ
                dc, dt = (dl+dr)/2.0, (dr-dl)/self.wheelbase
                self.x += dc*math.cos(self.theta+dt/2)
                self.y += dc*math.sin(self.theta+dt/2)
                self.theta = (self.theta+dt) % (2*math.pi)
                if self.theta > math.pi: self.theta -= 2*math.pi
                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_footprint'
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation.z = math.sin(self.theta/2)
                odom.pose.pose.orientation.w = math.cos(self.theta/2)
                self.pub_odom.publish(odom)
                self.last_el, self.last_er = el, er
            except: pass

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.write(b'L0\nR0\nM0\n')
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
