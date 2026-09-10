#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelSplitter(Node):
    def __init__(self):
        super().__init__('cmd_vel_splitter')
        
        # Subscriber lắng nghe topic tổng /cmd_vel
        self.sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        
        # Publisher cho phần di chuyển (linear.x, linear.y, angular.z)
        self.pub_drive = self.create_publisher(Twist, '/diff_drive/cmd_vel', 10)
        
        # Publisher cho cơ cấu nâng (chỉ lấy linear.z)
        self.pub_lift = self.create_publisher(Twist, '/lift/cmd_vel', 10)
        
        self.get_logger().info("CmdVel Splitter Node đã sẵn sàng!")

    def cmd_vel_callback(self, msg: Twist):
        # 1. Lọc dữ liệu di chuyển cho di động
        drive_msg = Twist()
        drive_msg.linear.x = msg.linear.x
        drive_msg.linear.y = msg.linear.y
        drive_msg.angular.z = msg.angular.z
        self.pub_drive.publish(drive_msg)

        # 2. Lọc dữ liệu nâng/hạ cho Motor 3 (Lift)
        lift_msg = Twist()
        lift_msg.linear.z = msg.linear.z
        self.pub_lift.publish(lift_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
