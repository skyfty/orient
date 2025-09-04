import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import os

class UdpCmdvelNode(Node):
    def __init__(self):
        super().__init__('udp_cmdvel_node')
        self.declare_parameter('client_ip', os.getenv('ORIENT_HOSTIP', ''))
        self.declare_parameter('angular_x', 0.5)
        self.declare_parameter('steering', True)

        self.udp_ip = self.get_parameter('client_ip').get_parameter_value().string_value
        if not self.udp_ip:
            self.get_logger().error("未设置客户端IP地址，请通过环境变量 ORIENT_HOSTIP 设置")
            raise ValueError("客户端IP地址未设置")
        
        self.angular_x = self.get_parameter('angular_x').get_parameter_value().double_value
        self.udp_port = 1234
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.steering = self.get_parameter('steering').get_parameter_value().bool_value

        self.get_logger().info(f"UDP命令速度节点已启动 angular_x:{self.angular_x} steering:{self.steering}")
        # 订阅 /cmd_vel 话题
        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.velocity_callback, 
            10
        )
        self.subscription  # 防止被 Python 的垃圾回收机制回收

        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel2', 
            self.velocity_callback2, 
            10
        )
        self.subscription  # 防止被 Python 的垃圾回收机制回收


    def velocity_callback(self, msg):
        """处理 /cmd_vel 话题的回调函数"""
        if self.steering:
            message = f"linear_x: {msg.linear.x},angular_x: {msg.angular.x},angular_z: {msg.angular.z}"
        else:
            message = f"linear_x: {msg.linear.x},angular_x: 0.0,angular_z: {msg.angular.z}"

        self.sock.sendto(message.encode(), (self.udp_ip, self.udp_port))
        self.get_logger().info(f"发送UDP消息: {message} 到 {self.udp_ip}:{self.udp_port}")

    def velocity_callback2(self, msg):
        msg.angular.x = msg.angular.z
        msg.angular.z = self.angular_x
        self.velocity_callback(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UdpCmdvelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()