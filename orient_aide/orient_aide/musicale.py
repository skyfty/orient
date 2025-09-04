#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from orient_interfaces.msg import CollisionAction
from nav_msgs.msg import Odometry
import os

class MusicaleNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.collision_action_type = None
        self.decimal_value = None
        self.get_logger().info("MusicaleNode initialized and ready.")

        # 前进速度阈值配置
        self.declare_parameter('speed_threshold', 0.2)
        self.speed_threshold = self.get_parameter('speed_threshold').get_parameter_value().double_value

        self.declare_parameter('enabled', True)
        self.enable = self.get_parameter('enabled').get_parameter_value().bool_value

        self.subscription = self.create_subscription(CollisionAction,'collision/action', self.collision_action_callback,1)
        self.pose_subscription = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odometry_callback,
            1
        )

        # 检查所有 GPIO 路径是否存在
        self.gpio_done = True
        self.gpio_paths = [
            "/sys/class/gpio/gpio843/value",
            "/sys/class/gpio/gpio844/value",
            "/sys/class/gpio/gpio845/value",
            "/sys/class/gpio/gpio846/value"
        ]
        for path in self.gpio_paths:
            if not os.path.exists(path):
                self.gpio_done = False
                self.get_logger().error(f"{path} does not exist.")
                break
        if self.gpio_done:
            self.get_logger().info("All GPIO paths are valid.")
        

    def odometry_callback(self, msg):
        if self.collision_action_type is not None:
            return
        # 如果没有碰撞动作，则根据 Odometry 的线速度来设置扬声器状态
        # 判读 Odometry 是否在转向、倒车还是前进
        if abs(msg.twist.twist.linear.x) > self.speed_threshold:
            if msg.twist.twist.linear.x > 0.0:
                # 前进
                self.set_speaker_state(1)
            else:
                # 倒车
                self.set_speaker_state(2)
        else:
            # 停止
            self.set_speaker_state(0)

    
    def collision_action_callback(self, msg):
        self.collision_action_type = msg.action_type
        if msg.action_type == CollisionAction.ACTION_STOP:
            self.set_speaker_state(3)
        elif msg.action_type == CollisionAction.ACTION_SLOWDOWN:
            self.set_speaker_state(2)
        elif msg.action_type == CollisionAction.ACTION_APPROACH:
            self.set_speaker_state(3)
        else:
            self.collision_action_type = None
            self.set_speaker_state(0)


    def set_speaker_state(self, decimal_value):
        if not self.gpio_done or not self.enable or self.decimal_value == decimal_value:
            return
        self.decimal_value = decimal_value
        # 将十进制值转换为二进制字符串，并确保为 4 位（不足时前面补零）
        binary_value = format(decimal_value, '04b') 

        speaker_io1 = '1' if binary_value[3] == '0' else '0'
        speaker_io2 = '1' if binary_value[2] == '0' else '0'
        speaker_io3 = '1' if binary_value[1] == '0' else '0'
        speaker_io4 = '1' if binary_value[0] == '0' else '0'
        
        with open("/sys/class/gpio/gpio843/value", "w") as f:
            f.write(speaker_io1)
        with open("/sys/class/gpio/gpio844/value", "w") as f:
            f.write(speaker_io2)
        with open("/sys/class/gpio/gpio845/value", "w") as f:
            f.write(speaker_io3)
        with open("/sys/class/gpio/gpio846/value", "w") as f:
            f.write(speaker_io4)
    

def main(args=None):
    rclpy.init(args=args)
    node = MusicaleNode("musicale_node")
    rclpy.spin(node) 
    rclpy.shutdown() 