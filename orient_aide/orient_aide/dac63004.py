#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class Dac63004Node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Dac63004Node initialized and ready.")
    
        self.cmd_subscription = self.create_subscription(
            String,
            'dac63004/cmd',
            self.cmd_callback,
            1
        )

    def cmd_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        # 根据接收到的命令执行相应的操作
        self.exec_command(msg.data)
        
        if msg.data == "dac63004App-down":
            self.set_gpio_value(826, 1)
            self.set_gpio_value(843, 1)
            self.set_gpio_value(827, 1)
            self.get_logger().info("Executed dac63004App-down command.")
        elif msg.data == "dac63004App-up":
            self.set_gpio_value(827, 1)
            self.set_gpio_value(826, 0)
            self.set_gpio_value(843, 0)
            self.get_logger().info("Executed dac63004App-up command.")
        else:
            self.get_logger().warn(f"Unknown command: {msg.data}")

    def exec_command(self, command):
        pid = os.fork()
        if pid == 0:
            os.execl(command, command)
            os._exit(0)
        elif pid < 0:
            self.get_logger().error(f"Failed to fork process for command: {command}")
        else:
            os.waitpid(pid, 0)
            self.get_logger().info(f"Executed command: {command}")

    def set_gpio_value(self, gpio_number, value):
        gpio_path = f"/sys/class/gpio/gpio{gpio_number}/value"
        if not os.path.exists(gpio_path):
            self.get_logger().error(f"GPIO path {gpio_path} does not exist.")
            return
        try:
            with open(gpio_path, 'w') as f:
                f.write(str(value))
            self.get_logger().info(f"Set GPIO {gpio_number} to {value}.")
        except Exception as e:
            self.get_logger().error(f"Failed to set GPIO {gpio_number}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Dac63004Node("dac63004_node")
    rclpy.spin(node) 
    rclpy.shutdown() 