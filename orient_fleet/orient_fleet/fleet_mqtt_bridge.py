# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
import sys

from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
import paho.mqtt.client as mqtt
from rmf_fleet_msgs.msg import  RobotState
from orient_interfaces.msg import Task
from rmf_fleet_msgs.msg import DockSummary, ModeRequest

import yaml
from rclpy.qos import qos_profile_system_default

class FleetRobotManagerMQTTBridge(Node):
    def __init__(self, argv=sys.argv):
        super().__init__('orient_agent_mqtt')

        self.robot_state_topic = self.get_or_declare_parameter('robot_state_topic', 'robot_state').string_value
        self.robot_task_requests_topic = self.get_or_declare_parameter('robot_task_requests_topic', 'robot_task_requests').string_value
        self.robot_mode_requests_topic = self.get_or_declare_parameter('robot_mode_requests_topic', 'action_execution_notice').string_value

        self.broker_host = self.get_or_declare_parameter('broker.host', 'localhost').string_value
        self.broker_port = self.get_or_declare_parameter('broker.port', 1883).integer_value

        self.robot_state_publisher = self.create_publisher(RobotState,self.robot_state_topic,qos_profile=10)
        self.robot_task_request_subscriber = self.create_subscription(
            Task, self.robot_task_requests_topic, self.on_robot_task_request, qos_profile=qos_profile_system_default)
        self.robot_mode_requests_publisher = self.create_publisher(ModeRequest, self.robot_mode_requests_topic, qos_profile=qos_profile_system_default)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.get_logger().info(f"MQTT Broker Host: {self.broker_host}, Port: {self.broker_port}")
        try:
            # Connect to the MQTT broker
            self.mqtt_client.connect(self.broker_host, self.broker_port)
        except ConnectionRefusedError as e:
            print(f"MQTT connection to {self.broker_host} failed.")
            print("Please check that the MQTT server is running!")
            raise(e)
        self.mqtt_client.loop_start()
        
    def get_or_declare_parameter(self, name, default_value):
        # Check if parameter is already declared
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)
        return self.get_parameter(name).get_parameter_value()
    
    def on_robot_task_request(self, msg):
        # Serialize the Task message to bytes
        serialized_msg = serialize_message(msg)
        # Publish the serialized message to the MQTT broker
        self.mqtt_client.publish(self.robot_task_requests_topic, serialized_msg, qos=1)

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT Broker: {self.broker_host}:{self.broker_port}")
        self.mqtt_client.subscribe(self.robot_state_topic, qos=1)
        self.mqtt_client.subscribe(self.robot_mode_requests_topic, qos=1)

    def on_message(self, client, userdata, msg):
        if msg.topic == self.robot_state_topic:
            deserialized_msg = deserialize_message(msg.payload, RobotState)
            self.robot_state_publisher.publish(deserialized_msg)
        if msg.topic == self.robot_mode_requests_topic:
            deserialized_msg = deserialize_message(msg.payload, ModeRequest)
            self.robot_mode_requests_publisher.publish(deserialized_msg)

def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = FleetRobotManagerMQTTBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
