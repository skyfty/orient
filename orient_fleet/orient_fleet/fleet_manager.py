#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
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

import sys
import math
import yaml
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import RobotState, Location, PathRequest,  DockSummary, RobotMode

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry
from orient_interfaces.msg import Task

import numpy as np

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

import threading
app = FastAPI()


class Request(BaseModel):
    map_name: Optional[str] = None
    task: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str

# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_task_request = None
        self.last_completed_request = None

    def is_expected_task_id(self, task_id):
        if self.last_task_request is not None:
            if task_id != self.last_task_request.path_request.task_id:
                return False
        return True


class FleetManager(Node):
    def __init__(self, config):
        self.debug = False
        self.config = config
        self.fleet_name = self.config["rmf_fleet"]["name"]

        super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.docks = {}  # Map dock name to waypoints

        for robot_name, robot_config in self.config["robots"].items():
            self.robots[robot_name] = State()
        assert(len(self.robots) > 0)

        profile = traits.Profile(geometry.make_final_convex_circle(
            self.config['rmf_fleet']['profile']['footprint']),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        self.vehicle_traits.differential.reversible = self.config['rmf_fleet']['reversible']

        self.create_subscription(RobotState, 'robot_state', self.robot_state_cb, qos_profile=10)
        
        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)
        self.create_subscription( DockSummary, 'dock_summary', self.dock_summary_cb, qos_profile=transient_qos)
        self.task_pub = self.create_publisher(Task, 'robot_task_requests', qos_profile=qos_profile_system_default)

        @app.get('/open-rmf/rmf_orient_fm/status/', response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }
            if robot_name is None:
                response['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    response['data']['all_robots'].append(self.get_robot_state(state, robot_name))
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                response['data'] = self.get_robot_state(state, robot_name)
            response['success'] = True
            return response
        
        def make_task_request(robot_name: str, behavior:str, path_request: PathRequest):
            task = Task()
            task.clientid = robot_name
            task.behavior = behavior
            task.path_request = path_request            
            return task
        
        @app.post('/open-rmf/rmf_orient_fm/navigate/', response_model=Response)
        async def navigate(robot_name: str, cmd_id: int, dest: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or len(dest.destination) < 1):
                return response
            robot = self.robots[robot_name]

            target_loc = Location()
            target_loc.t = self.get_clock().now().to_msg()
            target_loc.x = dest.destination['x']
            target_loc.y = dest.destination['y']
            target_loc.yaw = dest.destination['yaw']
            target_loc.level_name =  dest.map_name
            if dest.speed_limit > 0:
                target_loc.obey_approach_speed_limit = True
                target_loc.approach_speed_limit = dest.speed_limit
                
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            task_request = make_task_request(robot_name, "follow_path.xml", path_request)
            self.task_pub.publish(task_request)

            robot.last_task_request = task_request
            robot.destination = target_loc
            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_orient_fm/start_action/', response_model=Response)
        async def start_action(robot_name: str, cmd_id: int, dest: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots):
                return response
            robot = self.robots[robot_name]
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(cmd_id)
            path_request.path = []
            target_loc = Location()
            target_loc.t = self.get_clock().now().to_msg()
            target_loc.x = dest.destination['x']
            target_loc.y = dest.destination['y']
            target_loc.yaw = dest.destination['yaw']
    
            path_request.path.append(target_loc)

            behavior = dest.task.endswith('.xml') and dest.task or f"{dest.task}.xml"
            self.get_logger().info(f"Starting action {behavior} for robot {robot_name} at {target_loc.x}, {target_loc.y}, {target_loc.yaw}")
            task_request = make_task_request(robot_name, behavior, path_request)
            self.task_pub.publish(task_request)

            robot.last_task_request = task_request
            robot.destination = target_loc
            response['success'] = True
            return response
        
        @app.get('/open-rmf/rmf_orient_fm/stop_robot/', response_model=Response)
        async def stop(robot_name: str, cmd_id: int):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            path_request = PathRequest()
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path = []
            path_request.path.append(robot.state.location)
            path_request.path.append(robot.state.location)
            path_request.task_id = str(cmd_id)
            task_request = make_task_request(robot_name, "follow_path.xml", path_request)
            self.task_pub.publish(task_request)
            
            robot.last_task_request = task_request
            robot.destination = None
            response['success'] = True
            return response

        @app.post('/open-rmf/rmf_orient_fm/start_task/', response_model=Response)
        async def start_task(robot_name: str, cmd_id: int, task: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or len(task.task) < 1 or  task.task not in self.docks):
                return response

            robot = self.robots[robot_name]
            path_request = PathRequest()
            target_loc = Location()
            for wp in self.docks[task.task]:
                target_loc = wp
                path_request.path.append(target_loc)

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.task_id = str(cmd_id)
            task_request = make_task_request(robot_name, "follow_path.xml", path_request)
            self.task_pub.publish(task_request)
            
            robot.last_task_request = task_request
            robot.destination = target_loc
            response['success'] = True
            return response


    def robot_state_cb(self, msg):
        if (msg.name in self.robots):
            robot = self.robots[msg.name]
            robot.state = msg
            if not robot.is_expected_task_id(msg.task_id):
                # This message is out of date, so disregard it.
                if robot.last_task_request is not None:
                    # Resend the latest task request for this robot, in case
                    # the message was dropped.
                    self.task_pub.publish(robot.last_task_request)
                return

            # Check if robot has reached destination
            if robot.destination is None:
                return
            if (( msg.mode.mode == RobotMode.MODE_IDLE or msg.mode.mode == RobotMode.MODE_CHARGING) and len(msg.path) == 0):
                robot = self.robots[msg.name]
                robot.destination = None
                completed_request = int(msg.task_id)
                robot.last_completed_request = completed_request

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, robot: State, robot_name):
        data = {}
        position = [robot.state.location.x, robot.state.location.y]
        angle = robot.state.location.yaw
        data['fleet_name'] = self.fleet_name
        data['robot_name'] = robot_name
        data['map_name'] = robot.state.location.level_name
        data['position'] = {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = robot.state.battery_percent
        if (robot.destination is not None and robot.last_task_request is not None):
            destination = robot.destination
            # calculate arrival estimate
            dist_to_target = self.disp(position, [destination.x, destination.y])
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (dist_to_target /
                        self.vehicle_traits.linear.nominal_velocity +
                        ori_delta /
                        self.vehicle_traits.rotational.nominal_velocity)
            cmd_id = int(robot.last_task_request.path_request.task_id)
            data['destination_arrival'] = {
                'cmd_id': cmd_id,
                'duration': duration
            }
        else:
            data['destination_arrival'] = None

        data['last_completed_request'] = robot.last_completed_request
        if (robot.state.mode.mode == RobotMode.MODE_WAITING or robot.state.mode.mode == RobotMode.MODE_ADAPTER_ERROR):
            data['replan'] = True
        else:
            data['replan'] = False

        return data

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(prog="fleet_adapter", description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True, help="Path to the config.yaml file")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet manager...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()
    fleet_manager_config = config['rmf_fleet']['fleet_manager']
    uvicorn.run(app, host=fleet_manager_config['ip'], port=fleet_manager_config['port'], log_level='warning')


if __name__ == '__main__':
    main(sys.argv)
