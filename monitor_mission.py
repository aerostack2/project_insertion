#!/usr/bin/env python3

# Copyright 2025 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""monitor_mission.py"""

__authors__ = 'Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import argparse
import random
import time
import yaml

from as2_msgs.msg import MissionUpdate, PlatformInfo, PlatformStatus
from as2_python_api.mission_interpreter.mission import InterpreterState, InterpreterStatus
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    qos_profile_sensor_data,
    qos_profile_system_default,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


def platform_status_to_str(status: PlatformStatus) -> str:
    """Convert PlatformStatus to string"""
    if status == PlatformStatus.DISARMED:
        return 'DISARMED'
    elif status == PlatformStatus.LANDED:
        return 'LANDED'
    elif status == PlatformStatus.TAKING_OFF:
        return 'TAKING_OFF'
    elif status == PlatformStatus.FLYING:
        return 'FLYING'
    elif status == PlatformStatus.LANDING:
        return 'LANDING'
    elif status == PlatformStatus.EMERGENCY:
        return 'EMERGENCY'
    else:
        return 'UNKNOWN'


class InsertionMonitor(Node):
    """ROS 2 Node for monitoring INSERTION missions."""

    def __init__(self, drone_target: str, timer_max_wait_time: int,
                 timer_min_wait_time: int, max_samples: int, use_sim_time=False):
        super().__init__('monitor')

        self.param_use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.drone_target = drone_target
        self.timer_max_wait_time = timer_max_wait_time  # seconds
        self.timer_min_wait_time = timer_min_wait_time  # seconds
        self.max_samples = max_samples

        self.last_status: InterpreterStatus = None

        # MISSION STATUS
        self.connected: bool = False
        self.platform_status: PlatformStatus = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10
        )

        status_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )

        self.mission_status_sub = self.create_subscription(
            String,
            f'/{drone_target}/mission_status',
            self.mission_status_callback,
            status_qos_profile,
        )
        self.mission_update_pub = self.create_publisher(
            MissionUpdate, f'/{drone_target}/mission_update', qos_profile_system_default
        )

        self.platform_info_sub = self.create_subscription(
            PlatformInfo,
            '/' + drone_target + '/platform/info',
            self.platform_info_callback,
            qos_profile_system_default,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            f'/{drone_target}/sensor_measurements/gps',
            self.gps_callback,
            qos_profile_sensor_data,
        )

        timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=timer_cb_group)
        self.timer_cont = 0
        self.timer_goal = random.randint(self.timer_min_wait_time, self.timer_max_wait_time)
        self.sample_cont = 0

        self.is_idle = False

        self.get_logger().info('Monitor initialized')

    def platform_info_callback(self, msg: PlatformInfo):
        """Platform info callback"""
        self.connected = msg.connected
        if msg.status.state != self.platform_status:
            self.get_logger().info(f'Platform Status: {platform_status_to_str(msg.status.state)}')
        self.platform_status = msg.status.state

    def gps_callback(self, msg: NavSatFix):
        """New GPS message from drone"""
        self.drone_alt = msg.altitude

    def mission_status_callback(self, msg: String):
        """New mission status update"""
        status = InterpreterStatus.parse_raw(msg.data)
        # self.get_logger().info(str(status))

        self.is_idle = status.state == InterpreterState.IDLE

        if self.last_status != status or self.last_status is None:
            self.get_logger().info(f'Interpreter Status: {status}')

        self.last_status = status

    def publish_cmd(self, cmd: int, mission_id: int = 0):
        """Publish a mission command"""
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=mission_id, action=cmd)
        self.mission_update_pub.publish(msg)

    def perform_take_sample_mission(self):
        """Perform take sample mission"""
        keep_item_id = self.last_status.done_items - 1
        self.get_logger().info(f'Stop scan mission, {keep_item_id}')
        self.publish_cmd(MissionUpdate.STOP)

        while not self.is_idle:
            time.sleep(0.1)

        time.sleep(0.5)
        self.get_logger().info('Start take sample mission')
        self.publish_cmd(MissionUpdate.START, mission_id=1)

        time.sleep(0.5)
        while self.last_status.current_item != None:
            time.sleep(0.1)

        self.get_logger().info(f'Resume scan mission, {keep_item_id}')
        msg = MissionUpdate(
            drone_id=self.drone_target,
            mission_id=0,
            item_id=keep_item_id,
            action=MissionUpdate.START,
        )
        self.mission_update_pub.publish(msg)

    def perform_boat_mission(self):
        """Perform boat mission"""
        keep_item_id = self.last_status.done_items - 2  # Go back 1 waypoints
        self.get_logger().info(f'Stop scan mission, {keep_item_id}')
        self.publish_cmd(MissionUpdate.STOP)

        while not self.is_idle:
            time.sleep(0.1)

        time.sleep(0.5)
        self.get_logger().info('Start boat mission')
        self.publish_cmd(MissionUpdate.START, mission_id=2)

        time.sleep(0.5)
        while self.last_status.current_item != None:
            time.sleep(0.1)

        self.get_logger().info(f'Resume scan mission, {keep_item_id}')
        msg = MissionUpdate(
            drone_id=self.drone_target,
            mission_id=0,
            item_id=keep_item_id,
            action=MissionUpdate.START,
        )
        self.mission_update_pub.publish(msg)

    def timer_callback(self):
        """Timer callback"""
        if self.platform_status != PlatformStatus.FLYING:
            return

        self.timer_cont += 1
        if self.timer_cont >= self.timer_goal:
            self.get_logger().info(f'Timer goal reached: {self.timer_goal}')
            self.timer_cont = 0
            self.timer_goal = random.randint(self.timer_min_wait_time, self.timer_max_wait_time)

            if (
                self.last_status is not None
                and self.last_status.state == InterpreterState.RUNNING
                and (self.last_status.current_item.behavior == 'go_to' or
                     self.last_status.current_item.behavior == 'go_to_gps')
            ):
                self.perform_take_sample_mission()
                self.sample_cont += 1
                if self.sample_cont >= self.max_samples:
                    self.get_logger().info('Max samples reached, starting boat mission')
                    self.perform_boat_mission()
                    self.sample_cont = 0


def main():
    """Node spin"""
    argument_parser = parser.parse_args()
    use_sim_time = argument_parser.use_sim_time
    print(f'{use_sim_time=}')

    with open(argument_parser.config, 'r', encoding='utf-8') as file:
        yaml_data = yaml.safe_load(file)
        print(yaml_data)

    rclpy.init()

    node = InsertionMonitor(
        drone_target=argument_parser.n, timer_max_wait_time=yaml_data['timer_max_wait_time'],
        timer_min_wait_time=yaml_data['timer_min_wait_time'], max_samples=yaml_data['max_samples'],
        use_sim_time=argument_parser.use_sim_time
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=str, default='drone0', help='Namespace')
    parser.add_argument('--use_sim_time', action='store_true', help='Use sim time')
    parser.add_argument('--config', type=str, default='config/monitor.yaml',
                        help='Monitor config file')
    main()
