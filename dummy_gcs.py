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

"""dummy_gcs.py"""

__authors__ = 'Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2025 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'


import argparse
import threading
import time
from as2_msgs.msg import MissionUpdate
from as2_python_api.mission_interpreter.mission import Mission, MissionItem, InterpreterStatus
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_system_default
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
import rclpy


class AreaScanMission(Mission):
    """Area Scan mission using go_to waypoints"""

    def __init__(self, target, area_corners, altitude, speed):
        super().__init__(target=target)

        takeoff_item = MissionItem(behavior='takeoff',
                                   args={'height': altitude, 'speed': speed})
        self.plan.append(takeoff_item)

        for corner in area_corners:
            go_to_item = MissionItem(behavior='go_to', method='go_to_point_path_facing',
                                     args={'point': corner + [altitude], 'speed': speed})
            self.plan.append(go_to_item)

        go_to_item = MissionItem(behavior='go_to', method='go_to_point_path_facing',
                                 args={'point': [0.0, 0.0, altitude], 'speed': speed,
                                       'frame_id': f'{target}/map'})
        self.plan.append(go_to_item)

        land_item = MissionItem(behavior='land', args={'speed': speed})
        self.plan.append(land_item)


class AreaScanFollowPathMission(Mission):
    """Area Scan mission using follow_path behavior"""

    def __init__(self, target, area_corners, altitude, speed):
        super().__init__(target=target)

        takeoff_item = MissionItem(behavior='takeoff',
                                   args={'height': altitude, 'speed': speed})
        self.plan.append(takeoff_item)

        path = []
        for corner in area_corners:
            path.append([corner[0], corner[1], altitude])

        follow_path_item = MissionItem(behavior='follow_path', method='follow_path_with_path_facing',
                                       args={'path': path, 'speed': speed})
        self.plan.append(follow_path_item)

        land_item = MissionItem(behavior='land', args={'speed': speed})
        self.plan.append(land_item)


class TakeSampleMission(Mission):
    """Take Sample mission, go down and up"""

    def __init__(self, target, altitude, speed):
        super().__init__(target=target)

        go_to_item = MissionItem(behavior='go_to', method='go_to_point',
                                 args={'point': [0.0, 0.0, -altitude], 'speed': speed,
                                       'frame_id': f'{target}/base_link'})
        self.plan.append(go_to_item)

        go_to_item = MissionItem(behavior='go_to', method='go_to_point',
                                 args={'point': [0.0, 0.0, altitude], 'speed': speed,
                                       'frame_id': f'{target}/base_link'})
        self.plan.append(go_to_item)


class BoatMission(Mission):
    """Boat mission, go to a point in boat frame"""

    def __init__(self, target, altitude, speed):
        super().__init__(target=target)

        go_to_item = MissionItem(behavior='go_to', method='go_to_point',
                                 args={'point': [0.0, 0.0, altitude], 'speed': speed,
                                       'frame_id': 'boat'})
        self.plan.append(go_to_item)


class DummyGCS(Node):
    """Dummy Ground Control Station node"""

    def __init__(self, use_sim_time=False, drone_target='drone0'):
        super().__init__('dummy_gcs')

        self.drone_target = drone_target
        self.param_use_sim_time = Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([self.param_use_sim_time])

        self.__resume_client = self.create_client(
            Trigger, f'/{drone_target}/FollowPathBehavior/_behavior/resume')

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                         depth=10)

        self.uplink = self.create_publisher(
            MissionUpdate, f'/{drone_target}/mission_update', qos_profile_system_default)

        self.status = None
        self.downlink = self.create_subscription(
            String, f'/{drone_target}/mission_status', self.downlink_callback, qos)

        self.get_logger().info("Dummy GCS ready")

    def publish_mission(self):
        """Load all three missions to the drone"""
        # scan_mission = AreaScanFollowPathMission(
        #     target=self.drone_target,
        #     area_corners=[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
        #     altitude=2.0, speed=1.0)
        scan_mission = AreaScanMission(
            target=self.drone_target,
            area_corners=[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
            altitude=2.0, speed=1.0)

        msg = MissionUpdate(drone_id=self.drone_target, mission_id=0, action=MissionUpdate.LOAD,
                            mission=scan_mission.json())
        self.get_logger().info(f"Publishing Area Scan mission")
        self.get_logger().debug(f"{scan_mission.json()}")
        self.uplink.publish(msg)

        time.sleep(0.5)
        take_sample_mission = TakeSampleMission(target=self.drone_target, altitude=1.0, speed=0.5)
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=1, action=MissionUpdate.LOAD,
                            mission=take_sample_mission.json())
        self.get_logger().info(f"Publishing Take Sample mission")
        self.get_logger().debug(f"{take_sample_mission.json()}")
        self.uplink.publish(msg)

        time.sleep(0.5)
        boat_mission = BoatMission(target=self.drone_target, altitude=1.0, speed=0.5)
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=2, action=MissionUpdate.LOAD,
                            mission=boat_mission.json())
        self.get_logger().info(f"Publishing Boat mission")
        self.get_logger().debug(f"{boat_mission.json()}")
        self.uplink.publish(msg)

    def publish_cmd(self, cmd: int):
        """Publish a command to the mission with mission_id=0"""
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=0, action=cmd)
        self.uplink.publish(msg)

    def perform_take_sample_mission(self):
        """Stop current mission and start take sample mission and resume afterwards"""
        keep_item_id = self.status.done_items - 1
        self.get_logger().info(f"Stop scan mission, {keep_item_id}")
        self.publish_cmd(MissionUpdate.STOP)

        time.sleep(0.5)
        self.get_logger().info("Start take sample mission")
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=1, action=MissionUpdate.START)
        self.uplink.publish(msg)

        time.sleep(0.5)
        while self.status.current_item != None:
            time.sleep(0.1)

        self.get_logger().info(f"Resume scan mission, {keep_item_id}")
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=0,
                            item_id=keep_item_id, action=MissionUpdate.START)
        self.uplink.publish(msg)

    def perform_take_sample_mission_with_follow_path(self):
        """NOT WORKING. Stop current mission and start take sample mission and resume afterwards"""
        self.get_logger().info(f"Pause scan mission")
        self.publish_cmd(MissionUpdate.PAUSE)
        self.publish_cmd(MissionUpdate.STOP)  # esto mata el behavior follow_path, goal canceled

        time.sleep(0.5)
        self.get_logger().info("Start take sample mission")
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=1, action=MissionUpdate.START)
        self.uplink.publish(msg)

        time.sleep(0.5)
        while self.status.current_item != None:
            time.sleep(0.1)

        self.get_logger().info("Resume scan mission")
        self.publish_cmd(MissionUpdate.RESUME)
        response = self.__resume_client.call(Trigger.Request())
        print(f"{response=}")

    def perform_boat_mission(self):
        """Stop current mission and start boat mission and resume afterwards"""
        keep_item_id = self.status.done_items - 2  # Go back 1 waypoints
        self.get_logger().info(f"Stop scan mission, {keep_item_id}")
        self.publish_cmd(MissionUpdate.STOP)

        time.sleep(0.5)
        self.get_logger().info("Start boat mission")
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=2, action=MissionUpdate.START)
        self.uplink.publish(msg)

        time.sleep(0.5)
        while self.status.current_item != None:
            time.sleep(0.1)

        self.get_logger().info(f"Resume scan mission, {keep_item_id}")
        msg = MissionUpdate(drone_id=self.drone_target, mission_id=0,
                            item_id=keep_item_id, action=MissionUpdate.START)
        self.uplink.publish(msg)

    def downlink_callback(self, msg: String):
        """New mission status update"""
        self.status = InterpreterStatus.parse_raw(msg.data)
        self.get_logger().info(str(self.status))


def main():
    """Main function, node spin"""
    argument_parser = parser.parse_args()
    use_sim_time = argument_parser.use_sim_time
    print(f"{use_sim_time=}")

    rclpy.init()

    gcs = DummyGCS(use_sim_time=use_sim_time, drone_target=argument_parser.n)
    msg_input = """
    l:      load mission
    s:      start mission
    p:      pause mission
    r:      resume mission
    a:      abort mission
    c:      send take sample mission
    b:      send boat mission
    q:      quit
    \n"""

    while rclpy.ok():
        def input_thread(node: DummyGCS):
            while rclpy.ok():
                user_input = input(msg_input)
                if user_input.lower() == "q":
                    rclpy.shutdown()
                    break
                if user_input.lower() == "l":
                    node.publish_mission()
                elif user_input.lower() == "s":
                    node.publish_cmd(MissionUpdate.START)
                elif user_input.lower() == "n":
                    node.publish_cmd(MissionUpdate.STOP)
                elif user_input.lower() == "p":
                    node.publish_cmd(MissionUpdate.PAUSE)
                elif user_input.lower() == "r":
                    node.publish_cmd(MissionUpdate.RESUME)
                elif user_input.lower() == "a":
                    node.publish_cmd(MissionUpdate.STOP)
                elif user_input.lower() == "c":
                    node.perform_take_sample_mission()
                    # node.perform_take_sample_mission_with_follow_path()
                elif user_input.lower() == "b":
                    node.perform_boat_mission()

        thread = threading.Thread(target=input_thread, args=(gcs,))
        thread.start()

        rclpy.spin(gcs)

    gcs.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=str, default='drone0', help='Namespace')
    parser.add_argument('--use_sim_time', action='store_true', help='Use sim time')

    main()
