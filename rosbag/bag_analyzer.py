"""
bag_analyzer.py
"""

from dataclasses import dataclass, field
from math import sqrt
from pathlib import Path
from matplotlib.lines import Line2D
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import json
from itertools import groupby
import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.time import Duration
from tf2_ros.buffer import Buffer
from as2_msgs.msg import PlatformInfo, MissionUpdate, BehaviorStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from std_msgs.msg import Header, String
from tf2_msgs.msg import TFMessage
import tf2_geometry_msgs
from as2_python_api.mission_interpreter.mission import InterpreterStatus, InterpreterState, \
    MissionItem

from bag_reader import read_rosbag, deserialize_msgs, deserialize_tfs


def timestamp_to_float(header: Header) -> float:
    """Parse timestamp from header and convert float"""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def int_to_interpreter_state(state: int) -> InterpreterState:
    """Convert int to InterpreterState"""
    if state == BehaviorStatus.IDLE:
        return InterpreterState.IDLE
    elif state == BehaviorStatus.RUNNING:
        return InterpreterState.RUNNING
    elif state == BehaviorStatus.PAUSED:
        return InterpreterState.PAUSED
    else:
        raise ValueError(f"Unknown state: {state}")


@dataclass
class LogData:
    """Data read from rosbag file"""
    filename: Path
    poses: dict[str, list[PoseStamped]] = field(default_factory=dict)
    twists: dict[str, list[TwistStamped]] = field(default_factory=dict)
    mission_status: dict[str, list[InterpreterStatus]] = field(default_factory=dict)
    mission_update: dict[str, list[MissionUpdate]] = field(default_factory=dict)
    platform_info: dict[str, list[PlatformInfo]] = field(default_factory=dict)

    @classmethod
    def from_rosbag(cls, rosbag: Path) -> 'LogData':
        """Read the rosbag"""
        log_data = cls(rosbag)
        rosbag_msgs = read_rosbag(str(rosbag))

        # buffer = Buffer(cache_time=Duration(seconds=1200))
        # print('Processing /tf_static...')
        # buffer = deserialize_tfs(rosbag_msgs['/tf_static'], buffer)
        # print('Processing /tf...')
        # buffer = deserialize_tfs(rosbag_msgs['/tf'], buffer)

        # tf_static: dict[str, TransformStamped] = {}
        # tfs = deserialize_msgs(rosbag_msgs['/tf_static'], TFMessage)
        # for tf in tfs:
        #     for transform in tf.transforms:
        #         k = f'{transform.header.frame_id}_{transform.child_frame_id}'
        #         tf_static[k] = transform

        for topic, msgs in rosbag_msgs.items():
            if "self_localization/pose" in topic:
                drone_id = topic.split("/")[1]
                log_data.poses[drone_id] = deserialize_msgs(msgs, PoseStamped)
            elif "self_localization/twist" in topic:
                drone_id = topic.split("/")[1]
                log_data.twists[drone_id] = deserialize_msgs(msgs, TwistStamped)
            elif "/tf" == topic:
                continue
            elif '/tf_static' == topic:
                continue
            elif 'mission_status' in topic:
                status_str_list: list[tuple[String, int]] = deserialize_msgs(
                    msgs, String, keep_stamp=True)
                drone_id = topic.split("/")[1]
                log_data.mission_status[drone_id] = []
                for status_str, ts in status_str_list:
                    data = json.loads(status_str.data)
                    data['state'] = int_to_interpreter_state(data['state'])
                    status = InterpreterStatus(**data)

                    if status.current_item is not None:
                        if status.current_item.method == 'land':
                            status.current_item.behavior = 'land'
                    # be careful, this is a list of tuples
                    log_data.mission_status[drone_id].append((status, ts))
            elif 'mission_update' in topic:
                update_list: list[tuple[String, int]] = deserialize_msgs(
                    msgs, MissionUpdate, keep_stamp=True)
                drone_id = topic.split("/")[1]
                log_data.mission_update[drone_id] = []
                for update, ts in update_list:
                    log_data.mission_update[drone_id].append((update, ts))
            elif 'platform_info' in topic:
                drone_id = topic.split("/")[1]
                log_data.platform_info[drone_id] = deserialize_msgs(msgs, PlatformInfo)
            else:
                print(f"Ignored topic: {topic}")
                continue
            print(f'Processed {topic}')

        return log_data

    def __str__(self):
        """Print stats"""
        text = f"{self.filename.stem}\n"
        return text


def plot_path(data: LogData):
    """Plot paths"""
    fig, ax = plt.subplots()
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        # https://stackoverflow.com/questions/52773215
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        ax.plot(x, y, label=drone)

    ax.set_title(f'Path {data.filename.stem}')
    ax.set_xlabel('y (m)')
    ax.set_ylabel('x (m)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/path_{data.filename.stem}.png")
    return fig


def plot_z(data: LogData,
           drone_behavior_ranges: dict[str, dict[str, list[tuple[int, int]]]],
           color_map: dict[str, str]):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    elem = []
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        t00 = poses[0].header.stamp.sec
        behavior_ranges = drone_behavior_ranges[drone]
        for behavior, ranges in behavior_ranges.items():
            for start, end in ranges:
                print(poses[0].header.stamp.sec - t00)
                print(start - t00)
                print(poses[-1].header.stamp.sec - t00)
                print(end - t00)
                z = [pose.pose.position.z for pose in poses if start <= pose.header.stamp.sec <= end]
                ts = [pose.header.stamp.sec - t00 for pose in poses if start <=
                      pose.header.stamp.sec <= end]
                ax.plot(ts, z, label=behavior, color=color_map[behavior])
            elem.append(Line2D([], [], color=color_map[behavior], label=behavior))

    seen = set()
    unique = [obj for obj in elem if obj.get_label() not in seen and not seen.add(obj.get_label())]

    # ax.set_title(f'X {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('z (m)')
    ax.legend(handles=unique)
    ax.grid()
    fig.savefig(f"/tmp/z_{data.filename.stem}.png")
    return fig


def plot_twist(data: LogData):
    """Plot twists"""
    fig, ax = plt.subplots()
    for drone, twists in zip(data.twists.keys(), data.twists.values()):
        sp = [sqrt(twist.twist.linear.x**2 + twist.twist.linear.y ** 2 + twist.twist.angular.z**2)
              for twist in twists]
        ts = [timestamp_to_float(twist.header) - timestamp_to_float(twists[0].header)
              for twist in twists]
        ax.plot(ts, sp, label=drone)

    ax.set_title(f'Twists {data.filename.stem}')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('twist (m/s)')
    ax.legend()
    ax.grid()
    fig.savefig(f"/tmp/twist_{data.filename.stem}.png")
    return fig


def plot_broken_barh_mission_status(data: LogData, color_map: dict[str, str],
                                    y_pose_map: dict[str, float],
                                    behaviors_allowed: list[str] = None):
    """Plot mission status"""
    fig, ax = plt.subplots()
    patches = []
    for drone, status_with_ts in zip(data.mission_status.keys(), data.mission_status.values()):
        test = [key for key, group in groupby(status_with_ts, lambda x: x[0].current_item)]
        t00 = status_with_ts[0][1]

        i = 0
        behaviors = {}
        for e in test:
            if e is None:
                continue
            if behaviors_allowed is not None and e.behavior not in behaviors_allowed:
                continue
            indexes = [i for i, x in enumerate(status_with_ts) if x[0].current_item == e]
            if e.behavior not in behaviors:
                behaviors[e.behavior] = []
            behaviors[e.behavior] += indexes

        # print(drone, behaviors)

        for beh in behaviors.keys():
            behaviors[beh] = sorted(behaviors[beh])
            indexes = set(behaviors[beh])

            idx_ranges = []
            ts_ranges = []
            for _, g in groupby(enumerate(indexes), lambda k: k[0] - k[1]):
                start = next(g)[1]
                end = list(v for _, v in g) or [start]
                idx_ranges.append(range(start, end[-1] + 1))

                t0 = status_with_ts[start][1]
                tf = status_with_ts[end[-1]][1]
                # print(drone, beh, round(t0 * 1e-9), round(tf * 1e-9))
                duration = (tf - t0) * 1e-9
                t0 = t0 if duration > 2.0 else t0 - 1.0
                duration = duration if duration > 2.0 else 2.0
                ts_ranges.append(((t0 - t00) * 1e-9, duration))

            ax.broken_barh(ts_ranges, (i + y_pose_map[drone], 0.2), color=color_map[drone])
            i += 1
        patches.append(mpatches.Patch(color=color_map[drone], label=drone))

    y_labels = behaviors.keys()
    y_labels = ["_".join(x.split('_')[:-1]) if '_gps' in x else x for x in y_labels]

    # ax.set_title(f'Mission status {data.filename.stem}')
    ax.set_xlabel('time (s)')
    # ax.set_ylabel('status')
    ax.set_yticks(range(len(behaviors.keys())), labels=y_labels, rotation=45)
    ax.grid()
    ax.legend(handles=patches, loc='lower right')
    fig.savefig(f"/tmp/mission_status_{data.filename.stem}.png")
    return fig


def plot_broken_barh_mission_all(data: LogData,
                                 behavior_color_map: dict[str, str],
                                 behaviors_allowed: list[str] = None):
    """Plot mission status"""
    fig, ax = plt.subplots()
    i = -0.10
    for drone, status_with_ts in zip(data.mission_status.keys(), data.mission_status.values()):
        test = [key for key, group in groupby(status_with_ts, lambda x: x[0].current_item)]
        t00 = status_with_ts[0][1]

        behaviors = {}
        for e in test:
            if e is None:
                continue
            if behaviors_allowed is not None and e.behavior not in behaviors_allowed:
                continue
            indexes = [i for i, x in enumerate(status_with_ts) if x[0].current_item == e]
            beh_name = e.behavior if '_gps' not in e.behavior else e.behavior[:-4]
            if beh_name not in behaviors:
                behaviors[beh_name] = []
            behaviors[beh_name] += indexes

        print(drone, behaviors)

        for beh in behaviors.keys():
            if beh not in behavior_color_map.keys():
                continue
            behaviors[beh] = sorted(behaviors[beh])
            indexes = set(behaviors[beh])

            idx_ranges = []
            ts_ranges = []
            for _, g in groupby(enumerate(indexes), lambda k: k[0] - k[1]):
                start = next(g)[1]
                end = list(v for _, v in g) or [start]
                idx_ranges.append(range(start, end[-1] + 1))

                t0 = status_with_ts[start][1]
                tf = status_with_ts[end[-1]][1]
                # DEBUG: useful to check behavior times
                print(drone, beh, round(t0 * 1e-9), round(tf * 1e-9))
                duration = (tf - t0) * 1e-9
                t0 = t0 if duration > 2.0 else t0 - 1.0
                duration = duration if duration > 2.0 else 2.0
                ts_ranges.append(((t0 - t00) * 1e-9, duration))

            ax.broken_barh(ts_ranges, (i, 0.2), color=behavior_color_map[beh])
        i += 0.5

    i = 0
    for drone, updates in zip(data.mission_update.keys(), data.mission_update.values()):
        for update, ts in updates:
            ts = (ts - t00) * 1e-9
            if update.action == 1:
                ax.scatter(ts, i, color='c', marker='x', label='LOAD')
            elif update.action == 3:
                ax.scatter(ts, i, color='r', marker='x', label='PAUSE')
            elif update.action == 4:
                ax.scatter(ts, i, color='g', marker='x', label='RESUME')
            elif update.action == 5:
                ax.scatter(ts, i, color='k', marker='x', label='STOP')
            else:
                ax.scatter(ts, i, color='m', marker='x', label='START')
        i += 0.5

    patches = []
    for key, value in behavior_color_map.items():
        if behaviors_allowed is not None and key not in behaviors_allowed:
            continue
        patches.append(mpatches.Patch(color=value, label=key))
    patches.append(Line2D([], [], color='c', label='LOAD', marker='x', ls=''))
    patches.append(Line2D([], [], color='m', label='START', marker='x', ls=''))
    patches.append(Line2D([], [], color='r', label='PAUSE', marker='x', ls=''))
    patches.append(Line2D([], [], color='k', label='STOP', marker='x', ls=''))

    # ax.set_title(f'Mission status {data.filename.stem}')
    ax.set_xlabel('time (s)')
    # ax.set_ylabel('status')
    ax.set_yticks(np.linspace(0, 0.5, len(data.mission_status.keys())),
                  labels=data.mission_status.keys())
    ax.set_ybound(-0.6, 0.6)
    ax.grid()
    ax.legend(handles=patches)
    fig.savefig(f"/tmp/mission_status_{data.filename.stem}.png")
    return fig


def plot_3d_path(data: LogData):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        t0 = poses[0].header.stamp.sec
        x = [pose.pose.position.x for pose in poses]
        y = [pose.pose.position.y for pose in poses]
        z = [pose.pose.position.z for pose in poses]
        ax.plot(x, y, z, label=drone)

    # ax.set_title(f'3D Path {data.filename.stem}')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.legend()
    fig.savefig(f"/tmp/3d_path_{data.filename.stem}.png")
    return fig


def plot_3d_path_behaviors(data: LogData,
                           drone_behavior_ranges: dict[str, dict[str, list[tuple[int, int]]]],
                           color_map: dict[str, str]):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    elem = []
    for drone, poses in zip(data.poses.keys(), data.poses.values()):
        behavior_ranges = drone_behavior_ranges[drone]
        for behavior, ranges in behavior_ranges.items():
            if behavior == 'point_gimbal':
                target = min(enumerate(data.poses[drone]), key=lambda x: abs(
                    ranges[0][0] - x[1].header.stamp.sec))
                ax.scatter(data.poses[drone][target[0]].pose.position.x,
                           data.poses[drone][target[0]].pose.position.y,
                           data.poses[drone][target[0]
                                             ].pose.position.z, color=color_map[behavior],
                           marker='D', label=behavior)
                elem.append(Line2D([], [], color=color_map[behavior],
                            label=behavior, marker='D', ls=''))
                continue
            for start, end in ranges:
                x = [pose.pose.position.x for pose in poses if start <= pose.header.stamp.sec <= end]
                y = [pose.pose.position.y for pose in poses if start <= pose.header.stamp.sec <= end]
                z = [pose.pose.position.z for pose in poses if start <= pose.header.stamp.sec <= end]
                ax.plot(x, y, z, label=behavior, color=color_map[behavior])
            elem.append(Line2D([], [], color=color_map[behavior], label=behavior))

    seen = set()
    unique = [obj for obj in elem if obj.get_label() not in seen and not seen.add(obj.get_label())]
    # ax.set_title(f'3D Path {data.filename.stem}')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_zlabel('z (m)')
    ax.legend(handles=unique)
    fig.savefig(f"/tmp/3d_path_{data.filename.stem}.png")
    return fig


def put_plane_in_3D_plot(axis, xyz: np.ndarray, rpy: np.ndarray, dim: float = 2.0, color: str = 'grey'):

    xyz_ = xyz
    rpy_ = rpy
    # Rotation matrix from Euler angles
    rotation_matrix = R.from_euler('xyz', rpy_).as_matrix()

    # Create a grid for the plane
    x = np.linspace(-dim, dim, 100)
    y = np.linspace(-dim, dim, 100)
    X, Y = np.meshgrid(x, y)
    Z = np.zeros_like(X)

    # Apply rotation to the grid points
    points = np.stack([X.flatten(), Y.flatten(), Z.flatten()], axis=1)
    rotated_points = (rotation_matrix @ points.T).T

    translated_points = rotated_points + xyz_

    Xt = translated_points[:, 0].reshape(X.shape)
    Yt = translated_points[:, 1].reshape(Y.shape)
    Zt = translated_points[:, 2].reshape(Z.shape)

    axis.plot_surface(Xt, Yt, Zt, alpha=0.5, color=color)


def test_status(data: LogData):
    """Test mission status"""
    for drone, status_with_ts in zip(data.mission_status.keys(), data.mission_status.values()):
        test = [key for key, group in groupby(status_with_ts, lambda x: x[0].current_item)]

        # for msg in status_with_ts:
        #     print(msg[0].current_item, msg[0].state, msg[1])

        for e in test:
            if e is None:
                continue
            indexes = [i for i, x in enumerate(status_with_ts) if x[0].current_item == e]
            print(indexes)
            t00 = status_with_ts[0][1]
            t0 = status_with_ts[indexes[0]][1]
            tf = status_with_ts[indexes[-1]][1]
            print(e.behavior, (t0 - t00) * 1e-9, (tf - t00) * 1e-9)

        # last_item = ''
        # ts_0 = status_with_ts[0][1]
        # for status, ts in status_with_ts:
        #     if last_item != status.current_item:
        #         print(f'[{(ts-ts_0)*1e-9}] {drone}: {status.current_item}')
        #     last_item = status.current_item
        print()


def test_mission_updates(data: LogData):
    """Test mission status"""
    for drone, mission_updates in zip(data.mission_update.keys(), data.mission_update.values()):
        for update, ts in mission_updates:
            print(f'[{round(ts*1e-9)}] {drone}: {update.action}')


def plot_experiment_A():
    rosbag = Path('rosbags/rosbag_20250918_101422')
    drone_color_map = {'drone0': 'tab:blue'}
    y_pose_map = {'drone0': 0.0}
    drone_behavior_ranges = {'drone0': {'takeoff': [(0, 24)],
                                        'go_to': [(24, 96)],
                                        'land': [(97, 105)]}}
    color_map = {'takeoff': 'green', 'go_to': 'red',
                 'point_gimbal': 'black', 'take_photo': 'blue', 'land': 'y'}

    log_files = list(rosbag.iterdir())
    log_files = [log_file for log_file in log_files if log_file.suffix == '.db3']

    fig, fig2 = None, None
    for log in log_files:
        data = LogData.from_rosbag(log)

        print(data.poses['drone0'][0])
        print(data.mission_status['mission_status'][0])

        # test_status(data)
        test_mission_updates(data)

        # plot_broken_barh_mission_status(data, drone_color_map, y_pose_map)
        plot_broken_barh_mission_all(data, color_map)

        plot_3d_path(data)
        plot_z(data, drone_behavior_ranges, color_map)

        # fig = plot_3d_path_behaviors(data, drone_behavior_ranges, color_map)
        # ax = fig.gca()
        # ax.text(data.poses['drone0'][0].pose.position.x,
        #         data.poses['drone0'][0].pose.position.y,
        #         data.poses['drone0'][0].pose.position.z, 'drone0', size=15, zorder=1, color='k')
        plt.show()


def plot_experiment_B():
    rosbag = Path('rosbags/HIL/test_mission_1')
    drone_color_map = {'M300': 'tab:blue'}
    y_pose_map = {'M300': 0.0}
    drone_behavior_ranges = {'M300': {'takeoff': [(1758206947, 1758206957)],
                                      'go_to': [(1758206958, 1758207027)],
                                      'land': [(1758207027, 1758207037),]}}
    color_map = {'takeoff': 'green', 'go_to': 'red',
                 'point_gimbal': 'black', 'take_photo': 'blue', 'land': 'y'}

    log_files = list(rosbag.iterdir())
    log_files = [log_file for log_file in log_files if log_file.suffix == '.db3']

    fig, fig2 = None, None
    for log in log_files:
        data = LogData.from_rosbag(log)

        # test_status(data)
        test_mission_updates(data)

        # plot_broken_barh_mission_status(data, drone_color_map, y_pose_map)
        plot_broken_barh_mission_all(data, color_map)

        # plot_3d_path(data)
        plot_z(data, drone_behavior_ranges, color_map)

        fig = plot_3d_path_behaviors(data, drone_behavior_ranges, color_map)
        ax = fig.gca()
        ax.text(data.poses['M300'][0].pose.position.x,
                data.poses['M300'][0].pose.position.y,
                data.poses['M300'][0].pose.position.z, 'drone0', size=15, zorder=1, color='k')
        plt.show()


def plot_experiment_C():
    rosbag = Path('rosbags/20250923_pantano_insertion/flight_23')
    drone_color_map = {'M350': 'tab:blue'}
    y_pose_map = {'M350': 0.0}
    drone_behavior_ranges = {'M350': {'takeoff': [(1757333225, 1757333226)],
                                      'go_to': [(1757333228, 1757333290),
                                                (1757333294, 1757333300),
                                                (1757333304, 1757333310),
                                                (1757333314, 1757333332)],
                                      'take_photo': [(1757333291, 1757333293),
                                                     (1757333301, 1757333303),
                                                     (1757333311, 1757333313),
                                                     ]}}
    color_map = {'takeoff': 'green', 'go_to': 'red',
                 'point_gimbal': 'black', 'take_photo': 'blue', 'land': 'y'}

    log_files = list(rosbag.iterdir())
    log_files = [log_file for log_file in log_files if log_file.suffix == '.db3']

    fig, fig2 = None, None
    for log in log_files:
        data = LogData.from_rosbag(log)

        # test_status(data)
        # test_mission_updates(data)

        # plot_broken_barh_mission_status(data, drone_color_map, y_pose_map)
        plot_broken_barh_mission_all(data, color_map)

        plot_3d_path(data)
        # plot_z(data, drone_behavior_ranges, color_map)

        # fig = plot_3d_path_behaviors(data, drone_behavior_ranges, color_map)
        # ax = fig.gca()
        # ax.text(data.poses['M350'][0].pose.position.x,
        #         data.poses['M350'][0].pose.position.y,
        #         data.poses['M350'][0].pose.position.z, 'M350', size=15, zorder=1, color='k')
        plt.show()


def main(log_file: str):
    """Main function"""
    if Path(log_file).is_dir():
        log_files = list(Path(log_file).iterdir())
        for child in Path(log_file).iterdir():
            if child.is_file() and child.suffix == ".db3":
                log_files = [Path(log_file)]
                break
    elif Path(log_file).is_file():
        raise NotADirectoryError(f"{log_file} is not a directory")

    # DEFAULT FOR TWO DRONES
    drone_color_map = {'drone0': 'tab:blue'}
    y_pose_map = {'drone0': 0.0}

    fig, fig2 = None, None
    for log in log_files:
        data = LogData.from_rosbag(log)

        # test_status(data)
        plot_broken_barh_mission_status(data, drone_color_map, y_pose_map)

        fig = plot_3d_path(data)
        # fig = plot_3d_path_behaviors(data)
        # fig2 = plot_twist(data)
        # plot_x(data)
        # print(data.stats(25.0))
        plt.show()


if __name__ == "__main__":
    # plot_experiment_A()
    # plot_experiment_B()
    plot_experiment_C()
