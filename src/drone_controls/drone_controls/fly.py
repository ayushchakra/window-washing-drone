import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3, Pose, Point
import math
import time
from enum import Enum, auto
import numpy as np
from dataclasses import dataclass
from typing import List
from pathlib import Path
import json
from abc import ABC
from functools import cached_property

PANNING_STEP_SIZE = 0.30  # m
DIST_THRESH = 0.20  # m
TAKEOFF_HEIGHT_THRESHOLD = 0.7  # m
DIST_FROM_WALL = 0.6  # m
Z_OFFSET_FROM_WINDOW = 0.2  # m
DESIRED_MAP_FEATURES = [
    # "WINDOW_ONE",
    # "WINDOW_TWO",
    "WINDOW_THREE",
    # "CORNER_ONE",
    # "WINDOW_FOUR",
    # "WINDOW_FIVE",
    # "WINDOW_SIX",
    # "WINDOW_SEVEN",
    # "CORNER_TWO",
    # "WINDOW_EIGHT",
]
ENABLE_LOGGING = True


class DroneState(Enum):
    INIT = auto()
    TAKEOFF = auto()
    TRAJECTORY_FOLLOWING = auto()
    TURNING = auto()
    LANDING = auto()


def euler_from_quaternion(x, y, z, w):
    t1 = +2.0 * (w * z + x * y)
    t2 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t1, t2)

    return yaw


class TrajectoryPlanner(ABC):
    name: str

    @cached_property
    def trajectory_points(self) -> np.ndarray:
        raise NotImplementedError


@dataclass
class WindowDatum(TrajectoryPlanner):
    name: str
    left_top_corner: List[float]
    right_bottom_corner: List[float]
    required_yaw: float

    @cached_property
    def trajectory_points(self) -> np.ndarray:
        if self.left_top_corner[0] == self.right_bottom_corner[0]:
            x_const = True
        else:
            x_const = False

        num_pts_by_axis = (
            np.abs(np.array(self.left_top_corner) - np.array(self.right_bottom_corner))
            // PANNING_STEP_SIZE
            + 1
        )
        x_vals = np.linspace(
            self.left_top_corner[0],
            self.right_bottom_corner[0],
            int(num_pts_by_axis[0]),
        )
        y_vals = np.linspace(
            self.left_top_corner[1],
            self.right_bottom_corner[1],
            int(num_pts_by_axis[1]),
        )
        z_vals = (
            np.linspace(
                self.left_top_corner[2],
                self.right_bottom_corner[2],
                int(num_pts_by_axis[2]),
            )
            + Z_OFFSET_FROM_WINDOW
        )
        pts = []
        reverse = False
        for z in z_vals:
            if reverse:
                iter = -1
            else:
                iter = 1
            reverse = not reverse
            if x_const:
                for y in y_vals[::iter]:
                    pts.append(
                        np.array(
                            [
                                x_vals[0] - DIST_FROM_WALL * np.cos(self.required_yaw),
                                y,
                                z,
                                self.required_yaw,
                            ]
                        )
                    )
            else:
                for x in x_vals[::iter]:
                    pts.append(
                        np.array(
                            [
                                x,
                                y_vals[0] - DIST_FROM_WALL * np.sin(self.required_yaw),
                                z,
                                self.required_yaw,
                            ]
                        )
                    )
        return np.array(pts)


@dataclass
class CornerDatum(TrajectoryPlanner):
    name: str
    point: List[float]
    required_yaw: float

    @cached_property
    def trajectory_points(self) -> np.ndarray:
        ret = np.zeros(4)
        ret[0:3] = self.point
        ret[3] = self.required_yaw
        return np.array([ret])


class FlyNode(Node):
    def __init__(self):
        super().__init__("fly_node")

        self.takeoff = self.create_publisher(Empty, "/drone/takeoff", 10)
        self.land = self.create_publisher(Empty, "/drone/land", 10)
        self.vel_cmd = self.create_publisher(Twist, "/drone/cmd_vel", 10)
        self.create_subscription(Pose, "/drone/gt_pose", self.update_state, 10)
        self.yaw = 0
        self.pose: Point = None

        self.create_timer(0.1, self.run_loop)
        self.state = DroneState.INIT

        with open(Path(__file__).parent / "data/map.json", "r") as file:
            self.window_corner_positions = json.load(file)["window_corner_positions"]

        self.map_features: List[TrajectoryPlanner] = []
        for obj in self.window_corner_positions:
            if obj["name"] not in DESIRED_MAP_FEATURES:
                continue
            if obj["name"].startswith("WINDOW"):
                self.map_features.append(WindowDatum(**obj))
            else:
                self.map_features.append(CornerDatum(**obj))
        self.map_features[0].trajectory_points
        self.feature_idx = -1
        self.trajectory_pt_idx = -1

        if ENABLE_LOGGING:
            self.log_data = {}

    def update_state(self, msg: Pose):
        self.pose = msg.position
        self.yaw = euler_from_quaternion(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        )

    @property
    def curr_map_feature(self):
        return self.map_features[self.feature_idx]

    @property
    def curr_trajectory_pts(self):
        return self.curr_map_feature.trajectory_points

    @property
    def curr_setpoint(self):
        if self.trajectory_pt_idx == -1:
            return None
        return self.curr_trajectory_pts[self.trajectory_pt_idx]

    def run_loop(self):
        print(self.state)
        print(self.curr_setpoint)
        if ENABLE_LOGGING and self.trajectory_pt_idx > 0:
            if not isinstance(self.curr_map_feature, WindowDatum):
                raise RuntimeError("Invalid Type for the current map feature")
            if self.curr_map_feature.name not in self.log_data.keys():
                self.log_data[self.curr_map_feature.name] = {
                    "window_coordinates": {
                        "top_left_corner": self.curr_map_feature.left_top_corner,
                        "right_buttom_corner": self.curr_map_feature.right_bottom_corner,
                    },
                    "drone_poses": [],
                }
            self.log_data[self.curr_map_feature.name]["drone_poses"].append(
                [self.pose.x, self.pose.y, self.pose.z]
            )

        if self.state == DroneState.INIT:
            self.takeoff.publish(Empty())
            self.state = DroneState.TAKEOFF
        if self.state == DroneState.TAKEOFF:
            if self.pose and self.pose.z >= TAKEOFF_HEIGHT_THRESHOLD:
                self.state = DroneState.TRAJECTORY_FOLLOWING
                self.feature_idx = 0
                self.trajectory_pt_idx = 0
        if self.state == DroneState.TRAJECTORY_FOLLOWING:
            if (
                (self.pose.x - self.curr_setpoint[0]) ** 2
                + (self.pose.y - self.curr_setpoint[1]) ** 2
                + (self.pose.z - self.curr_setpoint[2]) ** 2
            ) ** 0.5 < DIST_THRESH:
                if self.trajectory_pt_idx + 1 >= len(self.curr_trajectory_pts):
                    if self.feature_idx + 1 == len(self.map_features):
                        self.state = DroneState.LANDING
                        return
                    else:
                        self.feature_idx += 1
                        self.trajectory_pt_idx = 0
                else:
                    self.trajectory_pt_idx += 1
            vel = np.array(
                [
                    (self.curr_setpoint[0] - self.pose.x) / 2.5,
                    (self.curr_setpoint[1] - self.pose.y) / 2.5,
                ]
            )
            rot = np.array(
                [
                    [np.cos(-self.yaw), np.sin(-self.yaw)],
                    [-np.sin(-self.yaw), np.cos(-self.yaw)],
                ]
            )
            drone_vel = vel @ rot
            self.vel_cmd.publish(
                Twist(
                    linear=Vector3(
                        x=drone_vel[0],
                        y=drone_vel[1],
                        z=(self.curr_setpoint[2] - self.pose.z) / 2,
                    ),
                    angular=Vector3(
                        z=((self.curr_setpoint[3] - self.yaw) + np.pi) % (2 * np.pi)
                        - np.pi
                    ),
                )
            )
        if self.state == DroneState.LANDING:
            self.land.publish(Empty())
            with open(Path(__file__).parent / "data/drone_panning.json", "w") as file:
                json.dump(self.log_data, file, indent=4)

            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FlyNode()
    rclpy.spin(node)
    rclpy.shutdown()
