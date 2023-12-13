import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3, Pose, Point
from enum import Enum, auto
import numpy as np
from typing import List
from pathlib import Path
import json
from .helpers import *

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
    """
    Finest State Machine tracking the different states of the Drone as it travels over
    the scene.
    """

    INIT = auto()
    TAKEOFF = auto()
    TRAJECTORY_FOLLOWING = auto()
    TURNING = auto()
    LANDING = auto()


class FlyNode(Node):
    def __init__(self):
        super().__init__("fly_node")

        # Create publishers to command drone behavior
        self.takeoff = self.create_publisher(Empty, "/drone/takeoff", 10)
        self.land = self.create_publisher(Empty, "/drone/land", 10)
        self.vel_cmd = self.create_publisher(Twist, "/drone/cmd_vel", 10)

        # Drone State Estimation for Closed Loop Control
        self.create_subscription(Pose, "/drone/state_estimate", self.update_state, 10)
        # Odometry variables
        self.yaw = 0
        self.pose: Point = None

        self.create_timer(0.1, self.run_loop)
        self.state = DroneState.INIT

        # Load in pre-saved window coordinated from the map
        with open(Path(__file__).parent / "data/map.json", "r") as file:
            self.window_corner_positions = json.load(file)["window_corner_positions"]

        # Extract "features" (windows) for the given map
        self.map_features: List[TrajectoryPlanner] = []
        for obj in self.window_corner_positions:
            if obj["name"] not in DESIRED_MAP_FEATURES:
                continue
            if obj["name"].startswith("WINDOW"):
                self.map_features.append(WindowDatum(**obj))
            else:
                self.map_features.append(CornerDatum(**obj))

        # Initialize tracking variables to prepare for trajectory following
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
    def curr_map_feature(self) -> WindowDatum:
        """
        Returns the current WindowDatum object
        """
        return self.map_features[self.feature_idx]

    @property
    def curr_trajectory_pts(self) -> np.ndarray:
        """
        Returns a list of waypoints based on the current feature window
        """
        return self.curr_map_feature.trajectory_points

    @property
    def curr_setpoint(self) -> np.ndarray:
        """
        Returns the current x,y,z,yaw array to navigate to
        """
        if self.trajectory_pt_idx == -1:
            return None
        return self.curr_trajectory_pts[self.trajectory_pt_idx]

    def run_loop(self):
        # Store drone positions relative to the window to analyze window
        # sprayer coverage in post-processing
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
            # Initialize the drone to takeoff
            self.takeoff.publish(Empty())
            self.state = DroneState.TAKEOFF
        if self.state == DroneState.TAKEOFF:
            # Transitions to trajectory following once the drone has reached a
            # certain height from the takeoff command
            if self.pose and self.pose.z >= TAKEOFF_HEIGHT_THRESHOLD:
                self.state = DroneState.TRAJECTORY_FOLLOWING
                self.feature_idx = 0
                self.trajectory_pt_idx = 0
        if self.state == DroneState.TRAJECTORY_FOLLOWING:
            # Checks if the current setpoint has been reached
            if (
                (self.pose.x - self.curr_setpoint[0]) ** 2
                + (self.pose.y - self.curr_setpoint[1]) ** 2
                + (self.pose.z - self.curr_setpoint[2]) ** 2
            ) ** 0.5 < DIST_THRESH:
                # If the distance threshold has been met, check if there are any more
                # waypoints to be navigated to for the current window, if not, then
                # start navigating to the next window
                if self.trajectory_pt_idx + 1 >= len(self.curr_trajectory_pts):
                    if self.feature_idx + 1 == len(self.map_features):
                        self.state = DroneState.LANDING
                        return
                    else:
                        self.feature_idx += 1
                        self.trajectory_pt_idx = 0
                else:
                    self.trajectory_pt_idx += 1

            # Use a basic p-controller to command velocities to reach the current
            # setpoint
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
            # Send landing command
            self.land.publish(Empty())
            # Store the drone trajectory data to a json for post processing
            with open(Path(__file__).parent / "data/drone_panning.json", "w") as file:
                json.dump(self.log_data, file, indent=4)

            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FlyNode()
    rclpy.spin(node)
    rclpy.shutdown()
