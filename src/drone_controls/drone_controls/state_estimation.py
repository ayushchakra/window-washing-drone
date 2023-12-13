import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import (
    Transform,
    Quaternion,
    Vector3,
    Pose,
    Point,
    Vector3Stamped,
)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Empty
from typing import List, Dict, Tuple
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
import numpy as np
from pathlib import Path
import json


@dataclass
class TagDatum:
    april_tag: Transform

    @property
    def translation(self) -> np.ndarray:
        return np.array(
            [
                self.april_tag.translation.x,
                self.april_tag.translation.y,
                self.april_tag.translation.z,
            ]
        )

    @property
    def rot_mat(self) -> np.ndarray:
        return R.from_quat(
            [
                self.april_tag.rotation.x,
                self.april_tag.rotation.y,
                self.april_tag.rotation.z,
                self.april_tag.rotation.w,
            ]
        ).as_matrix()

    @property
    def rpy(self) -> np.ndarray:
        """
        global -> transform
        y -> x
        """
        return R.from_quat(
            [
                self.april_tag.rotation.x,
                self.april_tag.rotation.y,
                self.april_tag.rotation.z,
                self.april_tag.rotation.w,
            ]
        ).as_euler("xyz")

    @property
    def homogeneous_pose(self) -> np.ndarray:
        ret = np.zeros([4, 4])
        ret[0:3, 0:3] = self.rot_mat
        ret[0:3, 3] = self.translation
        ret[3, 3] = 1

        return ret


class MapTagDatum(TagDatum):
    def __init__(self, xyz_rpy: Dict):
        quat = R.from_euler("xyz", [xyz_rpy["rpy"]]).as_quat()[0]
        rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        translation = Vector3(
            x=xyz_rpy["xyz"][0], y=xyz_rpy["xyz"][1], z=xyz_rpy["xyz"][2]
        )
        self.april_tag = Transform(translation=translation, rotation=rotation)


class StateEstimationNode(Node):
    def __init__(self):
        super().__init__("state_estimation_node")

        self.delta_t = 0.1
        self.timer = self.create_timer(self.delta_t, self.run_loop)
        self.create_subscription(Vector3Stamped, "/drone/gps/vel", self.process_gps, 10)
        self.create_subscription(Imu, "/drone/imu/out", self.process_imu, 10)
        self.create_subscription(Pose, "/drone/gt_pose", self.process_gt_pose, 10)
        self.create_subscription(Empty, "/drone/takeoff", self.process_takeoff, 10)
        self.create_subscription(NavSatFix, "/drone/data", self.process_altitude, 10)
        self.create_subscription(Empty, "/drone/land", self.process_land, 10)
        self.pos_pub = self.create_publisher(Pose, "/drone/state_estimate", 10)
        self.vel_est: Vector3 = None
        self.prev_vel_est: Vector3 = None
        self.ang_vel_z: float = None
        self.prev_ang_vel_z: float = None
        self.curr_pose = Pose()
        self.curr_yaw = 0
        self.gt_pose = Pose()
        self.in_flight = False
        self.odom_data = {"gt_poses": [], "est_poses": []}

    def process_gps(self, msg: Vector3Stamped):
        self.prev_vel_est = self.vel_est
        self.vel_est = Vector3(x=-msg.vector.x, y=-msg.vector.y, z=msg.vector.z)

    def process_imu(self, msg: Imu):
        self.prev_ang_vel_z = self.ang_vel_z
        self.ang_vel_z = msg.angular_velocity.z

    def process_gt_pose(self, msg: Pose):
        self.gt_pose = msg

    def process_takeoff(self, msg: Empty):
        self.in_flight = True

    def process_altitude(self, msg: NavSatFix):
        self.curr_pose.position.z = msg.altitude

    def process_land(self, msg: Empty):
        with open(Path(__file__).parent / "data/odom_benchmark.json", "w") as file:
            json.dump(self.odom_data, file)

    def run_loop(self):
        if not self.in_flight:
            self.pos_pub.publish(Pose())
        else:
            if self.prev_vel_est:
                self.curr_pose.position.x += (
                    (self.vel_est.x + self.prev_vel_est.x) / 2 * self.delta_t
                )
                self.curr_pose.position.y += (
                    (self.vel_est.y + self.prev_vel_est.y) / 2 * self.delta_t
                )
            if self.prev_ang_vel_z:
                self.curr_yaw += (
                    (self.ang_vel_z + self.prev_ang_vel_z) / 2 * self.delta_t
                )
                self.curr_yaw % (2 * np.pi)

            self.odom_data["est_poses"].append(
                [
                    self.curr_pose.position.x,
                    self.curr_pose.position.y,
                    self.curr_pose.position.z,
                ]
            )
            self.odom_data["gt_poses"].append(
                [
                    self.gt_pose.position.x,
                    self.gt_pose.position.y,
                    self.gt_pose.position.z,
                ]
            )

            self.pos_pub.publish(self.curr_pose)


def main(args=None):
    np.set_printoptions(precision=4)
    rclpy.init(args=args)
    node = StateEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()
