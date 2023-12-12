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
from sensor_msgs.msg import Imu
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
        self.create_subscription(TFMessage, "/tf", self.process_april_tag, 10)
        self.create_subscription(Vector3Stamped, "/drone/gps/vel", self.process_gps, 10)
        self.create_subscription(Imu, "/drone/imu/out", self.process_imu, 10)
        self.create_subscription(Pose, "/drone/gt_pose", self.process_gt_pose, 10)
        self.pos_pub = self.create_publisher(Pose, "/drone/position_estimate", 10)
        self.new_april_tag = False
        self.april_tags: List[Tuple[str, TagDatum]] = []
        self.vel_est: Vector3 = None
        self.prev_vel_est: Vector3 = None
        self.ang_vel_z: float = None
        self.prev_ang_vel_z: float = None
        self.curr_pose = Pose()
        self.gt_pose = Pose()

        with open(Path(__file__).parent / "data/map.json", "r") as file:
            self.tag_lookup = {
                tag_name: MapTagDatum(xyz_rpy=pose)
                for (tag_name, pose) in json.load(file)["april_tag_positions"].items()
            }

    def process_april_tag(self, msg: TFMessage):
        self.new_april_tag = True
        for tag in msg.transforms:
            self.april_tags.append((tag.child_frame_id, TagDatum(tag.transform)))

    def process_gps(self, msg: Vector3Stamped):
        self.prev_vel_est = self.vel_est
        self.vel_est = Vector3(x=-msg.vector.x, y=-msg.vector.y, z=msg.vector.z)

    def process_imu(self, msg: Imu):
        self.prev_ang_vel_z = self.ang_vel_z
        self.ang_vel_z = msg.angular_velocity.z

    def process_gt_pose(self, msg: Pose):
        self.gt_pose = msg

    def run_loop(self):
        if self.new_april_tag:
            # Currently assumes only one tag has been detected in a given frame.
            # This is a reasonable assumption based on the layout of the AprilTags
            # in cafe.world
            # tag_id, tag_datum = self.april_tags.pop()
            # tag_global_pos = self.tag_lookup[tag_id]
            # global_pos_est = (
            #     tag_global_pos.homogeneous_pose @ tag_datum.homogeneous_pose
            # )
            # global_pos_est_trans = global_pos_est[0:3, 3]
            # global_pos_est_rot = R.from_matrix(global_pos_est[0:3, 0:3]).as_quat()
            # self.curr_pose = Pose(
            #     position=Point(
            #         x=global_pos_est_trans[0],
            #         y=global_pos_est_trans[1],
            #         z=global_pos_est_trans[2],
            #     ),
            #     orientation=Quaternion(
            #         x=global_pos_est_rot[0],
            #         y=global_pos_est_rot[1],
            #         z=global_pos_est_rot[2],
            #         w=global_pos_est_rot[3],
            #     ),
            # )
            self.curr_pose = self.gt_pose
            self.new_april_tag = False
        else:
            if self.prev_ang_vel_z:
                self.curr_pose
            if self.prev_vel_est:
                self.curr_pose.position.x += (
                    self.vel_est.x - self.prev_vel_est.x
                ) * self.delta_t
                self.curr_pose.position.y += (
                    self.vel_est.y - self.prev_vel_est.y
                ) * self.delta_t
                self.curr_pose.position.z += (
                    self.vel_est.z - self.prev_vel_est.z
                ) * self.delta_t

        self.pos_pub.publish(self.curr_pose)


def main(args=None):
    np.set_printoptions(precision=4)
    rclpy.init(args=args)
    node = StateEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()
