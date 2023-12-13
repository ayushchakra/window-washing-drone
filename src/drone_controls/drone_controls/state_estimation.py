import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    Quaternion,
    Vector3,
    Pose,
    Vector3Stamped,
)
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation as R
import numpy as np
from pathlib import Path
import json

RECORD_POSES = False


class StateEstimationNode(Node):
    def __init__(self):
        super().__init__("state_estimation_node")

        self.delta_t = 0.1
        self.timer = self.create_timer(self.delta_t, self.run_loop)

        # Subscribe to various on-board sensors to be fused to create the state estimation
        self.create_subscription(Vector3Stamped, "/drone/gps/vel", self.process_gps, 10)
        self.create_subscription(Imu, "/drone/imu/out", self.process_imu, 10)
        self.create_subscription(Pose, "/drone/gt_pose", self.process_gt_pose, 10)
        self.create_subscription(Empty, "/drone/takeoff", self.process_takeoff, 10)
        self.create_subscription(NavSatFix, "/drone/data", self.process_altitude, 10)
        self.create_subscription(Empty, "/drone/land", self.process_land, 10)

        # Publish the estimated state
        self.pos_pub = self.create_publisher(Pose, "/drone/state_estimate", 10)

        # Track the current and previous sensor values
        self.vel_est: Vector3 = None
        self.prev_vel_est: Vector3 = None
        self.ang_vel_z: float = None
        self.prev_ang_vel_z: float = None
        self.curr_pose = Pose()
        self.curr_yaw = 0
        self.gt_pose = Pose()
        self.in_flight = False

        # Log the GT poses from the simulation and the estimated poses to be post-processed
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
        if RECORD_POSES:
            with open(Path(__file__).parent / "data/odom_benchmark.json", "w") as file:
                json.dump(self.odom_data, file)

    def run_loop(self):
        if not self.in_flight:
            self.pos_pub.publish(Pose())
        else:
            if self.prev_vel_est:
                # Integrate the x and y linear velocities over time, using a midpoint
                # riemann sum approximation
                self.curr_pose.position.x += (
                    (self.vel_est.x + self.prev_vel_est.x) / 2 * self.delta_t
                )
                self.curr_pose.position.y += (
                    (self.vel_est.y + self.prev_vel_est.y) / 2 * self.delta_t
                )
            if self.prev_ang_vel_z:
                # Integrate the z acceleration from the IMU
                self.curr_yaw += (
                    (self.ang_vel_z + self.prev_ang_vel_z) / 2 * self.delta_t
                )
                self.curr_yaw % (2 * np.pi)
                yaw_quat = R.from_euler("xyz", [0, 0, self.curr_yaw]).as_quat()
                self.curr_pose.orientation = Quaternion(
                    x=yaw_quat[0], y=yaw_quat[1], z=yaw_quat[2], w=yaw_quat[3]
                )

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
    rclpy.init(args=args)
    node = StateEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()
