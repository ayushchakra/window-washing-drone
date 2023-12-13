from dataclasses import dataclass
from functools import cached_property
from abc import ABC
import math
from typing import List
import numpy as np

PANNING_STEP_SIZE = 0.40  # m
DIST_THRESH = 0.20  # m
TAKEOFF_HEIGHT_THRESHOLD = 0.7  # m
DIST_FROM_WALL = 0.6  # m
Z_OFFSET_FROM_WINDOW = 0.2  # m


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
        """
        Create a linspace of waypoints for navigation based on a parameter
        step size, and two corner positions of the window.
        """
        # Determines if the window has constant x dimensions or y.
        if self.left_top_corner[0] == self.right_bottom_corner[0]:
            x_const = True
        else:
            x_const = False

        # Construct the linspace across each axis
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

        # Create the back and forth motion based on the computed linspaces
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
    """
    Corners represent single point datums to facilitate the transitions
    between window features and avoid walls
    """

    name: str
    point: List[float]
    required_yaw: float

    @cached_property
    def trajectory_points(self) -> np.ndarray:
        ret = np.zeros(4)
        ret[0:3] = self.point
        ret[3] = self.required_yaw
        return np.array([ret])
