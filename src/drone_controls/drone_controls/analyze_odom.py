import matplotlib.pyplot as plt
from pathlib import Path
import json
import numpy as np


def main():
    with open(Path(__file__).parent / "data/odom_benchmark.json", "r") as file:
        data = json.load(file)
    gt_poses = np.array(data["gt_poses"])
    est_poses = np.array(data["est_poses"])

    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot(gt_poses[:, 0], gt_poses[:, 1], gt_poses[:, 2], "k", label="GT Poses")
    ax.plot(
        est_poses[:, 0], est_poses[:, 1], est_poses[:, 2], "r", label="Estimated Poses"
    )
    ax.legend()
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")

    ax.set_title("GT Pose vs. Estimated Pose")
    ax = fig.add_subplot(1, 2, 2)
    ax.set_xlabel("Pose Index")
    ax.set_ylabel("Mean Squared Error (m)")
    ax.set_title("MSE of Estimated Pose")
    mse = np.linalg.norm(gt_poses - est_poses, axis=1)
    plt.plot(mse)
    plt.show()


if __name__ == "__main__":
    main()
