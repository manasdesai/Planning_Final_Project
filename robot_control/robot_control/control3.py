#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class ClosedLoopTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("closed_loop_trajectory_follower")

        self._trajectories = [
            [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],
            [0.2, -0.8, 1.2, 0.1, 1.2, 0.3],
            [-0.3, -1.2, 1.8, -0.1, 0.8, 0.6],
            [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # self.kp = 1.5
        self.tolerance = 0.01

        self.kp = 5
        self.tolerance = 0.1

        self.current_position = np.zeros(len(self.joint_names))
        self.joint_received = False
        self.current_target_index = 0

        self.publisher_ = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.subscriber_ = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

    def joint_state_callback(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        try:
            self.current_position = np.array(
                [positions[name] for name in self.joint_names]
            )
            self.joint_received = True
        except KeyError:
            self.get_logger().warn("Joint state missing expected joint names")

    def control_loop(self):
        if not self.joint_received or self.current_target_index >= len(
            self._trajectories
        ):
            return

        target = np.array(self._trajectories[self.current_target_index])
        error = target - self.current_position

        if np.all(np.abs(error) < self.tolerance):
            self.get_logger().info(
                f"Reached target {self.current_target_index + 1}/{len(self._trajectories)}"
            )
            self.current_target_index += 1
            return

        # P-controller for next position
        next_position = self.current_position + self.kp * error * 0.1  # dt = 0.1s

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = next_position.tolist()
        point.time_from_start.sec = 1
        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(
            f"Moving to target {self.current_target_index + 1}: {target.round(3).tolist()}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopTrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
