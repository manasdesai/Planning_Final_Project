#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np


class ClosedLoopTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("closed_loop_trajectory_follower")

        self._trajectories =  [[0., 0., 0., 0., 0., 0.],
                [-1.41676495, -0.08502147, -0.77376657, -0.17377489, -0.44743411, -0.03214425],
                [-0.89670221, -0.29152778, -1.46246705, -1.25166692, -0.63305188, -0.63735725],
                [-1.12005874, -0.05637656, -1.75455204, -0.81418333, -0.60207674, -1.52630983],
                [-1.45913798, -0.50836614, -1.3414889,  -0.38920551, -1.1352861,  -1.94283063],
                [-1.94144708, -0.62595784, -1.70016405, -0.70587617, -1.02567317, -2.72013383],
                [-2.37000504, -1.42265913, -1.3608887,  -1.35968222, -1.04178208, -2.82965688],
                [-1.81861782, -1.54672164, -1.67409782, -1.63175719, -1.572515,   -2.71065885],
                [-2.43177897, -1.24268743, -1.88267826, -2.15830153, -2.12468244, -2.89429456],
                [-2.44435127, -1.62391323, -1.63169247, -2.33736547, -2.98732659, -3.2074331 ],
                [-3.12715491, -1.96207623, -2.07769007, -2.88244187, -3.00878353, -3.53981402],
                [-2.51004079, -2.52320659, -1.92358539, -2.66495185, -3.59439967, -3.30914331],
                [-3.01440459, -2.70031357, -2.68695014, -2.51396126, -3.64650969, -3.16921926],
                [-3.14159265, -3.14159265, -3.14159265, -3.14159265, -3.14159265, -3.14159265]
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
