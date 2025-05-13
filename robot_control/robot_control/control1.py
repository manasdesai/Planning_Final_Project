#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time


class ArmTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('arm_trajectory_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.publish_trajectories()

    def publish_trajectories(self):
        trajectories = [
            [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],
            [0.2, -0.8, 1.2, 0.1, 1.2, 0.3],
            [-0.3, -1.2, 1.8, -0.1, 0.8, 0.6],
            [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],  # return to initial
        ]

        delay_between = 3  # seconds between trajectories

        for i, positions in enumerate(trajectories):
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start.sec = 2  # duration for this movement
            traj_msg.points.append(point)

            self.publisher_.publish(traj_msg)
            self.get_logger().info(f'Published trajectory {i+1}')
            time.sleep(delay_between)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTrajectoryPublisher()
    rclpy.spin_once(node, timeout_sec=1)  # Only spin briefly to allow publishing
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
