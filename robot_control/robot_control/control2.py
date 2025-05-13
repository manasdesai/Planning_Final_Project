#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class ClosedLoopArmController(Node):
    def __init__(self):
        super().__init__('closed_loop_arm_controller')

        # Target joint positions
        self._trajectories = [
            [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],
            [0.2, -0.8, 1.2, 0.1, 1.2, 0.3],
            [-0.3, -1.2, 1.8, -0.1, 0.8, 0.6],
            [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],  # return to initial
        ]
        
        self.target_position = np.array([0.5, -1.2, 1.0, 0.2, 0.3, 0.1])
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.kp = 1.5  # proportional gain
        self.tolerance = 0.01  # convergence threshold

        self.current_position = np.zeros(len(self.joint_names))
        self.joint_received = False

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.subscriber_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Main control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def joint_state_callback(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        try:
            self.current_position = np.array([positions[name] for name in self.joint_names])
            self.joint_received = True
        except KeyError:
            self.get_logger().warn('Joint names mismatch in /joint_states')

    def control_loop(self):
        if not self.joint_received:
            return

        error = self.target_position - self.current_position

        if np.all(np.abs(error) < self.tolerance):
            self.get_logger().info("Target reached.")
            return

        correction = self.current_position + self.kp * error * 0.1  # simple proportional step

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = correction.tolist()
        point.time_from_start.sec = 1
        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(f"Publishing corrected positions: {correction}")

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
