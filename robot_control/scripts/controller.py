#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from std_msgs.msg import Float64MultiArray

class ClosedLoopTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("closed_loop_trajectory_follower")
        self.get_logger().info("Closed Loop Trajectory Follower Node Initialized")

        # Subscribe to waypoints from the topic
        self.waypoint_subscriber = self.create_subscription(
            Float64MultiArray, "/waypoints", self.waypoint_callback, 10
        )

        # Publisher for sending joint trajectory commands
        self.publisher_ = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        # Joint states subscriber
        self.subscriber_ = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # UR robot joint names
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Controller parameters
        self.kp = 5.0
        self.tolerance = 0.05
        self.current_position = np.zeros(len(self.joint_names))
        self.joint_received = False
        self.waypoints = []
        self.current_target_index = 0
        self.get_logger().info("Waiting for waypoints...")

    def waypoint_callback(self, msg: Float64MultiArray):
        # Convert the received flat list into waypoints
        data = np.array(msg.data).reshape(-1, len(self.joint_names))
        self.waypoints = data.tolist()
        self.current_target_index = 0
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")
        self.get_logger().info(f"First waypoint: {self.waypoints[0]}")

    def joint_state_callback(self, msg: JointState):
        # Update current joint positions
        positions = dict(zip(msg.name, msg.position))
        try:
            self.current_position = np.array(
                [positions[name] for name in self.joint_names]
            )
            self.joint_received = True
        except KeyError:
            self.get_logger().warn("Joint state missing expected joint names")

    def control_loop(self):
        if not self.joint_received:
            self.get_logger().warn("Joint states not received yet")
            return

        # Check if waypoints are available
        if len(self.waypoints) == 0:
            self.get_logger().warn("No waypoints received")
            return

        # Check if all waypoints are processed
        if self.current_target_index >= len(self.waypoints):
            self.get_logger().info("All waypoints processed. Returning to home position.")
            return

        # Target waypoint
        target = np.array(self.waypoints[self.current_target_index])
        error = target - self.current_position

        # Check if the current target waypoint is reached
        if np.all(np.abs(error) < self.tolerance):
            self.get_logger().info(
                f"Reached waypoint {self.current_target_index + 1}/{len(self.waypoints)}"
            )
            self.current_target_index += 1
            if self.current_target_index >= len(self.waypoints):
                self.get_logger().info("All waypoints reached.")
                return
            target = np.array(self.waypoints[self.current_target_index])

        # Proportional control to calculate the next joint position
        next_position = self.current_position + self.kp * error * 0.1  # dt = 0.1s

        # Publish the next joint trajectory
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = next_position.tolist()
        point.time_from_start.sec = 1
        traj_msg.points.append(point)

        self.publisher_.publish(traj_msg)
        self.get_logger().info(
            f"Moving to waypoint {self.current_target_index + 1}: {target.round(3).tolist()}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopTrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
