#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import numpy as np

class ClosedLoopTrajectoryFollower(Node):
    def __init__(self):
        super().__init__('closed_loop_trajectory_follower')
        self.get_logger().info("Controller node up; waiting for /waypoints + /joint_states…")

        # State
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.current_position = np.zeros(6)
        self.joint_received = False
        self.waypoints = []           # list of 6‐float targets
        self.waypoints_received = False
        self.idx = 0                  # current waypoint index
        self.kp = 5.0
        self.tol = 0.05

        # Subs & pubs
        self.create_subscription(Float64MultiArray, '/waypoints',
                                 self.waypoint_cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_state_cb, 10)
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        # start control loop
        self.create_timer(0.1, self.control_loop)

    def waypoint_cb(self, msg: Float64MultiArray):
        flat = np.array(msg.data, dtype=float)
        n = flat.size // 6
        self.waypoints = flat.reshape(n, 6).tolist()
        self.waypoints_received = True
        self.idx = 0
        self.get_logger().info(f"Loaded {n} waypoints from astar_node")

    def joint_state_cb(self, msg: JointState):
        m = {n:p for n,p in zip(msg.name, msg.position)}
        try:
            self.current_position = np.array([m[j] for j in self.joint_names])
            self.joint_received = True
        except KeyError:
            self.get_logger().warn("Joint state missing some names")

    def control_loop(self):
        if not (self.joint_received and self.waypoints_received):
            return

        if self.idx >= len(self.waypoints):
            self.get_logger().info("All waypoints reached.")
            return

        target = np.array(self.waypoints[self.idx])
        error = target - self.current_position

        # if within tolerance, advance index
        if np.all(np.abs(error) < self.tol):
            self.get_logger().info(f"Reached waypoint {self.idx+1}/{len(self.waypoints)}")
            self.idx += 1
            return

        # P‐control step
        delta = self.kp * error * 0.1
        next_pos = (self.current_position + delta).tolist()

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = next_pos
        pt.time_from_start.sec = 1
        traj.points.append(pt)

        self.pub.publish(traj)
        self.get_logger().info(f"Moving → wp {self.idx+1}: {list(np.round(target,3))}")

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopTrajectoryFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
