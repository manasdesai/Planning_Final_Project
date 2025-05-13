#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from astar_algo import AStar
from obstacles import Obstacles
from prm import PRM
from robot import RobotArm

PRM_NODES = 20

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/waypoints', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)  # Publish every second
        self.get_logger().info('Waypoint Publisher Node has been started.')
        self.waypoint_list = []
        self.generate_waypoints()
        self.waypoint_index = 0

    def generate_waypoints(self):
        robot = RobotArm()
        ob = Obstacles()
        prm = PRM(robot, ob.obstacles, N=PRM_NODES, K=5)

        # waypoints = [
        #     [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],
        #     [0.2, -0.8, 1.2, 0.1, 1.2, 0.3],
        #     [-0.3, -1.2, 1.8, -0.1, 0.8, 0.6],
        #     [0.0, -1.0, 1.5, 0.0, 1.0, 0.5],
        #     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        # ]
        waypoints = [[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                    [0.785, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
                    [1.57, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
                    # [-1.57, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
                    # [-0.785, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
                    [0.785, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
                    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
                    ]
        # waypoints = [[0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        #              [1.57, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
        #              [3.14, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
        #              [-1.57, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
        #              [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        #             ]
        
        for i in range(len(waypoints)):
            start = waypoints[i]
            goal = waypoints[0] if i == len(waypoints) - 1 else waypoints[i + 1]

            astr = AStar(prm, start, goal)
            path = astr._search()
            if path is not None:
                self.waypoint_list.extend(path)
            else:
                # self.get_logger().error(f"Failed to generate path from {start} to {goal}")
                self.get_logger().info(f"Failed to generate path from {start} to {goal}")


        self.get_logger().info('Generated waypoints for UR manipulator.')
        self.get_logger().info(f"Number of waypoints: {len(self.waypoint_list)}")

    def publish_waypoints(self):
        if self.waypoint_index >= len(self.waypoint_list):
            self.get_logger().info('All waypoints published.')
            self.waypoint_index = 0  # Reset to continuously publish
            return

        # Prepare the message
        msg = Float64MultiArray()
        waypoint = self.waypoint_list[self.waypoint_index]

        # Flatten the waypoint if necessary
        if isinstance(waypoint, (list, tuple, np.ndarray)):
            flat_waypoint = np.array(waypoint).flatten().tolist()
        else:
            flat_waypoint = [float(waypoint)]

        # Assign the flattened waypoint to the message
        msg.data = flat_waypoint

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published waypoint {self.waypoint_index + 1}: {msg.data}')
        self.waypoint_index += 1

def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
