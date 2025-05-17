#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

from astar_algo import AStar
from obstacles import Obstacles
from prm import PRM
from robot import RobotArm

PRM_NODES = 20

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.pub = self.create_publisher(Float64MultiArray, '/waypoints', 10)
        self.get_logger().info('Starting PRM+A* and publishing full waypoint list…')
        self.publish_all_waypoints()
    
    def publish_all_waypoints(self):
        robot = RobotArm()
        ob = Obstacles()
        prm = PRM(robot, ob.obstacles, N=PRM_NODES, K=5)

        # your “key” waypoints around the tables
        seeds = [
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
            [0.785, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
            [1.57, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
            [0.785, 0.13849, 2.38544, 0.62393, 1.57, 0.0],
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        ]

        full_path = []
        for i, start in enumerate(seeds):
            goal = seeds[0] if i == len(seeds)-1 else seeds[i+1]
            astr = AStar(prm, start, goal)
            segment = astr._search()
            if segment is None:
                self.get_logger().warn(f"no path from {start} → {goal}")
            else:
                full_path.extend(segment)

        flat = [float(x) for wp in full_path for x in wp]
        msg = Float64MultiArray(data=flat)
        self.pub.publish(msg)
        self.get_logger().info(f"Published {len(full_path)} waypoints (_flat len={len(flat)})")
        # keep node alive so subscriber can latch
        # no timer needed; we’re done

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
