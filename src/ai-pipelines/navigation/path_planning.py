#!/usr/bin/env python3

"""
Path Planning Pipeline for Humanoid Robots
This module implements path planning algorithms for humanoid navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
import math
from typing import List, Tuple


class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        # Publishers
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.status_pub = self.create_publisher(String, 'path_planning_status', 10)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            'move_base_simple/goal',
            self.goal_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Timer for path planning
        self.planning_timer = self.create_timer(1.0, self.plan_path_if_needed)

        # State variables
        self.map_data = None
        self.current_goal = None
        self.robot_pose = None
        self.last_path_time = self.get_clock().now()
        self.path_valid = False

        # Planning parameters
        self.planning_frequency = 1.0  # Hz
        self.path_timeout = 5.0  # seconds
        self.min_distance_to_goal = 0.2  # meters

        self.get_logger().info('Path Planning Node Initialized')

    def map_callback(self, msg):
        """Handle map updates"""
        self.map_data = {
            'data': np.array(msg.data).reshape(msg.info.height, msg.info.width),
            'info': msg.info
        }
        self.get_logger().info('Map received')

    def goal_callback(self, msg):
        """Handle new navigation goals"""
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'New goal received: {self.current_goal}')
        self.path_valid = False  # Invalidate existing path
        self.plan_path_if_needed()  # Plan immediately

    def scan_callback(self, msg):
        """Handle laser scan data for local obstacle avoidance"""
        # This could be used for local path adjustment
        pass

    def plan_path_if_needed(self):
        """Plan a new path if needed"""
        if self.current_goal and self.map_data and not self.path_valid:
            self.plan_path()

    def plan_path(self):
        """Plan path using A* algorithm"""
        if not self.map_data or not self.current_goal:
            return

        # Get map dimensions and resolution
        map_width = self.map_data['info'].width
        map_height = self.map_data['info'].height
        resolution = self.map_data['info'].resolution
        origin = self.map_data['info'].origin

        # Convert world coordinates to map coordinates
        start_x, start_y = self.world_to_map(0, 0)  # Assuming robot is at origin initially
        goal_x, goal_y = self.world_to_map(self.current_goal[0], self.current_goal[1])

        # Validate coordinates
        if not self.is_valid_coordinate(start_x, start_y, map_width, map_height) or \
           not self.is_valid_coordinate(goal_x, goal_y, map_width, map_height):
            self.get_logger().error('Invalid coordinates for path planning')
            return

        # Run A* algorithm
        path = self.a_star_search(start_x, start_y, goal_x, goal_y)

        if path:
            # Convert path back to world coordinates
            world_path = self.path_to_world(path, origin, resolution)
            self.publish_path(world_path)
            self.path_valid = True
            self.get_logger().info('Path planned successfully')
        else:
            self.get_logger().error('No path found to goal')
            self.publish_status('No path found')

    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates"""
        if not self.map_data:
            return 0, 0

        origin = self.map_data['info'].origin
        resolution = self.map_data['info'].resolution

        map_x = int((x - origin.position.x) / resolution)
        map_y = int((y - origin.position.y) / resolution)

        return map_x, map_y

    def path_to_world(self, path, origin, resolution):
        """Convert path in map coordinates to world coordinates"""
        world_path = []
        for x, y in path:
            world_x = x * resolution + origin.position.x
            world_y = y * resolution + origin.position.y
            world_path.append((world_x, world_y))
        return world_path

    def is_valid_coordinate(self, x, y, width, height):
        """Check if coordinates are within map bounds and not occupied"""
        if x < 0 or x >= width or y < 0 or y >= height:
            return False

        # Check if cell is occupied (value > 50 means occupied in OccupancyGrid)
        if self.map_data['data'][y, x] > 50:
            return False

        return True

    def a_star_search(self, start_x, start_y, goal_x, goal_y):
        """A* pathfinding algorithm implementation"""
        # Get map dimensions
        width = self.map_data['info'].width
        height = self.map_data['info'].height

        # Initialize open and closed sets
        open_set = [(start_x, start_y)]
        closed_set = set()

        # Initialize g_score and f_score
        g_score = {}
        f_score = {}
        g_score[(start_x, start_y)] = 0
        f_score[(start_x, start_y)] = self.heuristic(start_x, start_y, goal_x, goal_y)

        # Parent tracking for path reconstruction
        came_from = {}

        # A* main loop
        while open_set:
            # Find node with lowest f_score
            current = min(open_set, key=lambda pos: f_score.get(pos, float('inf')))

            # Check if we reached the goal
            if current == (goal_x, goal_y):
                return self.reconstruct_path(came_from, current)

            # Move current from open to closed
            open_set.remove(current)
            closed_set.add(current)

            # Check neighbors (8-connected)
            for dx, dy in [(-1,-1), (-1,0), (-1,1), (0,-1), (0,1), (1,-1), (1,0), (1,1)]:
                neighbor = (current[0] + dx, current[1] + dy)

                # Skip if neighbor is in closed set or invalid
                if neighbor in closed_set or not self.is_valid_coordinate(neighbor[0], neighbor[1], width, height):
                    continue

                # Calculate tentative g_score
                tentative_g_score = g_score.get(current, float('inf')) + self.distance(current, neighbor)

                # If this path to neighbor is better
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor[0], neighbor[1], goal_x, goal_y)

                    if neighbor not in open_set:
                        open_set.append(neighbor)

        # No path found
        return None

    def heuristic(self, x1, y1, x2, y2):
        """Heuristic function (Euclidean distance)"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def distance(self, pos1, pos2):
        """Distance between two positions"""
        return math.sqrt((pos2[0] - pos1[0])**2 + (pos2[1] - pos1[1])**2)

    def reconstruct_path(self, came_from, current):
        """Reconstruct path from came_from dictionary"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path):
        """Publish the planned path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_status(self, status):
        """Publish path planning status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    """Main function to run the path planning node"""
    rclpy.init(args=args)

    # Create the path planning node
    planning_node = PathPlanningNode()

    try:
        # Keep the node running
        rclpy.spin(planning_node)
    except KeyboardInterrupt:
        planning_node.get_logger().info('Shutting down path planning node...')
    finally:
        # Clean up
        planning_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()