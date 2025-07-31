#!/usr/bin/env python3
"""
Direct Map Publisher for Multi-Map Navigation.
This node generates and publishes occupancy grid maps directly for multiple rooms.
Used as a debugging fallback when map_server fails.
"""

import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class DirectMapPublisher(Node):
    def __init__(self):
        super().__init__('direct_map_publisher')
        self.get_logger().info('Direct Map Publisher started')

        # === QoS Setup ===
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # === Publishers for each room ===
        self.map_publishers = {
            'room1': self.create_publisher(OccupancyGrid, 'map_room1', qos_profile),
            'room2': self.create_publisher(OccupancyGrid, 'map_room2', qos_profile),
            'room3': self.create_publisher(OccupancyGrid, 'map_room3', qos_profile),
        }

        # === Subscription to current map ===
        self.current_map = 'room1'
        self.create_subscription(String, '/multi_map_nav/current_map', self.map_callback, 10)

        # === Timer to publish maps ===
        self.timer = self.create_timer(0.2, self.publish_maps)
        self.publish_count = 0
        self.get_logger().info('Publishing maps at 5Hz')

        # Initial push
        self.publish_maps()

    def map_callback(self, msg):
        self.current_map = msg.data
        self.get_logger().info(f'Current map updated: {self.current_map}')
        self.publish_maps()

    def publish_maps(self):
        for room in ['room1', 'room2', 'room3']:
            map_msg = self.generate_map(room)
            self.map_publishers[room].publish(map_msg)

        self.publish_count += 1
        if self.publish_count % 10 == 0:
            self.get_logger().info(f'Published all maps (count: {self.publish_count})')

    def generate_map(self, room_name, width=200, height=200):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = 0.05
        map_msg.info.width = width
        map_msg.info.height = height
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.orientation.w = 1.0

        data = np.zeros(width * height, dtype=np.int8)
        
        # === Wall boundaries ===
        data[0:width] = 100
        data[(height - 1)*width : height*width] = 100
        for i in range(height):
            data[i*width] = 100
            data[i*width + width - 1] = 100

        # === Doorways per room ===
        self.insert_doorways(data, room_name, width, height)
        # === Unique features per room ===
        self.insert_features(data, room_name, width, height)

        map_msg.data = data.tolist()
        return map_msg

    def insert_doorways(self, data, room_name, w, h):
        if room_name == 'room1':
            for i in range(int(h*0.4), int(h*0.6)):
                data[i*w + w - 1] = 0
        elif room_name == 'room2':
            for i in range(int(h*0.4), int(h*0.6)):
                data[i*w] = 0
            for i in range(int(w*0.4), int(w*0.6)):
                data[i] = 0
        elif room_name == 'room3':
            for i in range(int(w*0.4), int(w*0.6)):
                data[(h-1)*w + i] = 0

    def insert_features(self, data, room_name, w, h):
        mid_x, mid_y = w // 2, h // 2
        if room_name == 'room1':
            for y in range(mid_y - 5, mid_y + 5):
                for x in range(mid_x - 30, mid_x + 30):
                    data[y*w + x] = 100
            for y in range(mid_y - 30, mid_y + 30):
                for x in range(mid_x - 5, mid_x + 5):
                    data[y*w + x] = 100

        elif room_name == 'room2':
            radius = 30
            for y in range(h):
                for x in range(w):
                    if radius - 3 < np.hypot(x - mid_x, y - mid_y) < radius:
                        data[y*w + x] = 100

        elif room_name == 'room3':
            size = 30
            for y in range(mid_y - size, mid_y + size):
                for x in range(mid_x - size, mid_x + size):
                    if abs(x - mid_x) > size - 3 or abs(y - mid_y) > size - 3:
                        data[y*w + x] = 100


def main(args=None):
    rclpy.init(args=args)
    node = DirectMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Direct Map Publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

