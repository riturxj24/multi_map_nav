#!/usr/bin/env python3

import os
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.utilities import perform_substitutions

class MapVerifier(Node):
    def __init__(self):
        super().__init__('map_verifier')
        self.get_logger().info('Map Verifier node started')

        # State flags
        self.map_received = False
        self.room_maps_received = {'room1': False, 'room2': False, 'room3': False}
        self.current_map_name = "room1"

        self._init_subscribers()
        self._init_publishers()
        self._check_map_files()
        self.timer = self.create_timer(1.0, self._publish_test_map)

    def _init_subscribers(self):
        self.create_subscription(OccupancyGrid, '/map', self._main_map_cb, 10)
        self.create_subscription(OccupancyGrid, '/map_room1', lambda msg: self._room_map_cb(msg, 'room1'), 10)
        self.create_subscription(OccupancyGrid, '/map_room2', lambda msg: self._room_map_cb(msg, 'room2'), 10)
        self.create_subscription(OccupancyGrid, '/map_room3', lambda msg: self._room_map_cb(msg, 'room3'), 10)
        self.create_subscription(String, '/multi_map_nav/current_map', self._current_map_cb, 10)

    def _init_publishers(self):
        self.test_map_pub = self.create_publisher(OccupancyGrid, '/test_map', 10)

    def _main_map_cb(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info(f'Main map received! Size: {len(msg.data)} | Dimensions: {msg.info.width}x{msg.info.height} | Frame: {msg.header.frame_id}')
        else:
            self.get_logger().debug('Map update received.')

    def _room_map_cb(self, msg, room_name):
        if not self.room_maps_received[room_name]:
            self.room_maps_received[room_name] = True
            self.get_logger().info(f'{room_name.capitalize()} map received! Size: {len(msg.data)}')

    def _current_map_cb(self, msg):
        self.current_map_name = msg.data
        self.get_logger().info(f'Current map is: {self.current_map_name}')

    def _check_map_files(self):
        try:
            pkg_share = perform_substitutions(None, [FindPackageShare('multi_map_nav')])
            maps_dir = os.path.join(pkg_share, 'maps')
            self.get_logger().info(f"Checking map files in: {maps_dir}")

            for room in ['room1', 'room2', 'room3']:
                pgm = os.path.join(maps_dir, f"{room}.pgm")
                yaml = os.path.join(maps_dir, f"{room}.yaml")

                if not os.path.exists(pgm):
                    self.get_logger().error(f"Missing PGM file: {pgm}")
                else:
                    size = os.path.getsize(pgm)
                    self.get_logger().info(f"PGM exists: {pgm} ({size} bytes)")
                    if size < 100:
                        self.get_logger().warn(f"PGM may be too small: {size} bytes")

                if not os.path.exists(yaml):
                    self.get_logger().error(f"Missing YAML file: {yaml}")
                else:
                    self.get_logger().info(f"YAML exists: {yaml}")

        except Exception as e:
            self.get_logger().error(f"Error while checking map files: {e}")

    def _publish_test_map(self):
        test_map = OccupancyGrid()
        test_map.header.stamp = self.get_clock().now().to_msg()
        test_map.header.frame_id = "map"
        test_map.info.resolution = 0.05
        test_map.info.width = 20
        test_map.info.height = 20
        test_map.info.origin.position.x = -5.0
        test_map.info.origin.position.y = -5.0
        test_map.info.origin.orientation.w = 1.0

        test_map.data = [0] * (20 * 20)

        for i in range(20):
            test_map.data[i] = 100
            test_map.data[(19) * 20 + i] = 100
            test_map.data[i * 20] = 100
            test_map.data[i * 20 + 19] = 100

        self.test_map_pub.publish(test_map)
        self.get_logger().debug('Published test map')

    def _status_report(self, final=False):
        tag = 'FINAL STATUS' if final else 'STATUS UPDATE'
        self.get_logger().info(f'=== {tag} ===')
        self.get_logger().info(f'Main map received: {self.map_received}')
        for room, received in self.room_maps_received.items():
            self.get_logger().info(f'{room.capitalize()} map received: {received}')
        self.get_logger().info(f'Current map: {self.current_map_name}')
        if final and not self.map_received:
            self.get_logger().error('No map received on /map topic!')
        elif final:
            self.get_logger().info('Map verification completed successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = MapVerifier()

    try:
        start = time.time()
        while rclpy.ok() and (time.time() - start) < 15.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if int(time.time() - start) % 5 == 0:
                node._status_report()

        node._status_report(final=True)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

