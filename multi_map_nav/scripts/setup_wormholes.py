#!/usr/bin/env python3

import sqlite3
import rclpy
from rclpy.node import Node


class WormholeSetup(Node):
    def __init__(self):
        super().__init__('wormhole_setup')

        self.db_path = self.declare_parameter('db_path', '/tmp/wormhole.db')\
                            .get_parameter_value().string_value
        self.get_logger().info(f"Using database at: {self.db_path}")

        if self.setup_wormholes():
            self.get_logger().info("Wormhole setup completed successfully!")
        else:
            self.get_logger().error("Wormhole setup failed!")

    def setup_wormholes(self) -> bool:
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()

            self._create_schema(cursor)
            self._insert_wormholes(cursor)

            conn.commit()

            count = cursor.execute('SELECT COUNT(*) FROM wormholes').fetchone()[0]
            self.get_logger().info(f"Number of wormholes in database: {count}")

            for row in cursor.execute('SELECT source_map, target_map, source_x, source_y, target_x, target_y FROM wormholes'):
                self.get_logger().info(
                    f"Wormhole: {row[0]} -> {row[1]} at ({row[2]}, {row[3]}) -> ({row[4]}, {row[5]})"
                )

            conn.close()
            return True

        except sqlite3.Error as e:
            self.get_logger().error(f"SQLite error: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
            return False

    def _create_schema(self, cursor):
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS wormholes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                source_map TEXT NOT NULL,
                target_map TEXT NOT NULL,
                source_x REAL NOT NULL, source_y REAL NOT NULL, source_z REAL NOT NULL,
                source_qx REAL NOT NULL, source_qy REAL NOT NULL, source_qz REAL NOT NULL, source_qw REAL NOT NULL,
                target_x REAL NOT NULL, target_y REAL NOT NULL, target_z REAL NOT NULL,
                target_qx REAL NOT NULL, target_qy REAL NOT NULL, target_qz REAL NOT NULL, target_qw REAL NOT NULL,
                transition_cost REAL NOT NULL,
                UNIQUE(source_map, target_map)
            )
        ''')

    def _insert_wormholes(self, cursor):
        wormholes = [
            # room1 <-> room2
            ('room1', 'room2', 4.5, 2.0, -4.5, 2.0),
            ('room2', 'room1', -4.5, 2.0, 4.5, 2.0),
            # room2 <-> room3
            ('room2', 'room3', 2.0, 4.5, 2.0, -4.5),
            ('room3', 'room2', 2.0, -4.5, 2.0, 4.5),
        ]

        for src_map, tgt_map, src_x, src_y, tgt_x, tgt_y in wormholes:
            cursor.execute('''
                INSERT OR REPLACE INTO wormholes 
                (source_map, target_map,
                 source_x, source_y, source_z, source_qx, source_qy, source_qz, source_qw,
                 target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw,
                 transition_cost)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                src_map, tgt_map,
                src_x, src_y, 0.0, 0.0, 0.0, 0.0, 1.0,
                tgt_x, tgt_y, 0.0, 0.0, 0.0, 0.0, 1.0,
                1.0
            ))


def main():
    rclpy.init()
    node = WormholeSetup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

