"""
Launches the core components of the multi-map navigation system:
- Navigation action server (handles wormhole logic)
- Map switcher node (remaps and switches maps dynamically)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # === Launch Arguments ===
    db_path = LaunchConfiguration("db_path")
    current_map = LaunchConfiguration("current_map")

    return LaunchDescription([
        # === Declare Arguments ===
        DeclareLaunchArgument(
            "db_path",
            default_value="/tmp/wormhole.db",
            description="Path to SQLite wormhole database"
        ),
        DeclareLaunchArgument(
            "current_map",
            default_value="room1",
            description="Initial map name"
        ),

        # === Navigation Server ===
        Node(
            package="multi_map_nav",
            executable="navigation_server",
            name="multi_map_navigation_server",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"db_path": db_path},
                {"current_map": current_map}
            ]
        ),

        # === Map Switcher ===
        Node(
            package="multi_map_nav",
            executable="map_switcher",
            name="map_switcher",
            output="screen",
            emulate_tty=True,
            remappings=[("map", "/map")]  # Ensure map topic is global
        ),
    ])

