"""
Launch file to start static map servers for room1, room2, and room3.
Uses a lifecycle manager to manage their lifecycle transitions.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # === Configurable Launch Argument ===
    pkg_share = FindPackageShare("multi_map_nav")
    default_map_dir = PathJoinSubstitution([pkg_share, "maps"])
    map_dir = LaunchConfiguration("map_dir")

    return LaunchDescription([
        # === Declare Argument ===
        DeclareLaunchArgument(
            "map_dir",
            default_value=default_map_dir,
            description="Directory containing the map YAML + PGM files"
        ),

        # === Debug Output (Optional) ===
        LogInfo(msg=["[MultiMapNav] Loading maps from: ", map_dir]),
        ExecuteProcess(
            cmd=["ls", "-la", map_dir],
            output="screen"
        ),

        # === Static Map Servers ===
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server_room1",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "yaml_filename": PathJoinSubstitution([map_dir, "room1.yaml"]),
                "topic": "map_room1",
                "frame_id": "map",
                "use_sim_time": False
            }]
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server_room2",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "yaml_filename": PathJoinSubstitution([map_dir, "room2.yaml"]),
                "topic": "map_room2",
                "frame_id": "map",
                "use_sim_time": False
            }]
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server_room3",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "yaml_filename": PathJoinSubstitution([map_dir, "room3.yaml"]),
                "topic": "map_room3",
                "frame_id": "map",
                "use_sim_time": False
            }]
        ),

        # === Lifecycle Manager for Map Servers ===
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_maps",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "autostart": True,
                "node_names": [
                    "map_server_room1",
                    "map_server_room2",
                    "map_server_room3"
                ]
            }]
        )
    ])

