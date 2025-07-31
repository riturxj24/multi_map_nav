"""
Full system launch for the multi-map navigation system.

Includes:
- Room map servers
- Wormhole setup (SQLite DB)
- Navigation server + switcher
- Map visualizer
- TF publisher + robot state publisher
- RViz (optional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # === Launch Arguments ===
    db_path = LaunchConfiguration("db_path")
    current_map = LaunchConfiguration("current_map")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    pkg_share = FindPackageShare("multi_map_nav")

    return LaunchDescription([
        # === Declare Arguments ===
        DeclareLaunchArgument(
            "db_path",
            default_value="/tmp/wormhole.db",
            description="Path to the SQLite database file"
        ),
        DeclareLaunchArgument(
            "current_map",
            default_value="room1",
            description="Initial map to load"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Whether to launch RViz"
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([pkg_share, "config", "multi_map_nav.rviz"]),
            description="RViz configuration file"
        ),

        # === Launch Map Loading ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, "launch", "load_maps.launch.py"])
            )
        ),

        # === Launch Navigation Server + Switcher ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, "launch", "multi_map_nav.launch.py"])
            ),
            launch_arguments={
                "db_path": db_path,
                "current_map": current_map
            }.items()
        ),

        # === Wormhole Setup Node ===
        Node(
            package="multi_map_nav",
            executable="setup_wormholes.py",
            name="setup_wormholes",
            output="screen",
            parameters=[{"db_path": db_path}]
        ),

        # === Map Visualization Node ===
        Node(
            package="multi_map_nav",
            executable="map_visualizer",
            name="map_visualizer",
            output="screen",
            parameters=[{"db_path": db_path}]
        ),

        # === TF Publisher ===
        Node(
            package="multi_map_nav",
            executable="tf_publisher_node",
            name="tf_publisher_node",
            output="screen"
        ),

        # === Robot State Publisher ===
        Node(
            package="multi_map_nav",
            executable="robot_state_publisher_node",
            name="robot_state_publisher_node",
            output="screen"
        ),

        # === RViz (optional) ===
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        ),
    ])

