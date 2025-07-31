To LauncH the Full System
In a Terminal1 open the workspace where the repository is located then
source install/setup.bash (source the workspace)
ros2 launch multi_map_nav multi_map_full.launch.py

To run the Map Publisher
In a Terminal2 open the workspace where the repository is located then
source install/setup.bash (source the workspace)
ros2 run multi_map_nav direct_map_publisher.py


To WORMHOLE ROBOT into different rooms map
In a Terminal3 open the workspace where the repository is located then
source install/setup.bash (source the workspace)
to room 1
ros2 launch multi_map_nav send_goal.launch.py map_name:=room1 x:=1.0 y:=3.0
to room 2
ros2 launch multi_map_nav send_goal.launch.py map_name:=room2 x:=2.0 y:=2.0
to room 3
ros2 launch multi_map_nav send_goal.launch.py map_name:=room3 x:=3.0 y:=1.0

The WORMHOLE  happens between room goes like
room1 to room2
room2 to room3


