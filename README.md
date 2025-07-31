To LauncH the Full System
source install/setup.bash
ros2 launch multi_map_nav multi_map_full.launch.py


To run the Map Publisher
source install/setup.bash
ros2 run multi_map_nav direct_map_publisher.py


To Navigate  into different rooms map
source install/setup.bash
to room 1
ros2 launch multi_map_nav send_goal.launch.py map_name:=room1 x:=1.0 y:=3.0
to room 2
ros2 launch multi_map_nav send_goal.launch.py map_name:=room2 x:=2.0 y:=2.0
to room 3
ros2 launch multi_map_nav send_goal.launch.py map_name:=room3 x:=3.0 y:=1.0

The navigation happens between room goes like
room1 to room2
room2 to room3


