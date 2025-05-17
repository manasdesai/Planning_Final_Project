Navigate to your workspace
`cd ~/project5_ws/src`

Clone the packages using 
- `git clone https://github.com/manasdesai/Planning_Final_Project`
  
Build the packages using
- `cd ~/project5_ws && colcon build --symlink-install`
  
To launch the gazebo simulation, run
- `ros2 launch robot_control gazebo.launch.py

Turn the physics off in Gazebo, and try to place the robot on the cafe table if not placed already.

In another terminal, source the workspace and run
- `ros2 run robot_control controller.py`

In a separate terminal, source the workspace and run
- `ros2 run robot_control astar_node.py`  
