# PRM Path Planner Demonstration for UR10e Manipulator

## Summary 
This repository implements a multi‑query Probabilistic Roadmap (PRM) path planner and A* query for a UR10e 6‑DOF manipulator. Running the demo will build a collision‑free roadmap in joint‑space, search for a feasible path between a hard‑coded start and goal, and visualize both the end‑effector roadmap and the full robot link configurations against workspace obstacles using Matplotlib.

## Dependencies 
Python, NumPy, SciPy, Matplotlib 

```
pip install numpy scipy matplotlib 
```
## Part1
If you just want to test the Python implementation of PRM and A*, go to the prm-manipulator folder using
-  `cd Project5_Codes/prm-manipulator`

Second, from the project root, execute the following:
```
python3 main.py
```

Third, examine the generated plots.

## Part2
To run the Gazebo simulation,

Navigate to the project5_ws workspace
`cd Project5_Codes/project5_ws/`
  
Build the packages using
- `colcon build --symlink-install`
  
To launch the gazebo simulation, source the workspace and run
- `ros2 launch robot_control gazebo.launch.py`

Turn the physics off in Gazebo, and try to place the robot on the cafe table if not placed already.

In another terminal, source the workspace and run
- `ros2 run robot_control controller.py`

In a separate terminal, source the workspace and run
- `ros2 run robot_control astar_node.py`  
