# fish_hpurv

## Installation and Setup

Follow these steps to set up and use the `fish_hpurv` package in ROS 2:

1. Clone the package repository:  
   ```bash
   git clone https://github.com/lohithvarma2004/fish_hpurv.git
   ```
2. Navigate to your ROS 2 workspace and build the package using colcon:  
   ```bash
   colcon build
   ```
3. Source the workspace (consider adding it to your `~/.bashrc` for convenience):  
   ```bash
   source install/setup.bash
   ```
4. Launch the robot model in Gazebo:  
   ```bash
   ros2 launch fish_hpurv display.launch.py
   ```

## Package Structure

```
fish_hpurv/
│── urdf/            # Contains the Xacro file for the robot description
│── meshes/          # Stores STL files for the robot model
│── launch/          # Python launch file for spawning the robot
│── world/           # Fish_world to manually check SDF functionality
│── models/          
│   └── fish_hpurv/  # Model folder containing the necessary files
│       ├── model.config   # Model configuration file
│       ├── fish.sdf       # SDF file for Gazebo simulation
│       └── *.stl          # Additional STL files
```

This package allows the simulation of a bio-inspired fish robot in an underwater Gazebo world.
