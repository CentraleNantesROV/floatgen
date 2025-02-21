# Simulation of a world with floating wind turbines

Uses Gazebo (or Ignition) and [simple_launch](https://github.com/oKermorgant/simple_launch).


## Spawning Ignition with a wind farm

The following command will spawn an empty world with buoyancy plugin, and a wind farm with joint state publisher:

`ros2 launch floatgen floatgen_world_launch.py`

parameters:

- `nx, ny`: number of turbines in x and y directions (default 1, 1)
- `x, y`: position of the first turbine (default 0, 0)
- `scale`: distance between two turbines (default 200)
- `yaw`: orientation of the farm (default 0)
- `velocity`: angular velocity of the turbines (default -15)
