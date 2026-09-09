# px4ctrl ROS 2 Humble

This package is the ROS 2 Humble port of the controller in `px4ctrl_ros`.

Required ROS 2 packages:

- `rclcpp`
- `mavros` and `mavros_msgs`
- `quadrotor_msgs` with `PositionCommand`, `TakeoffLand`, and `Px4ctrlDebug`

The ROS 2 `PositionCommand` message used here has the fields `pos`, `vel`, and
`acc`. A ROS 1 `quadrotor_msgs` package cannot be used directly.

Build from the workspace root after sourcing Humble, MAVROS, and the ROS 2
`quadrotor_msgs` workspace:

```bash
colcon build --symlink-install --packages-select px4ctrl
source install/setup.bash
ros2 launch px4ctrl run_ctrl_launch.py
```

The controller subscribes to:

- `/mavros/state`
- `/mavros/extended_state`
- `odom` (remapped by the launch file)
- `cmd` (remapped by the launch file)
- `/mavros/imu/data`
- `/mavros/rc/in`
- `/mavros/battery`
- `takeoff_land`

It publishes attitude commands to `/mavros/setpoint_raw/attitude`, trajectory
start triggers to `/traj_start_trigger`, and debug messages to `/debugPx4ctrl`.
