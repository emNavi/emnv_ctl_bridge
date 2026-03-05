# Usage Details


## State Machine
ctrl_bridge provides an upper-level state machine. The state is published to the `/ctrl_bridge/bridge_status` topic.
After landing, the state machine resets and returns to IDLE, waiting for the next takeoff.
> Some odometry systems such as VINS may no longer be ready for takeoff after completing a flight, requiring a restart of the odometry system.


## Setting Odometry Information

By modifying `ref_odom` in the launch file, you can use:
- VIO or LIO odometry information
- `/mavros/local_position/odom`

## Control

### ctrl_mode

`ctrl_mode` defines the type of control command provided by the upper-level planning and control system to ctrl_bridge. Options are `POSY`, `ATTI`, `RATE`.
- `POSY` is the most common: pvay (position, velocity, acceleration, yaw)
- `ATTI`: the planning/control system provides a quaternion attitude and thrust setpoint
- `RATE`: the planning/control system provides 3-axis angular rates and thrust setpoint


### cmd_out_level
`cmd_out_level` is the command type that ctrl_bridge sends to PX4. Options are:
- `POSY`: desired position + desired velocity + desired acceleration + yaw + yaw rate (3+3+3+1+1); the position loop runs on PX4
- `ATTI`: quaternion attitude + normalized throttle thrust (4+1)
- `RATE`: body-frame angular rates + normalized throttle thrust (3+1)

> Note: when using POSY mode, `local_position` must output a valid value at 30 Hz (i.e., PX4 has received valid position information).

### Dependency Between ctrl_mode and cmd_out_level

- When `ctrl_mode` is `RATE`, `cmd_out_level` can only be `RATE`
- When `ctrl_mode` is `ATTI`, `cmd_out_level` can be `ATTI` or `RATE`
- When `ctrl_mode` is `POSY`, `cmd_out_level` can be `POSY`, `ATTI`, or `RATE`

If the combination is invalid, the program exits and throws the following error:
```bash
[ERROR] [1751528439.591269879]: Invalid correspondence between "ctrl_mode" and "ctrl_level"
```

1. When `ctrl_mode` is `POSY` and `cmd_out_level` is `POSY`, control is computed by PX4.
1. When `ctrl_mode` is `ATTI` and `cmd_out_level` is `ATTI`, control is computed by PX4.
1. When `ctrl_mode` is `RATE` and `cmd_out_level` is `RATE`, control is computed by PX4.
1. When `ctrl_mode` is `POSY` and `cmd_out_level` is `ATTI`, the `linear_controller` in ctrl_bridge takes effect — the position loop is computed by ctrl_bridge.
1. When `ctrl_mode` is `POSY` and `cmd_out_level` is `RATE`, both `linear_controller` and `AttitudeController` in ctrl_bridge take effect — the position loop and attitude loop are computed by ctrl_bridge.
1. When `ctrl_mode` is `ATTI` and `cmd_out_level` is `RATE`, the `AttitudeController` in ctrl_bridge takes effect — the attitude loop is computed by ctrl_bridge.




### Landing Detection
- Descent speed remains below `-0.1 m/s` for 2 consecutive seconds
- When hover thrust estimation is active, the estimated hover thrust stays below `0.1` for 2 consecutive seconds

### Takeoff and Landing Commands

ctrl_bridge provides quick takeoff and landing functionality:
  - `/ctrl_bridge/takeoff` Bool msg
  - `/ctrl_bridge/land`  Bool msg
```bash
# source src/control_for_gym/Tools/help_func.sh # already included in devel/setup.bash by default
source devel/setup.bash
takeoff drone
land drone
# "drone" is a parameter (drone_name) configured in the launch file
# You can also take off or land multiple drones at once
takeoff drone1,drone2
land drone1,drone2
```

The takeoff and landing phases have the following characteristics:
> 1. During takeoff and landing, the user's upper-level planning/control program does not intervene in control (control commands will be rejected by ctrl_bridge)
> 2. Takeoff and landing commands are mutually exclusive — only the last received command type takes effect
> 3. A takeoff can be directly interrupted by a landing command



## State Estimation
### Hover Thrust Estimation

When `cmd_out_level` is set to `ATTI` or `RATE`, ctl_bridge needs to perform hover thrust estimation.
> The thrust estimation depends on:
> - The z-axis component of the normalized throttle in the world coordinate frame
> - The z-axis acceleration measured in the world coordinate frame

Hover thrust estimation uses a single-variable EKF. $a_z^{meas}$ is the measured z-axis acceleration in the world frame, $u_{hover}$ is the hover thrust (0~1), $u$ is the normalized throttle, and $g$ is gravitational acceleration. The measurement equation is:
$$
a_z^{meas}  =  \frac{u}{u_{hover}}g+noise
$$




In the code:
- The acceleration obtained from `/mavros/imu/data` is in the baselink frame; a coordinate transform is applied to get the z-axis measurement in the world frame.
- $u$ uses the throttle setpoint before entering the mixer

The estimation module exposes several configuration parameters:
```yaml
hover_thrust_ekf:
  init_hover_thrust: 0.6
  hover_thrust_max: 0.8
  hover_thrust_noise: 0.1
  process_noise: 0.0036
```
- `init_hover_thrust`: the initial estimate of hover thrust. If unknown, set to 0.1. If set too low, takeoff will be sluggish; if set too high, takeoff may overshoot. If `cmd_out_level` is `ATTI` or `RATE`, you can check the real-time hover thrust estimate after successful takeoff and hover via `rostopic echo /ctrl_bridge/hover_thrust`.
- `hover_thrust_max`: upper bound on the hover thrust estimate
- `hover_thrust_noise`: hover thrust measurement noise
- `process_noise`: IMU acceleration measurement noise

> Reference: PX4 hover thrust estimation

## Remote Control
### Forced Landing via Remote Control
TODO
- You can configure a switch to toggle between program control and remote control. We assume you have a switch configured as a cmd_valid toggle. If state estimation is normal, you can use the switch to force a switch to landing mode. -->



## Multi-UAV Deployment

For multi-UAV simulation, ctrl_bridge is adapted to use ROS `<group>` namespaces. All mavros topics subscribed by mavutils will automatically include the group prefix.


## Trajectory Generation Module
### Fifth-Order Polynomial Trajectory
A fifth-order polynomial trajectory generator is available for quickly testing control performance. Note that:
- The trajectory does not support obstacle avoidance
- The trajectory does not account for physical space constraints — the generated shape depends only on the waypoints and execution time settings
- For more details, see [Trajectory Generation Module](./docs/ploy_traj.md)
### Lemniscate
Generates a figure-eight trajectory.
