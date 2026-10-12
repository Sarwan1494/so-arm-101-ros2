# SO-101 Robot Arm with ROS 2 and MoveIt 2

ROS 2 Jazzy packages to plan motions for the SO-101 6-axis arm in MoveIt 2 and run them on the real robot. The arm uses Feetech STS3215 bus servos. A custom `ros2_control` hardware plugin talks to the servos directly over USB serial, so a trajectory planned in RViz is executed by the physical arm.

## Packages

| Package | Contents |
|---|---|
| `so_arm_101_description` | URDF/xacro model, STL meshes, RViz config, the ST3215 servo driver library (`st3215_driver`) and `motor_node`, a standalone node used as a torque-off calibration tool |
| `so_arm_101_hardware` | `ros2_control` `SystemInterface` plugin (`so_arm_101_hardware/SoArm101HardwareInterface`) that reads and writes all 6 servos through `st3215_driver` |
| `so_arm_config` | MoveIt 2 configuration (SRDF, kinematics, controllers, joint limits) and launch files, including `real_hardware.launch.py` |
| `so_arm_101_startup` | `startup_sequence` node that moves the arm through the named poses `startup_move` and `ready` with MoveGroupInterface |

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy
- MoveIt 2, `ros2_control`, `ros2_controllers`
- SO-101 arm on `/dev/ttyACM0` (set in `so_arm_config/config/so101_new_calib.ros2_control.xacro`)

## Build

```bash
mkdir -p ~/ws_moveit2/src
cd ~/ws_moveit2/src
git clone git@github.com:Sarwan1494/so-arm-101-ros2.git .
cd ~/ws_moveit2
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Run

Simulation only (mock hardware, no robot needed):

```bash
ros2 launch so_arm_config real_hardware.launch.py mode:=mock
```

Real robot:

```bash
ros2 launch so_arm_config real_hardware.launch.py mode:=real
```

Drag the interactive marker in RViz, then click **Plan & Execute**. The real arm follows the planned trajectory.

Run the startup sequence once `move_group` is up:

```bash
ros2 run so_arm_101_startup startup_sequence
```

## Calibration

Servo IDs 1 to 6 map to `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` and `gripper`. Each joint has an `offset` (raw encoder ticks at the URDF zero pose, 4096 ticks per revolution) and a `dir` (+1 or -1) in `so_arm_config/config/so101_new_calib.ros2_control.xacro`.

To measure new offsets:

1. Launch `ros2 launch so_arm_101_description display_robot.launch.py`. This starts `motor_node` with torque off, plus the joint slider GUI and RViz.
2. Set all sliders to 0 and move the real arm by hand into the same pose.
3. Read the raw servo positions:
   ```bash
   ros2 topic pub -1 /orientation std_msgs/msg/String "{data: 'Position'}"
   ```
   The values appear in the `motor_node` log.
4. Enter the values as `offset` in the xacro file and rebuild.

**Tip:** Before launching with `mode:=real`, place the arm near the zero pose. When torque turns on, the servos hold the current position. If the arm has sagged outside the joint limits, MoveIt rejects the start state.

## License

See the `<license>` tag in each package's `package.xml`. `so_arm_101_description` includes an Apache 2.0 `LICENSE` file.
