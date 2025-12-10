# Gijitaiken (疑似体験)
Gijitaiken (疑似体験, simulated experience) is a ROS2 package that aims to simulate robots from real life to a simulator and vice versa using the MuJoCo simulator.

## Features
* Emulate real hardware compatible with `aruku` and `tachimawari` packages.
* Provide synthetic IMU and joint state feedback for closed-loop control.
* Broadcast ground truth odometry for strategy debugging.
* Enable interactive physics manipulation via native MuJoCo viewer.
* Control robot manually using keyboard teleoperation.

## Requirements
Ensure that the following packages are already in your workspace:
* mujoco
* numpy
* pynput
* aruku_interfaces
* tachimawari_interfaces
* kansei_interfaces

## Installation
```bash
$ colcon build --packages-select gijitaiken --symlink-install
$ source install/setup.bash
````

## Usage
**1. Run Simulator:**

```bash
$ ros2 run gijitaiken start
```

**2. Run Teleoperation (Optional):**
To control the robot manually using a keyboard, run in a separate terminal:

```bash
$ ros2 run gijitaiken teleop
```

  * **Controls:** `W`/`S` (Move X), `A`/`D` (Move Y), `Q`/`E` (Turn).

## Configuration

**config/joint\_map.yaml**

This file maps motor IDs to actuator names in MuJoCo.

> **Important:** If you want to change the robot model (XML), make sure the joint names in this file match the actuator names in the XML.

## ROS2 Interface

### Subscriber

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/joint/set_joints` | `tachimawari_interfaces/SetJoints` | Receives servo position targets from the Aruku package |

### Publisher

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/imu/unit` | `kansei_interfaces/Unit` | Sends Gyro & Accelerometer data for balance |
| `/walking/set_odometry` | `aruku_interfaces/Point2` | Sends robot X,Y position data from the simulator |
| `/joint_states` | `sensor_msgs/JointState` | Sends real-time joint angle data |
| `/walking/set_walking` | `aruku_interfaces/SetWalking` | (Optional) Sends keyboard input to aruku |