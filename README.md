# Interfacing Haptic Glove and Simulated Robotic Arm

## Project Overview
This project is part of the **EEL 5934 - Autonomous Robots** course at the University of Florida. Our goal is to interface a **Hiwonder Wireless Glove** with a simulated **Hiwonder MaxArm Robot Arm** in **Gazebo**. The glove will serve as an input device to manipulate the simulated robot arm via ROS 2.

## Team Members
- **Artem Bisliouk**
- **Seonbeom Kim**
- **Yongkyoon Park**
- **Byeongkwan Jeon**

## Hardware and Simulation
- **Hiwonder Wireless Glove** (connected via USB)
- **Hiwonder MaxArm Robot Arm** (simulated in Gazebo due to hardware malfunction)
- The automatic connection modules were removed — full ROS 2-based integration is required.

## Project Goal
- Use glove inputs to control the simulated robot arm in real time.
- Translate gestures (e.g., hand movement, flex sensors) into robot joint commands.
- Ensure bidirectional communication using custom ROS 2 nodes.

## Project Structure
```
hiwonder_ws/
├── src/
│   ├── glove_driver/               # Reads and publishes glove sensor data
│   ├── maxarm_sim/                 # URDF, xacro, and Gazebo world configuration
│   ├── glove_to_arm_controller/   # Converts glove data into robot arm commands
```

## ROS 2 Implementation Plan

### 1. Glove Driver Node
- Reads glove sensor data over USB serial.
- Parses and publishes data to `/glove/data`.

### 2. MaxArm Simulation
- Load URDF/xacro robot model in Gazebo.
- Use ROS 2 controllers (e.g., `JointTrajectoryController`) for controlling joints.

### 3. Gesture to Command Mapping
- ROS 2 node subscribes to `/glove/data`.
- Converts input to joint command messages.
- Publishes to `/arm_controller/command`.

### 4. Calibration and Testing
- Calibrate flex/IMU data to joint ranges.
- Visualize data with `rqt_plot`, `rviz2`, or `ros2 topic echo`.

### 5. Optional Enhancements
- Implement gesture recognition (e.g., fist = reset).
- Feedback loop to adjust motion based on real-time simulation.

## Requirements
- ROS 2 (preferably Humble or Iron)
- Gazebo Classic or Gazebo Fortress
- Python 3.8+ or C++17 support for nodes
- Serial communication library (e.g., `pyserial` for Python)

## Resources
- [Hiwonder Wireless Glove](https://www.hiwonder.com/products/wireless-glove-open-source-somatosensory-mechanical-glove?variant=40936077590615)
- [Hiwonder MaxArm](https://www.hiwonder.com/products/maxarm?variant=40008714092631)
- [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [Gazebo Documentation](https://classic.gazebosim.org/tutorials)

