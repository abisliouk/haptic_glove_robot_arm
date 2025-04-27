# Interfacing Haptic Glove and Simulated Robotic Arm

## Project Overview
This project is part of the **EEL 5934 - Autonomous Robots** course at the University of Florida. Our goal is to interface a **Hiwonder Wireless Glove** with a simulated **Hiwonder MaxArm Robot Arm** in **PyBullet** using ROS 2. The glove serves as an input device to manipulate the simulated robot arm via ROS 2 nodes.

## Team Members
- **Artem Bisliouk**
- **Seonbeom Kim**
- **Yongkyoon Park**
- **Byeongkwan Jeon**

## Hardware and Simulation
- **Hiwonder Wireless Glove** (connected via USB)
- **Hiwonder MaxArm Robot Arm** (simulated in PyBullet due to hardware malfunction)
- Full ROS 2-based integration for real-time communication between the glove and the robot arm.

## Project Goal
- Use glove inputs to control the simulated robot arm in real time.
- Translate gestures (e.g., hand movement, flex sensors) into robot joint commands.
- Ensure bidirectional communication using custom ROS 2 nodes.

## Project Structure
```
ros2_ws/
├── src/
│   ├── haptic_glove_robot_arm/     # ROS 2 package for glove and arm integration
│       ├── glove_publisher.py      # Publishes glove sensor data
│       ├── arm_subscriber.py       # Subscribes to glove data and controls the robot arm
│       ├── package.xml             # ROS 2 package metadata
│       ├── setup.py                # Python package setup
```

## ROS 2 Implementation

### 1. Glove Publisher Node
- **File**: `glove_publisher.py`
- **Functionality**: Reads glove sensor data over USB serial and publishes it to the `/glove_data` topic.
- **Topic**: `/glove_data` (type: `std_msgs/String`)

### 2. Arm Subscriber Node
- **File**: `arm_subscriber.py`
- **Functionality**: Subscribes to `/glove_data`, parses the data, and controls the robot arm in PyBullet.
- **Topic**: `/glove_data` (type: `std_msgs/String`)

### 3. PyBullet Simulation
- The robot arm is simulated in PyBullet.
- The glove data is used to control the arm's joints and gripper in real time.

## Installation and Setup

### Prerequisites
- ROS 2 (preferably Humble or Iron)
- Python 3.8+ with `pyserial` and `pybullet` installed
- A working ROS 2 workspace (`ros2_ws`)

### Steps
1. **Clone the Repository**:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url> haptic_glove_robot_arm
   ```

2. **Install Dependencies**:
   Ensure you have the required Python libraries:
   ```bash
   pip install pyserial pybullet
   ```

3. **Build the Package**:
   Navigate to your ROS 2 workspace and build the package:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

4. **Run the Nodes**:
   Open two terminals and run the following commands:

   - **Terminal 1**: Start the glove publisher node:
     ```bash
     ros2 run haptic_glove_robot_arm glove_publisher
     ```

   - **Terminal 2**: Start the arm subscriber node:
     ```bash
     ros2 run haptic_glove_robot_arm arm_subscriber
     ```

5. **Verify the Simulation**:
   - The PyBullet GUI should open, showing the simulated robot arm.
   - Move the glove to see the robot arm respond in real time.

## Testing and Debugging

### Debugging Glove Data
- Use `ros2 topic echo` to verify the glove data being published:
  ```bash
  ros2 topic echo /glove_data
  ```

### Visualizing Joint States
- Use the PyBullet GUI to observe joint movements and gripper actions.

### Adjusting Parameters
- Modify the `gain` or other parameters in `arm_subscriber.py` to fine-tune the robot's responsiveness.

## Future Enhancements
- Add gesture recognition (e.g., fist = reset arm position).
- Implement a feedback loop to adjust motion based on real-time simulation.
- Integrate with Gazebo for a more realistic simulation environment.

## Requirements
- ROS 2 (Humble or Iron)
- Python 3.8+ with `pyserial` and `pybullet`
- A USB connection for the Hiwonder Wireless Glove

## Resources
- [Hiwonder Wireless Glove](https://www.hiwonder.com/products/wireless-glove-open-source-somatosensory-mechanical-glove?variant=40936077590615)
- [Hiwonder MaxArm](https://www.hiwonder.com/products/maxarm?variant=40008714092631)
- [ROS 2 Documentation](https://docs.ros.org/en/rolling/index.html)
- [PyBullet Documentation](https://pybullet.org/wordpress/)