<img width="689" alt="Project Banner" src="https://github.com/abisliouk/haptic_glove_robot_arm/blob/main/demo/Project%20banner.jpeg">

# Haptic Glove to Simulated Robot Arm Interface

## Project Summary

This project enables **real-time teleoperation** of a simulated robotic arm using a **haptic glove**. The glove transmits spatial and finger position data over **Wi-Fi**, which is consumed by a **ROS 2 node** that controls a **simulated robot arm in PyBullet**. The architecture supports full simulation, testing on a single PC without ROS, or mocking the glove input.

---

## Team Members
- **Seonbeom Kim**
- **Artem Bisliouk**
- **Yongkyoon Park**
- **Byeongkwan Jeon**

---


## 🗂️ Repository Structure

```
haptic_glove_robot_arm/
├── haptic_glove_robot_arm/          # ROS 2 package (main integration)
│   ├── glove_publisher.py           # ROS node: receives glove data via socket, publishes to /glove_data
│   ├── arm_subscriber.py           # ROS node: subscribes to /glove_data, simulates arm in PyBullet
│
├── haptic_glove_robot_arm_mock/     # Standalone scripts (no ROS)
│   ├── glove_arm_simulator_no_ros.py # Direct glove-to-arm simulation via serial
│   ├── glove_publisher_mock.py      # Mock publisher node for testing without hardware
│
├── haptic_glove_setup/              # Setup scripts for glove device
│   ├── glove_arduino_setup.ino      # Arduino firmware for glove
│   ├── glove_data_wifi_transition.py # Reads serial glove data, sends to ROS node over Wi-Fi
│
├── test/                            # Code quality tests
├── demo/                            # Folder with the banner of the project and a demo
├── resource/, LICENSE, setup.py, etc.
```

---

## ⚙️ System Architecture & Workflow

### 🧪 Option A: Full System (Two PCs)
1. **PC 1** (USB-Glove connected):
   - Flash glove with `glove_arduino_setup.ino`
   - Run `glove_data_wifi_transition.py` to send glove data over Wi-Fi

2. **PC 2** (ROS 2 system):
   - Start `glove_publisher.py` to receive glove data over socket and publish to `/glove_data`
   - Start `arm_subscriber.py` to control the simulated robot arm via PyBullet

---

### 🔬 Option B: Standalone Simulation (One PC, no ROS)
- Run `glove_arm_simulator_no_ros.py`
- Reads glove data from serial, directly controls simulated robot arm

---

### 🧪 Option C: Mocked Simulation (ROS + Mock)
- Replace glove input by launching:
  - `glove_publisher_mock.py` (simulates glove data)
  - `arm_subscriber.py` as usual

---

## 🧰 Hardware

### 1. **Hiwonder Wireless Glove**
- Used for capturing **finger flex** and **tilt orientation** (pitch, roll)
- Connected via **USB serial** to the first PC (setup device)
- Flashed with `glove_arduino_setup.ino` to stream data in structured format
- Transmits data to another PC over Wi-Fi using `glove_data_wifi_transition.py`

### 2. **PC 1 – Glove Interface Host**
- Must support USB serial communication (e.g., `/dev/ttyUSB0` or `COM5`)
- Runs the glove setup and Wi-Fi forwarding script
- Sends sensor data to PC 2 over TCP/IP

### 3. **PC 2 – Simulation Host**
- Must have **ROS 2 (Humble)** and **PyBullet** installed
- Receives data from PC 1 and publishes it via `glove_publisher.py`
- Subscribes and visualizes behavior via `arm_subscriber.py` using the **Franka Panda** model in PyBullet

### 4. **(Optional) Single-PC Setup**
- If using `glove_arm_simulator_no_ros.py`, only one PC is needed that supports USB serial input and runs PyBullet
- Suitable for testing glove response without ROS

## 🛠️ Installation & Setup

### 1. Clone the Repo
```bash
cd ~/ros2_ws/src
git clone <your_repo_url> haptic_glove_robot_arm
```
where `ros2_ws` is your set up ros2 workspace

### 2. Install Dependencies
```bash
sudo apt update
sudo apt install ros-humble-rclpy python3-pybullet python3-serial
pip install pybullet pyserial
```

### 3. Build the ROS Package
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🚀 How to Run

### Option A: Two-PC Glove-to-Robot Pipeline

**PC 1 – Glove Serial to Wi-Fi**
```bash
cd haptic_glove_setup
python3 glove_data_wifi_transition.py
```

**PC 2 – ROS Receiver**
```bash
# Terminal 1
ros2 run haptic_glove_robot_arm glove_publisher

# Terminal 2
ros2 run haptic_glove_robot_arm arm_subscriber
```

---

### Option B: Run Entire Simulation on One PC (No ROS)
```bash
python3 haptic_glove_robot_arm_mock/glove_arm_simulator_no_ros.py
```

---

### Option C: Run ROS Simulation with Mocked Glove
```bash
# Terminal 1
ros2 run haptic_glove_robot_arm_mock glove_publisher_mock

# Terminal 2
ros2 run haptic_glove_robot_arm arm_subscriber
```

---

## 📡 Communication Flow

1. `glove_data_wifi_transition.py` reads serial → sends over TCP socket
2. `glove_publisher.py` accepts socket → publishes `/glove_data`
3. `arm_subscriber.py` subscribes → moves robot joints in PyBullet

---

## 🧪 Testing

### View Published Glove Data
```bash
ros2 topic echo /glove_data
```

### Adjust Gripper Logic
- Tune `grip_val` and `gain` in `arm_subscriber.py` for desired responsiveness

---

## 📌 Notes
- Ensure both PCs are on the same network
- Serial port (e.g., `COM5`) must match your system
- You can extend to more fingers, gestures, or IMU feedback with minor edits

---

## 🔗 Resources
- [Hiwonder Wireless Glove](https://www.hiwonder.com/products/wireless-glove-open-source-somatosensory-mechanical-glove)
- [PyBullet Docs](https://pybullet.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)

---

## 📄 License

This project is licensed under the MIT License.
