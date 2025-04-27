import pybullet as p
import pybullet_data
import time
import math
import socket

# === socket server setting ===
server_ip = '127.0.0.1'
server_port = 9999
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((server_ip, server_port))
server_socket.listen(1)

print(f"Waiting for connection on port {server_port}...")
client_socket, addr = server_socket.accept()
print(f"Connected by {addr}")

# ✅ PyBullet init
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.1])

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=0,
    cameraPitch=-40,
    cameraTargetPosition=[0.55, -0.35, 0.2]
)

# ✅ joint init
controlled_joints = [0, 1, 2, 3, 5, 6]
sliders = {}
for joint_id in controlled_joints:
    if joint_id not in [0, 1]:
        joint_name = p.getJointInfo(pandaUid, joint_id)[1].decode('utf-8')
        sliders[joint_id] = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0.0)
        p.changeDynamics(pandaUid, joint_id, linearDamping=0.04, angularDamping=0.04)

# ✅ tilt sensitivity & init grip
gain = 3.0
grip_val = 0.04  # base: open
first_data = True  # initial pitch/roll record plag

# ✅ buffer
buffer = ""

# ✅ simulation loop
try:
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)

        # === Receiving data and handling buffer ===
        data = client_socket.recv(1024)
        if not data:
            break
        buffer += data.decode()

        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            decoded_data = line.strip()
            if not decoded_data:
                continue

            print(f"Received line: {decoded_data}")

            # === receiving data parsing ===
            fingers = [0] * 5
            pitch = roll = 0.0

            if "FINGER:" in decoded_data:
                finger_part = decoded_data.split("FINGER:")[-1].split("|")[0]
                for f in finger_part.split(","):
                    if ":" in f:
                        idx, val = f.split(":")
                        fingers[int(idx) - 1] = int(val)

            if "TILT:" in decoded_data:
                tilt_part = decoded_data.split("TILT:")[-1]
                tilt_values = tilt_part.split(",")
                if len(tilt_values) >= 2:
                    pitch_str, roll_str = tilt_values[0], tilt_values[1]
                    pitch = float(pitch_str)
                    roll = float(roll_str)

            # === Store initial reference values from the first data
            if first_data:
                initial_pitch = pitch
                initial_roll = roll
                first_data = False

            # === tilt 
            relative_pitch = pitch - initial_pitch
            relative_roll = roll - initial_roll

            # === controll gripper
            if all(fingers[i] >= 50 for i in range(4)):
                grip_val = 0.0  # close
            elif all(fingers[i] < 50 for i in range(4)):
                grip_val = 0.04  # open

            p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, grip_val, force=10)
            p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, grip_val, force=10)

            # === joint mode by finger pose
            if fingers[1] < 50 and all(fingers[i] >= 50 for i in [0, 2, 3]):
                pitch_rad = max(min(relative_pitch / 30.0 * gain, 15), -15)
                roll_rad  = max(min(relative_roll  / 30.0 * gain, 1.5), -1.5)

                current_0 = p.getJointState(pandaUid, 0)[0]
                current_1 = p.getJointState(pandaUid, 1)[0]

                new_0 = (1 - 0.2) * current_0 + 0.2 * pitch_rad
                new_1 = (1 - 0.2) * current_1 + 0.2 * roll_rad

                p.resetJointState(pandaUid, 0, new_0)
                p.resetJointState(pandaUid, 1, new_1)

            elif fingers[1] < 50 and fingers[2] < 50 and all(fingers[i] >= 50 for i in [0, 3]):
                pitch_rad = max(min(relative_pitch / 30.0 * gain, 15), -15)
                roll_rad  = max(min(relative_roll  / 30.0 * gain, 1.5), -1.5)

                current_2 = p.getJointState(pandaUid, 2)[0]
                current_3 = p.getJointState(pandaUid, 3)[0]

                new_2 = (1 - 0.2) * current_2 + 0.2 * pitch_rad
                new_3 = (1 - 0.2) * current_3 + 0.2 * roll_rad

                p.resetJointState(pandaUid, 2, new_2)
                p.resetJointState(pandaUid, 3, new_3)

            else:
                pitch_rad = max(min(relative_pitch / 30.0 * gain, 15), -15)
                roll_rad  = max(min(relative_roll  / 30.0 * gain, 10), -10)

                current_5 = p.getJointState(pandaUid, 5)[0]
                current_6 = p.getJointState(pandaUid, 6)[0]

                new_6 = (1 - 0.2) * current_6 + 0.2 * pitch_rad
                new_5 = (1 - 0.2) * current_5 + 0.2 * roll_rad

                p.resetJointState(pandaUid, 5, new_5)
                p.resetJointState(pandaUid, 6, new_6)

except KeyboardInterrupt:
    pass

client_socket.close()
server_socket.close()
