import pybullet as p
import pybullet_data
import time
import math
import serial

#  glove serial port(this port may need to changed to fit your laptop)
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)

# ✅ glove data parsing function
def read_glove_data():
    try:
        line = ser.readline().decode().strip()
        fingers = [0] * 5
        pitch = roll = 0.0

        if "FINGER:" in line:
            finger_part = line.split("FINGER:")[-1].split("|")[0]
            for f in finger_part.split(","):
                if ":" in f:
                    idx, val = f.split(":")
                    fingers[int(idx) - 1] = int(val)

        if "TILT:" in line:
            tilt_part = line.split("TILT:")[-1]
            pitch_str, roll_str = tilt_part.split(",")
            pitch = float(pitch_str)
            roll = float(roll_str)

        return fingers, pitch, roll
    except:
        return [0]*5, 0.0, 0.0

# ✅ PyBullet init
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])
pandaUid = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, -0.1], useFixedBase=True)
objectUid = p.loadURDF(
    "cube_small.urdf",
    basePosition=[0.7, 0, 0.05],
    globalScaling=1.5
)
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=0,
    cameraPitch=-40,
    cameraTargetPosition=[0.55, -0.35, 0.2]
)

# ✅ joint&slider
controlled_joints = [0, 1, 2, 3,5,6]
sliders = {}
for joint_id in controlled_joints:
    if joint_id not in [0, 1]:  # joint 0, 1 controlled by tilt
        joint_name = p.getJointInfo(pandaUid, joint_id)[1].decode('utf-8')
        sliders[joint_id] = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0.0)
        p.changeDynamics(pandaUid, joint_id, linearDamping=0.04, angularDamping=0.04)

# ✅ tilt sensitivity & init gripper status
gain = 1.5
grip_val = 0.04  # base: open


# ✅ store initial tilt before simulation loop
fingers, initial_pitch, initial_roll = read_glove_data()

# ✅ simulation loop
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)

    fingers, pitch, roll = read_glove_data()

    # ✅ calculate relative tilt
    relative_pitch = pitch - initial_pitch
    relative_roll = roll - initial_roll

    #  # === Control gripper (fingers 1~4, responds when all fingers are extended or all are bent)
    if all(fingers[i] < 50 for i in [0, 1, 2, 3]):
        grip_val = 0.05  # open
    elif all(fingers[i] >= 50 for i in [0, 1, 2, 3]):
        grip_val = 0  # close

    p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, grip_val, force=10)
    p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, grip_val, force=10)

    if all(fingers[i] < 50 for i in [0, 1, 2, 3]):
        continue
    # === only second finger extended → only joint 4 controlled by slider
    elif fingers[1] < 50 and all(fingers[i] >= 50 for i in [0, 2, 3]):
        
        pitch_rad = max(min(relative_pitch / 30.0 * gain, 10), -10) # rotate
        roll_rad  = max(min(relative_roll  / 30.0 * gain, 1.0), -1.0)

        current_0 = p.getJointState(pandaUid, 0)[0]
        current_1 = p.getJointState(pandaUid, 1)[0]

        new_0 = (1 - 0.5) * current_0 + 0.5 * pitch_rad
        new_1 = (1 - 0.5) * current_1 + 0.5 * roll_rad

        p.resetJointState(pandaUid, 0, new_0)
        p.resetJointState(pandaUid, 1, new_1)
        
    elif fingers[1] < 50 and fingers[2] < 50 and all(fingers[i] >= 50 for i in [0, 3]):
        
        pitch_rad = max(min(relative_pitch / 30.0 * gain, 10), -10) # rotate
        roll_rad  = max(min(relative_roll  / 30.0 * gain, 1.0), -1.0)

        current_2 = p.getJointState(pandaUid, 2)[0]
        current_3 = p.getJointState(pandaUid, 3)[0]

        new_2 = (1 - 0.5) * current_2 + 0.5 * pitch_rad
        new_3 = (1 - 0.5) * current_3 + 0.5 * roll_rad

        p.resetJointState(pandaUid, 2, new_2)
        p.resetJointState(pandaUid, 3, new_3)
        
    else:
        
        pitch_rad = max(min(relative_pitch / 30.0 * gain, 10), -10) # rotate
        roll_rad  = max(min(relative_roll  / 30.0 * gain, 10), -10)

        current_5 = p.getJointState(pandaUid, 5)[0]
        current_6 = p.getJointState(pandaUid, 6)[0]

        new_6 = (1 - 0.5) * current_6 + 0.5 * pitch_rad
        new_5 = (1 - 0.5) * current_5 + 0.5 * roll_rad

        p.resetJointState(pandaUid, 5, new_5)
        p.resetJointState(pandaUid, 6, new_6)
        
