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
pandaUid = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
objectUid = p.loadURDF("random_urdfs/000/000.urdf", basePosition=[0.7, 0, 0.1])

p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=0,
    cameraPitch=-40,
    cameraTargetPosition=[0.55, -0.35, 0.2]
)

# ✅ joint&slider
controlled_joints = [0, 1, 3, 4, 5]
sliders = {}
for joint_id in controlled_joints:
    if joint_id not in [0, 1]:  # joint 0, 1 controlled by tilt
        joint_name = p.getJointInfo(pandaUid, joint_id)[1].decode('utf-8')
        sliders[joint_id] = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0.0)
        p.changeDynamics(pandaUid, joint_id, linearDamping=0.04, angularDamping=0.04)

# ✅ tilt sensitivity & init gripper status
gain = 2.0
grip_val = 0.04  # base: open

# ✅ simulation loop
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)

    fingers, pitch, roll = read_glove_data()

    #  # === Control gripper (fingers 1~4, responds when all fingers are extended or all are bent)
    if all(fingers[i] <= 50 for i in range(4)):
        grip_val = 0.0  # close
    elif all(fingers[i] > 50 for i in range(4)):
        grip_val = 0.04  # open

    p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, grip_val, force=10)
    p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, grip_val, force=10)

    # === only second finger extended → only joint 4 contoll by slider
    if fingers[1] < 50 and all(fingers[i] >= 50 for i in [0, 2, 3]):
        target = p.readUserDebugParameter(sliders[4])
        current = p.getJointState(pandaUid, 4)[0]
        interp = (1 - 0.2) * current + 0.2 * target
        p.resetJointState(pandaUid, 4, interp)
    else:
        # === basic: joint 0, 1 contolled with tilt
        pitch_rad = max(min(pitch / 30.0 * gain, 1.5), -1.5)
        roll_rad  = max(min(roll  / 30.0 * gain, 1.5), -1.5)

        current_0 = p.getJointState(pandaUid, 0)[0]
        current_1 = p.getJointState(pandaUid, 1)[0]

        new_0 = (1 - 0.2) * current_0 + 0.2 * pitch_rad
        new_1 = (1 - 0.2) * current_1 + 0.2 * roll_rad

        p.resetJointState(pandaUid, 0, new_0)
        p.resetJointState(pandaUid, 1, new_1)

    # === slider controll for joint 3, 5 
    for joint_id in [3, 5]:
        target = p.readUserDebugParameter(sliders[joint_id])
        current = p.getJointState(pandaUid, joint_id)[0]
        interp = (1 - 0.2) * current + 0.2 * target
        p.resetJointState(pandaUid, joint_id, interp)
