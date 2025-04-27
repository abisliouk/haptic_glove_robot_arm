import pybullet as p
import pybullet_data
import time
import math
import serial

# ✅ glove sirial
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # 시리얼 안정화

def read_tilt_from_glove():
    try:
        line = ser.readline().decode().strip()
        if line.startswith("TILT:") or "|TILT:" in line:
            tilt_part = line.split("TILT:")[-1]
            pitch_str, roll_str = tilt_part.split(",")
            pitch = float(pitch_str)
            roll = float(roll_str)
            return pitch, roll
    except:
        pass
    return 0.0, 0.0

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

controlled_joints = [0, 1, 3, 5]
sliders = {}

# make slider (only joint 3, 5) <- we have to change slider -> glove
for joint_id in controlled_joints:
    if joint_id not in [0, 1]:  # joint 0,1 is controled by glove
        joint_name = p.getJointInfo(pandaUid, joint_id)[1].decode('utf-8')
        sliders[joint_id] = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0.0)
        p.changeDynamics(pandaUid, joint_id, linearDamping=0.04, angularDamping=0.04)

gripper_slider = p.addUserDebugParameter("Gripper", 0.0, 0.04, 0.04)

# ✅ simulation loop
while True:
    p.stepSimulation()
    time.sleep(1. / 240.)

    # === glove tilt (joint 0, 1)
    pitch, roll = read_tilt_from_glove()
    pitch_rad = max(min(pitch / 90.0 * math.pi, math.pi), -math.pi)
    roll_rad = max(min(roll / 90.0 * math.pi, math.pi), -math.pi)

    p.resetJointState(pandaUid, 1, roll_rad)   # joint 0 ← roll
    p.resetJointState(pandaUid, 0, pitch_rad)  # joint 1 ← pitch

    # === slide joint control
    for joint_id in [3, 5]:
        target = p.readUserDebugParameter(sliders[joint_id])
        current = p.getJointState(pandaUid, joint_id)[0]
        interp = (1 - 0.2) * current + 0.2 * target
        p.resetJointState(pandaUid, joint_id, interp)

    # === gripper
    grip_val = p.readUserDebugParameter(gripper_slider)
    p.setJointMotorControl2(pandaUid, 9, p.POSITION_CONTROL, grip_val, force=10)
    p.setJointMotorControl2(pandaUid, 10, p.POSITION_CONTROL, grip_val, force=10)
