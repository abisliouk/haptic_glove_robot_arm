# This script is a ROS 2 node that subscribes to glove data and controls a robotic arm in a PyBullet simulation.
# It uses the PyBullet physics engine to simulate the arm and an object, and it applies forces to the arm's joints based on the glove data.
# The glove data includes finger positions and tilt angles, which are parsed and used to control the arm's movements.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pybullet as p
import pybullet_data
import time
import math

class ArmSubscriber(Node):
    def __init__(self):
        super().__init__('arm_subscriber')
        self.subscription = self.create_subscription(String, 'glove_data', self.listener_callback, 10)

        # PyBullet init
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        self.tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])
        self.pandaUid = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06])
        visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06], rgbaColor=[1, 0, 0, 1])
        self.objectUid = p.createMultiBody(baseMass=3, baseCollisionShapeIndex=colBoxId, baseVisualShapeIndex=visBoxId, basePosition=[0.7, 0, 0.1])
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55, -0.35, 0.2])

        self.controlled_joints = [0, 1, 2, 3, 5, 6]
        for joint_id in self.controlled_joints:
            if joint_id not in [0, 1]:
                p.changeDynamics(self.pandaUid, joint_id, linearDamping=0.04, angularDamping=0.04)

        self.gain = 2.5
        self.grip_val = 0.04
        self.object_constraint = None
        self.initial_pitch = None
        self.initial_roll = None

        # State machine
        self.mode = 'idle'
        self.fingers = [0] * 5
        self.pitch = 0.0
        self.roll = 0.0

        self.create_timer(0.05, self.control_loop)

    def listener_callback(self, msg):
        self.fingers, self.pitch, self.roll = self.parse_glove_data(msg.data)

        if self.initial_pitch is None:
            self.initial_pitch = self.pitch
            self.initial_roll = self.roll

        rel_pitch = self.pitch - self.initial_pitch
        rel_roll = self.roll - self.initial_roll

        target_pos, _ = p.getBasePositionAndOrientation(self.objectUid)
        gripper_pos = p.getLinkState(self.pandaUid, 8)[0]
        distance = math.sqrt(sum([(a - b) ** 2 for a, b in zip(target_pos, gripper_pos)]))

        hand_closed = all(self.fingers[i] >= 50 for i in range(4))
        hand_opened = all(self.fingers[i] < 50 for i in range(4))

        if hand_closed and distance < 0.2:
            self.grip_val = 0.0
        elif hand_opened:
            self.grip_val = 0.05

        p.setJointMotorControl2(self.pandaUid, 9, p.POSITION_CONTROL, self.grip_val, force=10)
        p.setJointMotorControl2(self.pandaUid, 10, p.POSITION_CONTROL, self.grip_val, force=10)

        if self.grip_val == 0.0 and self.object_constraint is None and distance < 0.2:
            self.object_constraint = p.createConstraint(
                parentBodyUniqueId=self.pandaUid,
                parentLinkIndex=8,
                childBodyUniqueId=self.objectUid,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=[0, 0, 0]
            )
            p.changeConstraint(self.object_constraint, maxForce=500, erp=0.9)

        elif self.grip_val > 0.04 and self.object_constraint is not None:
            p.removeConstraint(self.object_constraint)
            self.object_constraint = None

        # Mode switching
        if all(f >= 50 for f in self.fingers):
            self.mode = 'idle'
        elif self.fingers[1] < 50 and all(self.fingers[i] >= 50 for i in [0, 2, 3]):
            self.mode = 'mode_0_1'
        elif self.fingers[1] < 50 and self.fingers[2] < 50 and all(self.fingers[i] >= 50 for i in [0, 3]):
            self.mode = 'mode_2_3'
        else:
            self.mode = 'mode_5_6'

    def control_loop(self):
        rel_pitch = self.pitch - self.initial_pitch
        rel_roll = self.roll - self.initial_roll

        if self.mode == 'mode_0_1':
            self.control_joint_mode(0, 1, rel_pitch, rel_roll)
        elif self.mode == 'mode_2_3':
            self.control_joint_mode(2, 3, rel_pitch, rel_roll)
        elif self.mode == 'mode_5_6':
            self.control_joint_continuous(5, 6, rel_pitch, rel_roll)

        p.stepSimulation()

    def control_joint_mode(self, joint_a, joint_b, rel_pitch, rel_roll):
        pitch_rad = max(min(rel_pitch / 30.0 * self.gain, 10), -10)
        roll_rad = max(min(rel_roll / 30.0 * self.gain, 1.0), -1.0)

        current_a = p.getJointState(self.pandaUid, joint_a)[0]
        current_b = p.getJointState(self.pandaUid, joint_b)[0]

        new_a = (1 - 0.1) * current_a + 0.1 * pitch_rad
        new_b = (1 - 0.1) * current_b + 0.1 * roll_rad

        p.resetJointState(self.pandaUid, joint_a, new_a)
        p.resetJointState(self.pandaUid, joint_b, new_b)

    def control_joint_continuous(self, joint_a, joint_b, rel_pitch, rel_roll):
        pitch_rad = max(min(rel_pitch / 30.0 * self.gain, 10), -10)
        roll_rad = max(min(rel_roll / 30.0 * self.gain, 10), -10)

        current_a = p.getJointState(self.pandaUid, joint_a)[0]
        current_b = p.getJointState(self.pandaUid, joint_b)[0]

        new_a = (1 - 0.1) * current_a + 0.1 * pitch_rad
        new_b = (1 - 0.1) * current_b + 0.1 * roll_rad

        p.resetJointState(self.pandaUid, joint_a, new_a)
        p.resetJointState(self.pandaUid, joint_b, new_b)

    def parse_glove_data(self, line):
        fingers = [0] * 5
        pitch = roll = 0.0
        try:
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
        except:
            pass
        return fingers, pitch, roll

def main(args=None):
    rclpy.init(args=args)
    node = ArmSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()