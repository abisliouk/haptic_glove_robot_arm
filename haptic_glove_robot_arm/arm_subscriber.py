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
        self.subscription  # prevent unused variable warning

        # PyBullet initialization
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")
        self.tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])
        self.pandaUid = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, -0.1], useFixedBase=True)
        self.colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06])
        self.visBoxId = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06], rgbaColor=[1, 0, 0, 1])
        self.objectUid = p.createMultiBody(
            baseMass=0.1,
            baseCollisionShapeIndex=self.colBoxId,
            baseVisualShapeIndex=self.visBoxId,
            basePosition=[0.7, 0, 0.1]
        )
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=0,
            cameraPitch=-40,
            cameraTargetPosition=[0.55, -0.35, 0.2]
        )

        self.controlled_joints = [0, 1, 2, 3, 5, 6]
        self.sliders = {}
        for joint_id in self.controlled_joints:
            if joint_id not in [0, 1]:  # joint 0, 1 controlled by tilt
                joint_name = p.getJointInfo(self.pandaUid, joint_id)[1].decode('utf-8')
                self.sliders[joint_id] = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0.0)
                p.changeDynamics(self.pandaUid, joint_id, linearDamping=0.04, angularDamping=0.04)

        self.gain = 1.5
        self.grip_val = 0.04  # base: open
        self.object_constraint = None
        self.initial_pitch = 0.0
        self.initial_roll = 0.0

    def listener_callback(self, msg):
        line = msg.data
        fingers, pitch, roll = self.parse_glove_data(line)

        # Calculate relative tilt
        relative_pitch = pitch - self.initial_pitch
        relative_roll = roll - self.initial_roll

        target_pos, _ = p.getBasePositionAndOrientation(self.objectUid)
        gripper_state = p.getLinkState(self.pandaUid, 8)
        gripper_pos = gripper_state[0]

        distance = math.sqrt(
            (target_pos[0] - gripper_pos[0]) ** 2 +
            (target_pos[1] - gripper_pos[1]) ** 2 +
            (target_pos[2] - gripper_pos[2]) ** 2
        )

        hand_closed = all(fingers[i] >= 50 for i in [0, 1, 2, 3])
        hand_opened = all(fingers[i] < 50 for i in [0, 1, 2, 3])

        if hand_closed and distance < 1:
            self.grip_val = 0.0  # Close gripper
        elif hand_opened:
            self.grip_val = 0.05  # Open gripper

        p.setJointMotorControl2(self.pandaUid, 9, p.POSITION_CONTROL, self.grip_val, force=10)
        p.setJointMotorControl2(self.pandaUid, 10, p.POSITION_CONTROL, self.grip_val, force=10)

        if self.grip_val == 0.0 and self.object_constraint is None:
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

        if self.grip_val == 0.05 and self.object_constraint is not None:
            p.removeConstraint(self.object_constraint)
            self.object_constraint = None

        # Control logic for joints
        if fingers[1] < 50 and all(fingers[i] >= 50 for i in [0, 2, 3]):
            self.control_joint(0, 1, relative_pitch, relative_roll)
        elif fingers[1] < 50 and fingers[2] < 50 and all(fingers[i] >= 50 for i in [0, 3]):
            self.control_joint(2, 3, relative_pitch, relative_roll)
        else:
            self.control_joint(5, 6, relative_pitch, relative_roll)

    def parse_glove_data(self, line):
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

    def control_joint(self, joint_a, joint_b, relative_pitch, relative_roll):
        pitch_rad = max(min(relative_pitch / 30.0 * self.gain, 10), -10)
        roll_rad = max(min(relative_roll / 30.0 * self.gain, 1.0), -1.0)

        current_a = p.getJointState(self.pandaUid, joint_a)[0]
        current_b = p.getJointState(self.pandaUid, joint_b)[0]

        new_a = (1 - 0.5) * current_a + 0.5 * pitch_rad
        new_b = (1 - 0.5) * current_b + 0.5 * roll_rad

        p.resetJointState(self.pandaUid, joint_a, new_a)
        p.resetJointState(self.pandaUid, joint_b, new_b)

def main(args=None):
    rclpy.init(args=args)
    node = ArmSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()