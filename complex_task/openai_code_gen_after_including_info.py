import math
import time
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data

p.connect(p.GUI) #or p.GUI for graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

plane_id = p.loadURDF("plane.urdf")
kuka_id = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000, -0.200000, 0.600000, 0.000000, 0.000000, 0.000000, 1.000000)
kuka_gripper_id = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.2, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
cube_id = p.loadURDF("cube.urdf", basePosition=[0.85, -0.5, 0.65], globalScaling=0.05)
drawer_id = p.loadURDF("../assets/drawer/drawer_with_tray_inside.urdf", basePosition=[0.85, 0.2, 0.65], globalScaling=0.2)

# attach gripper to kuka arm
kuka_cid = p.createConstraint(kuka_id, 6, kuka_gripper_id, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
kuka_cid2 = p.createConstraint(kuka_gripper_id, 4, kuka_gripper_id, 6, jointType=p.JOINT_GEAR, jointAxis=[1,1,1], parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(kuka_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)

def move_to_target(kuka_id, target_pos, target_ori, num_joints=7):
    joint_poses = p.calculateInverseKinematics(kuka_id, num_joints - 1, target_pos, target_ori)
    for i in range(num_joints):
        p.setJointMotorControl2(kuka_id, i, p.POSITION_CONTROL, joint_poses[i])
    for _ in range(100):
        p.stepSimulation()

def control_gripper(gripper_id, open=True):
    position = 0.04 if open else 0.0  # Open or close gripper
    p.setJointMotorControl2(gripper_id, 4, p.POSITION_CONTROL, position)
    p.setJointMotorControl2(gripper_id, 6, p.POSITION_CONTROL, -position)
    for _ in range(50):
        p.stepSimulation()

def control_drawer(drawer_id, open=True):
    position = -0.2 if open else 0.0  # Move drawer out or back in
    p.setJointMotorControl2(drawer_id, 0, p.POSITION_CONTROL, position)
    for _ in range(100):
        p.stepSimulation()

# Open the drawer
control_drawer(drawer_id, open=True)

# Move above the cube
move_to_target(kuka_id, [0.85, -0.5, 0.7], p.getQuaternionFromEuler([0, np.pi, 0]))

# Pick up the cube
control_gripper(kuka_gripper_id, open=True)
move_to_target(kuka_id, [0.85, -0.5, 0.65], p.getQuaternionFromEuler([0, np.pi, 0]))
control_gripper(kuka_gripper_id, open=False)

# Move cube to drawer
move_to_target(kuka_id, [0.85, 0.2, 0.7], p.getQuaternionFromEuler([0, np.pi, 0]))
move_to_target(kuka_id, [0.85, 0.2, 0.65], p.getQuaternionFromEuler([0, np.pi, 0]))
control_gripper(kuka_gripper_id, open=True)

# Retract arm and close drawer
move_to_target(kuka_id, [0.85, 0.2, 0.7], p.getQuaternionFromEuler([0, np.pi, 0]))
control_drawer(drawer_id, open=False)
