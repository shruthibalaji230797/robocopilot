import math
import time
import pybullet as p
import pybullet_data

# Connect to PyBullet
p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Load the plane and objects
plane_id = p.loadURDF("plane.urdf")
kuka_id = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.4, -0.2, 0.6, 0, 0, 0, 1)
kuka_gripper_id = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.2, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
cube_id = p.loadURDF("cube.urdf", basePosition=[0.85, -0.5, 0.65], globalScaling=0.05)
drawer_id = p.loadURDF("../assets/drawer/drawer_with_tray_inside.urdf", basePosition=[0.85, 0.2, 0.65], globalScaling=0.2)

# Attach gripper to Kuka arm
kuka_cid = p.createConstraint(kuka_id, 6, kuka_gripper_id, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
kuka_cid2 = p.createConstraint(kuka_gripper_id, 4, kuka_gripper_id, 6, jointType=p.JOINT_GEAR, jointAxis=[1, 1, 1], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(kuka_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)

# Function to open the drawer
def open_drawer():
    p.setJointMotorControl2(drawer_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.5)  # Adjust joint index as needed
    time.sleep(1)

# Function to close the drawer
def close_drawer():
    p.setJointMotorControl2(drawer_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0)  # Adjust joint index as needed
    time.sleep(1)

# Function to move the robot to a specific position using inverse kinematics
def move_robot_to(position):
    # Get the end effector link index
    end_effector_index = 6  # Kuka's end effector index
    joint_positions = p.calculateInverseKinematics(kuka_id, end_effector_index, position)
    p.setJointMotorControlArray(kuka_id, jointIndices=range(7), controlMode=p.POSITION_CONTROL, targetPositions=joint_positions)
    time.sleep(1)

# Function to pick up the cube
def pick_up_cube():
    # Move the robot to the cube's position
    move_robot_to([0.85, -0.5, 0.65])  # Position above the cube
    p.setJointMotorControl2(kuka_gripper_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.1)  # Close gripper
    time.sleep(1)

# Function to place the cube inside the drawer
def place_cube_in_drawer():
    # Move the robot to the drawer's position
    move_robot_to([0.85, 0.2, 0.65])  # Position above the drawer
    p.setJointMotorControl2(kuka_gripper_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.0)  # Open gripper
    time.sleep(1)

# Main task sequence
try:
    # Open the drawer
    open_drawer()

    # Pick up the cube
    pick_up_cube()

    # Place the cube inside the drawer
    place_cube_in_drawer()

    # Close the drawer
    close_drawer()

finally:
    p.disconnect()