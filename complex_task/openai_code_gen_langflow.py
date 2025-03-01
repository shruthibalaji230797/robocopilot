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
kuka_id = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.400000, -0.200000, 0.600000, 0.000000, 0.000000, 0.000000, 1.000000)
kuka_gripper_id = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.2, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
cube_id = p.loadURDF("cube.urdf", basePosition=[0.85, -0.5, 0.65], globalScaling=0.05)
drawer_id = p.loadURDF("../assets/drawer/drawer_with_tray_inside.urdf", basePosition=[0.85, 0.2, 0.65], globalScaling=0.2)

# Attach gripper to Kuka arm
kuka_cid = p.createConstraint(kuka_id, 6, kuka_gripper_id, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
kuka_cid2 = p.createConstraint(kuka_gripper_id, 4, kuka_gripper_id, 6, jointType=p.JOINT_GEAR, jointAxis=[1, 1, 1], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
p.changeConstraint(kuka_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)

# Function to move the robot to a specific position using inverse kinematics
def move_robot_to(position):
    end_effector_index = 6  # Kuka's end effector index
    joint_positions = p.calculateInverseKinematics(kuka_id, end_effector_index, position)
    p.setJointMotorControlArray(kuka_id, jointIndices=range(7), controlMode=p.POSITION_CONTROL, targetPositions=joint_positions)
    time.sleep(1)

# Task 1: Robot moves to the location of the drawer handle and opens the drawer
def open_drawer():
    # Target pose above the drawer handle
    handle_pose = [0.85, 0.2, 0.75]  # Adjust height as necessary
    move_robot_to(handle_pose)
    # Pull and open the drawer
    p.setJointMotorControl2(drawer_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.5)  # Assume joint index 0 is for opening the drawer
    time.sleep(1)

# Task 2: Robot moves to a location above the cube and picks it up
def pick_up_cube():
    # Move to position above the cube
    cube_pose = [0.85, -0.5, 0.65]  # Position above the cube
    move_robot_to(cube_pose)
    # Close gripper to pick up the cube
    p.setJointMotorControl2(kuka_gripper_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.1)  # Close gripper
    time.sleep(1)

# Task 3: Robot places the cube inside the drawer
def place_cube_in_drawer():
    # Move to position above the drawer
    drawer_pose = [0.85, 0.2, 0.65]  # Position above the drawer
    move_robot_to(drawer_pose)
    # Open gripper to release the cube
    p.setJointMotorControl2(kuka_gripper_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0.0)  # Open gripper
    time.sleep(1)

# Task 4: Robot closes the drawer by placing the gripper on the handle and pushing it to closure
def close_drawer():
    # Move to position above the drawer handle for closing
    handle_pose_close = [0.85, 0.2, 0.75]  # Adjust height as necessary
    move_robot_to(handle_pose_close)
    # Push the drawer to close
    p.setJointMotorControl2(drawer_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0)  # Close the drawer
    time.sleep(1)

# Main task sequence
try:
    open_drawer()          # Task 1: Open the drawer
    pick_up_cube()        # Task 2: Pick up the cube
    place_cube_in_drawer()# Task 3: Place the cube inside the drawer
    close_drawer()        # Task 4: Close the drawer
finally:
    p.disconnect()