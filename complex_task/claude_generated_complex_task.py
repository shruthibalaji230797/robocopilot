import pybullet as p
import pybullet_data
import time
import math
import numpy as np

def setup_simulation():
    # Connect to PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Load plane
    p.loadURDF("plane.urdf")
    
    # Load KUKA robot
    robot_start_pos = [0, 0, 0]
    robot = p.loadURDF("kuka_iiwa/model.urdf", robot_start_pos, useFixedBase=True)
    
    # Load gripper
    gripper = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
    
    # Attach gripper to robot
    robot_ee_idx = 6  # End effector link index
    p.createConstraint(robot, robot_ee_idx, gripper, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
    
    # Load table
    table = p.loadURDF("table/table.urdf", [1, 0, 0], useFixedBase=True)
    
    # Load drawer (using a custom URDF or simplified box)
    drawer_base = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.3, 0.1])
    drawer_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.3, 0.1], rgbaColor=[0.8, 0.6, 0.4, 1])
    drawer_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=drawer_base, 
                                  baseVisualShapeIndex=drawer_visual, 
                                  basePosition=[1.2, 0, 0.7])
    
    # Create sliding drawer
    drawer_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15, 0.25, 0.08])
    drawer_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.25, 0.08], rgbaColor=[0.6, 0.4, 0.2, 1])
    drawer = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=drawer_shape, 
                             baseVisualShapeIndex=drawer_visual, 
                             basePosition=[1.2, 0, 0.7])
    
    # Create prismatic joint for drawer
    drawer_joint = p.createConstraint(drawer_body, -1, drawer, -1, p.JOINT_PRISMATIC, 
                                    [0, 1, 0], [0, 0, 0], [0, 0, 0],
                                    [0, -0.25, 0], [0, 0.25, 0])
    
    # Load cube
    cube_size = 0.05
    cube = p.loadURDF("cube.urdf", [1.0, 0.3, 0.75], globalScaling=cube_size)
    
    return robot, gripper, drawer, drawer_joint, cube

def move_arm_to_target(robot, target_pos, target_orn):
    """Move robot arm to target position and orientation"""
    joint_poses = p.calculateInverseKinematics(robot, 6, target_pos, target_orn)
    for i in range(7):
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, joint_poses[i])
    return joint_poses

def control_gripper(gripper, open_close):
    """Control gripper (0 = close, 1 = open)"""
    p.setJointMotorControl2(gripper, 1, p.POSITION_CONTROL, open_close * 0.05)
    p.setJointMotorControl2(gripper, 3, p.POSITION_CONTROL, -open_close * 0.05)

def move_drawer(drawer_joint, position):
    """Move drawer to specified position"""
    p.setJointMotorControl2(drawer_joint, p.JOINT_PRISMATIC, p.POSITION_CONTROL, 
                           targetPosition=position, force=500)

def main():
    robot, gripper, drawer, drawer_joint, cube = setup_simulation()
    
    # Wait for simulation to stabilize
    for _ in range(100):
        p.stepSimulation()
    
    # Task 1: Open drawer
    print("Opening drawer...")
    move_arm_to_target(robot, [1.2, -0.2, 0.7], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    time.sleep(2)
    move_drawer(drawer_joint, 0.25)  # Open drawer
    time.sleep(2)
    
    # Task 2: Pick up cube
    print("Picking up cube...")
    # Move above cube
    move_arm_to_target(robot, [1.0, 0.3, 0.85], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    time.sleep(1)
    # Open gripper
    control_gripper(gripper, 1)
    time.sleep(1)
    # Move down to cube
    move_arm_to_target(robot, [1.0, 0.3, 0.77], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    time.sleep(1)
    # Close gripper
    control_gripper(gripper, 0)
    time.sleep(1)
    
    # Task 3: Place cube in drawer
    print("Placing cube in drawer...")
    # Move up
    move_arm_to_target(robot, [1.0, 0.3, 0.85], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    time.sleep(1)
    # Move to drawer
    move_arm_to_target(robot, [1.2, 0, 0.85], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    time.sleep(1)
    # Lower into drawer
    move_arm_to_target(robot, [1.2, 0, 0.77], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    time.sleep(1)
    # Release cube
    control_gripper(gripper, 1)
    time.sleep(1)
    
    # Task 4: Close drawer
    print("Closing drawer...")
    # Move to drawer handle
    move_arm_to_target(robot, [1.2, 0.25, 0.7], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    time.sleep(1)
    # Close drawer
    move_drawer(drawer_joint, 0)
    time.sleep(2)
    
    # Move arm to home position
    move_arm_to_target(robot, [0, 0, 1], p.getQuaternionFromEuler([0, -math.pi/2, 0]))
    
    # Keep simulation running
    while p.isConnected():
        p.stepSimulation()
        time.sleep(1./240.)

if __name__ == "__main__":
    main()