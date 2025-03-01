# "Create a Python script using PyBullet that simulates a KUKA robotic arm. The robot should pick up a cube from a specified location and place it in an adjacent position. The script should include the following:

# A properly initialized PyBullet environment with physics simulation.
# Loading a KUKA robot model and a cube in a predefined position.
# Using inverse kinematics (IK) or motion planning to move the robotic arm.
# Grasping the cube, moving it smoothly, and placing it in an adjacent location.
# Ensuring proper physics interactions and stability.
# Commented code for clarity."


import pybullet as p
import pybullet_data
import time
import numpy as np

def setup_simulation():
    """Initializes PyBullet simulation with the KUKA robot and a cube."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    plane_id = p.loadURDF("plane.urdf")
    kuka_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0])
    cube_id = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0, 0.05])
    
    return kuka_id, cube_id

def get_ik_solution(robot_id, target_position, target_orientation):
    """Computes inverse kinematics to find joint positions."""
    end_effector_index = 6  # KUKA's last joint
    joint_positions = p.calculateInverseKinematics(robot_id, end_effector_index, target_position, target_orientation)
    return joint_positions

def move_robot(robot_id, joint_positions):
    """Applies joint positions to move the robot."""
    for i in range(7):  # KUKA has 7 DOF
        p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_positions[i])
    
    for _ in range(100):  # Allow motion to complete
        p.stepSimulation()
        time.sleep(1 / 240)

def pick_and_place(kuka_id, cube_id):
    """Controls the KUKA arm to pick up the cube and place it in an adjacent location."""
    # Define pick position (above the cube, then grip)
    pick_position = [0.5, 0, 0.15]
    grip_position = [0.5, 0, 0.05]
    place_position = [0.7, 0, 0.05]
    
    orientation = p.getQuaternionFromEuler([0, np.pi, 0])
    
    # Move above the cube, then grip
    move_robot(kuka_id, get_ik_solution(kuka_id, pick_position, orientation))
    move_robot(kuka_id, get_ik_solution(kuka_id, grip_position, orientation))
    
    # Attach cube to gripper (simple method using constraints)
    constraint = p.createConstraint(kuka_id, 6, cube_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
    
    # Move to place position and release
    move_robot(kuka_id, get_ik_solution(kuka_id, place_position, orientation))
    p.removeConstraint(constraint)  # Drop the cube

def main():
    kuka_id, cube_id = setup_simulation()
    time.sleep(1)
    pick_and_place(kuka_id, cube_id)
    
    time.sleep(3)  # Allow user to observe result
    p.disconnect()

if __name__ == "__main__":
    main()
