import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load plane and table
p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf", [0, 0, 0])

# Load KUKA robot
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0.7])

# Load a cube
cube_size = 0.05
cube_mass = 1
visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                  halfExtents=[cube_size]*3,
                                  rgbaColor=[1, 0, 0, 1])
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                        halfExtents=[cube_size]*3)
cubeId = p.createMultiBody(baseMass=cube_mass,
                          baseCollisionShapeIndex=collisionShapeId,
                          baseVisualShapeIndex=visualShapeId,
                          basePosition=[0.5, 0, 1.0])

# Create a constraint for the gripper
constraint = p.createConstraint(kukaId,
                              6,
                              -1,
                              -1,
                              p.JOINT_FIXED,
                              [0, 0, 0],
                              [0, 0, 0.1],
                              [0, 0, 0])

def reset_robot():
    # Reset joint positions
    joint_positions = [0, 0, 0, -1.57, 0, 1.57, 0]
    for i in range(7):
        p.resetJointState(kukaId, i, joint_positions[i])

def move_to_position(target_pos, target_orn):
    # Calculate inverse kinematics
    joint_poses = p.calculateInverseKinematics(kukaId,
                                             6,
                                             target_pos,
                                             target_orn)
    
    # Move to target position
    for i in range(7):
        p.setJointMotorControl2(bodyIndex=kukaId,
                              jointIndex=i,
                              controlMode=p.POSITION_CONTROL,
                              targetPosition=joint_poses[i])
    
    # Wait for the movement to complete
    time.sleep(2)

def pick_and_place():
    # Reset robot to initial position
    reset_robot()
    
    # Pick position (above the cube)
    pick_pos = [0.5, 0, 1.2]
    # Place position (adjacent location)
    place_pos = [0.5, 0.3, 1.2]
    # Fixed orientation (gripper facing down)
    orientation = p.getQuaternionFromEuler([0, -np.pi, 0])
    
    # Move to position above cube
    move_to_position(pick_pos, orientation)
    
    # Move down to grab
    pick_pos[2] = 1.05
    move_to_position(pick_pos, orientation)
    
    # Create constraint between cube and robot
    grasp_constraint = p.createConstraint(kukaId,
                                        6,
                                        cubeId,
                                        -1,
                                        p.JOINT_FIXED,
                                        [0, 0, 0],
                                        [0, 0, 0],
                                        [0, 0, 0])
    
    # Lift cube
    pick_pos[2] = 1.2
    move_to_position(pick_pos, orientation)
    
    # Move to place position
    move_to_position(place_pos, orientation)
    
    # Lower and release
    place_pos[2] = 1.05
    move_to_position(place_pos, orientation)
    
    # Remove constraint to release cube
    p.removeConstraint(grasp_constraint)
    
    # Move up
    place_pos[2] = 1.2
    move_to_position(place_pos, orientation)
    
    # Return to initial position
    reset_robot()

# Main simulation loop
p.setRealTimeSimulation(1)
time.sleep(1)  # Wait for simulation to stabilize

# Execute pick and place
pick_and_place()

# Keep the simulation running
while p.isConnected():
    p.stepSimulation()
    time.sleep(1./240.)