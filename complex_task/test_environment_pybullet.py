import math
from PIL import Image
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import cv2
from base64 import b64encode
from IPython.display import HTML
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from simToRag.generate_sdf import generate_scene_sdf
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

# reset kuka
jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
for jointIndex in range(p.getNumJoints(kuka_id)):
    p.resetJointState(kuka_id, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(kuka_id, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)

# reset gripper
p.resetBasePositionAndOrientation(kuka_gripper_id, [0.923103, -0.200000, 1.250036], [-0.000000, 0.964531, -0.000002, -0.263970])
jointPositions = [0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000]
for jointIndex in range(p.getNumJoints(kuka_gripper_id)):
    p.resetJointState(kuka_gripper_id, jointIndex, jointPositions[jointIndex])
    p.setJointMotorControl2(kuka_gripper_id, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)

num_joints = p.getNumJoints(kuka_id)
kuka_end_effector_idx = 6

# camera parameters
cam_target_pos = [1.0, 0.0, 0.0]
cam_distance = 3.5
cam_yaw, cam_pitch, cam_roll = 0, -60, 0
cam_width, cam_height = 1024, 768

cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 75

#video = cv2.VideoWriter('vid.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (cam_width, cam_height)) # Does not seem to support h264!
# vid = imageio_ffmpeg.write_frames('vid.mp4', (cam_width, cam_height), fps=30)
# vid.send(None) # seed the video writer with a blank frame
sdf_output_dir = "scene_sdf_states"
for t in range(1400):
    print(f'\rtimestep {t}...', end='')

    if t % 8 == 0: # PyBullet default simulation time step is 240fps, but we want to record video at 30fps.
        cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
        cam_projection_matrix = p.computeProjectionMatrixFOV(cam_fov, cam_width*1./cam_height, cam_near_plane, cam_far_plane)
        image = p.getCameraImage(cam_width, cam_height, cam_view_matrix, cam_projection_matrix)[2][:, :, :3]
        img = Image.fromarray(image)

        # Create directory for images if it doesn't exist
        image_dir = "scene_images"
        if not os.path.exists(image_dir):
            os.makedirs(image_dir)
        img.save(os.path.join(image_dir, "initial_scene_ceiling_view.png"))

        #video.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        # vid.send(np.ascontiguousarray(image))
    
    target_pos, gripper_val = [0.85, 0.1, 0.97], 1 # Home pose
    if t >= 100 and t < 200:
        target_pos, gripper_val = [0.85, 0.1, 0.97], 1 # Go to drawer pose
        generate_scene_sdf(sdf_output_dir + "/scene_drawer_pose_" + str(t) + ".sdf", "drawer_pose")
    elif t >= 200 and t < 230:
        target_pos, gripper_val = [0.85, (0.1 - (t-200)/100.), 0.97], 1 # Open drawer
        generate_scene_sdf(sdf_output_dir + "/scene_open_drawer_" + str(t) + ".sdf", "open_drawer")
    elif t >= 230 and t < 330:
        target_pos, gripper_val = [0.85, 0.0, 1.1], 0 # Come back to home pose
        generate_scene_sdf(sdf_output_dir + "/scene_home_pose_" + str(t) + ".sdf", "home_pose")
    elif t >= 330 and t < 430:
        target_pos, gripper_val = [0.85, -0.5, 0.97], 0 # Go to object grab pose
        generate_scene_sdf(sdf_output_dir + "/scene_go_to_bject_grab_pose_" + str(t) + ".sdf", "go_to_bject_grab_pose")
    elif t >= 430 and t < 530:
        target_pos, gripper_val = [0.85, -0.5, 0.97], 1 # Close gripper
        generate_scene_sdf(sdf_output_dir + "/scene_close_gripper_object_grab_pose_" + str(t) + ".sdf", "close_gripper_object_grab_pose")
    elif t >= 530 and t < 650:
        z = 0.97 + 0.001*(t-530)
        if z >= 1.1:
            target_pos, gripper_val = [0.85, -0.5, 1.1], 1
        else:
            target_pos, gripper_val = [0.85, -0.5, z], 1
        generate_scene_sdf(sdf_output_dir + "/scene_object_grab_pose_" + str(t) + ".sdf", "object_grab_pose")
    elif t >= 650 and t < 800:
        y = -0.5 + (t-650)*0.004
        if y>=0.03:
            target_pos, gripper_val = [0.85, 0.03, 1.1], 1
        else:
            target_pos, gripper_val = [0.85, y, 1.1], 1 # Move to an intermediate position
        generate_scene_sdf(sdf_output_dir + "/scene_move_to_intermediate_pose_" + str(t) + ".sdf", "move_to_intermediate_pose")
    # elif t >= 720 and t < 800:
    #     target_pos, gripper_val = [0.85, 0.0 + 0.4*(t-650)/500., 1.1], 1 # move to target position
    elif t >= 800 and t < 900:
        target_pos, gripper_val = [0.85, 0.03, 1.0], 1 # stop at target position
        generate_scene_sdf(sdf_output_dir + "/scene_stop_at_target_pose_" + str(t) + ".sdf", "stop_at_target_pose")
    elif t >= 900 and t < 1000:
        target_pos, gripper_val = [0.85, 0.03, 1.0], 0 # drop object
        generate_scene_sdf(sdf_output_dir + "/scene_drop_object_pose_" + str(t) + ".sdf", "drop_object_pose")
    elif t >= 1000 and t < 1100:
        target_pos, gripper_val = [0.85, 0.0, 1.1], 1 # Come back to home pose
        generate_scene_sdf(sdf_output_dir + "/scene_home_pose_" + str(t) + ".sdf", "home_pose")
    elif t >= 1100 and t < 1200:
        target_pos, gripper_val = [0.85, -0.19, 0.97], 1 # Close drawer
        generate_scene_sdf(sdf_output_dir + "/scene_close_drawer_pose_" + str(t) + ".sdf", "close_drawer_pose")
    elif t >= 1200 and t < 1300:
        y = -0.19 + (t-1200)*0.005
        if y>=0.08:
            target_pos, gripper_val = [0.85, 0.08, 0.97], 1
        else:
            target_pos, gripper_val = [0.85, y, 0.97], 1
        generate_scene_sdf(sdf_output_dir + "/scene_close_drawer_action_" + str(t) + ".sdf", "close_drawer_action")
    else:
        target_pos, gripper_val = [0.85, 0.0, 1.1], 0 # Come back to home pose
        generate_scene_sdf(sdf_output_dir + "/scene_home_pose_" + str(t) + ".sdf", "home_pose")
    print(f'target_pos: {target_pos}, gripper_val: {gripper_val}')

    target_orn = p.getQuaternionFromEuler([0, 1.01*math.pi, 0])
    joint_poses = p.calculateInverseKinematics(kuka_id, kuka_end_effector_idx, target_pos, target_orn)
    for j in range (num_joints):
        p.setJointMotorControl2(bodyIndex=kuka_id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=joint_poses[j])
    
    p.setJointMotorControl2(kuka_gripper_id, 4, p.POSITION_CONTROL, targetPosition=gripper_val*0.05, force=100)
    p.setJointMotorControl2(kuka_gripper_id, 6, p.POSITION_CONTROL, targetPosition=gripper_val*0.05, force=100)

    p.stepSimulation()

plt.imshow(Image.fromarray(image)) # show the last frame

# vid.close()
#video.release()
p.disconnect()
