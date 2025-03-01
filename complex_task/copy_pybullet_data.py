import os
import shutil
import pybullet_data

def copy_pybullet_data(target_dir="../assets/bullet-objects"):
    """Copy required URDF/SDF files from pybullet_data to local directory"""
    # List of files to copy with their relative paths
    files_to_copy = [
        "plane.urdf",
        "kuka_iiwa/model_vr_limits.urdf",
        "kuka_iiwa/model.urdf",
        "kuka_iiwa/meshes/",  # Directory containing mesh files
        "gripper/wsg50_one_motor_gripper_new_free_base.sdf",
        "table/table.urdf",
        "cube.urdf"
    ]

    # Get pybullet data path
    pybullet_path = pybullet_data.getDataPath()
    print(f"Copying files from: {pybullet_path}")

    # Create target directory if it doesn't exist
    os.makedirs(target_dir, exist_ok=True)

    for file_path in files_to_copy:
        src_path = os.path.join(pybullet_path, file_path)
        dst_path = os.path.join(target_dir, file_path)

        # Create destination directory if needed
        os.makedirs(os.path.dirname(dst_path), exist_ok=True)

        if os.path.isdir(src_path):
            # If it's a directory, copy the entire directory
            print(f"Copying directory: {file_path}")
            if os.path.exists(dst_path):
                shutil.rmtree(dst_path)
            shutil.copytree(src_path, dst_path)
        else:
            # If it's a file, copy the file
            print(f"Copying file: {file_path}")
            shutil.copy2(src_path, dst_path)

    print(f"\nAll files copied to: {os.path.abspath(target_dir)}")

if __name__ == "__main__":
    copy_pybullet_data()