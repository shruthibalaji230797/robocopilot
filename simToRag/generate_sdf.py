import pybullet as p
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
from pathlib import Path

def generate_scene_sdf(output_path, world_name="simulation_world"):
    """Generate SDF file from current PyBullet simulation state including all objects"""
    # Create SDF structure
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name=world_name)
    
    # Add physics properties
    physics = ET.SubElement(world, 'physics', type='ode')
    gravity = ET.SubElement(physics, 'gravity')
    # gravity.text = ' '.join(map(str, p.getGravity()))
    gravity.text = '0 0 -9.81'
    # Get all objects in the scene
    num_bodies = p.getNumBodies()
    for body_id in range(num_bodies):
        body_info = p.getBodyInfo(body_id)
        body_name = body_info[1].decode('utf-8')
        
        # Create model for each object
        model = ET.SubElement(world, 'model', name=f'{body_name}_{body_id}')
        
        # Get base pose
        base_pos, base_orn = p.getBasePositionAndOrientation(body_id)
        
        # Get dynamics info
        dynamics_info = p.getDynamicsInfo(body_id, -1)
        print("dynamics info: ", dynamics_info)
        
        # Add static flag if mass is 0
        if dynamics_info[0] == 0:  # mass
            static = ET.SubElement(model, 'static')
            static.text = 'true'
        
        # Add base link
        base_link = ET.SubElement(model, 'link', name='base_link')
        
        # Set pose (combining position and orientation)
        pose = ET.SubElement(base_link, 'pose')
        pose.text = f"{' '.join(map(str, base_pos))} {' '.join(map(str, base_orn))}"
        
        # Add inertial properties
        inertial = ET.SubElement(base_link, 'inertial')
        
        # Mass
        mass = ET.SubElement(inertial, 'mass')
        mass.text = str(dynamics_info[0])
        
        # Inertia (diagonal terms from local inertia diagonal)
        inertia = ET.SubElement(inertial, 'inertia')
        ixx = ET.SubElement(inertia, 'ixx')
        ixx.text = str(dynamics_info[2][0])
        ixy = ET.SubElement(inertia, 'ixy')
        ixy.text = '0.0'
        ixz = ET.SubElement(inertia, 'ixz')
        ixz.text = '0.0'
        iyy = ET.SubElement(inertia, 'iyy')
        iyy.text = str(dynamics_info[2][1])
        iyz = ET.SubElement(inertia, 'iyz')
        iyz.text = '0.0'
        izz = ET.SubElement(inertia, 'izz')
        izz.text = str(dynamics_info[2][2])
        
        # Add physical properties
        surface = ET.SubElement(base_link, 'surface')
        friction = ET.SubElement(surface, 'friction')
        ode = ET.SubElement(friction, 'ode')
        
        # Friction properties
        mu = ET.SubElement(ode, 'mu')  # lateral friction
        mu.text = str(dynamics_info[1])
        mu2 = ET.SubElement(ode, 'mu2')  # rolling friction
        mu2.text = str(dynamics_info[5])
        fdir1 = ET.SubElement(ode, 'fdir1')
        fdir1.text = '0 0 0'
        slip1 = ET.SubElement(ode, 'slip1')
        slip1.text = '0.0'
        slip2 = ET.SubElement(ode, 'slip2')
        slip2.text = '0.0'
        
        # Contact properties
        contact = ET.SubElement(surface, 'contact')
        ode_contact = ET.SubElement(contact, 'ode')
        kp = ET.SubElement(ode_contact, 'kp')  # contact stiffness
        kp.text = str(max(0.0, dynamics_info[8]))  # use 0.0 if -1
        kd = ET.SubElement(ode_contact, 'kd')  # contact damping
        kd.text = str(dynamics_info[7])
        
        # Damping
        damping = ET.SubElement(base_link, 'damping')
        angular = ET.SubElement(damping, 'angular')
        angular.text = str(dynamics_info[3][0])  # using first component of angular damping
        
        # Add visual properties
        visual_data = p.getVisualShapeData(body_id)
        if visual_data:
            for v_id, visual in enumerate(visual_data):
                visual_elem = ET.SubElement(base_link, 'visual', name=f'visual_{v_id}')
                geometry = ET.SubElement(visual_elem, 'geometry')
                
                # Handle different geometry types
                if visual[2] == p.GEOM_MESH:
                    mesh = ET.SubElement(geometry, 'mesh')
                    uri = ET.SubElement(mesh, 'uri')
                    uri.text = visual[4].decode('utf-8')
                    if visual[3][0] != 1.0:  # If scale is not 1
                        scale = ET.SubElement(mesh, 'scale')
                        scale.text = ' '.join(map(str, visual[3]))
                elif visual[2] == p.GEOM_BOX:
                    box = ET.SubElement(geometry, 'box')
                    size = ET.SubElement(box, 'size')
                    size.text = ' '.join(map(str, [s*2 for s in visual[3]]))  # Box dimensions
                elif visual[2] == p.GEOM_CYLINDER:
                    cylinder = ET.SubElement(geometry, 'cylinder')
                    radius = ET.SubElement(cylinder, 'radius')
                    radius.text = str(visual[3][0])
                    length = ET.SubElement(cylinder, 'length')
                    length.text = str(visual[3][1])
                elif visual[2] == p.GEOM_SPHERE:
                    sphere = ET.SubElement(geometry, 'sphere')
                    radius = ET.SubElement(sphere, 'radius')
                    radius.text = str(visual[3][0])
                
                # Add material if available
                if len(visual) > 7:
                    material = ET.SubElement(visual_elem, 'material')
                    ambient = ET.SubElement(material, 'ambient')
                    ambient.text = ' '.join(map(str, visual[7]))
        
        # Process joints and additional links if it's an articulated object
        num_joints = p.getNumJoints(body_id)
        for joint_id in range(num_joints):
            joint_info = p.getJointInfo(body_id, joint_id)
            joint_name = joint_info[1].decode('utf-8')
            joint_state = p.getJointState(body_id, joint_id)
            
            # Add link for this joint
            link = ET.SubElement(model, 'link', name=f'link_{joint_name}')
            
            # Get link state
            link_state = p.getLinkState(body_id, joint_id)
            if link_state:
                link_pose = ET.SubElement(link, 'pose')
                pos = link_state[0]
                orn = link_state[1]
                link_pose.text = f"{' '.join(map(str, pos))} {' '.join(map(str, orn))}"
            
            # Add joint
            joint = ET.SubElement(model, 'joint', name=f'joint_{joint_name}')
            joint.set('type', get_joint_type(joint_info[2]))
            
            parent = ET.SubElement(joint, 'parent')
            parent.text = 'base_link' if joint_id == 0 else f'link_{p.getJointInfo(body_id, joint_id-1)[1].decode("utf-8")}'
            child = ET.SubElement(joint, 'child')
            child.text = f'link_{joint_name}'
            
            # Joint axis
            if joint_info[2] != p.JOINT_FIXED:
                axis = ET.SubElement(joint, 'axis')
                xyz = ET.SubElement(axis, 'xyz')
                xyz.text = ' '.join(map(str, joint_info[13]))
                
                # Joint limits
                limit = ET.SubElement(joint, 'limit')
                lower = ET.SubElement(limit, 'lower')
                lower.text = str(joint_info[8])
                upper = ET.SubElement(limit, 'upper')
                upper.text = str(joint_info[9])
                effort = ET.SubElement(limit, 'effort')
                effort.text = str(joint_info[10])
                velocity = ET.SubElement(limit, 'velocity')
                velocity.text = str(joint_info[11])
    
    # Write to file with nice formatting
    rough_string = ET.tostring(sdf, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    with open(output_path, 'w') as f:
        f.write(reparsed.toprettyxml(indent="  "))

def get_joint_type(pb_joint_type):
    """Convert PyBullet joint type to SDF joint type"""
    joint_types = {
        p.JOINT_REVOLUTE: 'revolute',
        p.JOINT_PRISMATIC: 'prismatic',
        p.JOINT_SPHERICAL: 'ball',
        p.JOINT_PLANAR: 'planar',
        p.JOINT_FIXED: 'fixed'
    }
    return joint_types.get(pb_joint_type, 'fixed')