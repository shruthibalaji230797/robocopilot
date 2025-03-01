import os
import json
import trimesh
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
import argparse
import sys
import yaml
from urdf_parser_py.urdf import URDF

class RobotYAMLLoader(yaml.SafeLoader):
    """Custom YAML Loader with robot-specific constructors"""
    pass

# Define constructors for custom tags
def construct_mesh(loader, node):
    return loader.construct_mapping(node)

def construct_revolute(loader, node):
    return loader.construct_mapping(node)

# Add constructors to our custom loader class
RobotYAMLLoader.add_constructor('!mesh', construct_mesh)
RobotYAMLLoader.add_constructor('!revolute', construct_revolute)

class ThreeDFileProcessor:
    def __init__(self, directory_path):
        self.directory = Path(directory_path)
        self.output_dir = self.directory / "processed_outputs"
        self.output_dir.mkdir(exist_ok=True)
        
    def process_all_files(self):
        """Process all supported files in the directory"""
        for file_path in self.directory.glob("*"):
            if file_path.suffix.lower() in ['.stl', '.mtl', '.sdf', '.urdf', '.yaml', '.yml', '.obj']:
                print(f"Processing: {file_path.name}")
                try:
                    self.process_file(file_path)
                except Exception as e:
                    print(f"Error processing {file_path.name}: {str(e)}")

    def process_file(self, file_path):
        """Route file to appropriate processor based on extension"""
        processors = {
            '.stl': self.process_stl,
            '.mtl': self.process_mtl,
            '.sdf': self.process_sdf,
            '.urdf': self.process_urdf,
            '.yaml': self.process_yaml,
            '.yml': self.process_yaml,
            '.obj': self.process_obj
        }
        
        processor = processors.get(file_path.suffix.lower())
        if processor:
            json_data, text_description = processor(file_path)
            self.save_outputs(file_path.stem, json_data, text_description)

    def process_stl(self, file_path):
        """Process STL files"""
        mesh = trimesh.load_mesh(str(file_path))
        
        # Extract geometric properties
        json_data = {
            "file_type": "STL",
            "name": file_path.stem,
            "geometry": {
                "vertices_count": len(mesh.vertices),
                "faces_count": len(mesh.faces),
                "volume": float(mesh.volume),
                "surface_area": float(mesh.area),
                "center_of_mass": mesh.center_mass.tolist(),
                "bounding_box": {
                    "min": mesh.bounds[0].tolist(),
                    "max": mesh.bounds[1].tolist()
                },
                "is_watertight": mesh.is_watertight,
                "is_convex": mesh.is_convex
            },
            "spatial_properties": {
                "dimensions": {
                    "width": float(mesh.extents[0]),
                    "length": float(mesh.extents[1]),
                    "height": float(mesh.extents[2])
                }
            }
        }
        
        # Create human-readable description
        text_description = f"""
        3D Model Description: {file_path.stem}
        Type: STL (Stereolithography) file
        
        Geometric Properties:
        - Contains {json_data['geometry']['vertices_count']} vertices and {json_data['geometry']['faces_count']} faces
        - Volume: {json_data['geometry']['volume']:.2f} cubic units
        - Surface Area: {json_data['geometry']['surface_area']:.2f} square units
        
        Spatial Characteristics:
        - Dimensions: {json_data['spatial_properties']['dimensions']['width']:.2f} x {json_data['spatial_properties']['dimensions']['length']:.2f} x {json_data['spatial_properties']['dimensions']['height']:.2f} units
        - Center of Mass: {[f'{x:.2f}' for x in json_data['geometry']['center_of_mass']]}
        
        Physical Properties:
        - {'Watertight' if json_data['geometry']['is_watertight'] else 'Not watertight'} mesh
        - {'Convex' if json_data['geometry']['is_convex'] else 'Non-convex'} shape
        """
        
        return json_data, text_description

    def process_mtl(self, file_path):
        """Process MTL (Material) files"""
        materials = {}
        current_material = None
        
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('newmtl '):
                    current_material = line.split()[1]
                    materials[current_material] = {}
                elif line.startswith(('Ka ', 'Kd ', 'Ks ', 'Ns ', 'd ')):
                    if current_material:
                        key, *values = line.split()
                        materials[current_material][key] = [float(v) for v in values]
                        
        json_data = {
            "file_type": "MTL",
            "name": file_path.stem,
            "materials": materials
        }
        
        # Create human-readable description
        text_description = f"""
        Material Library Description: {file_path.stem}
        Type: MTL (Material Template Library) file
        
        Materials:
        """
        
        for mat_name, properties in materials.items():
            text_description += f"\nMaterial: {mat_name}"
            for prop, values in properties.items():
                prop_names = {
                    'Ka': 'Ambient Color',
                    'Kd': 'Diffuse Color',
                    'Ks': 'Specular Color',
                    'Ns': 'Specular Exponent',
                    'd': 'Transparency'
                }
                text_description += f"\n- {prop_names.get(prop, prop)}: {values}"
        
        return json_data, text_description

    def process_sdf(self, file_path):
        """Process SDF (Simulation Description Format) files"""
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        def parse_element(element):
            """Recursively parse XML elements"""
            result = {}
            
            # Add attributes
            if element.attrib:
                result['attributes'] = element.attrib
            
            # Add text content if present
            if element.text and element.text.strip():
                result['value'] = element.text.strip()
            
            # Add children
            for child in element:
                if child.tag not in result:
                    result[child.tag] = parse_element(child)
                else:
                    if not isinstance(result[child.tag], list):
                        result[child.tag] = [result[child.tag]]
                    result[child.tag].append(parse_element(child))
            
            return result
        
        json_data = {
            "file_type": "SDF",
            "name": file_path.stem,
            "version": root.attrib.get('version', 'unknown'),
            "content": parse_element(root)
        }
        
        # Create human-readable description
        text_description = f"""
        Simulation Description: {file_path.stem}
        Type: SDF (Simulation Description Format) file
        Version: {json_data['version']}
        
        World Elements:
        """
        
        def describe_element(element, depth=0):
            description = ""
            indent = "  " * depth
            
            if isinstance(element, dict):
                for key, value in element.items():
                    if key == 'attributes':
                        description += f"\n{indent}Attributes: {value}"
                    elif key == 'value':
                        description += f": {value}"
                    else:
                        description += f"\n{indent}{key}"
                        description += describe_element(value, depth + 1)
            elif isinstance(element, list):
                for item in element:
                    description += describe_element(item, depth)
                    
            return description
        
        text_description += describe_element(json_data['content'])
        
        return json_data, text_description
    


    def process_yaml(self, file_path):
        """Process YAML configuration files"""
        try:
            with open(file_path, 'r') as f:
                data = yaml.load(f, Loader=RobotYAMLLoader)
            
            json_data = {
                "file_type": "YAML",
                "name": file_path.stem,
                "content": data,
                "structure": {
                    "top_level_keys": list(data.keys()),
                    "configuration_type": self._detect_config_type(data)
                }
            }
            
            # Create human-readable description
            text_description = f"""
            Configuration Description: {file_path.stem}
            Type: YAML Configuration File
            
            Configuration Type: {json_data["structure"]["configuration_type"]}
            
            Top-Level Elements:
            {self._generate_yaml_description(data)}
            
            Key Parameters:
            {self._generate_key_params_description(data)}
            """
            
            return json_data, text_description
            
        except Exception as e:
            raise Exception(f"Error processing YAML file: {str(e)}")
        
    def process_obj(self, file_path):
        """Process OBJ (3D Model) files"""
        try:
            vertices = []
            faces = []
            normals = []
            uvs = []
            materials = {}
            current_material = None
            
            with open(file_path, 'r') as f:
                for line in f:
                    if line.startswith('#'): continue  # Skip comments
                    
                    values = line.split()
                    if not values: continue
                    
                    if values[0] == 'v':  # Vertex
                        vertices.append([float(x) for x in values[1:4]])
                    elif values[0] == 'vn':  # Normal
                        normals.append([float(x) for x in values[1:4]])
                    elif values[0] == 'vt':  # Texture coordinate
                        uvs.append([float(x) for x in values[1:3]])
                    elif values[0] == 'f':  # Face
                        face = []
                        for v in values[1:]:
                            w = v.split('/')
                            # OBJ is 1-indexed, convert to 0-indexed
                            face.append([int(i)-1 if i else None for i in w])
                        faces.append(face)
                    elif values[0] == 'mtllib':  # Material library
                        materials['mtllib'] = values[1]
                    elif values[0] == 'usemtl':  # Use material
                        current_material = values[1]
                    elif values[0] == 'o':  # Object name
                        if len(values) > 1:
                            current_object = values[1]
            
            json_data = {
                "file_type": "OBJ",
                "name": file_path.stem,
                "geometry": {
                    "vertices_count": len(vertices),
                    "faces_count": len(faces),
                    "normals_count": len(normals),
                    "uvs_count": len(uvs)
                },
                "materials": {
                    "material_library": materials.get('mtllib'),
                    "materials_used": current_material
                },
                "bounds": {
                    "min": [min(v[i] for v in vertices) for i in range(3)],
                    "max": [max(v[i] for v in vertices) for i in range(3)]
                }
            }
            
            # Create human-readable description
            text_description = f"""
            3D Model Description: {file_path.stem}
            Type: OBJ (Wavefront) file
            
            Geometric Properties:
            - Vertices: {len(vertices)}
            - Faces: {len(faces)}
            - Normals: {len(normals)}
            - UV Coordinates: {len(uvs)}
            
            Material Information:
            - Material Library: {materials.get('mtllib', 'None')}
            - Active Material: {current_material}
            
            Bounding Box:
            - Min: {[f'{x:.3f}' for x in json_data['bounds']['min']]}
            - Max: {[f'{x:.3f}' for x in json_data['bounds']['max']]}
            
            Objects:
            - Current Object: {current_object if 'current_object' in locals() else 'None'}
            """
            
            return json_data, text_description
            
        except Exception as e:
            raise Exception(f"Error processing OBJ file: {str(e)}")

    # def process_urdf(self, file_path):
    #     """Process URDF files safely"""
    #     try:
    #         robot = URDF.from_xml_file(str(file_path))
            
    #         json_data = {
    #             "file_type": "URDF",
    #             "name": robot.name,
    #             "links": [],
    #             "joints": []
    #         }
            
    #         # Process links
    #         for link in robot.links:
    #             link_data = {
    #                 "name": link.name,
    #                 "inertial": self._process_inertial(link.inertial) if link.inertial else None,
    #                 "visual": self._process_visuals(link.visuals) if link.visuals else [],
    #                 "collision": self._process_collisions(link.collisions) if link.collisions else []
    #             }
    #             json_data["links"].append(link_data)
            
    #         # Process joints
    #         for joint in robot.joints:
    #             joint_data = {
    #                 "name": joint.name,
    #                 "type": joint.type,
    #                 "parent": joint.parent,
    #                 "child": joint.child,
    #                 "origin": self._process_joint_origin(joint),
    #                  "axis": joint.axis if joint.axis is not None else None,
    #                 "limits": self._process_joint_limits(joint.limit) if hasattr(joint, 'limit') else None,
    #                 "dynamics": self._process_joint_dynamics(joint) if hasattr(joint, 'dynamics') else None
    #             }
    #             json_data["joints"].append(joint_data)
            
    #         text_description = self._generate_urdf_description(json_data)
    #         return json_data, text_description
            
    #     except Exception as e:
    #         raise Exception(f"URDF processing error: {str(e)}")

    def process_urdf(self, file_path):
        """Process URDF files with support for all common elements"""
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()

            # Extract robot name
            robot_name = root.attrib.get("name", "Unknown Robot")
            
            links = []
            joints = []
            materials = []
            txt_output = f"Robot Name: {robot_name}\n\n"

            # Process materials
            txt_output += "=== MATERIALS ===\n"
            for material in root.findall(".//material"):
                material_name = material.attrib.get("name", "Unnamed Material")
                color_elem = material.find("color")
                if color_elem is not None:
                    rgba = color_elem.attrib.get("rgba", "0 0 0 0")
                    materials.append({
                        "name": material_name,
                        "rgba": [float(x) for x in rgba.split()]
                    })
                    txt_output += f"Material: {material_name}\n"
                    txt_output += f"  - RGBA: {rgba}\n"

            # Process links
            txt_output += "\n=== LINK DETAILS ===\n"
            for link in root.findall("link"):
                link_name = link.attrib.get("name", "Unnamed Link")
                link_data = {"name": link_name}

                # Process contact properties
                contact = link.find("contact")
                if contact is not None:
                    contact_data = {}
                    for prop in contact:
                        contact_data[prop.tag] = float(prop.attrib.get("value", 0))
                    link_data["contact"] = contact_data
                    txt_output += f"\nContact Properties for {link_name}:\n"
                    for prop, value in contact_data.items():
                        txt_output += f"  - {prop}: {value}\n"

                # Process inertial properties
                inertial = link.find("inertial")
                if inertial is not None:
                    inertial_data = {}
                    origin = inertial.find("origin")
                    if origin is not None:
                        inertial_data["origin"] = {
                            "xyz": [float(x) for x in origin.attrib.get("xyz", "0 0 0").split()],
                            "rpy": [float(x) for x in origin.attrib.get("rpy", "0 0 0").split()]
                        }
                    mass = inertial.find("mass")
                    if mass is not None:
                        inertial_data["mass"] = float(mass.attrib.get("value", "0"))
                    inertia = inertial.find("inertia")
                    if inertia is not None:
                        inertial_data["inertia"] = {k: float(v) for k, v in inertia.attrib.items()}
                    link_data["inertial"] = inertial_data

                # Process visual properties
                visuals = link.findall("visual")
                if visuals:
                    link_data["visual"] = []
                    for idx, visual in enumerate(visuals):
                        visual_data = {}
                        origin = visual.find("origin")
                        if origin is not None:
                            visual_data["origin"] = {
                                "xyz": [float(x) for x in origin.attrib.get("xyz", "0 0 0").split()],
                                "rpy": [float(x) for x in origin.attrib.get("rpy", "0 0 0").split()]
                            }
                        
                        geometry = visual.find("geometry")
                        if geometry is not None:
                            geo_data = {}
                            for geo_type in ['mesh', 'box', 'cylinder', 'sphere']:
                                geo_elem = geometry.find(geo_type)
                                if geo_elem is not None:
                                    geo_data["type"] = geo_type
                                    if geo_type == 'mesh':
                                        geo_data["filename"] = geo_elem.attrib.get("filename", "")
                                        geo_data["scale"] = [float(x) for x in geo_elem.attrib.get("scale", "1 1 1").split()]
                                    elif geo_type == 'box':
                                        geo_data["size"] = [float(x) for x in geo_elem.attrib.get("size", "1 1 1").split()]
                                    elif geo_type in ['cylinder', 'sphere']:
                                        for attr in ['radius', 'length']:
                                            if attr in geo_elem.attrib:
                                                geo_data[attr] = float(geo_elem.attrib[attr])
                            visual_data["geometry"] = geo_data

                        material = visual.find("material")
                        if material is not None:
                            material_data = {"name": material.attrib.get("name", "")}
                            color = material.find("color")
                            if color is not None:
                                material_data["color"] = [float(x) for x in color.attrib.get("rgba", "1 1 1 1").split()]
                            visual_data["material"] = material_data
                        
                        link_data["visual"].append(visual_data)

                # Process collision properties
                collisions = link.findall("collision")
                if collisions:
                    link_data["collision"] = []
                    for collision in collisions:
                        collision_data = {}
                        origin = collision.find("origin")
                        if origin is not None:
                            collision_data["origin"] = {
                                "xyz": [float(x) for x in origin.attrib.get("xyz", "0 0 0").split()],
                                "rpy": [float(x) for x in origin.attrib.get("rpy", "0 0 0").split()]
                            }
                        
                        geometry = collision.find("geometry")
                        if geometry is not None:
                            geo_data = {}
                            for geo_type in ['mesh', 'box', 'cylinder', 'sphere']:
                                geo_elem = geometry.find(geo_type)
                                if geo_elem is not None:
                                    geo_data["type"] = geo_type
                                    if geo_type == 'mesh':
                                        geo_data["filename"] = geo_elem.attrib.get("filename", "")
                                        geo_data["scale"] = [float(x) for x in geo_elem.attrib.get("scale", "1 1 1").split()]
                                    elif geo_type == 'box':
                                        geo_data["size"] = [float(x) for x in geo_elem.attrib.get("size", "1 1 1").split()]
                                    elif geo_type in ['cylinder', 'sphere']:
                                        for attr in ['radius', 'length']:
                                            if attr in geo_elem.attrib:
                                                geo_data[attr] = float(geo_elem.attrib[attr])
                            collision_data["geometry"] = geo_data
                        
                        link_data["collision"].append(collision_data)

                links.append(link_data)

            # Process joints (same as before)
            txt_output += "\n=== JOINT DETAILS ===\n"
            for joint in root.findall("joint"):
                joint_name = joint.attrib.get("name", "Unnamed Joint")
                joint_type = joint.attrib.get("type", "unknown")
                
                joint_data = {
                    "name": joint_name,
                    "type": joint_type
                }

                for elem in ["parent", "child"]:
                    element = joint.find(elem)
                    if element is not None:
                        joint_data[elem] = element.attrib.get("link", "")

                for elem in ["origin", "axis"]:
                    element = joint.find(elem)
                    if element is not None:
                        joint_data[elem] = {k: [float(x) for x in v.split()] 
                                        for k, v in element.attrib.items()}

                for elem in ["limit", "dynamics"]:
                    element = joint.find(elem)
                    if element is not None:
                        joint_data[elem] = {k: float(v) for k, v in element.attrib.items()}

                joints.append(joint_data)
                txt_output += f"\nJoint: {joint_name}\n"
                txt_output += f"  Type: {joint_type}\n"
                for key, value in joint_data.items():
                    if key not in ["name", "type"]:
                        txt_output += f"  {key}: {value}\n"

            json_data = {
                "robot_name": robot_name,
                "materials": materials,
                "links": links,
                "joints": joints
            }

            return json_data, txt_output

        except Exception as e:
            raise Exception(f"Error processing URDF file {file_path}: {str(e)}")
    def _process_inertial(self, inertial):
        """Process inertial data for KUKA LBR iiwa format"""
        if not inertial:
            return None
        
        try:
            return {
                "origin": {
                    "xyz": [float(x) for x in inertial.origin.xyz] if hasattr(inertial.origin, 'xyz') else [0, 0, 0],
                    "rpy": [float(x) for x in inertial.origin.rpy] if hasattr(inertial.origin, 'rpy') else [0, 0, 0]
                } if hasattr(inertial, 'origin') else None,
                "mass": float(inertial.mass),
                "inertia": {
                    "ixx": float(inertial.inertia.ixx),
                    "ixy": float(inertial.inertia.ixy),
                    "ixz": float(inertial.inertia.ixz),
                    "iyy": float(inertial.inertia.iyy),
                    "iyz": float(inertial.inertia.iyz),
                    "izz": float(inertial.inertia.izz)
                } if hasattr(inertial, 'inertia') else {}
            }
        except AttributeError as e:
            print(f"Warning: Could not process some inertial attributes: {e}")
            return {
                "mass": 0.0,
                "inertia": {},
                "origin": None
            }

    def _process_visuals(self, visuals):
        """Process visual elements"""
        if not visuals:
            return []
        
        visual_data = []
        for visual in visuals:
            data = {
                "origin": {
                    "xyz": [float(x) for x in visual.origin.xyz] if hasattr(visual.origin, 'xyz') else [0, 0, 0],
                    "rpy": [float(x) for x in visual.origin.rpy] if hasattr(visual.origin, 'rpy') else [0, 0, 0]
                } if hasattr(visual, 'origin') else None,
                "geometry": self._process_geometry(visual.geometry) if hasattr(visual, 'geometry') else None,
                "material": self._process_material(visual.material) if hasattr(visual, 'material') else None
            }
            visual_data.append(data)
        return visual_data

    def _process_geometry(self, geometry):
        """Process geometry elements"""
        if not geometry:
            return None
        
        geo_type = type(geometry).__name__.lower()
        if geo_type == 'box':
            return {
                "type": "box",
                "size": [float(x) for x in geometry.size]
            }
        elif geo_type == 'cylinder':
            return {
                "type": "cylinder",
                "radius": float(geometry.radius),
                "length": float(geometry.length)
            }
        elif geo_type == 'sphere':
            return {
                "type": "sphere",
                "radius": float(geometry.radius)
            }
        elif geo_type == 'mesh':
            return {
                "type": "mesh",
                "filename": geometry.filename
            }
        return None

    def _process_material(self, material):
        """Process material properties"""
        if not material:
            return None
        
        return {
            "name": material.name if hasattr(material, 'name') else None,
            "color": [float(x) for x in material.color.rgba] if hasattr(material, 'color') and hasattr(material.color, 'rgba') else None
        }

    def _process_joint_limits(self, limit):
        """Process joint limits"""
        if not limit:
            return None
        
        return {
            "effort": float(limit.effort) if hasattr(limit, 'effort') else None,
            "lower": float(limit.lower) if hasattr(limit, 'lower') else None,
            "upper": float(limit.upper) if hasattr(limit, 'upper') else None,
            "velocity": float(limit.velocity) if hasattr(limit, 'velocity') else None
        }

    def _process_joint_dynamics(self, joint):
        """Process joint dynamics"""
        if not hasattr(joint, 'dynamics'):
            return None
        
        return {
            "damping": float(joint.dynamics.damping) if hasattr(joint.dynamics, 'damping') else None,
            "friction": float(joint.dynamics.friction) if hasattr(joint.dynamics, 'friction') else None
        }

    def _process_joint_origin(self, joint):
        """Process joint origin"""
        if not hasattr(joint, 'origin'):
            return None
        
        return {
            "xyz": [float(x) for x in joint.origin.xyz] if hasattr(joint.origin, 'xyz') else [0, 0, 0],
            "rpy": [float(x) for x in joint.origin.rpy] if hasattr(joint.origin, 'rpy') else [0, 0, 0]
        }




    def _process_collisions(self, collisions):
        """Process collision elements"""
        collision_data = []
        for collision in collisions:
            collision_data.append({
                "name": collision.name,
                "geometry": self._process_geometry(collision.geometry),
                "origin": self._process_joint_origin(collision.origin) if collision.origin else None
            })
        return collision_data
    

    
    # Helper methods for YAML processing
    def _detect_config_type(self, yaml_data):
        """Detect the type of YAML configuration"""
        keys = set(yaml_data.keys())
        
        if {'robot', 'arm', 'gripper'}.intersection(keys):
            return "Robot Configuration"
        elif {'world', 'environment', 'scene'}.intersection(keys):
            return "Environment Configuration"
        elif {'controller', 'control', 'gains'}.intersection(keys):
            return "Controller Configuration"
        elif {'planning', 'motion_planning', 'trajectory'}.intersection(keys):
            return "Planning Configuration"
        return "Generic Configuration"

    def _generate_yaml_description(self, yaml_data, depth=0):
        """Generate human-readable description of YAML structure"""
        description = []
        indent = "  " * depth
        
        for key, value in yaml_data.items():
            if isinstance(value, dict):
                description.append(f"{indent}- {key}:")
                description.append(self._generate_yaml_description(value, depth + 1))
            elif isinstance(value, list):
                description.append(f"{indent}- {key}: {len(value)} items")
            else:
                description.append(f"{indent}- {key}: {value}")
                
        return "\n".join(description)

    def _generate_key_params_description(self, yaml_data):
        """Generate description of key parameters"""
        key_params = []
        
        def extract_key_params(data, prefix=""):
            for key, value in data.items():
                if isinstance(value, (int, float)) and any(k in key.lower() for k in 
                    ['max', 'min', 'limit', 'threshold', 'default', 'rate']):
                    key_params.append(f"{prefix}{key}: {value}")
                elif isinstance(value, dict):
                    extract_key_params(value, f"{prefix}{key}.")
                    
        extract_key_params(yaml_data)
        return "\n".join(key_params)

    def save_outputs(self, filename, json_data, text_description):
        """Save JSON and text outputs"""
        # Save JSON
        json_path = self.output_dir / f"{filename}_processed.json"
        with open(json_path, 'w') as f:
            json.dump(json_data, f, indent=2)
            
        # Save text description
        text_path = self.output_dir / f"{filename}_description.txt"
        with open(text_path, 'w') as f:
            f.write(text_description)

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description='Process 3D files (STL, MTL, SDF) to generate JSON and RAG-compatible text outputs'
    )
    
    parser.add_argument(
        '--dir', 
        type=str,
        required=True,
        help='Directory containing the 3D files to process'
    )
    
    parser.add_argument(
        '--verbose', 
        action='store_true',
        help='Print detailed processing information'
    )

    args = parser.parse_args()
    
    # Convert to Path object and resolve to absolute path
    input_dir = Path(args.dir).resolve()
    
    # Validate directory
    if not input_dir.exists():
        print(f"Error: Directory '{input_dir}' does not exist")
        sys.exit(1)
    
    if not input_dir.is_dir():
        print(f"Error: '{input_dir}' is not a directory")
        sys.exit(1)
        
    # Check for supported files
    supported_extensions = {'.stl', '.mtl', '.sdf', '.urdf', '.yaml', '.yml', '.obj'}
    files = [f for f in input_dir.glob("*") if f.suffix.lower() in supported_extensions]
    
    if not files:
        print(f"No supported files found in {input_dir}")
        print(f"Supported formats: {', '.join(supported_extensions)}")
        sys.exit(1)
    
    # Initialize processor
    processor = ThreeDFileProcessor(input_dir)
    
    # Process files
    print(f"Found {len(files)} supported files in {input_dir}")
    print("Starting processing...")
    
    if args.verbose:
        print("\nFiles to process:")
        for f in files:
            print(f"- {f.name}")
        print("\nProcessing...")
    
    processor.process_all_files()
    
    output_dir = input_dir / "processed_outputs"
    print("\nProcessing complete!")
    print(f"Output files are in: {output_dir}")
    
    if args.verbose:
        print("\nGenerated files:")
        for f in output_dir.glob("*"):
            print(f"- {f.name}")