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
            if file_path.suffix.lower() in ['.stl', '.mtl', '.sdf', '.urdf', '.yaml', '.yml']:
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
            '.yml': self.process_yaml
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
        """ Parses a URDF file and returns detailed JSON and TXT formatted data """
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()

            # Extract robot name
            robot_name = root.attrib.get("name", "Unknown Robot")
            
            links = []
            joints = []
            txt_output = f"Robot Name: {robot_name}\n\n"

            txt_output += "=== LINK DETAILS ===\n"
            for link in root.findall("link"):
                link_name = link.attrib.get("name", "Unnamed Link")
                
                # Extract visuals
                visuals = [visual.attrib.get("filename", "No Mesh Provided") 
                        for visual in link.findall("visual/geometry/mesh")]

                # Extract collisions
                collisions = [collision.attrib.get("filename", "No Mesh Provided") 
                            for collision in link.findall("collision/geometry/mesh")]

                # Extract inertial properties
                mass = 0.0
                inertia = {}
                inertial = link.find("inertial")
                if inertial is not None:
                    mass_elem = inertial.find("mass")
                    if mass_elem is not None:
                        mass = float(mass_elem.attrib.get("value", 0.0))
                    
                    inertia_elem = inertial.find("inertia")
                    if inertia_elem is not None:
                        inertia = {k: float(v) for k, v in inertia_elem.attrib.items()}

                link_data = {
                    "name": link_name,
                    "visuals": visuals if visuals else ["Not Provided"],
                    "collisions": collisions if collisions else ["Not Provided"],
                    "mass": mass if mass else "Not Provided",
                    "inertia": inertia if inertia else "Not Provided",
                }
                links.append(link_data)

                # Append to TXT output
                txt_output += f"\nLink Name: {link_name}\n"
                txt_output += f"  - Visual Elements: {', '.join(visuals) if visuals else 'Not Provided'}\n"
                txt_output += f"  - Collision Elements: {', '.join(collisions) if collisions else 'Not Provided'}\n"
                txt_output += f"  - Mass: {mass} kg\n"
                txt_output += f"  - Inertia Tensor: {inertia if inertia else 'Not Provided'}\n"

            txt_output += "\n=== JOINT DETAILS ===\n"
            for joint in root.findall("joint"):
                joint_name = joint.attrib.get("name", "Unnamed Joint")
                joint_type = joint.attrib.get("type", "Unknown Type")

                parent = joint.find("parent").attrib.get("link", "Unknown")
                child = joint.find("child").attrib.get("link", "Unknown")

                # Joint limits
                limit = {}
                limit_elem = joint.find("limit")
                if limit_elem is not None:
                    limit = {k: float(v) for k, v in limit_elem.attrib.items()}

                joint_data = {
                    "name": joint_name,
                    "type": joint_type,
                    "parent": parent,
                    "child": child,
                    "limit": limit if limit else "Not Provided",
                }
                joints.append(joint_data)

                # Append to TXT output
                txt_output += f"\nJoint Name: {joint_name}\n"
                txt_output += f"  - Type: {joint_type}\n"
                txt_output += f"  - Connects Parent: {parent} to Child: {child}\n"
                txt_output += f"  - Motion Limits: {limit if limit else 'Not Provided'}\n"

            # Convert structured data to JSON
            json_output = json.dumps({
                "robot_name": robot_name,
                "total_links": len(links),
                "total_joints": len(joints),
                "links": links,
                "joints": joints
            }, indent=4)

            return json_output, txt_output

        except Exception as e:
            print(f"Error parsing URDF: {e}")
            return None, None

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
    supported_extensions = {'.stl', '.mtl', '.sdf', '.urdf', '.yaml', '.yml'}
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