import os
import json
import trimesh
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
import argparse
import sys

class ThreeDFileProcessor:
    def __init__(self, directory_path):
        self.directory = Path(directory_path)
        self.output_dir = self.directory / "processed_outputs"
        self.output_dir.mkdir(exist_ok=True)
        
    def process_all_files(self):
        """Process all supported files in the directory"""
        for file_path in self.directory.glob("*"):
            if file_path.suffix.lower() in ['.stl', '.mtl', '.sdf']:
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
            '.sdf': self.process_sdf
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
    supported_extensions = {'.stl', '.mtl', '.sdf'}
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