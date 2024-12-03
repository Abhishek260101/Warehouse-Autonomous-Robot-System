import cv2
import numpy as np
import yaml
import os

def get_workspace_path():
    """Get the absolute path to the workspace root directory"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(current_dir))
    return workspace_dir

def load_map_from_yaml(yaml_file):
    """Load map data and image from a YAML file
    
    Args:
        yaml_file: Path to the YAML file containing map metadata
        
    Returns:
        tuple: (map image array, map metadata dict)
    """
    workspace_path = get_workspace_path()
    yaml_path = os.path.join(workspace_path, yaml_file)
    
    # Load map metadata from YAML
    with open(yaml_path, 'r') as f:
        map_data = yaml.safe_load(f)
    
    # Load corresponding PGM image    
    pgm_file = yaml_path.replace('.yaml', '.pgm')
    map_img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
    
    if map_img is None:
        raise ValueError(f"Failed to load map image from {pgm_file}")
    
    return map_img, map_data

def find_center_line(map_img):
    """Find the vertical center line of the explored area
    
    Args:
        map_img: Grayscale map image array
        
    Returns:
        int: x-coordinate of the center line
    """
    # Find where the map has data (not unknown space)
    explored = map_img != 205  # 205 represents unknown space
    cols = np.any(explored, axis=0)
    explored_cols = np.where(cols)[0]
    
    # Return center of explored region or image center if no explored area
    if len(explored_cols) > 0:
        return (explored_cols[0] + explored_cols[-1]) // 2
    return map_img.shape[1] // 2

def merge_hallway_maps(map1_yaml, map2_yaml):
    """Merge two hallway maps with overlap in the middle
    
    Args:
        map1_yaml: Path to first map's YAML file
        map2_yaml: Path to second map's YAML file
        
    Returns:
        tuple: (merged map image array, merged map metadata dict)
    """
    # Load both maps
    map1_img, map1_data = load_map_from_yaml(map1_yaml)
    map2_img, map2_data = load_map_from_yaml(map2_yaml)
    
    print(f"Loaded maps - Map1: {map1_img.shape}, Map2: {map2_img.shape}")
    
    # Find center lines for alignment
    center1 = find_center_line(map1_img)
    center2 = find_center_line(map2_img)
    
    print(f"Found centers - Map1: {center1}, Map2: {center2}")
    
    # Calculate dimensions for merged map
    total_width = map1_img.shape[1] + map2_img.shape[1] - center1 - center2
    total_height = max(map1_img.shape[0], map2_img.shape[0])
    
    # Create empty merged map filled with unknown space
    merged_map = np.ones((total_height, total_width), dtype=np.uint8) * 205
    
    def blend_values(val1, val2):
        """Blend two map cell values
        
        Args:
            val1: First cell value
            val2: Second cell value
            
        Returns:
            int: Blended cell value
        """
        if val1 == 205 and val2 == 205:
            return 205  # Both unknown
        elif val1 == 205:
            return val2  # First unknown
        elif val2 == 205:
            return val1  # Second unknown
        else:
            # For occupied/free space, take more confident value
            return min(val1, val2)
    
    # Copy first map entirely
    merged_map[0:map1_img.shape[0], 0:map1_img.shape[1]] = map1_img
    
    # Overlay second map with blending in overlap region
    for y in range(map2_img.shape[0]):
        for x in range(map2_img.shape[1]):
            target_x = x + (map1_img.shape[1] - center1 - center2)
            if 0 <= target_x < total_width:
                map2_val = map2_img[y, x]
                merged_val = merged_map[y, target_x]
                merged_map[y, target_x] = blend_values(merged_val, map2_val)
    
    # Enhance contrast of final map
    merged_map = cv2.normalize(merged_map, None, 0, 255, cv2.NORM_MINMAX)
    
    # Create merged metadata
    merged_data = map1_data.copy()
    merged_data['image'] = 'merged_map.pgm'
    merged_data['origin'] = [
        min(map1_data['origin'][0], map2_data['origin'][0]),
        min(map1_data['origin'][1], map2_data['origin'][1]),
        0.0
    ]
    
    return merged_map, merged_data

def save_merged_map(merged_map, merged_data, output_prefix='merged_map'):
    """Save merged map image and metadata
    
    Args:
        merged_map: Merged map image array
        merged_data: Merged map metadata dict
        output_prefix: Prefix for output filenames
    """
    workspace_path = get_workspace_path()
    output_path = os.path.join(workspace_path, output_prefix)
    
    # Save original merged map
    cv2.imwrite(f'{output_path}.pgm', merged_map)
    print(f"Saved merged map to {output_path}.pgm")
    
    # Save contrast-enhanced version for better visualization
    enhanced_map = cv2.normalize(merged_map, None, 0, 255, cv2.NORM_MINMAX)
    cv2.imwrite(f'{output_path}_enhanced.pgm', enhanced_map)
    print(f"Saved enhanced map to {output_path}_enhanced.pgm")
    
    # Save metadata YAML
    with open(f'{output_path}.yaml', 'w') as f:
        yaml.dump(merged_data, f)
    print(f"Saved merged map config to {output_path}.yaml")

if __name__ == '__main__':
    try:
        # Merge the two robot maps
        merged_map, merged_data = merge_hallway_maps('carter1_map.yaml', 'carter2_map.yaml')
        
        # Save merged results
        save_merged_map(merged_map, merged_data)
        
        # Display original merged map
        cv2.imshow('Merged Map', merged_map)
        print("Press any key to close the window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        # Display contrast-enhanced version
        enhanced_map = cv2.normalize(merged_map, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow('Enhanced Merged Map', enhanced_map)
        print("Press any key to close the window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"Error: {e}")