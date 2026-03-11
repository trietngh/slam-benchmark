#!/usr/bin/env python3

import argparse
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
    parser = argparse.ArgumentParser(description="Convert DA3 camera poses to TUM format.")
    parser.add_argument("--timestamps", required=True, help="Path to timestamps.txt")
    parser.add_argument("--camera_poses", required=True, help="Path to camera_poses.txt")
    parser.add_argument("--output", required=True, help="Path to save output tum file")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.timestamps):
        print(f"Error: Timestamps file not found at {args.timestamps}")
        return
        
    if not os.path.exists(args.camera_poses):
        print(f"Error: Camera poses file not found at {args.camera_poses}")
        return
        
    timestamps = []
    with open(args.timestamps, 'r') as f:
        for line in f:
            if line.strip():
                # Formats can vary, but typically "timestamp filename" or just "timestamp"
                parts = line.strip().split()
                timestamps.append(parts[0])
                
    poses_flat = []
    with open(args.camera_poses, 'r') as f:
        for line in f:
            if line.strip():
                poses_flat.append([float(x) for x in line.strip().split()])
                
    if len(timestamps) != len(poses_flat):
        print(f"Warning: Number of timestamps ({len(timestamps)}) does not match number of poses ({len(poses_flat)})")
        
    min_len = min(len(timestamps), len(poses_flat))
    
    with open(args.output, 'w') as f:
        for i in range(min_len):
            ts = timestamps[i]
            pose_flat = poses_flat[i]
            
            # The matrix is 4x4 (16 elements)
            if len(pose_flat) != 16:
                print(f"Error: Invalid pose format at line {i+1}. Expected 16 elements, got {len(pose_flat)}")
                continue
                
            pose_mat = np.array(pose_flat).reshape(4, 4)
            
            # Extract translation
            tx = pose_mat[0, 3]
            ty = pose_mat[1, 3]
            tz = pose_mat[2, 3]
            
            # Extract rotation
            rot_mat = pose_mat[:3, :3]
            
            # Convert rotation matrix to quaternion [x, y, z, w]
            rot = R.from_matrix(rot_mat)
            qx, qy, qz, qw = rot.as_quat()
            
            # Write to tum file
            f.write(f"{ts} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n")
            
    print(f"Successfully converted {min_len} poses to TUM format and saved to {args.output}")

if __name__ == "__main__":
    main()
