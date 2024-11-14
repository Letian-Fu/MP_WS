#!/usr/bin/env python
import numpy as np
from scipy import ndimage
import math

def signedDistanceField3D(ground_truth_map, cell_size):
    # regularize unknow area to open area
    cur_map = ground_truth_map > 0.75
    cur_map = cur_map.astype(int)

    if np.amax(cur_map) == 0:
        return np.ones(ground_truth_map.shape) * 1000

    # inverse map
    inv_map = 1 - cur_map

    # get signed distance from map and inverse map
    # since bwdist(foo) = ndimage.distance_transform_edt(1-foo)
    map_dist = ndimage.distance_transform_edt(inv_map)
    inv_map_dist = ndimage.distance_transform_edt(cur_map)

    field = map_dist - inv_map_dist

    # metric
    field = field * cell_size
    field = field.astype(float)

    return field


def saveSDFToSingleFile(sdf, output_file, origin, cell_size):
    with open(output_file, 'w') as file:
        # Write metadata
        file.write(f"Origin: {origin[0]} {origin[1]} {origin[2]}\n")
        file.write(f"FieldRows: {sdf.shape[0]}\n")
        file.write(f"FieldCols: {sdf.shape[1]}\n")
        file.write(f"FieldZ: {sdf.shape[2]}\n")
        file.write(f"CellSize: {cell_size}\n")
        for z in range(sdf.shape[2]):
            # Write each layer
            for x in range(sdf.shape[1]):
                for y in range(sdf.shape[0]):
                    file.write(f"{sdf[x, y, z]:.4f} ")
                file.write("\n")
            file.write("\n")  # Add an extra newline for clarity between layers

def add_obstacle(position, size, map):
    half_size_row = int(math.floor((size[0] - 1) / 2))
    half_size_col = int(math.floor((size[1] - 1) / 2))
    half_size_z = int(math.floor((size[2] - 1) / 2))

    # occupency grid
    map[position[0] - half_size_row - 1:position[0] + half_size_row,
        position[1] - half_size_col - 1:position[1] + half_size_col,
        position[2] - half_size_z - 1:position[2] + half_size_z, ] = np.ones(
            (2 * half_size_row + 1, 2 * half_size_col + 1,
             2 * half_size_z + 1))

    return map

# Example usage
if __name__ == "__main__":
    cols = 40
    rows = 40
    z = 30
    origin_x = -1.0
    origin_y = -1.0
    origin_z = -0.35
    cell_size = 0.05
    map = np.zeros((rows, cols, z))
    #
    origin = np.asarray([origin_x, origin_y, origin_z])
    map = add_obstacle([10, 20, 16], [3, 3, 3], map)
    # floor
    # map = add_obstacle([20, 20, 4], [30, 30, 3], map)

    # Compute the signed distance field
    sdf = signedDistanceField3D(map, cell_size)

    # Save the SDF to a single file
    output_file = "/home/roboert/MP_WS/src/gp_planner/sdf_data/sdf_data_static_py.txt"
    saveSDFToSingleFile(sdf, output_file, origin, cell_size)
    print(f"Signed distance field saved to {output_file}")