import struct
import numpy as np
import open3d as o3d

def rcs_to_obj(rcp_file: str, obj_file: str, lod: int=200):
    # with open(rcp_file, "rb") as f:
    #     # Read header (adjust based on your file structure)
    #     header_size = struct.unpack("<I", f.read(4))[0]
    #     # f.read(header_size)
    #
    #     # Read points and colors (adjust according to your data structure)
    #     points = []
    #     colors = []
    #     while True:
    #         try:
    #             x = struct.unpack("<f", f.read(4))[0]
    #             y = struct.unpack("<f", f.read(4))[0]
    #             z = struct.unpack("<f", f.read(4))[0]
    #             r = struct.unpack("<B", f.read(1))[0] / 255  # Assuming color as 8-bit values
    #             g = struct.unpack("<B", f.read(1))[0] / 255
    #             b = struct.unpack("<B", f.read(1))[0] / 255
    #             points.append((x, y, z))
    #             colors.append((r, g, b))
    #         except struct.error:  # End of file
    #             break
    #
    # print(points)
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(points)
    # o3d.visualization.draw_geometries([pcd])

    pcd = o3d.io.read_point_cloud(rcp_file)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=250),
    )
    pcd.orient_normals_consistent_tangent_plane(15)

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=8)

    target_num_vertices = int(len(mesh.vertices) * (lod / 1000))

    mesh = mesh.simplify_quadric_decimation(target_num_vertices)
    # Perform surface reconstruction (optional, but recommended for a solid model)
    # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(mesh.sample_points_uniformly(
    #     number_of_points=10000), depth=8)[0]  # Using Poisson for reconstruction
    o3d.io.write_triangle_mesh(obj_file, mesh)


if __name__ == "__main__":
    # Example usage:
    rcs_to_obj("model.pts", "model.obj", 200)
