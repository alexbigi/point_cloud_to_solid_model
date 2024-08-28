import struct
import numpy as np
import open3d as o3d

def cloud_to_mesh(rcp_file: str, obj_file: str):
    pcd = o3d.io.read_point_cloud(rcp_file)
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 2 * avg_dist
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(
        [radius, radius * 2]))
    # mesh = mesh.simplify_quadric_decimation(100000)
    # mesh.remove_degenerate_triangles()
    # mesh.remove_duplicated_triangles()
    # mesh.remove_duplicated_vertices()
    # mesh.remove_non_manifold_edges()
    o3d.visualization.draw_geometries([mesh])

    poisson_mesh = \
    o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10, width=0, scale=1.1, linear_fit=False)[0]
    o3d.visualization.draw_geometries([poisson_mesh])

    o3d.io.write_triangle_mesh(obj_file, mesh)


if __name__ == "__main__":
    # Example usage:
    cloud_to_mesh("model.pcd", "model.obj")
