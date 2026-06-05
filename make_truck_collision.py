import open3d as o3d
import numpy as np

PLY_IN = "assets/splats/truck.ply"
STL_OUT = "assets/scenes/truck_collision.stl"

pcd = o3d.io.read_point_cloud(PLY_IN)
print("original points:", np.asarray(pcd.points).shape)

# Downsample hard. Increase voxel_size if mesh is still too heavy.
pcd = pcd.voxel_down_sample(voxel_size=0.5)
print("downsampled points:", np.asarray(pcd.points).shape)

pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=2.0, max_nn=30)
)

mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd,
    depth=8
)

densities = np.asarray(densities)
keep = densities > np.quantile(densities, 0.10)
mesh = mesh.select_by_index(np.where(keep)[0])

mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=5000)
mesh.remove_degenerate_triangles()
mesh.remove_duplicated_triangles()
mesh.remove_duplicated_vertices()
mesh.remove_non_manifold_edges()
mesh.compute_vertex_normals()

print("triangles:", np.asarray(mesh.triangles).shape)

o3d.io.write_triangle_mesh(STL_OUT, mesh)
print("wrote:", STL_OUT)
