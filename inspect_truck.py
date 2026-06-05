import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("assets/splats/truck.ply")

pts = np.asarray(pcd.points)

print("points:", pts.shape)

print("min:", pts.min(axis=0))
print("max:", pts.max(axis=0))

bbox = pts.max(axis=0) - pts.min(axis=0)
print("size:", bbox)

o3d.visualization.draw_geometries([pcd])
