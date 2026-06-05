import coacd
import trimesh
import os

input_file = "untitled.stl"
output_dir = "truck_collision_parts"
os.makedirs(output_dir, exist_ok=True)

mesh = trimesh.load(input_file, force="mesh")

coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)

parts = coacd.run_coacd(
    coacd_mesh,
    threshold=0.01,
    max_convex_hull=1000,
    preprocess_mode="auto"
)

for i, part in enumerate(parts):
    vertices, faces = part
    part_mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    part_mesh.export(f"{output_dir}/truck_col_{i}.stl")

print(f"Exported {len(parts)} convex collision parts.")
