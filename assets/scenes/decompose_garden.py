import coacd
import trimesh
import os

input_file = "garden_collision.stl"
output_dir = "garden_collision_parts"

os.makedirs(output_dir, exist_ok=True)

mesh = trimesh.load(input_file, force="mesh")
coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)

parts = coacd.run_coacd(
    coacd_mesh,
    max_convex_hull=150
)

for i, part in enumerate(parts):
    vertices, faces = part

    trimesh.Trimesh(
        vertices=vertices,
        faces=faces
    ).export(
        f"{output_dir}/garden_col_{i}.stl"
    )

print(f"Exported {len(parts)} convex pieces")
