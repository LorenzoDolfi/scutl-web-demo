import coacd
import trimesh
import os

input_file = "construction_collision.stl"
output_dir = "construction_collision_parts"

os.makedirs(output_dir, exist_ok=True)

mesh = trimesh.load(input_file, force="mesh")
coacd_mesh = coacd.Mesh(mesh.vertices, mesh.faces)

parts = coacd.run_coacd(
    coacd_mesh,
    threshold=0.005,
    max_convex_hull=500
)

for i, part in enumerate(parts):
    vertices, faces = part

    trimesh.Trimesh(
        vertices=vertices,
        faces=faces
    ).export(
        f"{output_dir}/construction_col_{i}.stl"
    )

print(f"Exported {len(parts)} convex pieces")
