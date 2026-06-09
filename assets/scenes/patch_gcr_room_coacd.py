from pathlib import Path
import re

xml_path = Path("scutl.xml")
parts_dir = Path("gcr_room_collision_parts")
out_path = Path("scutl_gcr_room.xml")

xml = xml_path.read_text()

parts = sorted(
    parts_dir.glob("gcr_room_col_*.stl"),
    key=lambda p: int(re.search(r"gcr_room_col_(\d+)\.stl", p.name).group(1))
)

mesh_lines = []
for i, p in enumerate(parts):
    mesh_lines.append(
        f'    <mesh name="gcr_room_col_{i}" file="gcr_room_collision_parts/{p.name}" scale="1 1 1"/>'
    )

xml = xml.replace("  </asset>", "\n".join(mesh_lines) + "\n  </asset>", 1)

geom_lines = ['    <body name="gcr_room_collision_body" pos="0 0 0">']

for i in range(len(parts)):
    geom_lines.append(
        f'      <geom name="gcr_room_col_geom_{i}" type="mesh" mesh="gcr_room_col_{i}" '
        f'contype="1" conaffinity="1" rgba="1 0 0 0.4"/>'
    )

geom_lines.append("    </body>")

xml = xml.replace("  </worldbody>", "\n".join(geom_lines) + "\n  </worldbody>", 1)

out_path.write_text(xml)

print(f"Wrote {out_path}")
print(f"Added {len(parts)} GCR room collision geoms.")
