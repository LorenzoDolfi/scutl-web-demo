from pathlib import Path
import re

xml_path = Path("scutl.xml")
parts_dir = Path("construction_collision_parts")
out_path = Path("scutl_construction_coacd.xml")

xml = xml_path.read_text()

parts = sorted(
    parts_dir.glob("construction_col_*.stl"),
    key=lambda p: int(re.search(r"construction_col_(\d+)\.stl", p.name).group(1))
)

mesh_lines = []

for i, p in enumerate(parts):
    mesh_lines.append(
        f'    <mesh name="construction_col_{i}" file="construction_collision_parts/{p.name}" scale="1 1 1"/>'
    )

xml = xml.replace(
    "  </asset>",
    "\n".join(mesh_lines) + "\n  </asset>",
    1
)

geom_lines = [
    '    <body name="construction_collision_body" pos="0 0 0">'
]

for i in range(len(parts)):
    geom_lines.append(
        f'      <geom name="construction_col_geom_{i}" type="mesh" mesh="construction_col_{i}" contype="1" conaffinity="1" rgba="0 1 0 1"/>'
    )

geom_lines.append("    </body>")

xml = xml.replace(
    "  </worldbody>",
    "\n".join(geom_lines) + "\n  </worldbody>",
    1
)

out_path.write_text(xml)

print(f"Wrote {out_path}")
print(f"Added {len(parts)} construction collision geoms.")