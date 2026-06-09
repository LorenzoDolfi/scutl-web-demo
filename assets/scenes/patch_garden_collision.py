from pathlib import Path
import re

xml_path = Path("scutl.xml")
parts_dir = Path("garden_collision_parts")
out_path = Path("scutl_garden_coacd.xml")

xml = xml_path.read_text()

parts = sorted(
    parts_dir.glob("garden_col_*.stl"),
    key=lambda p: int(re.search(r"garden_col_(\d+)\.stl", p.name).group(1))
)

mesh_lines = [
    '    <mesh name="garden_visual" file="garden_collision.stl" scale="1 1 1"/>'
]

for i, p in enumerate(parts):
    mesh_lines.append(
        f'    <mesh name="garden_col_{i}" file="garden_collision_parts/{p.name}" scale="1 1 1"/>'
    )

xml = xml.replace(
    "  </asset>",
    "\n".join(mesh_lines) + "\n  </asset>",
    1
)

geom_lines = [
    '    <body name="garden_collision_body" pos="0 0 0">',
    '      <geom name="garden_visual_geom" type="mesh" mesh="garden_visual" contype="0" conaffinity="0" rgba="1 0 0 0.0"/>'
]

for i in range(len(parts)):
    geom_lines.append(
        f'      <geom name="garden_col_geom_{i}" type="mesh" mesh="garden_col_{i}" contype="1" conaffinity="1" rgba="0 1 0 0.25"/>'
    )

geom_lines.append("    </body>")

xml = xml.replace(
    "  </worldbody>",
    "\n".join(geom_lines) + "\n  </worldbody>",
    1
)

out_path.write_text(xml)

print(f"Wrote {out_path}")
print(f"Added {len(parts)} garden collision geoms.")