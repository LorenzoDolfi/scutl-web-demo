from pathlib import Path
import re

xml_path = Path("scutl.xml")  # <-- change this
parts_dir = Path("truck_collision_parts")

xml = xml_path.read_text()

parts = sorted(
    parts_dir.glob("truck_col_*.stl"),
    key=lambda p: int(re.search(r"truck_col_(\d+)\.stl", p.name).group(1))
)

# 1) Replace old truck mesh asset with visual + collision part assets
old_asset = '<mesh name="truck_collision" file="untitled.stl" scale="1 1 1"/>'

new_assets = ['<mesh name="truck_visual" file="untitled.stl" scale="1 1 1"/>']
for i, p in enumerate(parts):
    new_assets.append(
        f'<mesh name="truck_col_{i}" file="truck_collision_parts/{p.name}" scale="1 1 1"/>'
    )

xml = xml.replace(old_asset, "\n    ".join(new_assets))

# 2) Replace old single truck geom with visual-only + many collision geoms
old_geom_pattern = re.compile(
    r'<geom\s+name="truck_collision_geom"\s+type="mesh"\s+mesh="truck_collision"\s+contype="1"\s+conaffinity="1"\s+rgba="1 0 0 0\.25"\s*/>',
    re.MULTILINE
)

new_geoms = [
    '<geom name="truck_visual_geom" type="mesh" mesh="truck_visual" '
    'contype="0" conaffinity="0" rgba="1 0 0 0.25"/>'
]

for i in range(len(parts)):
    new_geoms.append(
        f'<geom name="truck_col_geom_{i}" type="mesh" mesh="truck_col_{i}" '
        f'contype="1" conaffinity="1" rgba="0 1 0 0.25"/>'
    )

xml = old_geom_pattern.sub("\n        ".join(new_geoms), xml)

out_path = xml_path.with_name(xml_path.stem + "_coacd.xml")
out_path.write_text(xml)

print(f"Wrote {out_path}")
print(f"Added {len(parts)} collision geoms.")
