from pathlib import Path

xml_path = Path("scutl.xml")
out_path = Path("scutl_gcr_room.xml")

xml = xml_path.read_text()

mesh_line = '    <mesh name="gcr_room_collision" file="gcr_room_collision.stl" scale="1 1 1"/>'

body_block = """
    <body name="gcr_room_collision_body" pos="0 0 0">
      <geom name="gcr_room_collision_geom"
            type="mesh"
            mesh="gcr_room_collision"
            contype="1"
            conaffinity="1"
            rgba="1 0 0 0.4"/>
    </body>
"""

if "mesh name=\"gcr_room_collision\"" not in xml:
    xml = xml.replace("  </asset>", mesh_line + "\n  </asset>", 1)

if "gcr_room_collision_body" not in xml:
    xml = xml.replace("  </worldbody>", body_block + "\n  </worldbody>", 1)

out_path.write_text(xml)

print(f"Wrote {out_path}")
print("Added visible gcr_room_collision mesh geom.")
