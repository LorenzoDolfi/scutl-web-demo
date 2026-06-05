#!/usr/bin/env python3
"""
ksplat_to_collision.py
======================
Converts a .ksplat file into a collision-ready .stl for MuJoCo.

Pipeline:
  .ksplat → parse Gaussian centers + alpha → filter by opacity
           → Poisson surface reconstruction → decimate → .stl

Usage:
  python ksplat_to_collision.py input.ksplat output.stl [options]

Options:
  --alpha-threshold   0..255  Minimum alpha to include a splat (default: 128)
  --poisson-depth     int     Poisson reconstruction depth (default: 9)
                              Higher = more detail but slower. 7-10 is typical.
  --target-triangles  int     Target triangle count after decimation (default: 4000)
  --no-filter-density         Skip density-based outlier removal
  --voxel-size        float   Voxel downsample size before reconstruction (default: auto)
                              Set to 0 to skip downsampling.
  --preview                   Open Open3D visualiser before saving (requires display)

Example:
  python ksplat_to_collision.py truck.ksplat truck_collision.stl --alpha-threshold 100 --target-triangles 3000
"""

import struct
import sys
import argparse
import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
#  ksplat binary format (reverse-engineered from mkkellogg/GaussianSplats3D)
#
#  File layout:
#    [0..4095]      Global header  (4096 bytes)
#    [4096..N]      Section headers, one per section (1024 bytes each)
#    [N..]          Section data blocks
#
#  Global header (views over same 4096 bytes, little-endian):
#    uint8[0]       versionMajor
#    uint8[1]       versionMinor
#    uint32[1]      maxSectionCount   (byte offset 4)
#    uint32[2]      sectionCount      (byte offset 8)
#    uint32[3]      maxSplatCount     (byte offset 12)
#    uint32[4]      splatCount        (byte offset 16)
#    uint16[10]     compressionLevel  (byte offset 20)
#    float32[6..8]  sceneCenter x,y,z (byte offsets 24,28,32)
#
#  Per-splat layout:
#    compressionLevel 0 (float32, 44 bytes/splat, SH=0):
#      [0..11]  center x,y,z   3× float32
#      [12..23] scale           3× float32
#      [24..39] rotation        4× float32
#      [40..43] color r,g,b,a   4× uint8
#
#    compressionLevel 1/2 (uint16 quantized, 24 bytes/splat, SH=0):
#      [0..5]   center x,y,z   3× uint16  (NOT float16 — raw quantized integers)
#      [6..11]  scale           3× uint16
#      [12..19] rotation        4× uint16
#      [20..23] color r,g,b,a   4× uint8
#      Decompression: world = (raw_uint16 - scaleRange) * scaleFactor + bucketCentre
# ─────────────────────────────────────────────────────────────────────────────

GLOBAL_HEADER_BYTES  = 4096
SECTION_HEADER_BYTES = 1024
BUCKET_STORAGE_FLOATS = 3   # x,y,z bucket centre
BUCKET_STORAGE_BYTES  = 12

COMPRESSION_LEVELS = {
    0: {"BytesPerCenter": 12, "ColorOffsetBytes": 40, "BytesPerSplat": 44, "ScaleRange": 1},
    1: {"BytesPerCenter":  6, "ColorOffsetBytes": 20, "BytesPerSplat": 24, "ScaleRange": 32767},
    2: {"BytesPerCenter":  6, "ColorOffsetBytes": 20, "BytesPerSplat": 24, "ScaleRange": 32767},
}


def read_uint8(buf, byte_offset):
    return struct.unpack_from("B", buf, byte_offset)[0]

def read_uint16(buf, byte_offset):
    return struct.unpack_from("<H", buf, byte_offset)[0]

def read_uint32(buf, byte_offset):
    return struct.unpack_from("<I", buf, byte_offset)[0]

def read_float32(buf, byte_offset):
    return struct.unpack_from("<f", buf, byte_offset)[0]


def parse_ksplat(path, alpha_threshold=128):
    """
    Parse a .ksplat file and return (positions, alphas) as numpy arrays.
    positions: (N,3) float32
    alphas:    (N,)  uint8
    """
    with open(path, "rb") as f:
        data = f.read()
    buf = memoryview(data)

    # ── Global header ───────────────────────────────────────────────────────
    version_major     = read_uint8(buf,  0)
    version_minor     = read_uint8(buf,  1)
    max_section_count = read_uint32(buf, 4)
    section_count     = read_uint32(buf, 8)
    max_splat_count   = read_uint32(buf, 12)
    splat_count       = read_uint32(buf, 16)
    compression_level = read_uint16(buf, 20)
    scene_cx = read_float32(buf, 24)
    scene_cy = read_float32(buf, 28)
    scene_cz = read_float32(buf, 32)

    print(f"  ksplat version: {version_major}.{version_minor}")
    print(f"  sections: {section_count}  splats: {splat_count}  compression: {compression_level}")
    print(f"  scene centre: ({scene_cx:.3f}, {scene_cy:.3f}, {scene_cz:.3f})")

    if compression_level not in COMPRESSION_LEVELS:
        raise ValueError(f"Unsupported compression level: {compression_level}")

    cl = COMPRESSION_LEVELS[compression_level]
    bytes_per_splat = cl["BytesPerSplat"]
    color_offset    = cl["ColorOffsetBytes"]
    scale_range     = cl["ScaleRange"]

    # ── Section headers ─────────────────────────────────────────────────────
    sections = []
    for i in range(max_section_count):
        sh = GLOBAL_HEADER_BYTES + i * SECTION_HEADER_BYTES
        sec_max_splats       = read_uint32(buf, sh + 4)
        bucket_size          = read_uint32(buf, sh + 8)
        bucket_count         = read_uint32(buf, sh + 12)
        bucket_block_size    = read_float32(buf, sh + 16)
        bucket_storage_sz    = read_uint16(buf, sh + 20)
        comp_scale_range     = read_uint32(buf, sh + 24) or scale_range
        full_bucket_count    = read_uint32(buf, sh + 32)
        partial_bucket_count = read_uint32(buf, sh + 36)

        sections.append({
            "maxSplatCount":             sec_max_splats,
            "bucketSize":                bucket_size,
            "bucketCount":               bucket_count,
            "bucketBlockSize":           bucket_block_size,
            "compressionScaleRange":     comp_scale_range,
            "fullBucketCount":           full_bucket_count,
            "partiallyFilledBucketCount": partial_bucket_count,
        })

    # ── Locate section data blocks ───────────────────────────────────────────
    sec_data_offsets = []
    cursor = GLOBAL_HEADER_BYTES + max_section_count * SECTION_HEADER_BYTES
    for sec in sections:
        sec_data_offsets.append(cursor)
        buckets_meta_bytes = sec["partiallyFilledBucketCount"] * 4
        buckets_data_bytes = sec["bucketCount"] * BUCKET_STORAGE_BYTES
        splat_data_bytes   = sec["maxSplatCount"] * bytes_per_splat
        cursor += buckets_meta_bytes + buckets_data_bytes + splat_data_bytes

    # ── Extract positions + alpha ────────────────────────────────────────────
    all_positions = []
    all_alphas    = []

    for sec_idx, sec in enumerate(sections):
        n = sec["maxSplatCount"]
        if n == 0:
            continue

        sec_data_base      = sec_data_offsets[sec_idx]
        buckets_meta_bytes = sec["partiallyFilledBucketCount"] * 4
        buckets_data_bytes = sec["bucketCount"] * BUCKET_STORAGE_BYTES

        # Bucket centres: array of (x,y,z) float32
        bucket_array_offset = sec_data_base + buckets_meta_bytes
        bucket_array = np.frombuffer(
            data, dtype=np.float32,
            count=sec["bucketCount"] * BUCKET_STORAGE_FLOATS,
            offset=bucket_array_offset
        ).reshape(-1, 3)

        splat_data_base   = sec_data_base + buckets_meta_bytes + buckets_data_bytes
        comp_scale_factor = sec["bucketBlockSize"] / sec["compressionScaleRange"]
        bucket_size       = sec["bucketSize"] if sec["bucketSize"] > 0 else 1

        # Read all splat data as a flat byte array for speed
        splat_bytes = np.frombuffer(data, dtype=np.uint8,
                                    count=n * bytes_per_splat,
                                    offset=splat_data_base)

        if compression_level == 0:
            # Float32 centers at offset 0, alpha at offset 43 (color+3)
            centers = splat_bytes.reshape(n, bytes_per_splat)[:, :12].view(np.float32).reshape(n, 3).copy()
            alphas  = splat_bytes.reshape(n, bytes_per_splat)[:, color_offset + 3].copy()
        else:
            # uint16 quantized centers, bucket-relative decompression
            # Raw uint16 values at byte offsets 0,2,4 per splat
            splat_uint16 = splat_bytes.reshape(n, bytes_per_splat).view(np.uint16)
            # Each splat row in uint16 view: [x,y,z, sx,sy,sz, r0,r1,r2,r3, R,G,B,A(as uint16 pairs)]
            # But color is uint8 so read separately
            xi = splat_uint16[:, 0].astype(np.float32)
            yi = splat_uint16[:, 1].astype(np.float32)
            zi = splat_uint16[:, 2].astype(np.float32)

            alphas_raw = splat_bytes.reshape(n, bytes_per_splat)[:, color_offset + 3].copy()

            sr = float(sec["compressionScaleRange"])
            sf = float(comp_scale_factor)

            # Bucket index per splat
            bucket_indices = np.arange(n) // bucket_size
            bucket_indices = np.clip(bucket_indices, 0, len(bucket_array) - 1)

            bx = bucket_array[bucket_indices, 0]
            by = bucket_array[bucket_indices, 1]
            bz = bucket_array[bucket_indices, 2]

            cx = (xi - sr) * sf + bx
            cy = (yi - sr) * sf + by
            cz = (zi - sr) * sf + bz

            centers = np.column_stack([cx, cy, cz]).astype(np.float32)
            alphas  = alphas_raw

        mask = alphas >= alpha_threshold
        all_positions.append(centers[mask])
        all_alphas.append(alphas[mask])

    if not all_positions:
        raise ValueError("No splats passed the alpha threshold — try lowering --alpha-threshold")

    positions = np.concatenate(all_positions, axis=0)
    alphas    = np.concatenate(all_alphas,    axis=0)
    return positions, alphas


def build_collision_mesh(positions, voxel_size=None, poisson_depth=9,
                          target_triangles=4000, filter_density=True, preview=False,
                          density_trim_percentile=10):
    try:
        import open3d as o3d
    except ImportError:
        print("ERROR: open3d not installed. Run: pip install open3d")
        sys.exit(1)

    print(f"\n[2] Building point cloud from {len(positions)} points...")
    print(f"    Bounds X: {positions[:,0].min():.3f} → {positions[:,0].max():.3f}")
    print(f"    Bounds Y: {positions[:,1].min():.3f} → {positions[:,1].max():.3f}")
    print(f"    Bounds Z: {positions[:,2].min():.3f} → {positions[:,2].max():.3f}")

    extent = np.ptp(positions, axis=0)
    print(f"    Extent: {extent}")

    if voxel_size is None:
        # Auto: target ~50k points, so voxel = extent / cbrt(50000)
        max_extent = float(np.max(extent))
        voxel_size = max_extent / 150.0
        print(f"    Auto voxel size: {voxel_size:.4f}")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(positions.astype(np.float64))

    if voxel_size > 0:
        print(f"[3] Voxel downsampling (size={voxel_size:.4f})...")
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f"    → {len(pcd.points)} points after downsampling")

    if filter_density:
        print("[4] Removing outliers...")
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        print(f"    → {len(pcd.points)} points after outlier removal")

    print("[5] Estimating normals...")
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=30)
    )
    pcd.orient_normals_consistent_tangent_plane(100)

    print(f"[6] Poisson surface reconstruction (depth={poisson_depth})...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=poisson_depth, linear_fit=False
    )
    print(f"    → {len(mesh.triangles)} triangles before density trim")

    densities = np.asarray(densities)
    density_threshold = np.percentile(densities, density_trim_percentile)
    vertices_to_remove = densities < density_threshold
    mesh.remove_vertices_by_mask(vertices_to_remove)
    print(f"    → {len(mesh.triangles)} triangles after density trim (percentile={density_trim_percentile})")

    print(f"[7] Decimating to {target_triangles} triangles...")
    mesh = mesh.simplify_quadric_decimation(target_number_of_triangles=target_triangles)
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    print(f"    → {len(mesh.triangles)} triangles final")

    if preview:
        print("[preview] Opening visualiser... close window to continue.")
        o3d.visualization.draw_geometries([mesh, pcd])

    return mesh


def save_stl(mesh, path):
    import open3d as o3d
    print(f"\n[8] Saving to {path}...")
    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()
    ok = o3d.io.write_triangle_mesh(path, mesh, write_ascii=False)
    if ok:
        import os
        print(f"    Done. ({path}, {os.path.getsize(path):,} bytes)")
    else:
        print("    ERROR: write failed!")


def main():
    parser = argparse.ArgumentParser(description="Convert .ksplat to collision .stl for MuJoCo")
    parser.add_argument("input",  help="Input .ksplat file")
    parser.add_argument("output", help="Output .stl file")
    parser.add_argument("--alpha-threshold",   type=int,   default=128)
    parser.add_argument("--poisson-depth",     type=int,   default=9)
    parser.add_argument("--target-triangles",  type=int,   default=4000)
    parser.add_argument("--voxel-size",        type=float, default=None,
                        help="Voxel downsample size (default: auto from scene extent)")
    parser.add_argument("--no-filter-density", action="store_true")
    parser.add_argument("--density-trim", type=int, default=10,
                        help="Percentile of low-density vertices to remove after Poisson (default: 10, try 25-50 to kill phantom hull)")
    parser.add_argument("--preview",           action="store_true")
    args = parser.parse_args()

    print(f"\n=== ksplat → collision STL pipeline ===")
    print(f"[1] Parsing {args.input}...")
    positions, alphas = parse_ksplat(args.input, alpha_threshold=args.alpha_threshold)
    print(f"    → {len(positions)} splats above alpha threshold {args.alpha_threshold}")

    mesh = build_collision_mesh(
        positions,
        voxel_size=args.voxel_size,
        poisson_depth=args.poisson_depth,
        target_triangles=args.target_triangles,
        filter_density=not args.no_filter_density,
        preview=args.preview,
        density_trim_percentile=args.density_trim,
    )

    save_stl(mesh, args.output)

    stem = args.output.replace(".stl", "")
    print(f"\n=== Complete! ===")
    print(f"In your MJCF:")
    print(f'  <mesh file="{args.output}"/>')
    print(f'  <geom type="mesh" mesh="{stem}" contype="1" conaffinity="1"/>')


if __name__ == "__main__":
    main()