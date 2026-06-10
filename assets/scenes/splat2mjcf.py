#!/usr/bin/env python3
"""
splat2mjcf — convert Gaussian splats into MuJoCo-ready collision assets.

Why this exists: MuJoCo replaces every mesh geom with its CONVEX HULL for
collision. A Poisson/marching-cubes STL of a whole scene therefore collides
as one giant blob. The representations MuJoCo is actually good at are:

  hfield  — native heightfield (best for terrain; single geom; exact contacts)
  boxes   — union of axis-aligned boxes from greedy-meshed voxels (no hulls!)
  mesh    — convex decomposition (CoACD / V-HACD) into many small hulls

Inputs:
  *.ply          3D Gaussian Splatting PLY (INRIA layout: opacity/scale/rot)
  *.splat        antimatter15 format
  *.glb/.gltf/.stl/.obj   meshes (e.g. splat-transform's *.collision.glb)
  (.ksplat/.spz/.sog: convert to .ply first with `splat-transform in.ksplat out.ply`)

Outputs (per mode) in -o OUTDIR:
  scene.xml               standalone test scene (load in simulate / mujoco_wasm)
  collision_include.xml   <asset>+<worldbody> snippet to merge into your model
  hfield.png / hfield.bin (hfield mode) file-based alternatives to inline data
  hulls/*.stl             (mesh mode, unless --inline)
  report.json             stats & parameters

Subcommand:
  splat2mjcf.py inline-stls DIR -o out.xml   pack a folder of convex-part STLs
                                             into ONE xml (vertex-only meshes)
"""
import argparse, json, math, os, struct, sys, time
import numpy as np

# ----------------------------------------------------------------------------- 
# small utils
# -----------------------------------------------------------------------------

def log(msg):
    print(f"[splat2mjcf] {msg}", flush=True)

def fmt(x):
    return f"{x:.6g}"

def quat_to_mat(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
        2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
        2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        axis=1).reshape(-1, 3, 3)

UP_ROT = {
    "z":  np.eye(3),
    "y":  np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),    # +Y -> +Z
    "-y": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),    # -Y -> +Z
    "x":  np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float).T,  # +X -> +Z
    "-z": np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float),   # -Z -> +Z
}

# -----------------------------------------------------------------------------
# readers
# -----------------------------------------------------------------------------

def read_gaussian_ply(path):
    from plyfile import PlyData
    ply = PlyData.read(path)
    v = ply["vertex"]
    names = set(v.data.dtype.names)
    xyz = np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64)
    op = (1.0 / (1.0 + np.exp(-np.asarray(v["opacity"], np.float64)))
          if "opacity" in names else np.ones(len(xyz)))
    scales = quats = None
    if {"scale_0", "scale_1", "scale_2"} <= names:
        s = np.stack([v["scale_0"], v["scale_1"], v["scale_2"]], 1).astype(np.float64)
        scales = np.exp(s)
        # some exporters store linear scales; detect absurd values after exp
        diag = float(np.linalg.norm(xyz.max(0) - xyz.min(0))) + 1e-9
        if np.median(scales) > 0.5 * diag:
            log("scales look linear (not log) — using raw values")
            scales = np.abs(s)
    if {"rot_0", "rot_1", "rot_2", "rot_3"} <= names:
        q = np.stack([v["rot_0"], v["rot_1"], v["rot_2"], v["rot_3"]], 1).astype(np.float64)
        q /= (np.linalg.norm(q, axis=1, keepdims=True) + 1e-12)
        quats = q
    return xyz, op, scales, quats


def read_dotsplat(path):
    raw = np.fromfile(path, np.uint8)
    n = len(raw) // 32
    rec = raw[: n * 32].reshape(n, 32)
    f = rec[:, :24].copy().view(np.float32).reshape(n, 6)
    xyz = f[:, :3].astype(np.float64)
    scales = np.abs(f[:, 3:6]).astype(np.float64)
    op = rec[:, 27].astype(np.float64) / 255.0
    q = (rec[:, 28:32].astype(np.float64) - 128.0) / 128.0
    q /= (np.linalg.norm(q, axis=1, keepdims=True) + 1e-12)
    return xyz, op, scales, q


def read_mesh_as_points(path, voxel):
    """Sample a mesh surface densely enough to voxelize at `voxel` resolution.
    Also returns the trimesh object so mesh-mode can use it directly."""
    import trimesh
    m = trimesh.load(path, force="mesh")
    n = int(min(6_000_000, max(50_000, 6.0 * m.area / (voxel * voxel))))
    pts, _ = trimesh.sample.sample_surface(m, n)
    return np.asarray(pts, np.float64), np.ones(n), None, None, m

# -----------------------------------------------------------------------------
# preprocessing
# -----------------------------------------------------------------------------

def preprocess(xyz, op, scales, quats, a):
    R = UP_ROT[a.up]
    xyz = xyz @ R.T
    if quats is not None:
        # only ellipsoid orientation matters for sampling; rotate via matrices
        pass  # handled in densify by rotating offsets then R
    xyz *= a.scale
    if scales is not None:
        scales = scales * a.scale

    keep = op >= a.min_opacity
    log(f"opacity filter (>= {a.min_opacity}): kept {keep.sum()}/{len(op)}")
    xyz, op = xyz[keep], op[keep]
    scales = scales[keep] if scales is not None else None
    quats = quats[keep] if quats is not None else None

    if a.crop:
        c = a.crop
        m = ((xyz[:, 0] >= c[0]) & (xyz[:, 0] <= c[1]) &
             (xyz[:, 1] >= c[2]) & (xyz[:, 1] <= c[3]) &
             (xyz[:, 2] >= c[4]) & (xyz[:, 2] <= c[5]))
        xyz, op = xyz[m], op[m]
        scales = scales[m] if scales is not None else None
        quats = quats[m] if quats is not None else None
        log(f"crop box: kept {m.sum()} splats")
    elif a.auto_crop > 0:
        lo = np.percentile(xyz, a.auto_crop, axis=0)
        hi = np.percentile(xyz, 100 - a.auto_crop, axis=0)
        pad = 0.05 * (hi - lo)
        lo, hi = lo - pad, hi + pad
        m = np.all((xyz >= lo) & (xyz <= hi), axis=1)
        xyz, op = xyz[m], op[m]
        scales = scales[m] if scales is not None else None
        quats = quats[m] if quats is not None else None
        log(f"auto-crop ({a.auto_crop}th pct): kept {m.sum()} splats, "
            f"bounds {np.round(lo,3)}..{np.round(hi,3)}")

    if a.outlier_k > 0 and len(xyz) > a.outlier_k + 1:
        from scipy.spatial import cKDTree
        d, _ = cKDTree(xyz).query(xyz, k=a.outlier_k + 1)
        dk = d[:, -1]
        thr = dk.mean() + a.outlier_sigma * dk.std()
        m = dk <= thr
        xyz, op = xyz[m], op[m]
        scales = scales[m] if scales is not None else None
        quats = quats[m] if quats is not None else None
        log(f"outlier removal: kept {m.sum()} splats (kNN dist <= {thr:.4f})")

    return xyz, op, scales, quats, R


def auto_voxel(xyz, requested, max_dim):
    if requested > 0:
        v = requested
    else:
        from scipy.spatial import cKDTree
        idx = np.random.default_rng(0).choice(len(xyz), min(len(xyz), 20000), replace=False)
        d, _ = cKDTree(xyz).query(xyz[idx], k=2)
        v = 2.0 * float(np.median(d[:, 1]))
        v = max(v, 1e-4)
        log(f"auto voxel size from splat spacing: {v:.4f}")
    ext = xyz.max(0) - xyz.min(0)
    need = float(ext.max()) / max_dim
    if v < need:
        log(f"voxel raised {v:.4f} -> {need:.4f} to respect --max-dim {max_dim}")
        v = need
    return v


def densify(xyz, op, scales, quats, R_up, voxel, rng, max_extra=4_000_000):
    """Add samples inside large gaussians so elongated splats fill voxels."""
    if scales is None:
        return xyz, op
    smax = scales.max(1)
    big = smax > 0.6 * voxel
    if not big.any():
        return xyz, op
    k = np.ceil(np.prod(np.clip(scales[big] / (0.6 * voxel), 1, None), axis=1)).astype(int)
    k = np.clip(k, 2, 96)
    tot = int(k.sum())
    if tot > max_extra:
        k = np.maximum(2, (k * (max_extra / tot)).astype(int))
        tot = int(k.sum())
    idx = np.repeat(np.nonzero(big)[0], k)
    # tight scatter: fill gaps along the splat surface without inflating it
    off = rng.normal(size=(tot, 3)) * (scales[idx] * 0.6)
    off = np.clip(off, -1.5 * scales[idx], 1.5 * scales[idx])
    if quats is not None:
        Rm = quat_to_mat(quats[idx])
        off = np.einsum("nij,nj->ni", Rm, off)
    off = off @ R_up.T
    pts = np.concatenate([xyz, xyz[idx] + off])
    w = np.concatenate([op, op[idx]])
    log(f"densified large splats: +{tot} samples ({big.sum()} splats affected)")
    return pts, w

# -----------------------------------------------------------------------------
# voxelization
# -----------------------------------------------------------------------------

def voxelize(pts, w, voxel, fill, close_iters, min_component, erode=0):
    from scipy import ndimage
    lo = pts.min(0) - voxel
    hi = pts.max(0) + voxel
    dims = np.maximum(2, np.ceil((hi - lo) / voxel).astype(int))
    ijk = np.clip(((pts - lo) / voxel).astype(int), 0, dims - 1)
    flat = np.ravel_multi_index((ijk[:, 0], ijk[:, 1], ijk[:, 2]), dims)
    acc = np.bincount(flat, weights=w, minlength=int(np.prod(dims)))
    occ = (acc.reshape(dims) >= 0.5)
    del acc
    log(f"grid {dims[0]}x{dims[1]}x{dims[2]} @ {voxel:.4f} m, "
        f"{int(occ.sum())} occupied voxels")

    if close_iters > 0:
        occ = ndimage.binary_closing(occ, structure=np.ones((3, 3, 3), bool),
                                     iterations=close_iters)
    if min_component > 0:
        lab, nlab = ndimage.label(occ)
        if nlab > 1:
            sizes = np.bincount(lab.ravel())
            keep = sizes >= min_component
            keep[0] = False
            occ = keep[lab]
            log(f"removed {int((~keep[1:]).sum())} small components "
                f"(< {min_component} voxels)")

    if fill == "floor":
        # fill every occupied column from its lowest voxel down to grid bottom
        has = occ.any(axis=2)
        first = np.argmax(occ, axis=2)               # lowest occupied k
        kk = np.arange(occ.shape[2])[None, None, :]
        occ |= (kk <= first[:, :, None]) & has[:, :, None]
        log(f"floor fill -> {int(occ.sum())} voxels")
    elif fill == "solid":
        ext = ~occ
        lab, _ = ndimage.label(ext)
        border = set(np.unique(np.concatenate([
            lab[0].ravel(), lab[-1].ravel(), lab[:, 0].ravel(), lab[:, -1].ravel(),
            lab[:, :, 0].ravel(), lab[:, :, -1].ravel()])))
        border.discard(0)
        outside = np.isin(lab, list(border))
        occ |= ext & ~outside
        log(f"solid fill -> {int(occ.sum())} voxels")
    if erode > 0:
        # counteract gaussian surface inflation (~0.5-1.5 voxels of splat sigma
        # ends up outside the true surface). plain erosion; thin features
        # (< 2*erode+1 voxels) will vanish, so keep erode=1 and voxel small.
        before = int(occ.sum())
        occ = ndimage.binary_erosion(occ, iterations=erode, border_value=0)
        log(f"eroded {erode} voxel(s): {before} -> {int(occ.sum())} voxels")
    return occ, lo, voxel

# -----------------------------------------------------------------------------
# backend: hfield
# -----------------------------------------------------------------------------

def backend_hfield(pts, a, outdir):
    from scipy import ndimage
    lo = pts.min(0); hi = pts.max(0)
    ext = hi - lo
    res = a.hfield_res
    if ext[0] >= ext[1]:
        ncol = res; nrow = max(8, int(round(res * ext[1] / max(ext[0], 1e-9))))
    else:
        nrow = res; ncol = max(8, int(round(res * ext[0] / max(ext[1], 1e-9))))
    ix = np.clip(((pts[:, 0] - lo[0]) / ext[0] * (ncol - 1)).astype(int), 0, ncol - 1)
    iy = np.clip(((pts[:, 1] - lo[1]) / ext[1] * (nrow - 1)).astype(int), 0, nrow - 1)
    H = np.full((nrow, ncol), -np.inf, np.float64)
    np.maximum.at(H, (iy, ix), pts[:, 2])
    mask = np.isfinite(H)
    if not mask.all():
        ind = ndimage.distance_transform_edt(~mask, return_indices=True)[1]
        H = H[ind[0], ind[1]]
        log(f"hfield: inpainted {int((~mask).sum())} empty cells")
    if a.hfield_smooth > 0:
        H = ndimage.gaussian_filter(H, a.hfield_smooth)
    h0, h1 = float(H.min()), float(H.max())
    zrange = max(h1 - h0, 1e-4)
    Hn = (H - h0) / zrange

    # files: PNG (top row = +y) and MuJoCo custom binary (row-major, row 0 = -y)
    from PIL import Image
    png = (np.flipud(Hn) * 65535.0 + 0.5).astype(np.uint16)
    Image.fromarray(png).save(os.path.join(outdir, "hfield.png"))
    with open(os.path.join(outdir, "hfield.bin"), "wb") as f:
        f.write(struct.pack("<ii", nrow, ncol))
        f.write(Hn.astype("<f4").tobytes())

    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    size = (ext[0] / 2, ext[1] / 2, zrange, a.hfield_base)
    # inline elevation: XML rows are top-to-bottom (+y first); mjModel flips.
    elev = " ".join(fmt(v) for v in np.flipud(Hn).ravel())
    asset_inline = (f'<hfield name="terrain" nrow="{nrow}" ncol="{ncol}" '
                    f'size="{fmt(size[0])} {fmt(size[1])} {fmt(size[2])} {fmt(size[3])}" '
                    f'elevation="{elev}"/>')
    asset_png = (f'<hfield name="terrain" file="hfield.png" '
                 f'size="{fmt(size[0])} {fmt(size[1])} {fmt(size[2])} {fmt(size[3])}"/>')
    geom = (f'<geom name="terrain" type="hfield" hfield="terrain" '
            f'pos="{fmt(cx)} {fmt(cy)} {fmt(h0)}" rgba="0.45 0.55 0.4 1"/>')
    stats = dict(nrow=nrow, ncol=ncol, zmin=h0, zmax=h1,
                 center=[cx, cy], size=list(size))
    return asset_inline, asset_png, geom, stats

# -----------------------------------------------------------------------------
# backend: boxes (greedy meshing)
# -----------------------------------------------------------------------------

def greedy_boxes(occ):
    occ = occ.copy()
    D0, D1, D2 = occ.shape
    boxes = []
    flat = occ.ravel()
    while True:
        p = int(np.argmax(flat))
        if not flat[p]:
            break
        i0, j0, k0 = np.unravel_index(p, occ.shape)
        i1 = i0
        while i1 + 1 < D0 and occ[i1 + 1, j0, k0]:
            i1 += 1
        j1 = j0
        while j1 + 1 < D1 and occ[i0:i1 + 1, j1 + 1, k0].all():
            j1 += 1
        k1 = k0
        while k1 + 1 < D2 and occ[i0:i1 + 1, j0:j1 + 1, k1 + 1].all():
            k1 += 1
        occ[i0:i1 + 1, j0:j1 + 1, k0:k1 + 1] = False
        boxes.append((i0, i1, j0, j1, k0, k1))
    return boxes


def backend_boxes(occ, lo, voxel, a):
    t0 = time.time()
    boxes = greedy_boxes(occ)
    log(f"greedy meshing: {len(boxes)} boxes in {time.time()-t0:.1f}s")
    if len(boxes) > 3000:
        log("WARNING: >3000 box geoms — increase --voxel or lower --max-dim")
    geoms = []
    for n, (i0, i1, j0, j1, k0, k1) in enumerate(boxes):
        c = lo + voxel * np.array([(i0 + i1 + 1) / 2, (j0 + j1 + 1) / 2, (k0 + k1 + 1) / 2])
        h = voxel * np.array([(i1 - i0 + 1) / 2, (j1 - j0 + 1) / 2, (k1 - k0 + 1) / 2])
        geoms.append(f'<geom name="vox{n}" type="box" pos="{fmt(c[0])} {fmt(c[1])} '
                     f'{fmt(c[2])}" size="{fmt(h[0])} {fmt(h[1])} {fmt(h[2])}" '
                     f'rgba="0.5 0.5 0.55 1"/>')
    return geoms, dict(n_boxes=len(boxes))

# -----------------------------------------------------------------------------
# backend: mesh (marching cubes -> decimate -> convex decomposition)
# -----------------------------------------------------------------------------

def marching_tile(field, lo, voxel, i0, i1, j0, j1, level):
    from skimage import measure
    sub = field[i0:i1, j0:j1, :]
    if sub.max() < level:
        return None
    v, f, _, _ = measure.marching_cubes(sub, level=level,
                                        spacing=(voxel, voxel, voxel))
    v = v + lo + voxel * np.array([i0, j0, 0])
    return v, f


def decompose(v, f, a):
    parts = []
    if a.backend == "coacd":
        import coacd
        coacd.set_log_level("error")
        mesh = coacd.Mesh(v.astype(np.float64), f.astype(np.int64))
        out = coacd.run_coacd(
            mesh, threshold=a.threshold, max_convex_hull=a.max_hulls_per_chunk,
            preprocess_resolution=a.preprocess_resolution, resolution=a.resolution,
            mcts_nodes=a.mcts_nodes, mcts_iterations=a.mcts_iterations,
            max_ch_vertex=a.max_hull_verts, seed=0)
        parts = [(np.asarray(pv), np.asarray(pf)) for pv, pf in out]
    else:
        import vhacdx
        out = vhacdx.compute_vhacd(
            v.astype(np.float64), f.astype(np.uint32).ravel(),
            maxConvexHulls=a.max_hulls_per_chunk, resolution=a.resolution_vhacd,
            maxNumVerticesPerCH=a.max_hull_verts,
            minimumVolumePercentErrorAllowed=a.vhacd_error)
        parts = [(np.asarray(pv), np.asarray(pf, np.int64)) for pv, pf in out]
    return parts


def backend_mesh(occ, lo, voxel, a, outdir):
    import trimesh
    from scipy import ndimage
    import fast_simplification

    field = occ.astype(np.float32)
    if a.fill == "none":
        field = ndimage.binary_dilation(occ, iterations=1).astype(np.float32)
    field = ndimage.gaussian_filter(field, a.mc_smooth)

    D0, D1 = occ.shape[:2]
    nx = ny = max(1, a.chunks)
    xs = np.linspace(0, D0, nx + 1).astype(int)
    ys = np.linspace(0, D1, ny + 1).astype(int)

    all_parts = []
    tile_id = 0
    for ti in range(nx):
        for tj in range(ny):
            i0 = max(0, xs[ti] - 1); i1 = min(D0, xs[ti + 1] + 1)
            j0 = max(0, ys[tj] - 1); j1 = min(D1, ys[tj + 1] + 1)
            mc = marching_tile(field, lo, voxel, i0, i1, j0, j1, level=0.5)
            if mc is None:
                continue
            v, f = mc
            target = max(400, a.target_faces // (nx * ny))
            if len(f) > target:
                v, f = fast_simplification.simplify(
                    v.astype(np.float32), f.astype(np.int64), target_count=target)
            t0 = time.time()
            parts = decompose(np.asarray(v, np.float64), np.asarray(f), a)
            log(f"tile {tile_id} ({len(f)} faces) -> {len(parts)} hulls "
                f"[{a.backend}, {time.time()-t0:.1f}s]")
            all_parts.extend(parts)
            tile_id += 1

    log(f"total convex parts: {len(all_parts)}")
    assets, geoms = [], []
    hull_dir = os.path.join(outdir, "hulls")
    if not a.inline:
        os.makedirs(hull_dir, exist_ok=True)
    for n, (pv, pf) in enumerate(all_parts):
        hull = trimesh.Trimesh(pv, pf, process=True).convex_hull
        name = f"part{n:03d}"
        if a.inline:
            vs = " ".join(fmt(x) for x in np.asarray(hull.vertices).ravel())
            assets.append(f'<mesh name="{name}" vertex="{vs}"/>')
        else:
            hull.export(os.path.join(hull_dir, f"{name}.stl"))
            assets.append(f'<mesh name="{name}" file="{name}.stl"/>')
        geoms.append(f'<geom type="mesh" mesh="{name}" rgba="0.55 0.5 0.6 1"/>')
    return assets, geoms, dict(n_hulls=len(all_parts), inline=a.inline)

# -----------------------------------------------------------------------------
# MJCF emit
# -----------------------------------------------------------------------------

SCENE_TMPL = """<mujoco model="splat_collision">
  <compiler angle="radian"{meshdir}/>
  <option timestep="0.002"/>
  <visual><headlight ambient="0.4 0.4 0.4"/></visual>
  <asset>
    {assets}
  </asset>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1"/>
    {geoms}
    <body name="probe" pos="{px} {py} {pz}">
      <freejoint/>
      <geom name="probe" type="sphere" size="0.05" density="300" rgba="0.2 0.6 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""

INCLUDE_TMPL = """<mujocoinclude>
  <asset>
    {assets}
  </asset>
  <worldbody>
    {geoms}
  </worldbody>
</mujocoinclude>
"""

def emit(outdir, assets, geoms, probe, meshdir=""):
    md = f' meshdir="{meshdir}"' if meshdir else ""
    scene = SCENE_TMPL.format(meshdir=md, assets="\n    ".join(assets),
                              geoms="\n    ".join(geoms),
                              px=fmt(probe[0]), py=fmt(probe[1]), pz=fmt(probe[2]))
    with open(os.path.join(outdir, "scene.xml"), "w") as f:
        f.write(scene)
    inc = INCLUDE_TMPL.format(assets="\n    ".join(assets),
                              geoms="\n    ".join(geoms))
    with open(os.path.join(outdir, "collision_include.xml"), "w") as f:
        f.write(inc)

# -----------------------------------------------------------------------------
# inline-stls subcommand
# -----------------------------------------------------------------------------

def inline_stls(d, out):
    import trimesh
    files = sorted(f for f in os.listdir(d) if f.lower().endswith((".stl", ".obj")))
    assets, geoms = [], []
    for n, fn in enumerate(files):
        m = trimesh.load(os.path.join(d, fn), force="mesh").convex_hull
        name = f"part{n:03d}"
        vs = " ".join(fmt(x) for x in np.asarray(m.vertices).ravel())
        assets.append(f'<mesh name="{name}" vertex="{vs}"/>')
        geoms.append(f'<geom type="mesh" mesh="{name}" group="3"/>')
    inc = INCLUDE_TMPL.format(assets="\n    ".join(assets), geoms="\n    ".join(geoms))
    with open(out, "w") as f:
        f.write(inc)
    log(f"packed {len(files)} STLs -> {out} "
        f"({os.path.getsize(out)/1e6:.2f} MB, gzip it for serving)")

# -----------------------------------------------------------------------------
# main
# -----------------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("input")
    p.add_argument("--mode", choices=["hfield", "boxes", "mesh"], default="hfield")
    p.add_argument("-o", "--outdir", default="out")
    # frame & cleanup
    p.add_argument("--up", choices=list(UP_ROT), default="z",
                   help="which input axis is 'up'; rotated to MuJoCo +Z")
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--min-opacity", type=float, default=0.4)
    p.add_argument("--crop", type=float, nargs=6, metavar=("X0","X1","Y0","Y1","Z0","Z1"))
    p.add_argument("--auto-crop", type=float, default=0.5,
                   help="percentile crop (0 disables)")
    p.add_argument("--outlier-k", type=int, default=8)
    p.add_argument("--outlier-sigma", type=float, default=3.0)
    # voxel grid
    p.add_argument("--voxel", type=float, default=0.0, help="0 = auto from spacing")
    p.add_argument("--max-dim", type=int, default=288)
    p.add_argument("--close-iters", type=int, default=1)
    p.add_argument("--min-component", type=int, default=40)
    p.add_argument("--erode", type=int, default=0, metavar="N",
        help="erode occupancy by N voxels after fill; counteracts gaussian "
             "surface inflation in boxes/mesh modes (try 1). thin features "
             "narrower than 2N+1 voxels are lost. hfield mode ignores this.")
    p.add_argument("--fill", choices=["none", "floor", "solid"], default="none",
                   help="floor: fill under lowest surface per column (terrain); "
                        "solid: fill enclosed cavities (objects/rooms)")
    # hfield
    p.add_argument("--hfield-res", type=int, default=192)
    p.add_argument("--hfield-smooth", type=float, default=1.0)
    p.add_argument("--hfield-base", type=float, default=0.2)
    # mesh / decomposition
    p.add_argument("--backend", choices=["coacd", "vhacd"], default="coacd")
    p.add_argument("--chunks", type=int, default=1,
                   help="split scene into NxN tiles before decomposition")
    p.add_argument("--target-faces", type=int, default=20000)
    p.add_argument("--threshold", type=float, default=0.06, help="CoACD concavity")
    p.add_argument("--max-hulls", type=int, default=256)
    p.add_argument("--max-hull-verts", type=int, default=64)
    p.add_argument("--mc-smooth", type=float, default=1.0)
    p.add_argument("--preprocess-resolution", type=int, default=40)
    p.add_argument("--resolution", type=int, default=2000)
    p.add_argument("--mcts-nodes", type=int, default=12)
    p.add_argument("--mcts-iterations", type=int, default=80)
    p.add_argument("--resolution-vhacd", type=int, default=200000)
    p.add_argument("--vhacd-error", type=float, default=1.0)
    p.add_argument("--inline", action="store_true",
                   help="embed hull vertices in the XML instead of STL files")
    p.add_argument("--seed", type=int, default=0)
    return p


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "inline-stls":
        q = argparse.ArgumentParser()
        q.add_argument("cmd"); q.add_argument("dir")
        q.add_argument("-o", "--out", default="collision_inline.xml")
        qa = q.parse_args()
        inline_stls(qa.dir, qa.out)
        return

    a = build_parser().parse_args()
    a.max_hulls_per_chunk = max(4, a.max_hulls // max(1, a.chunks * a.chunks))
    os.makedirs(a.outdir, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    t0 = time.time()

    ext = os.path.splitext(a.input)[1].lower()
    src_mesh = None
    if ext == ".ply":
        xyz, op, scales, quats = read_gaussian_ply(a.input)
    elif ext == ".splat":
        xyz, op, scales, quats = read_dotsplat(a.input)
    elif ext in (".glb", ".gltf", ".stl", ".obj"):
        v0 = a.voxel if a.voxel > 0 else 0.04
        xyz, op, scales, quats, src_mesh = read_mesh_as_points(a.input, v0)
    else:
        sys.exit(f"unsupported input {ext} (convert .ksplat/.spz/.sog to .ply "
                 f"with splat-transform first)")
    log(f"loaded {len(xyz)} splats/points from {a.input}")

    xyz, op, scales, quats, R_up = preprocess(xyz, op, scales, quats, a)
    if len(xyz) < 100:
        sys.exit("too few points after filtering — relax --min-opacity / crop")
    voxel = auto_voxel(xyz, a.voxel, a.max_dim)
    pts, w = densify(xyz, op, scales, quats, R_up, voxel, rng)

    report = dict(input=a.input, mode=a.mode, voxel=voxel,
                  n_points=len(pts), params=vars(a).copy())
    report["params"].pop("crop", None)

    if a.mode == "hfield":
        # use raw splat centers: max() over densified gaussian samples biases up ~2*sigma
        asset_inline, asset_png, geom, stats = backend_hfield(xyz, a, a.outdir)
        bounds = xyz.min(0), xyz.max(0)
        probe = ((bounds[0][0] + bounds[1][0]) / 2,
                 (bounds[0][1] + bounds[1][1]) / 2, bounds[1][2] + 0.5)
        emit(a.outdir, [asset_inline], [geom], probe)
        with open(os.path.join(a.outdir, "scene_png.xml"), "w") as f:
            f.write(SCENE_TMPL.format(meshdir="", assets=asset_png, geoms=geom,
                                      px=fmt(probe[0]), py=fmt(probe[1]), pz=fmt(probe[2])))
        report.update(stats)
    else:
        occ, lo, voxel = voxelize(pts, w, voxel, a.fill, a.close_iters, a.min_component, a.erode)
        bounds_hi = lo + voxel * np.array(occ.shape)
        probe = ((lo[0] + bounds_hi[0]) / 2, (lo[1] + bounds_hi[1]) / 2,
                 bounds_hi[2] + 0.5)
        if a.mode == "boxes":
            geoms, stats = backend_boxes(occ, lo, voxel, a)
            emit(a.outdir, ["<!-- boxes need no assets -->"], geoms, probe)
        else:
            assets, geoms, stats = backend_mesh(occ, lo, voxel, a, a.outdir)
            emit(a.outdir, assets, geoms, probe,
                 meshdir="" if a.inline else "hulls")
        report.update(stats)

    report["seconds"] = round(time.time() - t0, 1)
    with open(os.path.join(a.outdir, "report.json"), "w") as f:
        json.dump(report, f, indent=2, default=str)
    log(f"done in {report['seconds']}s -> {a.outdir}/")


if __name__ == "__main__":
    main()
