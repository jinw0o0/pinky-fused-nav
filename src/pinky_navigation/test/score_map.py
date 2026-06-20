#!/usr/bin/env python3
"""Score a saved occupancy map for the Pinky fused-scan verification (design doc §7).

Two modes:

  floor  --fused F.yaml --lidar L.yaml
      Measures depth-INTRODUCED false obstacles = cells occupied in the FUSED map
      but FREE in the LiDAR-only map (lidar can't see the floor, so anything it
      confirmed free that fused marks occupied is a depth false-positive).
      PASS if  depth-added/ swept-free <= --max-ratio  AND  no connected
      depth-added blob >= --blob-cells (a blob big enough to actually block a path).

  object --fused F.yaml [--lidar L.yaml] --objects objects.csv [--window 0.10]
      For each known object (world x,y) checks whether an occupied cell exists
      within +/- window metres in the FUSED map. PASS if every REQUIRED object is
      detected. With --lidar, also reports depth_unique (fused sees it, lidar does
      not) — the proof that depth adds value for sub-lidar-plane (<12.5cm) objects.

Maps are read in WORLD coordinates (via each .yaml origin/resolution), so the two
slam runs need not share image bounds. Deps: numpy, pyyaml (both in the ROS env).
Exit code 0 = PASS, 1 = FAIL, 2 = usage/IO error.
"""

import argparse
import os
import sys

import numpy as np
import yaml


# --------------------------------------------------------------------------- #
# Map loading
# --------------------------------------------------------------------------- #
def _read_pgm(path):
    """Minimal P5 (binary) / P2 (ascii) PGM reader -> HxW uint8 ndarray."""
    with open(path, "rb") as f:
        buf = f.read()

    def read_token(i):
        # skip whitespace and # comments
        while i < len(buf):
            c = buf[i:i + 1]
            if c.isspace():
                i += 1
            elif c == b"#":
                while i < len(buf) and buf[i:i + 1] not in (b"\n", b"\r"):
                    i += 1
            else:
                break
        j = i
        while j < len(buf) and not buf[j:j + 1].isspace():
            j += 1
        return buf[i:j], j

    magic, i = read_token(0)
    w_t, i = read_token(i)
    h_t, i = read_token(i)
    mv_t, i = read_token(i)
    w, h, maxval = int(w_t), int(h_t), int(mv_t)
    if maxval > 255:
        raise ValueError("16-bit PGM not supported (map_saver writes 8-bit)")

    if magic == b"P5":
        i += 1  # exactly one whitespace separates maxval from the binary block
        raw = buf[i:i + w * h]
        if len(raw) < w * h:
            raise ValueError("truncated P5 PGM body")
        return np.frombuffer(raw, dtype=np.uint8, count=w * h).reshape(h, w)
    if magic == b"P2":
        vals = []
        while len(vals) < w * h:
            t, i = read_token(i)
            vals.append(int(t))
        return np.array(vals, dtype=np.uint8).reshape(h, w)
    raise ValueError("unsupported PGM magic %r" % magic)


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img = meta["image"]
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)
    pgm = _read_pgm(img)
    negate = int(meta.get("negate", 0))
    occ_th = float(meta.get("occupied_thresh", 0.65))
    free_th = float(meta.get("free_thresh", 0.25))
    val = pgm.astype(np.float32)
    occ = val / 255.0 if negate else (255.0 - val) / 255.0
    occupied = occ > occ_th
    free = occ < free_th
    h, w = pgm.shape
    return {
        "res": float(meta["resolution"]),
        "ox": float(meta["origin"][0]),
        "oy": float(meta["origin"][1]),
        "h": h, "w": w,
        "occupied": occupied,
        "free": free,
    }


def world_to_px(m, wx, wy):
    col = int((wx - m["ox"]) / m["res"])
    row = m["h"] - 1 - int((wy - m["oy"]) / m["res"])
    return row, col


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def largest_blob(mask):
    """Largest 8-connected component size (cells) in a boolean mask."""
    h, w = mask.shape
    seen = np.zeros_like(mask)
    best = 0
    ys, xs = np.nonzero(mask)
    for sy, sx in zip(ys, xs):
        if seen[sy, sx]:
            continue
        size = 0
        stack = [(sy, sx)]
        seen[sy, sx] = True
        while stack:
            y, x = stack.pop()
            size += 1
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
        if size > best:
            best = size
    return best


def occupied_in_window(m, wx, wy, win_cells):
    r0, c0 = world_to_px(m, wx, wy)
    h, w = m["h"], m["w"]
    r1, r2 = max(0, r0 - win_cells), min(h, r0 + win_cells + 1)
    c1, c2 = max(0, c0 - win_cells), min(w, c0 + win_cells + 1)
    if r1 >= r2 or c1 >= c2:
        return False
    return bool(m["occupied"][r1:r2, c1:c2].any())


# --------------------------------------------------------------------------- #
# Scoring
# --------------------------------------------------------------------------- #
def depth_added_mask(fused, lidar):
    """Boolean mask (in fused grid) of cells occupied in fused but FREE in lidar."""
    h, w = fused["h"], fused["w"]
    rows, cols = np.nonzero(fused["occupied"])
    if len(rows) == 0:
        return np.zeros((h, w), dtype=bool)
    wx = fused["ox"] + (cols + 0.5) * fused["res"]
    wy = fused["oy"] + ((h - 1 - rows) + 0.5) * fused["res"]
    lcol = ((wx - lidar["ox"]) / lidar["res"]).astype(int)
    lrow = lidar["h"] - 1 - ((wy - lidar["oy"]) / lidar["res"]).astype(int)
    valid = (lrow >= 0) & (lrow < lidar["h"]) & (lcol >= 0) & (lcol < lidar["w"])
    is_free = np.zeros(len(rows), dtype=bool)
    is_free[valid] = lidar["free"][lrow[valid], lcol[valid]]
    out = np.zeros((h, w), dtype=bool)
    out[rows[is_free], cols[is_free]] = True
    return out


def score_floor(fused, lidar, max_ratio, blob_cells):
    added = depth_added_mask(fused, lidar)
    n_added = int(added.sum())
    swept = int(lidar["free"].sum())
    ratio = (n_added / swept) if swept else 0.0
    max_blob = largest_blob(added)
    ratio_ok = ratio <= max_ratio
    blob_ok = max_blob < blob_cells
    passed = ratio_ok and blob_ok

    print("=== FLOOR REJECTION ===")
    print(f"  swept (lidar-free) cells     : {swept}")
    print(f"  depth-added occupied cells   : {n_added}")
    print(f"  false-occupied ratio         : {ratio*100:.3f}%   (limit {max_ratio*100:.3f}%)  "
          f"[{'OK' if ratio_ok else 'FAIL'}]")
    print(f"  largest depth-added blob     : {max_blob} cells   (limit <{blob_cells})  "
          f"[{'OK' if blob_ok else 'FAIL'}]")
    print(f"  --> {'PASS' if passed else 'FAIL'}")
    return passed


def parse_objects(path):
    objs = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = [p.strip() for p in ln.split(",")]
            if len(parts) < 3:
                continue
            label = parts[0]
            x = float(parts[1])
            y = float(parts[2])
            height_cm = float(parts[3]) if len(parts) > 3 and parts[3] else None
            req_tok = (parts[4].lower() if len(parts) > 4 and parts[4] else "required")
            required = req_tok in ("required", "yes", "true", "1")
            objs.append({"label": label, "x": x, "y": y,
                         "height_cm": height_cm, "required": required})
    return objs


def score_objects(fused, lidar, objects, window):
    win_cells = max(0, int(round(window / fused["res"])))
    all_required_ok = True
    print("=== OBJECT DETECTION ===")
    print(f"  window +/- {window:.2f} m  ({win_cells} cells)")
    for o in objects:
        f_occ = occupied_in_window(fused, o["x"], o["y"], win_cells)
        l_occ = (occupied_in_window(lidar, o["x"], o["y"], win_cells)
                 if lidar is not None else None)
        depth_unique = f_occ and (l_occ is False)
        ok = f_occ or not o["required"]
        if o["required"] and not f_occ:
            all_required_ok = False
        tag = "REQ" if o["required"] else "opt"
        extra = ""
        if l_occ is not None:
            extra = f"  lidar={'Y' if l_occ else 'N'}  depth_unique={'Y' if depth_unique else 'N'}"
        ht = f"{o['height_cm']:.0f}cm" if o["height_cm"] is not None else "?"
        print(f"  [{tag}] {o['label']:<12} ({o['x']:.2f},{o['y']:.2f}) {ht:>5}  "
              f"fused={'Y' if f_occ else 'N'}{extra}  "
              f"[{'OK' if ok else 'FAIL'}]")
    print(f"  --> {'PASS' if all_required_ok else 'FAIL'}")
    return all_required_ok


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    pf = sub.add_parser("floor", help="floor false-positive score (needs fused + lidar)")
    pf.add_argument("--fused", required=True)
    pf.add_argument("--lidar", required=True)
    pf.add_argument("--max-ratio", type=float, default=0.005)
    pf.add_argument("--blob-cells", type=int, default=3)

    po = sub.add_parser("object", help="known-object detection score")
    po.add_argument("--fused", required=True)
    po.add_argument("--lidar", default=None)
    po.add_argument("--objects", required=True)
    po.add_argument("--window", type=float, default=0.10)

    a = ap.parse_args()
    try:
        if a.mode == "floor":
            fused = load_map(a.fused)
            lidar = load_map(a.lidar)
            ok = score_floor(fused, lidar, a.max_ratio, a.blob_cells)
        else:
            fused = load_map(a.fused)
            lidar = load_map(a.lidar) if a.lidar else None
            objs = parse_objects(a.objects)
            ok = score_objects(fused, lidar, objs, a.window)
    except (OSError, ValueError, KeyError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
