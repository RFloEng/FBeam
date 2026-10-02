"""Parametric silhouette body shell for the FBeam editor (regulations 6.2-6.7, 6.9).

Pure Python, standard library only (runs in Pyodide). FBeam axes: X left, Y rear, Z up,
metres; heights are from Z = 0, taken as the ground. The body is symmetric about X = 0.

The side profile is a closed polygon in the Y-Z plane (front bumper, bonnet, windscreen,
roof, rear window, boot, rear bumper, underside), extruded across the body width. Panel
mass comes from the skin area (both sides, roof, bonnet, boot, front, rear; the open
underside and the lower front chamfer are not skinned) times the panel material's mass
per square metre, so the body is paid for like everything else. Windows are counted as
panels (polycarbonate is close to thin steel per area); the body's mass, CG and size
replace the "body_panels" estimate (fbeam_editor_core.derive), so it is treated as a
uniform box there.

Body dict keys (see example_body): length, width, height (roof), front_overhang (front
bumper ahead of the front axle), ground_clearance, bonnet_height, boot_height (m),
windscreen_deg, rear_window_deg (from horizontal), roof_length (m), material (a MATERIALS
key), closed_roof. Checks: length 6.2, width 6.3, roof height 6.5, overhangs 6.6, closed
roof 6.7, ground clearance 6.9.
"""

import math

MATERIALS = {                       # kg/m^2 of panel skin
    "steel_0.8mm": 7850 * 0.0008,
    "aluminium_1.0mm": 2700 * 0.0010,
    "composite_2.0mm": 1600 * 0.0020,
}


def example_body():
    """Covet hatchback proportions (donor body nodes): 3.92 x 1.68 x 1.34 m."""
    return {"length": 3.92, "width": 1.68, "height": 1.34, "front_overhang": 0.75, "ground_clearance": 0.15,
            "bonnet_height": 0.86, "boot_height": 0.85, "windscreen_deg": 31, "rear_window_deg": 48,
            "roof_length": 1.45, "material": "steel_0.8mm", "closed_roof": True}


def profile(b, y_front_axle):
    """Side profile, front to rear over the top then back along the underside: [(y, z), ...].

    11 points: 0 front bumper bottom, 1-3 bumper top and bonnet, 4-5 roof, 6 boot, 7 rear
    bumper top (prof[7] is the rear bumper Y), 8-10 rear bumper bottom and underside.
    The bumpers and the underside corners are chamfered by 0.10 m.
    """
    yf = y_front_axle - b["front_overhang"]           # front bumper
    yr = yf + b["length"]                              # rear bumper
    gc, top = b["ground_clearance"], b["height"]
    ws = math.radians(b["windscreen_deg"])             # angle of the screen from horizontal
    rw = math.radians(b["rear_window_deg"])
    bonnet_z, boot_z = b["bonnet_height"], b["boot_height"]
    ws_run = (top - bonnet_z) / math.tan(ws)
    rw_run = (top - boot_z) / math.tan(rw)
    y_ws = yf + 0.10 + max(0.2, b["length"] * 0.24)    # screen base: end of the bonnet (bonnet ~24 % of the length)
    y_roof_f = y_ws + ws_run
    y_roof_r = y_roof_f + b["roof_length"]
    y_boot = min(y_roof_r + rw_run, yr - 0.10)
    return [(yf, gc + 0.10), (yf, bonnet_z - 0.10), (yf + 0.10, bonnet_z), (y_ws, bonnet_z), (y_roof_f, top),
            (y_roof_r, top), (y_boot, boot_z), (yr, boot_z - 0.05), (yr, gc + 0.10), (yr - 0.10, gc), (yf + 0.10, gc)]


def _polygon(prof):
    """(area, (cy, cz) centroid) of a closed polygon by the shoelace formula:
    A = sum(c_i) / 2, cy = sum((y_i + y_i+1) c_i) / 6A with c_i = y_i z_i+1 - y_i+1 z_i (signed, any winding)."""
    A = cy = cz = 0.0
    n = len(prof)
    for i in range(n):
        y0, z0 = prof[i]
        y1, z1 = prof[(i + 1) % n]
        c = y0 * z1 - y1 * z0
        A += c
        cy += (y0 + y1) * c
        cz += (z0 + z1) * c
    A /= 2
    return abs(A), (cy / (6 * A), cz / (6 * A))


def analyse(b, y_front_axle, rules):
    """Profile, skin area, panel mass and CG of body b, and its size checks.

    y_front_axle: front axle Y (m), which places the body. rules: full rules dict (dimensions).
    Returns {"profile", "side_area_m2", "skin_area_m2", "kg_per_m2", "mass" kg, "cg" [x, y, z],
    "size" [width, length, height above the underside] (the box used for inertia),
    "front_bumper", "rear_bumper" (Y, m), "issues"}.
    """
    prof = profile(b, y_front_axle)
    w = b["width"]
    side_area, (cy_side, cz_side) = _polygon(prof)
    # perimeter panels: every segment of the closed profile except the underside (9-10), which the
    # chassis floor closes; both lower chamfers (8-9 rear, 10-0 front) are skinned
    n = len(prof)
    segs = [(prof[i], prof[(i + 1) % n]) for i in range(n) if i != n - 2]
    lengths = [math.dist(a, c) for a, c in segs]
    top_len = sum(lengths)
    mids = [((a[0] + c[0]) / 2, (a[1] + c[1]) / 2) for a, c in segs]
    area_top = top_len * w
    area = 2 * side_area + area_top
    kg_m2 = MATERIALS[b["material"]]
    mass = area * kg_m2
    # CG of the skin: both sides at their polygon centroid, the wrap panels at their segment midpoints
    my = 2 * side_area * cy_side + sum(L * w * m[0] for L, m in zip(lengths, mids))
    mz = 2 * side_area * cz_side + sum(L * w * m[1] for L, m in zip(lengths, mids))
    cg = [0.0, my / area, mz / area]
    yf, yr = prof[0][0], prof[7][0]
    d = rules["dimensions"]
    issues = []
    if b["length"] * 1000 > d["max_length_mm"] + 1e-6:
        issues.append({"level": "ERROR", "rule": "6.2", "message": f"body length {b['length'] * 1000:.0f} mm > {d['max_length_mm']} mm"})
    if w * 1000 > d["max_body_width_mm"] + 1e-6:
        issues.append({"level": "ERROR", "rule": "6.3", "message": f"body width {w * 1000:.0f} mm > {d['max_body_width_mm']} mm"})
    if b["height"] < 1.30 - 1e-6:                      # 6.5 minimum roof height (not in the rules JSON)
        issues.append({"level": "ERROR", "rule": "6.5", "message": f"roof height {b['height'] * 1000:.0f} mm < 1300 mm"})
    if b["front_overhang"] * 1000 > d["max_front_overhang_mm"] + 1e-6:
        issues.append({"level": "ERROR", "rule": "6.6", "message": f"front overhang {b['front_overhang'] * 1000:.0f} mm > {d['max_front_overhang_mm']} mm"})
    return {"profile": prof, "side_area_m2": side_area, "skin_area_m2": area, "kg_per_m2": kg_m2, "mass": mass, "cg": cg,
            "size": [w, yr - yf, b["height"] - b["ground_clearance"]], "front_bumper": yf, "rear_bumper": yr,
            "issues": issues}


def overhangs_check(res, y_front_axle, y_rear_axle, rules, issues):
    """Rear overhang (6.6), appended to issues; the front overhang is checked in analyse().

    y_front_axle is unused (kept so both overhangs can be checked here)."""
    d = rules["dimensions"]
    rear = (res["rear_bumper"] - y_rear_axle) * 1000
    if rear > d["max_rear_overhang_mm"] + 1e-6:
        issues.append({"level": "ERROR", "rule": "6.6", "message": f"rear overhang {rear:.0f} mm > {d['max_rear_overhang_mm']} mm"})


def body_check(b, frame_nodes, issues):
    """Frame inside the body; roof closed (6.7); ground clearance (6.9). Appends to issues.

    frame_nodes: {node: [x, y, z]}; a node sticks out if it is wider than the body or above the
    roof (the side profile itself is not tested).
    """
    if not b.get("closed_roof", True):
        issues.append({"level": "ERROR", "rule": "6.7", "message": "the body must have a rigid, closed roof"})
    if b["ground_clearance"] < 0.10 - 1e-9:            # 6.9: 100 mm minimum
        issues.append({"level": "ERROR", "rule": "6.9", "message": f"body ground clearance {b['ground_clearance'] * 1000:.0f} mm < 100 mm"})
    hw = b["width"] / 2
    out = [n for n, p in frame_nodes.items() if abs(p[0]) > hw + 1e-6 or p[2] > b["height"] + 1e-6]
    if out:
        issues.append({"level": "WARN", "rule": "6.3", "message": f"{len(out)} frame joints stick out of the body shell (e.g. {out[0]})"})


def _ear_clip(poly):
    """Triangles of a simple polygon [(y, z), ...] by ear clipping.

    Works counter-clockwise; an ear is a convex corner with no other vertex inside its triangle.
    The guard stops after 1000 passes, so a degenerate polygon gives fewer triangles, not a hang.
    """
    pts = list(poly)
    area = sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))
    if area < 0:
        pts.reverse()                                     # counter-clockwise

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def inside(p, a, b, c):
        return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0

    out, guard = [], 0
    while len(pts) > 3 and guard < 1000:
        guard += 1
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            if cross(a, b, c) <= 1e-12:
                continue                                  # reflex corner: not an ear
            if any(inside(p, a, b, c) for p in pts if p not in (a, b, c)):
                continue
            out.append((a, b, c))
            del pts[i]
            break
    if len(pts) == 3:
        out.append(tuple(pts))
    return out


def mesh_triangles(res, width):
    """Triangles of the extruded shell (sides + wrap), FBeam coordinates: [(p1, p2, p3), ...].

    The wrap leaves out the same two segments as the skin area in analyse(). Winding is not
    made consistent, so the material is drawn double-sided (fbeam_gltf, the viewer).
    """
    prof = res["profile"]
    hw = width / 2
    tris = []
    n = len(prof)
    for a, b, c in _ear_clip(prof):                      # sides: the profile is concave at the screens
        for s in (hw, -hw):
            tris.append(([s, a[0], a[1]], [s, b[0], b[1]], [s, c[0], c[1]]))
    for i in range(n):                                   # wrap panels: all but the underside (9-10)
        if i == n - 2:
            continue
        a, c = prof[i], prof[(i + 1) % n]
        p1, p2, p3, p4 = [hw, a[0], a[1]], [-hw, a[0], a[1]], [-hw, c[0], c[1]], [hw, c[0], c[1]]
        tris += [(p1, p2, p3), (p1, p3, p4)]
    return tris
