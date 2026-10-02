"""Driveline: which axles are driven, the differentials, halfshafts and propshaft (regulations 4.6).

Pure Python, standard library only (runs in Pyodide). FBeam axes: X left, Y rear, Z up, metres.

The spec power unit is the Covet's: a transverse engine with the front differential in the
gearbox, driving the front wheels through unequal halfshafts (inner joints at the Covet's
tra2 / tra3 nodes). RWD and AWD can be drawn and analysed for testing; they are not legal
in v1 (4.6: the driven axle is the donor's).

For every driven axle:
  independent corner or de Dion  two halfshafts from inner joints on the chassis (the de Dion's
                                 chassis differential, DIFF) to the wheel centres
  live (solid) axle              the differential is in the axle; the propshaft runs to its pinion,
                                 which moves with the axle
A rear-driven car also gets a propshaft from the gearbox take-off to the rear differential.
Each shaft is analysed over the suspension travel: joint angle and plunge (length change).
The angle of a halfshaft is its inclination to the X axis, and of a propshaft its
inclination to the horizontal: the joints are assumed to sit on a lateral (or horizontal)
axis, so wheel camber and toe are not added. The right side is the mirror of the left.

analyse() returns {"layout", "driven", "gearbox_output", "shafts", "diffs", "issues",
"layouts"}; every issue is a WARN or ERROR tagged 4.6, and the limits below are typical
component figures, not rules.
"""

import math

import fbeam_suspension as fsus

LAYOUTS = {"FWD": ("front",), "RWD": ("rear",), "AWD": ("front", "rear")}     # driven axles
SPEC_LAYOUT = "FWD"                                                             # the Covet's (4.6)
CV_MAX_DEG = 25.0        # continuous working angle of a plunging CV joint (typical)
PROP_MAX_DEG = 12.0      # propshaft joint angle (typical for a two-piece shaft with UJs)
PLUNGE_MAX_MM = 40.0     # plunge a tripod / plunging CV joint takes (typical)
PINION_AHEAD = 0.15      # m: live-axle pinion offset from the axle line towards the middle of the car
                         # (ahead of a rear axle, behind a front one); also the default chassis rear diff input


def example():
    """Covet power unit, FWD: front differential in the gearbox (Covet differential_F nodes).

    Points in metres, FBeam axes. "rear" is a placeholder chassis rear diff for RWD / AWD tests.
    """
    return {
        "layout": "FWD",
        "gearbox_output": [0.05, -1.28, 0.28],              # transfer take-off for a propshaft (AWD / RWD tests)
        "front": {"diff": [-0.125, -1.278, 0.279],          # between the Covet's tra2 and tra3
                  "inner_L": [-0.003, -1.278, 0.279],       # Covet tra2: left halfshaft inner joint
                  "inner_R": [-0.25, -1.278, 0.279]},       # Covet tra3: right halfshaft inner joint
        "rear": {"diff": [0.0, 1.20, 0.30], "input": [0.0, 1.06, 0.30],
                 "inner_L": [0.13, 1.25, 0.29], "inner_R": [-0.13, 1.25, 0.29]},
    }


def _angle(v):
    """Angle (deg) between shaft vector v and the X axis."""
    return math.degrees(math.atan2(math.hypot(v[1], v[2]), abs(v[0])))


def _shaft_curve(inner, path, travel, usable, lateral=True):
    """Angle and plunge of a shaft from a fixed inner joint to a moving outer point.

    path: outer point per travel step (m), travel: the steps (mm), usable: (rebound, bump) in mm
    as signed travel; maxima and plunge range are taken over the usable travel only.
    lateral: True for a halfshaft (angle to X), False for a propshaft (angle to horizontal).
    Returns travel_mm, angle_deg, plunge_mm (length change from static), max_angle_deg,
    static_angle_deg, plunge_range_mm [min, max].
    """
    L0 = None
    ang, plunge = [], []
    i0 = travel.index(0.0) if 0.0 in travel else 0
    L0 = math.dist(inner, path[i0])
    for p in path:
        v = [p[k] - inner[k] for k in range(3)]
        ang.append(_angle(v) if lateral else math.degrees(math.atan2(abs(v[2]), math.hypot(v[0], v[1]))))
        plunge.append((math.dist(inner, p) - L0) * 1000)
    use = [i for i, t in enumerate(travel) if usable[0] - 1e-9 <= t <= usable[1] + 1e-9] or list(range(len(travel)))
    return {"travel_mm": travel, "angle_deg": ang, "plunge_mm": plunge,
            "max_angle_deg": max(ang[i] for i in use), "static_angle_deg": ang[i0],
            "plunge_range_mm": [min(plunge[i] for i in use), max(plunge[i] for i in use)]}


def analyse(model, sus):
    """Shafts, differentials and checks for the model's driveline and its analysed suspension.

    model: editor model (uses model["driveline"], else example()). sus: fbeam_editor_core
    suspension result, {"corners": {"front"/"rear": analyse() result + "type", "points"}}.
    Each shaft: id, label, kind ("half" / "prop"), end points a and b (static, m), b_ref (how
    the viewer moves b with the suspension) and the _shaft_curve() values. Each diff: axle,
    pos, live (True if it moves with a live axle).
    """
    d = model.get("driveline") or example()
    layout = d.get("layout", SPEC_LAYOUT)
    driven = LAYOUTS.get(layout)
    issues, shafts, diffs = [], [], []
    if driven is None:
        return {"layout": layout, "shafts": [], "diffs": [], "issues": [
            {"level": "ERROR", "rule": "4.6", "message": f"unknown driveline layout {layout!r} (FWD, RWD or AWD)"}]}
    if layout != SPEC_LAYOUT:
        issues.append({"level": "WARN", "rule": "4.6", "message":
                       f"{layout}: v1 drives the donor's axle only (the Covet is FWD); {layout} is for testing"})
    corners = (sus or {}).get("corners", {})
    gb = d.get("gearbox_output")
    for axle in driven:
        res = corners.get(axle)
        if not res:
            issues.append({"level": "WARN", "rule": "4.6", "message": f"{axle} axle is driven but its suspension is not analysed"})
            continue
        lay = fsus.TOPOLOGY[res["type"]]
        st, cu = res["static"], res["curves"]
        travel, wc = cu["travel_mm"], cu["wc"]
        usable = (-st["rebound_mm"], st["bump_mm"])
        conf = d.get(axle) or {}
        if lay.get("axle") and not lay.get("halfshafts"):              # live axle: the diff moves with it
            ahead = -PINION_AHEAD if axle == "rear" else PINION_AHEAD
            pin = [[0.0, p[1] + ahead, p[2]] for p in wc]
            diffs.append({"axle": axle, "pos": pin[travel.index(0.0)], "moving": {"corner": axle, "centre": True,
                                                                              "offset": [0.0, ahead, 0.0]}, "live": True})
            # propshaft only to the axle at the other end of the car from the gearbox (gb Y < 0: front)
            if gb and (axle == "rear") == (gb[1] < 0):
                c = _shaft_curve(gb, pin, travel, usable, lateral=False)
                shafts.append({"id": f"prop_{axle}", "label": f"Propshaft to the {axle} axle (live)", "a": gb,
                               "b": pin[travel.index(0.0)], "b_ref": {"corner": axle, "centre": True, "offset": [0.0, ahead, 0.0]},
                               "kind": "prop", **c})
            continue
        if lay.get("halfshafts") and "DIFF" in res["points"]:           # de Dion: its chassis differential
            inner_l = res["points"]["DIFF"]
            inner_r = [-inner_l[0], inner_l[1], inner_l[2]]
        else:                                                           # driveline joints, else the corner's DIFF
            inner_l = conf.get("inner_L") or res["points"].get("DIFF")
            inner_r = conf.get("inner_R") or (inner_l and [-inner_l[0], inner_l[1], inner_l[2]])
        if not inner_l:
            issues.append({"level": "WARN", "rule": "4.6", "message": f"{axle}: no halfshaft inner joints"})
            continue
        diffs.append({"axle": axle, "pos": conf.get("diff") or [(inner_l[0] + inner_r[0]) / 2, inner_l[1], inner_l[2]],
                      "live": False})
        for side, inner, path in (("L", inner_l, wc), ("R", inner_r, [[-p[0], p[1], p[2]] for p in wc])):
            c = _shaft_curve(inner, path, travel, usable)
            shafts.append({"id": f"half_{axle}_{side}", "label": f"Halfshaft, {axle} {'left' if side == 'L' else 'right'}",
                           "a": inner, "b": path[travel.index(0.0)], "b_ref": {"corner": axle, "side": 1 if side == "L" else -1},
                           "kind": "half", **c})
        if axle == "rear" and gb and gb[1] < 0:                           # front gearbox to a chassis rear diff (static only)
            inp = conf.get("input") or [0.0, inner_l[1] - PINION_AHEAD, inner_l[2]]
            v = [inp[k] - gb[k] for k in range(3)]
            a = math.degrees(math.atan2(abs(v[2]), math.hypot(v[0], v[1])))
            shafts.append({"id": "prop_rear", "label": "Propshaft to the rear differential", "a": gb, "b": inp, "b_ref": None,
                           "kind": "prop", "travel_mm": [0.0], "angle_deg": [a], "plunge_mm": [0.0], "max_angle_deg": a,
                           "static_angle_deg": a, "plunge_range_mm": [0.0, 0.0]})
    for s in shafts:
        limit = PROP_MAX_DEG if s["kind"] == "prop" else CV_MAX_DEG
        if s["max_angle_deg"] > limit:
            issues.append({"level": "WARN", "rule": "4.6", "message":
                           f"{s['label']}: joint angle reaches {s['max_angle_deg']:.0f}° over the usable travel "
                           f"(about {limit:.0f}° is the usual limit)"})
        lo, hi = s["plunge_range_mm"]
        if hi - lo > PLUNGE_MAX_MM:
            issues.append({"level": "WARN", "rule": "4.6", "message":
                           f"{s['label']}: plunge {hi - lo:.0f} mm over the usable travel (a plunging joint takes about {PLUNGE_MAX_MM:.0f} mm)"})
    return {"layout": layout, "driven": list(driven), "gearbox_output": gb, "shafts": shafts, "diffs": diffs,
            "issues": issues, "layouts": list(LAYOUTS)}
