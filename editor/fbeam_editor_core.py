"""Engineering core behind the HTML editor: the API that editor/app.js calls.

Runs unchanged in CPython and in the browser (Pyodide). Every number the editor
shows comes from here; it reuses fbeam_checker and examples/cage/cage_calc.py
and the editor/fbeam_*.py modules. Public functions take and return JSON
strings (the Pyodide boundary): evaluate, rigidity, suspension, suspension_layout,
pickups_to_joints, driveline, set_driveline, body, masses, fill_ballast, aero,
dynamics, section_preview, export_svj, export_svj_bundle, import_svj,
import_svj_bundle and build. default_model() returns the example car as a dict.

Regulations implemented here (docs/regulations.md): tube catalogue and cage
minimums (8.4, 8.9.1), mass from strength and the stability cap on springs
(8.2, 8.9.6), buckling length (8.9.7), bending beams and the global check
(8.10), pickups on braced joints (7.1.6), wheel travel (7.1.1), width over the
arches (6.4); body, mass and aero rules are delegated to fbeam_body, fbeam_mass
and fbeam_aero, and build() runs the real checker on the generated Submission.

Units: metres, kg, N, N/m, seconds, unless a key name says otherwise (_mm, _deg,
_N_per_mm, _Ns_per_m, _Nm_per_deg). Axes are BeamNG's: X left, Y rear, Z up, in
Ibishu Covet coordinates (front wheels at y = -1.25, rear at +1.25).

The model is plain JSON (saved by the editor as .fbeam.json). Only "nodes" and
"members" are required; full_chassis_model() fills every key.

    "nodes": {"MHt_L": [x, y, z], ...}
        Frame joints. Names ending _L / _R (FBeam) or l / r, ll / rr (BeamNG, as on
        the Covet) are mirror pairs (fbeam_checker.pickups.mirror_name).
    "members": [{"id": 0, "group": "Backstay", "a": "MHt_L", "b": "BSf_L",
                 "role": "other", "tube": "40x2"}, ...]
        One tube per entry. id is unique (written as fbeamMember "m<id>");
        role is "main" / "other" (cage, 8.9.1) or "frame"; tube is "<D>x<t>" in mm
        and must be in CATALOGUE[role]. Material is always MATERIAL.
    "bending_beams": "auto" | "off"          weightless 8.10 braces (default "auto")
    "axles": {"front": [joint, joint], "rear": [joint, joint]}
        Frame joints near each axle line: torsion / bending supports for rigidity()
        and the fallback axle lines when "hubs" is missing.
    "hubs": {"FL": [x, y, z], "FR": ..., "RL": ..., "RR": ...}
        Wheel centres; preferred over "axles" for the axle lines (_axles_y).
    "suspension": {"front": corner, "rear": corner}
        corner = {"type": a key of fbeam_suspension.TOPOLOGY (macpherson,
                  double_wishbone, multilink, trailing_arm, semi_trailing_arm,
                  trailing_arm_lateral, solid_axle_panhard, solid_axle_watt,
                  solid_axle_triangulated, de_dion_watt, de_dion_panhard),
                  "points": {key: [x, y, z] or {"node": joint}, ...},
                  "spring_N_per_mm", "damper_stroke_mm", "damper_bump_Ns_per_m",
                  "damper_rebound_Ns_per_m", "arb_Nm_per_deg",
                  front only: "rack_travel_mm", "rack_mm_per_rev";
                  optional: "arm_tube", "beam_tube" (twist beam), "tyre_radius"}
        Points are for the left (+X) corner; the right is mirrored. Keys are
        fbeam_suspension.LABEL (LIF, LBJ, WC, ...), required set REQUIRED[type].
        {"node": joint} attaches an inner pickup to a frame joint (7.1.6); a plain
        coordinate is a free point. Single points of an axle (Panhard, Watt) are
        not mirrored. DIFF is the halfshaft inner joint (from "driveline" except
        on a de Dion, which keeps its own).
    "driveline": {"layout": "FWD" | "RWD" | "AWD", "gearbox_output": [x, y, z],
                  "front": {"diff", "inner_L", "inner_R"},
                  "rear": {"diff", "input", "inner_L", "inner_R"}}  (fbeam_driveline)
    "components": [{"id", "kind", "mass", "position": [x, y, z],
                    "size": [dx, dy, dz], "estimate": bool, "note"}, ...]
        Lumped masses (kg). kind "body" is derived from "body", kind "ballast"
        gets its height from steel density (fbeam_mass.derive_ballast, 6.10).
    "unsprung_mass_per_corner": kg, spec wheel included
    "body": {"length", "width", "height", "front_overhang", "ground_clearance",
             "bonnet_height", "boot_height", "windscreen_deg", "rear_window_deg",
             "roof_length", "material", "closed_roof"}       (fbeam_body, 6.2-6.7)
    "aero": [{"id", "kind": "wing", "span", "chord", "aoa_deg", "endplates",
              "position"} | {"id", "kind": "splitter", "width", "depth",
              "position"}, ...]                              (fbeam_aero, 7.4)
    "svj_source", "svj_info"  set by import_svj: the imported SVJ, kept whole so
        export_svj can write it back (fbeam_svj)
Keys starting with "_" are temporary, added and removed during an export.
"""

import io
import json
import math
import shutil
import sys
import zipfile
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "examples" / "cage"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import cage_calc as cc  # noqa: E402
import fbeam_frame as frame  # noqa: E402
from fbeam_checker import bending as fbend  # noqa: E402
from fbeam_checker import buckling as fbuck  # noqa: E402
from fbeam_checker import pickups as fpick  # noqa: E402
import fbeam_suspension as susp  # noqa: E402
import fbeam_svj as svj  # noqa: E402
import fbeam_gltf as gltf  # noqa: E402
import fbeam_mass as fmass  # noqa: E402
import fbeam_dynamics as fdyn  # noqa: E402
import fbeam_aero as faero  # noqa: E402
import fbeam_vehicle as fveh  # noqa: E402
import fbeam_body as fbody  # noqa: E402
import fbeam_driveline as fdrive  # noqa: E402
from fbeam_checker.checks import run_all  # noqa: E402
from fbeam_checker.sections import parse_section  # noqa: E402
from fbeam_checker.submission import Submission  # noqa: E402

MATERIAL = "steel_1018"              # every editor tube; properties in rules/fbeam_v1.json
PART = "fbeam_editor_chassis"        # chassis part, vehicle folder and .pc model name of build()

# Sections offered per member role ("<D>x<t>", mm). Cage roles start at the FIA
# 253-8.3.3 minimums (regulations 8.9.1), so an undersized cage tube cannot be
# chosen (rules by construction). ROLE_OF_KIND maps cage_calc member kinds to roles.
CATALOGUE = {
    "main": ["45x2.5", "50x2", "50x2.5", "50x3"],
    "other": ["38x2.5", "40x2", "40x2.5", "45x2.5", "50x2", "50x2.5"],
    "frame": ["25x1.5", "25x2", "30x2", "35x2", "40x2", "40x2.5", "45x2.5", "50x2", "50x2.5"],
}
DEFAULT_TUBE = {"main": "45x2.5", "other": "40x2", "frame": "30x2"}
ROLE_LABEL = {"main": "Cage: main hoop / lateral rollbar", "other": "Cage: other member", "frame": "Chassis frame"}
ROLE_OF_KIND = {"main": "main", "other": "other", "diag": "other"}


# ---------------------------------------------------------------- example models

def cage_model():
    """The regulations 8.9.4 safety cage on its own (cage_calc's 18 tubes), moved into the Covet cabin.

    Returns a model dict with "nodes" and "members" only, recommended tubes. The worked example in
    examples/cage keeps its own (earlier, Civic-envelope) positions for the rulebook's figures.
    """
    tubes = cc.OPTIONS["recommended"]
    members = [{"id": i, "group": g, "a": a, "b": b, "role": ROLE_OF_KIND[k], "tube": tubes[k]}
               for i, (g, a, b, k) in enumerate(cc.MEMBERS)]
    nodes = {k: list(v) for k, v in cc.NODES.items()}
    for n, (x, y, z) in _CAGE_NODES.items():      # the Covet cabin positions (defined below)
        nodes[n], nodes[_mirror(n)] = [x, y, z], [-x, y, z]
    return {"nodes": nodes, "members": members}


# Tube-frame chassis for the Covet-based car (regulations 6): Covet coordinates, wheelbase 2.50 m.
# Rules it follows by construction:
#   * every suspension pickup is a joint braced by tubes in three dimensions (7.1.6);
#   * the joints the spec engine borrows from the Covet body (f11, f11rr, f13ll, f13rr, f15, f15l)
#     are at the Covet positions and carry its names, so the sealed engine part bolts on unchanged;
#     other joints at Covet engine-mount and pickup positions carry Covet names too
#     (f10ll, f12ll, f1l, f5l, fx1l, fx2l), so Covet mounts and suspension can be compared.
# Left side (+X) and centre-line joints, metres; the right side is mirrored (FBeam _L/_R,
# BeamNG l/r, ll/rr). Covet names and positions are from the donor files (rules/donor_covet.json).
_FRAME_NODES = {
    # front: radiator support (spec engine interface), y = -1.79
    "f11": (0.0, -1.79, 0.24), "f15": (0.0, -1.79, 0.67),
    "f11ll": (0.38, -1.79, 0.24), "f13ll": (0.38, -1.79, 0.47), "f15l": (0.38, -1.79, 0.67),
    # front subframe and rails
    "fx1l": (0.365, -1.647, 0.209),   # lower arm, front pickup (Covet fx1l)
    "f10ll": (0.38, -1.40, 0.26),     # lower rail, engine-mount level (Covet f10ll)
    "f12ll": (0.38, -1.40, 0.49),     # upper rail, engine mount (Covet f12ll)
    "fx2l": (0.341, -1.187, 0.217),   # lower arm, rear pickup (Covet fx2l)
    "RK_L": (0.288, -1.093, 0.301),   # rack end, tie-rod inner: Covet fh8l moved 100 mm in, 10 mm up (bump steer)
    "FT_L": (0.635, -1.220, 0.700),   # strut top, on the Covet steering axis
    "f1l": (0.38, -0.84, 0.20),       # bulkhead bottom (Covet f1l)
    "f5l": (0.38, -0.84, 0.49),       # bulkhead top (Covet f5l)
    "S0_L": (0.70, -0.84, 0.20),      # sill front
    # rear subframe
    "RF_L": (0.423, 0.589, 0.241),    # lower arm, front pickup (Covet trailing-arm pivot rx2l)
    "RX_L": (0.323, 1.304, 0.206),    # lower arm, rear pickup (Covet lower link rx1l)
    "RUF_L": (0.427, 1.020, 0.433),   # upper arm, front pickup (around Covet upper link rx3l)
    "RUR_L": (0.427, 1.300, 0.433),   # upper arm, rear pickup
    "RTI_L": (0.330, 1.450, 0.240),   # toe link, inner
    "RE_L": (0.40, 1.85, 0.30),       # rear frame end, bumper mount
}
# Safety cage (8.9.4 layout) moved to the Covet cabin: A-pillar at the bulkhead, main hoop at
# the B-pillar (y = 0.40), backstays down to the rear damper tops (the Covet rs1l spring tops).
# These override cage_calc.NODES of the same names.
_CAGE_NODES = {
    "APf_L": (0.70, -0.84, 0.45), "APt_L": (0.62, -0.20, 1.22),
    "MHf_L": (0.70, 0.40, 0.20), "MHm_L": (0.68, 0.40, 0.70), "MHt_L": (0.62, 0.40, 1.22),
    "BSf_L": (0.49, 1.34, 0.68),
}
# (group, a, b, tube): every _FRAME_MEMBERS entry also gets its mirror twin (both ends mirrored,
# centre-line ends unchanged). Members that are their own mirror (joining a joint to its twin, or
# on the centre line) are listed once, in _FRAME_CROSS; in _FRAME_MEMBERS they would be duplicated
# and fail _validate. All have role "frame".
_FRAME_MEMBERS = [
    # radiator support and front rails
    ("Radiator support, lower", "f11", "f11ll", "35x2"), ("Radiator support, post", "f11ll", "f13ll", "35x2"),
    ("Radiator support, post", "f13ll", "f15l", "35x2"), ("Radiator support, upper", "f15", "f15l", "35x2"),
    ("Radiator support, diagonal", "f11", "f13ll", "30x2"),
    ("Front lower rail", "f11ll", "fx1l", "40x2"), ("Front lower rail", "fx1l", "f10ll", "40x2"),
    ("Front lower rail", "f10ll", "fx2l", "40x2"), ("Front lower rail", "fx2l", "f1l", "40x2"),
    ("Front upper rail", "f13ll", "f12ll", "40x2"), ("Front upper rail", "f12ll", "f5l", "40x2"),
    ("Engine bay post", "f10ll", "f12ll", "35x2"), ("Subframe diagonal", "fx1l", "f12ll", "35x2"),
    ("Subframe diagonal", "fx2l", "f12ll", "35x2"),
    # strut tower and rack
    ("Strut tower, front", "FT_L", "f15l", "35x2"), ("Strut tower, inner", "FT_L", "f12ll", "35x2"),
    ("Strut tower, rear", "FT_L", "f5l", "35x2"), ("Scuttle rail", "FT_L", "APf_L", "40x2"),
    ("Rack mount", "RK_L", "fx2l", "30x2"), ("Rack mount", "RK_L", "f1l", "30x2"), ("Rack mount", "RK_L", "f5l", "30x2"),
    # bulkhead and floor
    ("Bulkhead post", "f1l", "f5l", "40x2"), ("Bulkhead, lower", "f1l", "S0_L", "40x2.5"),
    ("Bulkhead, upper", "f5l", "APf_L", "40x2"), ("Bulkhead diagonal", "f1l", "APf_L", "30x2"),
    ("A-pillar foot", "S0_L", "APf_L", "40x2.5"),
    ("Sill", "S0_L", "MHf_L", "50x2"), ("Floor diagonal", "S0_L", "MHf_R", "30x2"),
    # rear subframe
    ("Rear floor rail", "MHf_L", "RF_L", "40x2"), ("Rear lower rail", "RF_L", "RX_L", "40x2"),
    ("Rear lower rail", "RX_L", "RTI_L", "40x2"), ("Rear lower rail", "RTI_L", "RE_L", "35x2"),
    ("Rear subframe diagonal", "RF_L", "RUF_L", "35x2"), ("Rear upper rail", "RUF_L", "RUR_L", "35x2"),
    ("Rear upper rail", "RUR_L", "RE_L", "30x2"), ("Rear subframe post", "RX_L", "RUR_L", "35x2"),
    ("Rear subframe diagonal", "RX_L", "RUF_L", "30x2"), ("Toe-link mount", "RTI_L", "RUR_L", "30x2"),
    ("Rear damper tower", "BSf_L", "RUF_L", "35x2"), ("Rear damper tower", "BSf_L", "RUR_L", "35x2"),
    ("Rear floor post", "MHm_L", "RF_L", "35x2"),
]
_FRAME_CROSS = [
    ("Radiator support, centre post", "f11", "f15", "35x2"),
    ("Front subframe, front cross", "fx1l", "fx1r", "40x2"), ("Front subframe, rear cross", "fx2l", "fx2r", "40x2"),
    ("Rack crossmember", "RK_L", "RK_R", "30x2"), ("Strut brace", "FT_L", "FT_R", "30x2"),
    ("Bulkhead, lower cross", "f1l", "f1r", "40x2.5"), ("Bulkhead, upper cross", "f5l", "f5r", "40x2"),
    ("Floor crossmember", "MHf_L", "MHf_R", "40x2"),
    ("Rear subframe, front cross", "RF_L", "RF_R", "40x2"), ("Rear subframe, rear cross", "RX_L", "RX_R", "40x2"),
    ("Rear upper cross", "RUF_L", "RUF_R", "30x2"), ("Toe-link cross", "RTI_L", "RTI_R", "30x2"),
    ("Rear strut brace", "BSf_L", "BSf_R", "30x2"), ("Rear bumper crossmember", "RE_L", "RE_R", "35x2"),
]

# Covet wheel centre (left, rear): hub slot nodeOffset y 1.25, z 0.288; x between the rim's axle
# nodes (0.24 + 0.31 ... 0.57). Track 1.36 m; to confirm in-game (wheel placement on its axle).
# The front wheel centre is the same with y negated (wheelbase 2.50 m, 6.1).
COVET_WHEEL_CENTRE = (0.68, 1.25, 0.288)


def _mirror(n):
    """Name of the mirrored joint (a centre-line name is returned unchanged)."""
    return fpick.mirror_name(n)


def full_chassis_model():
    """The example car: cage + floor + front and rear subframes, suspension, driveline, body,
    aero and components, in Covet coordinates (wheelbase 2.50 m).

    Returns a complete model dict (schema in the module docstring) with the derived body
    panels and ballast already filled, so it meets the minimum Race Mass (6.11).
    """
    model = cage_model()
    for n, (x, y, z) in list(_CAGE_NODES.items()) + list(_FRAME_NODES.items()):
        model["nodes"][n] = [x, y, z]
        if _mirror(n) != n:
            model["nodes"][_mirror(n)] = [-x, y, z]
    nid = len(model["members"])
    for g, a, b, t in _FRAME_MEMBERS:
        for aa, bb in ((a, b), (_mirror(a), _mirror(b))):
            model["members"].append({"id": nid, "group": g, "a": aa, "b": bb, "role": "frame", "tube": t})
            nid += 1
    for g, a, b, t in _FRAME_CROSS:
        model["members"].append({"id": nid, "group": g, "a": a, "b": b, "role": "frame", "tube": t})
        nid += 1
    model["axles"] = {"front": ["FT_L", "FT_R"], "rear": ["BSf_L", "BSf_R"]}
    xw, yw, zw = COVET_WHEEL_CENTRE
    model["hubs"] = {"FL": [xw, -yw, zw], "FR": [-xw, -yw, zw], "RL": [xw, yw, zw], "RR": [-xw, yw, zw]}
    model["suspension"] = example_suspension()
    model["aero"] = faero.example_aero()
    model["body"] = fbody.example_body()
    model["components"] = fmass.example_components()
    model["unsprung_mass_per_corner"] = fmass.EXAMPLE_UNSPRUNG
    model["driveline"] = fdrive.example()
    derive(model)
    _, node_mass, _ = _design(model)
    fmass.fill_ballast(model, node_mass, cc.RULES["mass"]["min_race_mass_kg"])     # the example meets 6.11
    return model


def example_suspension():
    """The example car's {"front": corner, "rear": corner} (corner schema in the module docstring).

    Front MacPherson on the Covet's own hardpoints. Rear double wishbone laid out around the
    Covet's trailing-arm and lateral-link points; the Covet's own rear layout (trailing arm with
    lateral links, "trailing_arm_lateral") is offered too but not used here. Rates from the Covet
    rally_gravel setup. Every inner pickup is a frame joint (7.1.6).
    """
    xw, yw, zw = COVET_WHEEL_CENTRE
    return {
        "front": {"type": "macpherson", "spring_N_per_mm": 68.5, "damper_stroke_mm": 160,
                  "damper_bump_Ns_per_m": 3800, "damper_rebound_Ns_per_m": 5900, "arb_Nm_per_deg": 0,
                  "rack_travel_mm": 70, "rack_mm_per_rev": 65, "points": {
            "LIF": {"node": "fx1l"}, "LIR": {"node": "fx2l"}, "LBJ": [0.705, -1.244, 0.212],       # Covet fh1l
            "ST": {"node": "FT_L"}, "SB": [0.664, -1.230, 0.500],                                  # on the fh1l-fh2l axis
            "TRI": {"node": "RK_L"}, "TRO": [0.676, -1.121, 0.304],                                # Covet fh3l
            "WC": [xw, -yw, zw], "DIFF": [0.0, -1.278, 0.279]}},                                    # Covet tra2
        "rear": {"type": "double_wishbone", "spring_N_per_mm": 25.5, "damper_stroke_mm": 160,
                 "damper_bump_Ns_per_m": 1500, "damper_rebound_Ns_per_m": 2200, "arb_Nm_per_deg": 0, "points": {
            "LIF": {"node": "RF_L"}, "LIR": {"node": "RX_L"}, "LBJ": [0.663, 1.284, 0.197],        # Covet rh1l
            "UIF": {"node": "RUF_L"}, "UIR": {"node": "RUR_L"}, "UBJ": [0.600, 1.159, 0.439],      # Covet rh3l
            "TRI": {"node": "RTI_L"}, "TRO": [0.640, 1.450, 0.250],
            "DT": {"node": "BSf_L"}, "DB": [0.568, 1.290, 0.199], "WC": [xw, yw, zw]}},           # Covet rs1l, rsp2l
    }


default_model = full_chassis_model      # the model the editor opens with


# ---------------------------------------------------------------- engineering

def _validate(model):
    """Raise ValueError for a tube on a missing joint, of zero or < 10 mm length, duplicated,
    or with a role or section outside CATALOGUE."""
    nodes, seen = model["nodes"], set()
    for m in model["members"]:
        if m["a"] not in nodes or m["b"] not in nodes:
            raise ValueError(f"tube {m['id']} ends on a missing joint")
        if m["a"] == m["b"]:
            raise ValueError(f"tube {m['id']} starts and ends on the same joint")
        if m["role"] not in CATALOGUE:
            raise ValueError(f"tube {m['id']}: unknown role {m['role']!r}")
        if m["tube"] not in CATALOGUE[m["role"]]:
            raise ValueError(f"tube {m['id']}: {m['tube']} is not allowed for {ROLE_LABEL[m['role']].lower()}")
        pair = frozenset((m["a"], m["b"]))
        if pair in seen:
            raise ValueError(f"tube {m['id']} duplicates another tube between {m['a']} and {m['b']}")
        seen.add(pair)
        if math.dist(nodes[m["a"]], nodes[m["b"]]) < 0.01:
            raise ValueError(f"tube {m['id']} is shorter than 10 mm")


def buckling_lengths(model):
    """Unbraced length of every tube (regulations 8.9.7): {member id: buckling length in m}.

    The same rule as the checker (fbeam_checker.buckling): a run of tubes is followed through
    joints where exactly two tubes meet and the line turns by 30 degrees or less, so splitting
    a tube can never make it stronger.
    """
    return fbuck.buckling_lengths(model["nodes"], [(m["id"], m["a"], m["b"]) for m in model["members"]])


def _design(model):
    """Per-tube limits, node masses and stability-capped springs (regulations 8.2, 8.9.6).

    Returns (rows, node_mass, orphans): rows are the members with L, L_buckle (m), the
    Section.limits values (beamSpring N/m, beamDeform / beamStrength / yield_force /
    buckling_force N, mass kg), governs, kg_per_m, k_stable and k_set (N/m);
    node_mass is {joint: kg}; orphans are joints with no tube (left out of the jbeam).
    """
    _validate(model)
    nodes = {k: tuple(v) for k, v in model["nodes"].items()}
    Lb = buckling_lengths(model)
    rows, share = [], defaultdict(float)
    for m in model["members"]:
        sec = parse_section(f"tube {m['tube']} {MATERIAL}", cc.MATERIALS, cc.RULES)
        L = math.dist(nodes[m["a"]], nodes[m["b"]])
        lim = sec.limits(L, Lb[m["id"]])
        rows.append({**m, "L": L, "L_buckle": Lb[m["id"]], **lim,
                     "governs": "buckling" if lim["buckling_force"] < lim["yield_force"] else "yield",
                     "kg_per_m": cc.MATERIALS[MATERIAL].rho * sec.A})
        share[m["a"]] += lim["mass"] / 2
        share[m["b"]] += lim["mass"] / 2
    # half of each tube's mass to each end, plus the joint allowance (rules chassis.joint_allowance),
    # rounded up to 0.05 kg so the node can never be lighter than the rule
    node_mass = {n: math.ceil(s * (1 + cc.JOINT) * 20) / 20 for n, s in share.items()}
    dt = 1.0 / cc.PHYSICS_HZ
    for r in rows:
        ma, mb = node_mass[r["a"]], node_mass[r["b"]]
        # 8.9.6: k <= 0.25 * mu * (2 / dt)^2, mu the reduced mass of the two end nodes
        r["k_stable"] = cc.STABILITY_SAFETY * (ma * mb / (ma + mb)) * (2 / dt) ** 2
        r["k_set"] = min(r["beamSpring"], r["k_stable"])
    orphans = sorted(n for n in model["nodes"] if n not in node_mass)
    return rows, node_mass, orphans


def _section(r):
    """(A m^2, I m^4, D m, t m) of a row's tube, as fbeam_checker.bending.budgets expects."""
    sec = parse_section(f"tube {r['tube']} {MATERIAL}", cc.MATERIALS, cc.RULES)
    D, t = (float(x) / 1000 for x in r["tube"].split("x"))
    return sec.A, sec.I, D, t


def _material(r):
    """(E Pa, yield stress Pa) of a row's material, as fbeam_checker.bending.budgets expects."""
    m = cc.MATERIALS[MATERIAL]
    return m.E, m.sy


def bending_beams(model, rows, node_mass):
    """Weightless corner braces (8.10), budgeted from the tubes' bending and scaled like them (8.10.5).

    rows and node_mass come from _design. Returns a list of {"a", "b", "k_budget" (real
    stiffness, N/m), "k_set" (k_budget * scale), "scale" (smallest k_set / beamSpring of the
    tubes it represents), "beamDeform", "beamStrength" (N), "members" (tube ids), "corners"
    (joints)}. Empty when model["bending_beams"] is "off".
    """
    if model.get("bending_beams", "auto") == "off" or not rows:
        return []
    nodes = {n: model["nodes"][n] for n in node_mass}
    byid = {r["id"]: r for r in rows}
    out = []
    for (a, c), e in sorted(fbend.budgets(nodes, rows, _section, _material).items()):
        s = min(byid[i]["k_set"] / byid[i]["beamSpring"] for i in e["members"])
        out.append({"a": a, "b": c, "k_budget": e["k"], "k_set": e["k"] * s, "scale": s,
                    "beamDeform": e["F"], "beamStrength": fbend.STRENGTH_FACTOR * e["F"],
                    "members": e["members"], "corners": [x[0] for x in e["corners"]]})
    return out


def evaluate(model_json):
    """Everything the editor displays about the frame, as JSON (regulations 8).

    Returns {"cog": centre_of_gravity(), "members": [row + "jbeam" line], "bending":
    [bending beam + "jbeam" line], "node_mass": {joint: kg}, "orphans": [joint],
    "totals": {masses kg, weakest_kN, counts, maxwell_needed}, "catalogue",
    "default_tube", "role_label", "envelope": {"length", "width", "arch_width"} (m, 6.2-6.4)}.
    Raises ValueError (from _validate) for an invalid frame.
    """
    model = json.loads(model_json)
    rows, node_mass, orphans = _design(model)
    bends = bending_beams(model, rows, node_mass)
    keys = ("id", "group", "a", "b", "role", "tube", "L", "L_buckle", "mass", "kg_per_m", "beamSpring", "k_set",
            "yield_force", "buckling_force", "beamDeform", "beamStrength", "governs")
    weakest = min(rows, key=lambda r: r["beamDeform"]) if rows else None
    by_role = defaultdict(float)
    for r in rows:
        by_role[r["role"]] += r["mass"]
    d = cc.RULES["dimensions"]
    return json.dumps({
        "cog": centre_of_gravity(model, node_mass),
        "members": [dict({k: r[k] for k in keys}, jbeam=beam_line(r)) for r in rows],
        "bending": [dict(b, jbeam=bend_line(b)) for b in bends],
        "node_mass": node_mass,
        "orphans": orphans,
        "totals": {"tube_mass": sum(r["mass"] for r in rows), "node_mass": sum(node_mass.values()),
                   "cage_mass": by_role["main"] + by_role["other"], "frame_mass": by_role["frame"],
                   "weakest_kN": weakest["beamDeform"] / 1e3 if weakest else 0,
                   "weakest_group": weakest["group"] if weakest else "",
                   "buckling_members": sum(1 for r in rows if r["governs"] == "buckling"),
                   "members": len(rows), "joints": len(node_mass), "bending_beams": len(bends),
                   # Maxwell's rule: a rigid 3D pin-jointed truss needs at least 3j - 6 bars
                   "maxwell_needed": max(3 * len(node_mass) - 6, 0)},
        "catalogue": CATALOGUE, "default_tube": DEFAULT_TUBE, "role_label": ROLE_LABEL,
        "envelope": {"length": d["max_length_mm"] / 1000, "width": d["max_body_width_mm"] / 1000,
                     "arch_width": d["max_width_over_arches_mm"] / 1000},
    })


def centre_of_gravity(model, node_mass):
    """Structure CoG from node masses; weight split from the axle nodes if defined.

    Returns {"x", "y", "z" (m), "mass" (kg), "front_share" (0..1, only with model["axles"])},
    or None for a weightless frame. Frame tubes only: the full-car CG is masses().
    """
    total = sum(node_mass.values())
    if total <= 0:
        return None
    c = [sum(node_mass[n] * model["nodes"][n][i] for n in node_mass) / total for i in range(3)]
    out = {"x": c[0], "y": c[1], "z": c[2], "mass": total}
    ay = _axles_y(model)                     # wheel centres when there are hubs, else the axle joints
    if ay and abs(ay[1] - ay[0]) > 1e-9:
        yf, yr = ay
        out["front_share"] = (yr - c[1]) / (yr - yf)
    return out


def rigidity(model_json):
    """Torsion and bending stiffness of the chassis, three ways (regulations 8.10.6).

    frame      real welded tubes: beams with bending and twist (E, A, I, J)
    pinned     the same tubes pin-jointed: what triangulation alone gives (E*A/L)
    jbeam      pin-jointed with the stability-capped springs BeamNG will run
    With bending beams there is also pinned_bending (pinned plus the bending beams at their
    real budget stiffness), and jbeam includes the bending beams at their scaled k_set.

    Supports are model["axles"] (two joints per axle, they need tubes). The four 8.10.6 load cases:
    torsion between the axles; vertical bending at one low joint per side nearest mid-wheelbase;
    a sideways load and a roof load at the highest joint on each side (the main-hoop top), with
    the four axle joints held. Returns {"load_nodes", "top_nodes", "front", "rear",
    "bending_beams", <case>: {"torsion_Nm_per_deg", "bending_N_per_mm", "lateral_N_per_mm",
    "roof_N_per_mm", "mechanisms", "mechanism_joints"}, "jbeam_vs_frame_torsion",
    "pinned_vs_frame_torsion", "check_8_10_6": {"torsion_ratio", "bending_ratio",
    "lateral_ratio", "roof_ratio", "passed"} (with bending beams only)}. A stiffness is None
    when the load does not move the structure.
    """
    model = json.loads(model_json)
    rows, node_mass, orphans = _design(model)
    axles = model.get("axles") or {}
    front, rear = axles.get("front", []), axles.get("rear", [])
    if len(front) != 2 or len(rear) != 2 or not all(n in node_mass for n in front + rear):
        raise ValueError("set two front and two rear axle joints (with tubes) to compute rigidity")
    nodes = {n: model["nodes"][n] for n in node_mass}
    ymid = (sum(nodes[n][1] for n in front) + sum(nodes[n][1] for n in rear)) / 4
    load = []                    # per side: the joint nearest mid-wheelbase among those within 0.10 m of the lowest
    for side in (1, -1):
        cand = [n for n in nodes if nodes[n][0] * side > 0.05]
        if cand:
            zmin = min(nodes[n][2] for n in cand)
            low = [n for n in cand if nodes[n][2] <= zmin + 0.10]
            load.append(min(low, key=lambda n: abs(nodes[n][1] - ymid)))
    top = []                     # per side: the highest joint (the main-hoop top in a caged car)
    for side in (1, -1):
        cand = [n for n in nodes if nodes[n][0] * side > 0.05]
        if cand:
            top.append(max(cand, key=lambda n: nodes[n][2]))
    mat = cc.MATERIALS[MATERIAL]
    fr, pin, jb = [], [], []
    for r in rows:
        sec = parse_section(f"tube {r['tube']} {MATERIAL}", cc.MATERIALS, cc.RULES)
        fr.append({"a": r["a"], "b": r["b"], "E": mat.E, "A": sec.A, "I": sec.I, "J": 2 * sec.I})   # round tube: J = 2I
        pin.append({"a": r["a"], "b": r["b"], "k": r["beamSpring"]})
        jb.append({"a": r["a"], "b": r["b"], "k": r["k_set"]})
    bends = bending_beams(model, rows, node_mass)
    pinb = pin + [{"a": b["a"], "b": b["b"], "k": b["k_budget"]} for b in bends]
    jbb = jb + [{"a": b["a"], "b": b["b"], "k": b["k_set"]} for b in bends]
    out = {"load_nodes": load, "top_nodes": top, "front": front, "rear": rear, "bending_beams": len(bends)}
    cases = [("frame", fr, "frame"), ("pinned", pin, "truss")]
    cases += [("pinned_bending", pinb, "truss"), ("jbeam", jbb, "truss")] if bends else [("jbeam", jb, "truss")]
    for name, mem, kind in cases:
        st = frame.Structure(nodes, mem, kind)
        kt, mt = frame.torsion(st, front, rear)
        kb, mb = frame.bending(st, front, rear, load)
        kl, ml = frame.point_load(st, front + rear, top, 0, 1000.0) if top else (None, [])     # sideways at the hoop top
        kr, mr = frame.point_load(st, front + rear, top, 2, -1000.0) if top else (None, [])    # roof load, down
        mech = sorted({j for j, _ in mt + mb})
        out[name] = {"torsion_Nm_per_deg": kt, "bending_N_per_mm": kb, "lateral_N_per_mm": kl, "roof_N_per_mm": kr,
                     "mechanisms": max(len(mt), len(mb)), "mechanism_joints": mech}
    ratio = lambda a, b: a / b if a is not None and b else None    # None (JSON null) when either is missing
    out["jbeam_vs_frame_torsion"] = ratio(out["jbeam"]["torsion_Nm_per_deg"], out["frame"]["torsion_Nm_per_deg"])
    if bends:                    # 8.10.6: the jbeam structure at real stiffness may not beat the welded frame by > 10 %
        pb, f = out["pinned_bending"], out["frame"]
        rs = {f"{k}_ratio": ratio(pb[f"{k}_{u}"], f[f"{k}_{u}"])
              for k, u in (("torsion", "Nm_per_deg"), ("bending", "N_per_mm"), ("lateral", "N_per_mm"), ("roof", "N_per_mm"))}
        known = [v for v in rs.values() if v is not None]
        out["check_8_10_6"] = dict(rs, passed=bool(known) and rs["torsion_ratio"] is not None and max(known) <= 1.10)
    out["pinned_vs_frame_torsion"] = ratio(out["pinned"]["torsion_Nm_per_deg"], out["frame"]["torsion_Nm_per_deg"])
    return json.dumps(out)


def _point(model, v):
    """A suspension point as [x, y, z]: a coordinate, or the position of {"node": joint}."""
    return list(model["nodes"][v["node"]]) if isinstance(v, dict) else list(v)


def _dist_to_frame(model, p):
    """Distance (m) from a point to the nearest chassis joint or tube, and what it is
    ("joint <name>" or "tube <group>"), for the 7.1.6 messages."""
    best, what = float("inf"), None
    nodes = model["nodes"]
    for n, q in nodes.items():
        d = math.dist(p, q)
        if d < best:
            best, what = d, f"joint {n}"
    for m in model["members"]:
        a, b = nodes[m["a"]], nodes[m["b"]]
        ab = [y - x for x, y in zip(a, b)]
        L2 = sum(c * c for c in ab) or 1.0
        t = max(0.0, min(1.0, sum((pc - ac) * c for pc, ac, c in zip(p, a, ab)) / L2))
        d = math.dist(p, [ac + c * t for ac, c in zip(a, ab)])
        if d < best:
            best, what = d, f"tube {m['group']}"
    return best, what


def _tube_ends(model, joint):
    """Far-end positions of every tube meeting at a joint."""
    nodes = model["nodes"]
    return [nodes[m["b"] if m["a"] == joint else m["a"]] for m in model["members"] if joint in (m["a"], m["b"])]


def suspension(model_json):
    """Kinematic analysis of every corner, plus rule checks (7.1, 7.1.1, 7.1.6, 6.4).

    Checks: known layout and a steering link at the front (7.1), all points present,
    every inner pickup on a frame joint braced in three dimensions (7.1.6), usable
    wheel travel >= 150 mm (7.1.1), track + spec tyre width within the arch width (6.4).
    The DIFF point of a driven independent corner is taken from model["driveline"].

    Returns {"corners": {name: fbeam_suspension.analyse result + "mounts" (one entry per
    inner pickup: point, label, on_frame, joint, distance_mm, nearest, and tubes / span /
    braced when on a joint), "points" (resolved [x, y, z]), "type", "labels"}, "issues":
    [{"level", "rule", "message"}], "layouts": fbeam_suspension.describe()}.
    A corner with an error that stops the analysis is left out of "corners".
    """
    model = json.loads(model_json)
    out, issues = {}, []
    arch = cc.RULES["dimensions"]["max_width_over_arches_mm"]
    for name, corner in (model.get("suspension") or {}).items():
        if corner.get("type") not in susp.TOPOLOGY:
            issues.append({"level": "ERROR", "rule": "7.1", "message": f"{name}: unknown layout {corner.get('type')!r}"})
            continue
        if name == "front" and not susp.TOPOLOGY[corner["type"]]["toe"]:
            issues.append({"level": "ERROR", "rule": "7.1", "message":
                           f"front: a {susp.TOPOLOGY[corner['type']]['name'].lower()} has no steering link; the front wheels must steer"})
        need = susp.REQUIRED[corner["type"]]
        missing = [k for k in need if k not in corner["points"]]
        if missing:
            issues.append({"level": "ERROR", "rule": "7.1", "message": f"{name}: missing points {', '.join(missing)}"})
            continue
        pts = {k: _point(model, v) for k, v in corner["points"].items()}
        dl = model.get("driveline")
        if dl and not susp.TOPOLOGY[corner["type"]].get("halfshafts"):   # (a de Dion keeps its own DIFF)
            pts.pop("DIFF", None)
            driven = name in fdrive.LAYOUTS.get(dl.get("layout"), ())
            if driven and not susp.TOPOLOGY[corner["type"]].get("axle") and (dl.get(name) or {}).get("inner_L"):
                pts["DIFF"] = dl[name]["inner_L"]
        try:
            res = susp.analyse(corner, pts)
        except ValueError as exc:
            issues.append({"level": "ERROR", "rule": "7.1", "message": f"{name}: {exc}"})
            continue
        mounts = []
        for k in susp.INNER[corner["type"]]:
            ref = corner["points"][k]
            joint = ref["node"] if isinstance(ref, dict) and ref.get("node") in model["nodes"] else None
            d, what = _dist_to_frame(model, pts[k])
            entry = {"point": k, "label": susp.LABEL[k], "on_frame": joint is not None, "joint": joint,
                     "distance_mm": d * 1000, "nearest": what}
            if joint is None:
                hint = "split that tube there to make a joint" if what.startswith("tube") else "attach it to a joint"
                issues.append({"level": "ERROR", "rule": "7.1.6", "message":
                               f"{name} {susp.LABEL[k].lower()} is not on a frame joint ({d * 1000:.0f} mm from {what}); {hint}"})
            else:
                count, span, ok = fpick.joint_bracing(joint, model["nodes"][joint], _tube_ends(model, joint))
                entry.update(tubes=count, span=round(span, 3), braced=ok)
                if not ok:
                    issues.append({"level": "ERROR", "rule": "7.1.6", "message":
                                   f"{name} {susp.LABEL[k].lower()}: joint {joint} has {count} tubes that do not brace it "
                                   f"in three dimensions (span {span:.2f} < {fpick.MIN_SPAN}); add a tube out of their plane"})
            mounts.append(entry)
        st = res["static"]
        if st["usable_travel_mm"] < 150:
            issues.append({"level": "ERROR", "rule": "7.1.1", "message":
                           f"{name}: usable wheel travel {st['usable_travel_mm']:.0f} mm is under 150 mm"})
        if st["track_mm"] + susp.SPEC_TYRE["width"] * 1000 > arch:      # track + one tyre width = over the tyres
            issues.append({"level": "ERROR", "rule": "6.4", "message":
                           f"{name}: track {st['track_mm']:.0f} mm + tyre is wider than {arch} mm over the arches"})
        res["mounts"] = mounts
        res["points"] = pts
        res["type"] = corner["type"]
        res["labels"] = {k: susp.LABEL[k] for k in pts}
        out[name] = res
    return json.dumps({"corners": out, "issues": issues, "layouts": susp.describe()})


def driveline(model_json, suspension_json=None):
    """Driven axles, differentials, halfshafts and propshaft over the suspension travel (4.6).

    suspension_json is a suspension() result, recomputed when not given. Returns
    fbeam_driveline.analyse(): shaft angles (deg) and plunge (mm) per driven corner, issues.
    """
    model = json.loads(model_json)
    sus = json.loads(suspension_json) if suspension_json else json.loads(suspension(model_json))
    return json.dumps(fdrive.analyse(model, sus))


def set_driveline(model_json, layout):
    """Choose FWD, RWD or AWD; missing parts are filled from the Covet example.

    Returns the whole model JSON with model["driveline"]["layout"] set. Only FWD is legal
    in v1 (4.6); the others are for analysis.
    """
    model = json.loads(model_json)
    if layout not in fdrive.LAYOUTS:
        raise ValueError(f"unknown driveline layout {layout!r}")
    d = model.get("driveline") or fdrive.example()
    for k, v in fdrive.example().items():
        d.setdefault(k, v)
    d["layout"] = layout
    model["driveline"] = d
    return json.dumps(model)


def suspension_layout(model_json, corner_name, layout):
    """Switch a corner ("front" / "rear") to another layout (a fbeam_suspension.TOPOLOGY key),
    laid out around the same wheel centre as a sensible start (fbeam_suspension.default_points).

    The pickups start free, so 7.1.6 asks for them to be put on frame joints (pickups_to_joints).
    DIFF is kept. Rates and other corner fields are unchanged. Returns the whole model JSON.
    """
    model = json.loads(model_json)
    corner = model["suspension"][corner_name]
    if layout not in susp.TOPOLOGY:
        raise ValueError(f"unknown layout {layout!r}")
    old = corner["points"]
    wc = _point(model, old["WC"])
    new = susp.default_points(layout, wc, corner_name)
    if "DIFF" in old:
        new["DIFF"] = old["DIFF"]
    corner["type"] = layout
    corner["points"] = new
    return json.dumps(model)


def pickups_to_joints(model_json, corner_name):
    """Put a frame joint (and its mirror) at every free inner pickup of a corner and attach the pickup
    to it. The joints have no tubes yet: 7.1.6 then asks for three non-coplanar tubes at each.

    Joints are named <F|R><point>_L / _R (e.g. FLIF_L), with a number added if taken; single
    axle points (Panhard, Watt) and centre-line points get one joint without suffix.
    Returns the whole model JSON.
    """
    model = json.loads(model_json)
    corner = model["suspension"][corner_name]
    for k in susp.INNER[corner["type"]]:
        ref = corner["points"].get(k)
        if ref is None or isinstance(ref, dict):
            continue
        base = f"{corner_name[0].upper()}{k}"
        x, y, z = (round(c, 4) for c in ref)
        single = k in susp.TOPOLOGY[corner["type"]].get("single", []) or abs(x) < 1e-6

        def taken(name):                            # the name, or its mirror twin, is already a joint
            return name in model["nodes"] or (not single and fpick.mirror_name(name) in model["nodes"])
        n, i = (base if single else base + "_L"), 2
        while taken(n):
            n, i = (f"{base}{i}" if single else f"{base}{i}_L"), i + 1
        if k in susp.TOPOLOGY[corner["type"]].get("single", []):     # Panhard / Watt end: one joint, no mirror
            model["nodes"][n] = [x, y, z]
        elif abs(x) < 1e-6:                         # on the centre line: one joint
            model["nodes"][n] = [0.0, y, z]
        else:
            model["nodes"][n] = [x, y, z]
            model["nodes"][fpick.mirror_name(n)] = [-x, y, z]
        corner["points"][k] = {"node": n}
    return json.dumps(model)


def derive(model):
    """Values that follow from the design by construction: body panels (from the shell) and ballast (from steel).

    Changes the model in place; returns what was derived, for the editor to show:
    {"body": {"id", "mass" kg, "position", "size" m, "note"} (when there is a body and axle
    lines), "ballast": fbeam_mass.derive_ballast() result}. Rules 6.7 / 8.5 (panels), 6.10 (ballast).
    """
    out = {}
    b = model.get("body")
    ay = _axles_y(model)
    if b and ay:
        res = fbody.analyse(b, ay[0], cc.RULES)
        comps = model.setdefault("components", [])
        c = next((x for x in comps if x["kind"] == "body"), None)
        if c is None:
            c = {"id": "body_panels", "kind": "body", "mass": 0.0, "position": [0, 0, 0], "size": [1, 1, 1],
                 "estimate": False, "note": ""}
            comps.append(c)
        c.update({"mass": round(res["mass"], 3), "position": [round(v, 5) for v in res["cg"]],
                  "size": [round(v, 5) for v in res["size"]], "estimate": False,
                  "note": f"From the body shell: {res['skin_area_m2']:.1f} m² of {b['material']} ({res['kg_per_m2']:.2f} kg/m²)"})
        out["body"] = {"id": c["id"], "mass": c["mass"], "position": c["position"], "size": c["size"], "note": c["note"]}
    out["ballast"] = fmass.derive_ballast(model)
    return out


def body(model_json):
    """Body shell: profile, skin area, mass, CG and checks (6.2-6.7, 6.9).

    Needs model["body"] and axle lines (ValueError otherwise). Returns fbeam_body.analyse()
    with the overhang (6.6), frame-inside-body, closed-roof (6.7) and ground-clearance (6.9)
    issues added, and "triangles" (the shell mesh, [[p0, p1, p2], ...] in FBeam coordinates, m)
    for the 3D view and the SVJ bundle.
    """
    model = json.loads(model_json)
    ay = _axles_y(model)
    if not model.get("body"):
        raise ValueError("this model has no body shell")
    if ay is None:
        raise ValueError("set the front and rear axle joints to place the body")
    res = fbody.analyse(model["body"], ay[0], cc.RULES)
    fbody.overhangs_check(res, ay[0], ay[1], cc.RULES, res["issues"])
    rows, node_mass, _ = _design(model)
    fbody.body_check(model["body"], {n: model["nodes"][n] for n in node_mass}, res["issues"])
    res["triangles"] = fbody.mesh_triangles(res, model["body"]["width"])
    return json.dumps(res)


def masses(model_json):
    """Full-car mass, CG, inertia and mass-rule checks (4.4, 4.7, 6.7, 6.10, 6.11).

    Body panels and ballast are derived first (derive). Returns fbeam_mass.analyse() (kg, m,
    kg m^2; includes "front_share" when axle lines are known) plus "derived_ballast" and
    "derived_body", or null when there is nothing to weigh.
    """
    model = json.loads(model_json)
    rows, node_mass, _ = _design(model)
    der = derive(model)
    res = fmass.analyse(model, node_mass, cc.RULES["mass"]["min_race_mass_kg"])
    if res:
        res["derived_ballast"] = der["ballast"]
        res["derived_body"] = der.get("body")
    return json.dumps(res)


def fill_ballast(model_json):
    """The model with ballast added to reach the minimum Race Mass (6.11): {"model", "added_kg"}."""
    model = json.loads(model_json)
    derive(model)
    _, node_mass, _ = _design(model)
    added = fmass.fill_ballast(model, node_mass, cc.RULES["mass"]["min_race_mass_kg"])
    return json.dumps({"model": model, "added_kg": added})


def dynamics(model_json, suspension_json=None):
    """Steering geometry and steady-state roll / load transfer: {"steering", "roll", "notes"}.

    suspension_json is a suspension() result, recomputed when not given. Needs both corners;
    the wheelbase is the y distance between their wheel centres. "steering" is
    fbeam_dynamics.steering() (rack travel and mm per pinion turn from the front corner,
    defaults 70 mm / 65 mm), "roll" is fbeam_dynamics.roll_and_load_transfer() from the
    masses() result; either is null with a note when it cannot be computed.
    """
    model = json.loads(model_json)
    sus = json.loads(suspension_json) if suspension_json else json.loads(suspension(model_json))
    corners, notes, out = sus["corners"], [], {"steering": None, "roll": None, "notes": []}
    spec = (model.get("suspension") or {}).get("front")
    if "front" in corners and "rear" in corners:
        wb = abs(corners["rear"]["points"]["WC"][1] - corners["front"]["points"]["WC"][1])
        if spec:
            try:
                out["steering"] = fdyn.steering(spec, corners["front"]["points"], wb,
                                                spec.get("rack_travel_mm", 70), spec.get("rack_mm_per_rev", 65))
            except ValueError as exc:
                notes.append(f"steering: {exc}")
        ev = json.loads(masses(model_json))
        if ev and "front_share" in ev:
            arb = lambda n: math.degrees(model["suspension"][n].get("arb_Nm_per_deg", 0) or 0)   # N*m/deg -> N*m/rad
            out["roll"] = fdyn.roll_and_load_transfer(ev, corners, model.get("unsprung_mass_per_corner"), wb,
                                                      arb("front"), arb("rear"))
        else:
            notes.append("roll: needs the axle joints and a mass result")
    else:
        notes.append("needs a front and a rear corner")
    out["notes"] = notes
    return json.dumps(out)


def _axles_y(model):
    """(front, rear) axle line y in m: from the four hubs, else the axle joints, else None."""
    hubs = model.get("hubs") or {}
    if all(k in hubs for k in ("FL", "FR", "RL", "RR")):            # wheel centres are the axle lines
        return ((hubs["FL"][1] + hubs["FR"][1]) / 2, (hubs["RL"][1] + hubs["RR"][1]) / 2)
    ax = model.get("axles") or {}
    nodes = model["nodes"]
    if not (ax.get("front") and ax.get("rear") and all(n in nodes for n in ax["front"] + ax["rear"])):
        return None
    return (sum(nodes[n][1] for n in ax["front"]) / len(ax["front"]), sum(nodes[n][1] for n in ax["rear"]) / len(ax["rear"]))


def aero(model_json):
    """Wing and splitter: coefficients, forces, balance and 7.4 checks.

    The envelope passed on is the body width limit (6.3, narrowed to the body's width) and, with
    a body, (height, front bumper y, rear bumper y) in m. Returns fbeam_aero.analyse().
    """
    model = json.loads(model_json)
    ay = _axles_y(model)
    if ay is None:
        raise ValueError("set the front and rear axle joints to compute aero balance")
    d = cc.RULES["dimensions"]
    env = {"width": d["max_body_width_mm"] / 1000}
    if model.get("body"):
        b = model["body"]
        env["width"] = min(env["width"], b["width"])
        env["body"] = (b["height"], ay[0] - b["front_overhang"], ay[0] - b["front_overhang"] + b["length"])
    return json.dumps(faero.analyse(model, cc.RULES["aero"], ay, env))


def export_svj(model_json, rigidity_json=None):
    """SVJ document for this model (regulations and editor data mapped to SAE J670; docs/svj.md).

    rigidity_json is an optional rigidity() result. Results the SVJ needs (node masses,
    steering, aero) are passed to fbeam_svj.to_svj as temporary "_" keys of the model and
    removed from the embedded copy. Returns the SVJ as indented JSON text.
    """
    model = json.loads(model_json)
    derive(model)
    model_json = json.dumps(model)
    ev = json.loads(evaluate(model_json))
    sus = json.loads(suspension(model_json)) if model.get("suspension") else None
    rig = json.loads(rigidity_json) if rigidity_json else None
    model["_node_mass"] = ev["node_mass"]
    model["_steering"] = json.loads(dynamics(model_json, json.dumps(sus)))["steering"] if sus else None
    model["_aero"] = json.loads(aero(model_json)) if model.get("aero") and _axles_y(model) else None
    doc = svj.to_svj(model, sus, rig, ev["totals"], ev["cog"])
    for k in ("_node_mass", "_steering", "_aero"):
        doc["x_fbeam"]["model"].pop(k, None)
    return json.dumps(doc, indent=2)


# paths of the generated meshes inside an SVJ bundle (relative, as SVJ requires)
FRAME_MESH_URI = "meshes/fbeam_tube_frame.glb"
BODY_MESH_URI = "meshes/fbeam_body.glb"


def export_svj_bundle(model_json, rigidity_json, out_zip, extra_dir=None, name="fbeam_car"):
    """SVJ plus meshes as one zip (SVJ requires relative mesh paths, so they travel together).

    Writes <name>.svj.json, a generated .glb of the tube frame, and every mesh file
    found in extra_dir (meshes that came with an imported SVJ) at its original path.
    With a body and axle lines, a .glb of the body shell is added and bound to the
    body_panels mass body. Mesh coordinates are SAE (fbeam_svj.to_sae).

    Returns {"zip_path", "files", "frame_node" (glTF node of the frame mesh), "missing_meshes"
    (listed in the SVJ but not found in extra_dir), "svj" (SVJ version)}.
    """
    model = json.loads(model_json)
    derive(model)
    model_json = json.dumps(model)
    ev = json.loads(evaluate(model_json))
    sus = json.loads(suspension(model_json)) if model.get("suspension") else None
    rig = json.loads(rigidity_json) if rigidity_json else None
    model["_node_mass"] = ev["node_mass"]
    model["_steering"] = json.loads(dynamics(model_json, json.dumps(sus)))["steering"] if sus else None
    model["_aero"] = json.loads(aero(model_json)) if model.get("aero") and _axles_y(model) else None
    doc = svj.to_svj(model, sus, rig, ev["totals"], ev["cog"],
                     frame_mesh_uri=FRAME_MESH_URI if ev["members"] else None)
    for k in ("_steering", "_aero"):
        model.pop(k, None)
        doc["x_fbeam"]["model"].pop(k, None)
    model.pop("_node_mass")
    doc["x_fbeam"]["model"].pop("_node_mass", None)
    yf = svj._front_y(model)
    tubes = [(svj.to_sae(model["nodes"][m["a"]], yf), svj.to_sae(model["nodes"][m["b"]], yf),
              float(m["tube"].split("x")[0]) / 2000) for m in ev["members"]]     # radius m = D mm / 2000
    node = (doc["x_fbeam"].get("frame_mesh") or {}).get("node") or "SVJ::body::chassis"
    files = {f"{name}.svj.json": json.dumps(doc, indent=2).encode()}
    if tubes:
        files[FRAME_MESH_URI] = gltf.tubes_glb(tubes, node)
    if model.get("body") and _axles_y(model):                # body shell mesh, bound to its mass body (SVJ §22.3)
        bres = json.loads(body(model_json))
        tris = [tuple(svj.to_sae(p, yf) for p in t) for t in bres["triangles"]]
        files[BODY_MESH_URI] = gltf.triangles_glb(tris, "SVJ::body::body_panels", opacity=0.6)
        meshes = doc.setdefault("assets", {}).setdefault("meshes", [])
        meshes[:] = [a for a in meshes if a["id"] != "fbeam_body"] + [{"id": "fbeam_body", "uri": BODY_MESH_URI, "description": "FBeam body shell (generated)"}]
        for mb in doc["chassis"].get("mass_bodies", []):
            if mb["id"] == "body_panels":
                mb["visual"] = {"mesh_ref": "fbeam_body", "node": "SVJ::body::body_panels"}
        files[f"{name}.svj.json"] = json.dumps(doc, indent=2).encode()
    kept = []
    for a in (doc.get("assets") or {}).get("meshes", []):
        if a["id"] in (svj.FRAME_MESH_ID, "fbeam_body") or not extra_dir:
            continue
        src = Path(extra_dir) / a["uri"]
        if src.is_file():
            files[a["uri"]] = src.read_bytes()
            kept.append(a["uri"])
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for p, data in files.items():
            z.writestr(p, data)
    Path(out_zip).write_bytes(buf.getvalue())
    missing = [a["uri"] for a in (doc.get("assets") or {}).get("meshes", [])
               if a["id"] not in (svj.FRAME_MESH_ID, "fbeam_body") and a["uri"] not in kept]
    return json.dumps({"zip_path": str(out_zip), "files": sorted(files), "frame_node": node if tubes else None,
                       "missing_meshes": missing, "svj": doc["_metadata"]["version"]})


def _safe_extract(zpath, base):
    """Empty `base` (deleting it) and extract the zip into it, refusing paths that leave it."""
    if base.exists():
        shutil.rmtree(base)
    base.mkdir(parents=True)
    root = base.resolve()
    with zipfile.ZipFile(zpath) as z:
        for info in z.infolist():
            target = (base / info.filename).resolve()
            if root not in target.parents:
                raise ValueError(f"unsafe path in zip: {info.filename}")
            if not info.is_dir():
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(z.read(info))


def import_svj_bundle(path, work_dir, base_model_json=None, options_json=None):
    """Import an SVJ from a .zip bundle, or from a .svj.json whose meshes sit beside it.

    With base_model_json the file is applied to that model (apply_svj, keeping its frame;
    options_json as there) instead of replacing it.

    A zip is extracted into work_dir, which is deleted first. Mesh files are matched by
    their uri, else by file name anywhere under the SVJ's folder; their glTF node names are
    read so each visual binding can be checked.

    Returns {"model", "notes", "meshes": [{"id", "uri", "file", "nodes"}], "bindings", "axes",
    "base_dir", "y_front" (front axle y of the model, used to convert to SAE)}.
    """
    p, base = Path(path), Path(work_dir)
    if zipfile.is_zipfile(p):
        _safe_extract(p, base)
        cands = sorted(base.rglob("*.svj.json")) or [
            f for f in sorted(base.rglob("*.json")) if '"_metadata"' in f.read_text("utf-8", "ignore")[:4000]]
        if not cands:
            raise ValueError("no .svj.json file in the zip")
        svj_file = cands[0]
    else:
        svj_file = p
    doc = json.loads(svj_file.read_text(encoding="utf-8"))
    if base_model_json:
        res = json.loads(apply_svj(base_model_json, json.dumps(doc), options_json))
        opts = dict(APPLY_DEFAULTS, **(json.loads(options_json) if options_json else {}))
    else:
        res = json.loads(import_svj(json.dumps(doc)))
        opts = {"meshes": True}
    notes, meshes = res["notes"], []
    root = svj_file.parent
    for a in ((doc.get("assets") or {}).get("meshes", []) if opts["meshes"] else []):
        f = root / a["uri"]
        if not f.is_file():                           # loose files: match by file name
            hits = sorted(root.rglob(Path(a["uri"]).name))
            f = hits[0] if hits else None
        entry = {"id": a["id"], "uri": a["uri"], "file": str(f) if f else None, "nodes": []}
        if f:
            try:
                entry["nodes"] = gltf.node_names(f.read_bytes(), is_glb=f.suffix.lower() == ".glb")
            except (ValueError, UnicodeDecodeError) as exc:
                notes.append(f"mesh {a['uri']}: {exc}")
        else:
            notes.append(f"mesh {a['uri']} (id {a['id']}) is listed in the SVJ but was not provided")
        meshes.append(entry)
    bindings = svj.visual_bindings(doc) if opts["meshes"] else []
    by_id = {m["id"]: m for m in meshes}
    for b in bindings:
        m = by_id.get(b["mesh_ref"])
        if m is None:
            notes.append(f"{b['path']}: visual mesh_ref {b['mesh_ref']!r} is not in assets.meshes")
        elif m["file"] and b["node"] not in m["nodes"]:
            notes.append(f"{b['path']}: node {b['node']} not found in {m['uri']}")
    return json.dumps({"model": res["model"], "notes": notes, "meshes": meshes, "bindings": bindings,
                       "axes": svj.gltf_axes(doc), "base_dir": str(root), "y_front": svj._front_y(res["model"])})


def import_svj(svj_json):
    """FBeam model from an SVJ document, replacing the current one: {"model": ..., "notes": [...]}.

    SVJ gives wheel rate and motion ratio; FBeam stores the coil rate. The coil rate is set from
    the geometry's own motion ratio so the file's wheel rate is kept (wheel rate = k * MR^2).
    To apply a file to the current frame instead, use apply_svj.
    """
    model, notes = svj.from_svj(json.loads(svj_json))
    _fix_wheel_rates(model, notes)
    return json.dumps({"model": model, "notes": notes})


APPLY_DEFAULTS = {"move_joints": True, "layout": True, "meshes": True, "rates": True}
ATTACH_TOL = 0.005        # m: an SVJ pickup this close to an existing joint is attached to it
MOVE_TOL = 0.001          # m: a pickup that moved less than this is unchanged


def _free_joint_name(model, base, single):
    """A joint name not yet used: <base>_L (its mirror _R free too), or <base> for a single point."""
    def taken(n):
        return n in model["nodes"] or (not single and fpick.mirror_name(n) in model["nodes"])
    n, i = (base if single else base + "_L"), 2
    while taken(n):
        n, i = (f"{base}{i}" if single else f"{base}{i}_L"), i + 1
    return n


def _fix_wheel_rates(model, notes):
    """SVJ stores the wheel rate; FBeam the coil rate. Set each pending corner's coil rate from the
    geometry's own motion ratio so the file's wheel rate is kept (wheel rate = k * MR^2)."""
    pending = {n: c for n, c in (model.get("suspension") or {}).items() if "_svj_wheel_rate" in c}
    if pending:
        res = json.loads(suspension(json.dumps(model)))["corners"]
        for name, c in pending.items():
            wr, mr_decl = c.pop("_svj_wheel_rate"), c.pop("_svj_motion_ratio")
            mr = (res.get(name) or {}).get("static", {}).get("motion_ratio")
            if mr:
                c["spring_N_per_mm"] = round(wr / (mr * mr) / 1000, 2)     # the SVJ wheel rate is preserved
                if abs(mr - mr_decl) > 0.05 * mr_decl:
                    notes.append(f"{name}: motion ratio from geometry is {mr:.2f}, the file declares {mr_decl:.2f} "
                                 f"(over 5 %); kept the file's wheel rate {wr / 1000:.1f} N/mm")
    for c in (model.get("suspension") or {}).values():
        c.pop("_svj_wheel_rate", None), c.pop("_svj_motion_ratio", None)


def apply_svj(model_json, svj_json, options_json=None):
    """Apply an SVJ's suspension to the current model, keeping its frame.

    The SVJ's front axle is placed on the current front axle (wheel centres), so its hardpoints
    land where that file puts them relative to the front wheels. For every corner in the file:
      layout      with options "layout" the corner takes the file's layout (macpherson,
                  double_wishbone, or any FBeam layout written by FBeam); without it a corner
                  whose layout differs is left alone
      pickups     a pickup on a frame joint that moved by more than MOVE_TOL takes its joint
                  along when "move_joints" is on (the joint's mirror moves too, and every tube on
                  it follows); otherwise it is detached and left free (7.1.6 then reports it).
                  A pickup with no joint yet attaches to an existing joint within ATTACH_TOL, or
                  gets a new one, with no tubes (7.1.6 asks for bracing)
      moving pts  upright points and the wheel centre are set from the file; hubs follow
      rates       with "rates", springs, dampers and stroke come from the file (wheel rate kept)
    With "meshes" the file's glTF assets and its chassis visual binding are kept in
    model["svj_source"], so they show as reference geometry and are written on export; the
    rest of the file (tyres, powertrain, masses) is not taken.
    The file's own FBeam frame (x_fbeam.model), if any, is ignored.
    options_json: {"move_joints", "layout", "meshes", "rates"} (all default True).
    Returns {"model", "notes", "moved_joints": [...], "new_joints": [...], "attached": [...]}.
    """
    model = json.loads(model_json)
    doc = json.loads(svj_json)
    opt = dict(APPLY_DEFAULTS, **(json.loads(options_json) if options_json else {}))
    meta = doc.get("_metadata", {})
    if meta.get("specification") != "SVJ":
        raise ValueError("not an SVJ file (_metadata.specification is not 'SVJ')")
    yf = svj._front_y(model)
    notes, moved, created, attached = [], [], [], []
    if (doc.get("x_fbeam") or {}).get("model"):
        notes.append("The file's own FBeam frame is ignored: its suspension is applied to the current frame")
    model.setdefault("suspension", {})
    hubs = model.setdefault("hubs", {})
    moved_to = {}                                     # joint -> new position, this import

    def move_joint(j, p):
        model["nodes"][j] = [round(c, 5) for c in p]
        twin = fpick.mirror_name(j)
        if twin != j and twin in model["nodes"]:
            model["nodes"][twin] = [round(-p[0], 5), round(p[1], 5), round(p[2], 5)]
        moved_to[j] = p

    for name, (L, _R) in svj.CORNERS.items():
        corner_doc = (doc.get("suspension") or {}).get(L)
        if not corner_doc:
            continue
        spec, pts, n = svj._corner_from_svj(corner_doc, yf, name)
        notes += n
        if not spec:
            continue
        cur = model["suspension"].get(name)
        if cur and cur["type"] != spec["type"] and not opt["layout"]:
            notes.append(f"{name}: the file has a {susp.TOPOLOGY[spec['type']]['name'].lower()}, the car a "
                         f"{susp.TOPOLOGY[cur['type']]['name'].lower()}; kept the car's (layout option off)")
            continue
        same = bool(cur) and cur["type"] == spec["type"]
        new = dict(cur) if same else {k: v for k, v in (cur or {}).items() if k not in ("type", "points")}
        new["type"] = spec["type"]
        if opt["rates"] or not same:
            for k, v in spec.items():
                if k != "type":
                    new[k] = v
        old_pts = (cur or {}).get("points", {}) if same else {}
        points = {}
        single = set(susp.TOPOLOGY[spec["type"]].get("single", []))
        for k, p in pts.items():
            if k not in susp.INNER[spec["type"]]:
                points[k] = [round(c, 5) for c in p]            # upright points, wheel centre, DIFF
                continue
            ref = old_pts.get(k)
            if isinstance(ref, dict) and ref.get("node") in model["nodes"]:
                j = ref["node"]
                at = model["nodes"][j]
                if math.dist(at, p) < MOVE_TOL:
                    points[k] = ref
                    continue
                if opt["move_joints"] and (j not in moved_to or math.dist(moved_to[j], p) < MOVE_TOL):
                    move_joint(j, p)
                    moved.append(f"{j} ({math.dist(at, p) * 1000:.0f} mm, {name} {k})")
                    points[k] = ref
                    continue
                if opt["move_joints"]:
                    notes.append(f"{name} {k}: joint {j} was already moved for another pickup; this one gets its own joint")
            near = min(model["nodes"], key=lambda q: math.dist(model["nodes"][q], p), default=None)
            if near is not None and math.dist(model["nodes"][near], p) <= ATTACH_TOL:
                points[k] = {"node": near}
                attached.append(f"{name} {k} -> {near}")
                continue
            if not opt["move_joints"] and isinstance(ref, dict):
                points[k] = [round(c, 5) for c in p]            # detached: 7.1.6 will ask for a joint
                continue
            x = round(p[0], 5)
            is_single = k in single or abs(x) < 1e-6
            j = _free_joint_name(model, f"{name[0].upper()}{k}", is_single)
            model["nodes"][j] = [x, round(p[1], 5), round(p[2], 5)]
            if not is_single:
                model["nodes"][fpick.mirror_name(j)] = [-x, round(p[1], 5), round(p[2], 5)]
            points[k] = {"node": j}
            created.append(j)
        if "DIFF" in old_pts and "DIFF" not in points:
            points["DIFF"] = old_pts["DIFF"]                    # SVJ has no halfshaft inner joint for these
        new["points"] = points
        model["suspension"][name] = new
        wc = pts["WC"]
        tag = "F" if name == "front" else "R"
        hubs[tag + "L"] = [round(c, 5) for c in wc]
        hubs[tag + "R"] = [round(-wc[0], 5), round(wc[1], 5), round(wc[2], 5)]
    _fix_wheel_rates(model, notes)
    if opt["meshes"]:
        generated = (svj.FRAME_MESH_ID, "fbeam_body")        # FBeam's own frame / body meshes: regenerated on export
        assets = [a for a in (doc.get("assets") or {}).get("meshes", []) if a.get("id") not in generated]
        if assets:
            src = model.setdefault("svj_source", {})
            keep = [a for a in (src.get("assets") or {}).get("meshes", []) if a["id"] not in {b["id"] for b in assets}]
            src.setdefault("assets", {})["meshes"] = keep + assets
            vis = (doc.get("chassis") or {}).get("visual")
            if vis and vis.get("mesh_ref") not in generated:
                src.setdefault("chassis", {})["visual"] = vis
            cs = meta.get("coordinate_system")
            if isinstance(cs, dict):
                src.setdefault("_metadata", {})["coordinate_system"] = cs
            notes.append(f"{len(assets)} mesh asset(s) copied as reference geometry")
    if moved:
        notes.append("Moved frame joints with their pickups: " + ", ".join(moved))
    if created:
        notes.append("New frame joints (no tubes yet; brace them, 7.1.6): " + ", ".join(created))
    if attached:
        notes.append("Pickups attached to existing joints: " + ", ".join(attached))
    derive(model)
    return json.dumps({"model": model, "notes": notes, "moved_joints": moved, "new_joints": created, "attached": attached})


def section_preview(model_json, member_id, tube):
    """What changing one tube's section would do, before it is applied.

    member_id is a member "id", tube a section like "40x2". Returns {"d_mass" (kg),
    "d_deform_pct" (change of beamDeform, %), "new_deform_kN"}; springs are not stability-capped here.
    """
    model = json.loads(model_json)
    m = next(x for x in model["members"] if x["id"] == member_id)
    L = math.dist(model["nodes"][m["a"]], model["nodes"][m["b"]])
    Lb = buckling_lengths(model)[member_id]
    cur = parse_section(f"tube {m['tube']} {MATERIAL}", cc.MATERIALS, cc.RULES).limits(L, Lb)
    new = parse_section(f"tube {tube} {MATERIAL}", cc.MATERIALS, cc.RULES).limits(L, Lb)
    return json.dumps({"d_mass": new["mass"] - cur["mass"],
                       "d_deform_pct": (new["beamDeform"] - cur["beamDeform"]) / cur["beamDeform"] * 100,
                       "new_deform_kN": new["beamDeform"] / 1e3})


# ---------------------------------------------------------------- output

def beam_line(r):
    """The exact jbeam beam row the editor exports for one tube (8.3, 8.10.2).

    beamSpring is the stability-capped k_set; beamDeform and beamStrength are written 1 %
    under the limits so rounding can never take them over the checker's tolerance.
    """
    return (f'["{r["a"]}", "{r["b"]}", {{"fbeamSection": "tube {r["tube"]} {MATERIAL}", "fbeamMember": "m{r["id"]}", '
            f'"beamSpring": {r["k_set"]:.0f}, "beamDeform": {r["beamDeform"] * 0.99:.0f}, '
            f'"beamStrength": {r["beamStrength"] * 0.99:.0f}}}]')


def bend_line(b):
    """The jbeam row of a bending beam: weightless, names the tubes whose bending it reproduces (8.10.2).

    All three values are written 1 % under the budget, as in beam_line.
    """
    mem = ",".join(f"m{i}" for i in b["members"])
    return (f'["{b["a"]}", "{b["b"]}", {{"fbeamRole": "bending", "fbeamMember": "{mem}", '
            f'"beamSpring": {b["k_set"] * 0.99:.0f}, "beamDeform": {b["beamDeform"] * 0.99:.0f}, '
            f'"beamStrength": {b["beamStrength"] * 0.99:.0f}}}]')


def _write_submission(model, rows, node_mass, out):
    """Write a checkable Submission into `out` (a Path): vehicles/<PART>/ with the chassis jbeam,
    the suspension + wheel data jbeam (fbeam_vehicle), race.pc and info.json, plus fbeam.json
    (the checker manifest) and model.fbeam.json (the editor model). rows / node_mass from _design.
    """
    # suspension first: arms attached to chassis joints add their tube share to those joints
    sus = json.loads(suspension(json.dumps(model))) if model.get("suspension") else {"corners": {}}
    snodes, sbeams, _, extra = fveh.build_suspension(
        model, sus, node_mass, rows,
        lambda tube, L: parse_section(f"tube {tube} {MATERIAL}", cc.MATERIALS, cc.RULES).limits(L))
    # (k_set was capped with node_mass alone, so the extra arm mass only makes the springs safer)
    weights = {n: math.ceil((node_mass[n] + extra.get(n, 0.0)) * 20) / 20 for n in node_mass}
    nodes = ['["id", "posX", "posY", "posZ"]', '{"group": "fbeam_chassis"}']
    for n in sorted(node_mass):
        x, y, z = model["nodes"][n]
        nodes.append(f'["{n}", {x}, {y}, {z}, {{"nodeWeight": {weights[n]}}}]')
    beams = ['["id1:", "id2:"]', '{"beamDamp": 150}']
    for r in rows:
        beams.append(beam_line(r))
    for b in bending_beams(model, rows, node_mass):
        beams.append(bend_line(b))
    # suspension part (full car; unverified in-game) and the vehicle files BeamNG needs to spawn
    refn = fveh.ref_nodes({n: model["nodes"][n] for n in node_mass})
    slots = ""
    if sbeams:
        slots = ('    "slots": [\n        ["type", "default", "description"],\n'
                 f'        ["{fveh.PART}", "{fveh.PART}", "Suspension (FBeam editor)"]\n    ],\n')
    part = ('{\n"' + PART + '": {\n    "information": {"authors": "FBeam editor", "name": "FBeam tube-frame chassis"},\n'
            '    "slotType": "main",\n    // generated by the FBeam editor; edit the .fbeam.json model instead\n' + slots +
            f'    "refNodes": [\n        ["ref:", "back:", "left:", "up:"],\n        ["{refn["ref"]}", "{refn["back"]}", "{refn["left"]}", "{refn["up"]}"]\n    ],\n'
            '    "nodes": [\n        ' + ",\n        ".join(nodes) + '\n    ],\n    "beams": [\n        '
            + ",\n        ".join(beams) + "\n    ]\n}\n}\n")
    veh = out / "vehicles" / PART
    veh.mkdir(parents=True, exist_ok=True)
    (veh / f"{PART}.jbeam").write_text(part, encoding="utf-8", newline="\n")
    parts = {PART: PART}
    unsprung_nodes = []
    if sbeams:
        sn = ['["id", "posX", "posY", "posZ"]', '{"group": "fbeam_suspension"}']
        for n, d in sorted(snodes.items()):
            sn.append(f'["{n}", {d["pos"][0]}, {d["pos"][1]}, {d["pos"][2]}, {{"nodeWeight": {round(d["mass"], 3)}}}]')
            if d.get("upright"):
                unsprung_nodes.append(n)
        # stability cap per beam (8.9.6), from the final node masses: arm, upright and bracket beams on
        # light nodes (0.5 kg brackets, damper bottoms) would otherwise exceed it. Springs keep their rate.
        mass_of = dict(weights)
        mass_of.update({n: d["mass"] for n, d in snodes.items()})
        mass_of.update(fveh.rim_axle_node_masses())

        def capped(b):
            ma, mb = mass_of.get(b["a"]), mass_of.get(b["b"])
            if not ma or not mb:
                return b["k"]
            return min(b["k"], cc.STABILITY_SAFETY * (ma * mb / (ma + mb)) * (2 * cc.PHYSICS_HZ) ** 2)
        for b in sbeams:
            if b["role"] != "spring":
                b["k"] = capped(b)
        sb = ['["id1:", "id2:"]', '{"beamDamp": 150}']
        for i, b in enumerate(sbeams):
            if b["role"] == "member":                        # arm tubes: section-checked like the chassis (7.1.3)
                sb.append(f'["{b["a"]}", "{b["b"]}", {{"fbeamSection": "tube {b["tube"]} {MATERIAL}", "fbeamMember": "s{i}", '
                          f'"beamSpring": {b["k"]:.0f}, "beamDeform": {b["F"] * 0.99:.0f}, "beamStrength": {b["Fu"] * 0.99:.0f}}}]')
            elif b["role"] == "spring":                      # coilover: high finite limits so it never yields (8.1.3)
                sb.append(f'["{b["a"]}", "{b["b"]}", {{"fbeamRole": "spring", "beamSpring": {b["k"]:.0f}, '
                          f'"beamDamp": {b["damp"]:.0f}, "beamDeform": 1000000, "beamStrength": 1500000}}]')
            else:                                            # upright / mount: beamStrength = 1.2 x beamDeform
                sb.append(f'["{b["a"]}", "{b["b"]}", {{"fbeamRole": "{b["role"]}", "beamSpring": {b["k"]:.0f}, '
                          f'"beamDeform": {b["F"]:.0f}, "beamStrength": {b["F"] * 1.2:.0f}}}]')
        wslots, wrows, wbeams, wpc = fveh.build_wheels(sus, snodes)
        for b in wbeams:
            b["k"] = capped(b)
        slots2 = ""
        if wslots:
            slots2 = ('    "slots2": [\n        ["name", "allowTypes", "denyTypes", "default", "description"],\n        '
                      + ",\n        ".join(json.dumps(r) for r in wslots) + "\n    ],\n")
        spart = ('{\n"' + fveh.PART + '": {\n    "information": {"authors": "FBeam editor", "name": "FBeam suspension (unverified in-game)"},\n'
                 f'    "slotType": "{fveh.PART}",\n    // generated by the FBeam editor; steering hydro and powertrain attach via the donor interface\n'
                 + slots2 +
                 '    "nodes": [\n        ' + ",\n        ".join(sn) + '\n    ],\n    "beams": [\n        ' + ",\n        ".join(sb) + "\n    ]\n}")
        if wslots:
            # spec wheel (rules 5): rim axle nodes tied to the uprights; tyre and rim values come from the spec parts
            wb = ['["id1:", "id2:"]', '{"beamDamp": 150}'] + [
                f'["{b["a"]}", "{b["b"]}", {{"fbeamRole": "upright", "beamSpring": {b["k"]:.0f}, '
                f'"beamDeform": {b["F"]:.0f}, "beamStrength": {b["F"] * 1.2:.0f}}}]' for b in wbeams]
            pw = ['["name", "hubGroup", "group", "node1:", "node2:", "nodeS", "nodeArm:", "wheelDir"]'] + [json.dumps(r) for r in wrows]
            spart += (',\n"' + fveh.WHEELDATA_PART + '": {\n    "information": {"authors": "FBeam editor", "name": "FBeam wheel data (unverified in-game)"},\n'
                      f'    "slotType": "{fveh.WHEELDATA_PART}",\n'
                      '    "beams": [\n        ' + ",\n        ".join(wb) + '\n    ],\n'
                      '    "pressureWheels": [\n        ' + ",\n        ".join(pw) + "\n    ]\n}")
        (veh / f"{fveh.PART}.jbeam").write_text(spart + "\n}\n", encoding="utf-8", newline="\n")
        parts[fveh.PART] = fveh.PART
        parts.update(wpc)
    (veh / "race.pc").write_text(json.dumps({"format": 2, "model": PART, "parts": parts, "vars": {}},
                                            indent=2), encoding="utf-8", newline="\n")
    (veh / "info.json").write_text(json.dumps(fveh.info_json(PART), indent=2), encoding="utf-8", newline="\n")
    # chassis_parts: every part whose .pc entry is itself (chassis, suspension, wheel data), not the spec rims
    manifest = {"team": "Editor", "car_id": "FB-EDIT", "beamng_version": "TBD",
                "pc": f"vehicles/{PART}/race.pc", "chassis_parts": [p for p in parts if p == parts[p]], "unsprung_nodes": sorted(unsprung_nodes),
                # 7.1.6: every node of the suspension part moves with the wheels; the checker treats the rest as frame
                "suspension_nodes": sorted(snodes) if sbeams else []}
    axles = model.get("axles", {})
    if {"fhubi_L", "fhubi_R", "rhubi_L", "rhubi_R"} <= set(snodes if sbeams else ()):
        # hub nodes sit on the wheel centres: the wheelbase is measured where the wheels are (6.1)
        manifest["front_axle_nodes"], manifest["rear_axle_nodes"] = ["fhubi_L", "fhubi_R"], ["rhubi_L", "rhubi_R"]
    elif axles.get("front") and axles.get("rear") and all(n in node_mass for n in axles["front"] + axles["rear"]):
        manifest["front_axle_nodes"], manifest["rear_axle_nodes"] = axles["front"], axles["rear"]
    (out / "fbeam.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8", newline="\n")
    (out / "model.fbeam.json").write_text(json.dumps(model, indent=2), encoding="utf-8", newline="\n")


def build(model_json, out_dir):
    """Write the Submission into out_dir (deleted first), run the real checker, and zip it
    beside it as <out_dir>.zip.

    Returns {"passed" (no ERROR finding), "findings": [{"level", "rule", "message", ...}],
    "file_hash", "jbeam_digest" (10.2), "dimensions" (as measured by the checker), "zip_path"}.
    Raises ValueError for an invalid frame or one with no tubes.
    """
    model = json.loads(model_json)
    rows, node_mass, orphans = _design(model)
    if not rows:
        raise ValueError("there are no tubes to build")
    out = Path(out_dir)
    if out.exists():
        shutil.rmtree(out)
    _write_submission(model, rows, node_mass, out)
    findings, stats = run_all(Submission(out), cc.RULES)
    zpath = out.parent / (out.name + ".zip")      # (with_suffix would cut "build.v1" to "build.zip")
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(out.rglob("*")):
            if f.is_file():
                z.write(f, f.relative_to(out).as_posix())
    zpath.write_bytes(buf.getvalue())
    notes = [{"level": "WARN", "rule": "editor", "message": f"joint {n} has no tubes and was left out of the jbeam"}
             for n in orphans]
    return json.dumps({"passed": not any(f.level == "ERROR" for f in findings),
                       "findings": notes + [f.__dict__ for f in findings],
                       "file_hash": stats["file_hash"], "jbeam_digest": stats["jbeam_digest"],
                       "dimensions": stats.get("dimensions", {}), "zip_path": str(zpath)})
