"""Worked safety-cage design for a compact FBeam car (regulations 8.9.4 - 8.9.6).

The 12-node, 18-tube cage was laid out for the earlier Civic envelope (withdrawn,
regulations 6); the editor's example car (editor/fbeam_editor_core.py) uses the
same layout moved into the Ibishu Covet cabin. Computes every cage member from
its tube (8.2: stiffness, yield / buckling force and mass from the section; 8.9.6:
springs capped for integration stability), prints the tables used in the
regulations, and generates a checkable Submission in examples/demo_cage.

The editor also imports this module for RULES, MATERIALS, PHYSICS_HZ,
STABILITY_SAFETY, JOINT, NODES, OPTIONS and MEMBERS, so keep them importable
without side effects. Running it as a script with the recommended option
overwrites examples/demo_cage.

    python examples/cage/cage_calc.py                # recommended option, writes demo_cage
    python examples/cage/cage_calc.py --option fia_min   # 45x2.5 / 38x2.5 everywhere
    python examples/cage/cage_calc.py --option light     # 50x2.0 / 40x2.0 everywhere
"""

import json
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from fbeam_checker.sections import load_materials, parse_section  # noqa: E402

RULES = json.loads((ROOT / "rules" / "fbeam_v1.json").read_text(encoding="utf-8"))
MATERIALS = load_materials(RULES)

PHYSICS_HZ = 2000          # BeamNG physics rate (8.9.6; to confirm in-game)
STABILITY_SAFETY = 0.25    # fraction of the explicit-integration limit used (8.9.6)
JOINT = RULES["chassis"]["joint_allowance"]     # extra node mass for welds and brackets (0.15)

# BeamNG axes: X left(+)/right(-), Y forward(-)/rear(+), Z up. Metres. Civic-era positions;
# the editor's full_chassis_model() overrides them with Covet-cabin positions.
NODES = {
    "MHf_L": (0.60, 0.25, 0.20), "MHf_R": (-0.60, 0.25, 0.20),    # main hoop feet
    "MHm_L": (0.58, 0.25, 0.70), "MHm_R": (-0.58, 0.25, 0.70),    # main hoop, door-bar joint
    "MHt_L": (0.55, 0.25, 1.20), "MHt_R": (-0.55, 0.25, 1.20),    # main hoop top corners
    "APf_L": (0.60, -0.95, 0.35), "APf_R": (-0.60, -0.95, 0.35),  # A-pillar feet
    "APt_L": (0.55, -0.40, 1.18), "APt_R": (-0.55, -0.40, 1.18),  # A-pillar tops
    "BSf_L": (0.50, 1.15, 0.40), "BSf_R": (-0.50, 1.15, 0.40),    # backstay feet
}

# Tube per member kind (D x t, mm): FIA 253-8.3.3 minimums (8.9.1) are 45x2.5 or 50x2.0 (main),
# 38x2.5 or 40x2.0 (other). "diag" is the main-hoop diagonal, an "other" member. Compared in 8.9.5.
OPTIONS = {
    "recommended": {"main": "45x2.5", "other": "40x2", "diag": "50x2"},
    "fia_min": {"main": "45x2.5", "other": "38x2.5", "diag": "38x2.5"},
    "light": {"main": "50x2", "other": "40x2", "diag": "40x2"},
}

# (group, node a, node b, member kind); the 18 tubes of 8.9.4
MEMBERS = [
    ("Main hoop, lower leg", "MHf_L", "MHm_L", "main"), ("Main hoop, lower leg", "MHf_R", "MHm_R", "main"),
    ("Main hoop, upper leg", "MHm_L", "MHt_L", "main"), ("Main hoop, upper leg", "MHm_R", "MHt_R", "main"),
    ("Main hoop, top", "MHt_L", "MHt_R", "main"),
    ("Lateral rollbar, A-pillar", "APf_L", "APt_L", "main"), ("Lateral rollbar, A-pillar", "APf_R", "APt_R", "main"),
    ("Lateral rollbar, roof rail", "APt_L", "MHt_L", "main"), ("Lateral rollbar, roof rail", "APt_R", "MHt_R", "main"),
    ("Windscreen header", "APt_L", "APt_R", "other"),
    ("Backstay", "MHt_L", "BSf_L", "other"), ("Backstay", "MHt_R", "BSf_R", "other"),
    ("Main hoop diagonal", "MHt_L", "MHf_R", "diag"),
    ("Roof diagonal", "APt_L", "MHt_R", "other"),
    ("Door bar, upper", "APf_L", "MHm_L", "other"), ("Door bar, upper", "APf_R", "MHm_R", "other"),
    ("Door bar, lower", "APf_L", "MHf_L", "other"), ("Door bar, lower", "APf_R", "MHf_R", "other"),
]


def design(sizes):
    """Limits, node masses and stability-capped springs for one OPTIONS entry.

    sizes is {kind: "<D>x<t>"}. Returns (rows, node_mass): rows are the MEMBERS with tube,
    L (m), sec (Section) and Section.limits (beamSpring N/m, beamDeform, beamStrength,
    yield_force, buckling_force N, mass kg), plus k_stable, k_set (N/m) and governs;
    node_mass is {node: kg}. Buckling length is the tube length (no split tubes here).
    """
    steel = "steel_1018"
    rows = []
    share = defaultdict(float)
    for group, a, b, kind in MEMBERS:
        tube = sizes[kind]
        sec = parse_section(f"tube {tube} {steel}", MATERIALS, RULES)
        L = math.dist(NODES[a], NODES[b])
        lim = sec.limits(L)
        rows.append({"group": group, "a": a, "b": b, "tube": tube, "L": L, "sec": sec, **lim})
        share[a] += lim["mass"] / 2
        share[b] += lim["mass"] / 2
    node_mass = {n: math.ceil(s * (1 + JOINT) * 20) / 20 for n, s in share.items()}  # round up to 0.05 kg
    dt = 1.0 / PHYSICS_HZ
    for r in rows:
        ma, mb = node_mass[r["a"]], node_mass[r["b"]]
        mu = ma * mb / (ma + mb)                       # reduced mass of the two end nodes
        # 8.9.6: an explicit integrator is stable for k <= mu (2 / dt)^2; keep a 0.25 margin
        r["k_stable"] = STABILITY_SAFETY * mu * (2 / dt) ** 2
        r["k_set"] = min(r["beamSpring"], r["k_stable"])
        r["governs"] = "buckling" if r["buckling_force"] < r["yield_force"] else "yield"
    return rows, node_mass


def print_tables(rows, node_mass):
    """Print the member table (one row per group, as in 8.9.4), node masses and spring caps as Markdown."""
    print("| Member | Tube (mm) | L (m) | Mass (kg) | E·A/L (N/m) | Yield (kN) | Buckling (kN) | beamDeform (kN) | Governs | beamStrength (kN) |")
    print("| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |")
    seen = set()
    for r in rows:
        if r["group"] in seen:
            continue
        seen.add(r["group"])
        n = sum(1 for x in rows if x["group"] == r["group"])
        print(f"| {r['group']}{' (×' + str(n) + ')' if n > 1 else ''} | {r['tube'].replace('x', ' × ')} | {r['L']:.2f} | "
              f"{r['mass']:.2f} | {r['beamSpring']:.3g} | {r['yield_force'] / 1e3:.1f} | {r['buckling_force'] / 1e3:.1f} | "
              f"{r['beamDeform'] / 1e3:.1f} | {r['governs']} | {r['beamStrength'] / 1e3:.1f} |")
    tube_mass = sum(r["mass"] for r in rows)
    print(f"\ntube mass {tube_mass:.2f} kg, node mass incl. {JOINT:.0%} joints {sum(node_mass.values()):.2f} kg")
    print("\n| Node | Mass (kg) |\n| --- | --- |")
    for n in sorted(node_mass):
        print(f"| {n} | {node_mass[n]:.2f} |")
    print("\nspring: E·A/L vs stability cap")
    for r in rows:
        print(f"{r['a']}-{r['b']}: EA/L {r['beamSpring']:.3g}, stable {r['k_stable']:.3g}, set {r['k_set']:.3g}")


def write_submission(rows, node_mass, out):
    """Write the cage as a checkable Submission into `out` (a Path): jbeam, race.pc and fbeam.json.

    beamDeform and beamStrength are written 1 % under their limits so rounding stays inside
    the checker's tolerance.
    """
    nodes = ['["id", "posX", "posY", "posZ"]', '{"group": "fbeam_cage"}']
    for n, (x, y, z) in NODES.items():
        nodes.append(f'["{n}", {x}, {y}, {z}, {{"nodeWeight": {node_mass[n]}}}]')
    beams = ['["id1:", "id2:"]', '{"beamDamp": 150}']
    for r in rows:
        beams.append(f'["{r["a"]}", "{r["b"]}", {{"fbeamSection": "tube {r["tube"]} steel_1018", '
                     f'"beamSpring": {r["k_set"]:.0f}, "beamDeform": {r["beamDeform"] * 0.99:.0f}, '
                     f'"beamStrength": {r["beamStrength"] * 0.99:.0f}}}]')
    part = ('{\n"fbeam_demo_cage": {\n    "information": {"authors": "FBeam", "name": "FBeam demo safety cage"},\n'
            '    "slotType": "main",\n    // generated by examples/cage/cage_calc.py\n    "nodes": [\n        '
            + ",\n        ".join(nodes) + '\n    ],\n    "beams": [\n        ' + ",\n        ".join(beams)
            + "\n    ]\n}\n}\n")
    veh = out / "vehicles" / "fbeam_demo_cage"
    veh.mkdir(parents=True, exist_ok=True)
    (veh / "fbeam_demo_cage.jbeam").write_text(part, encoding="utf-8", newline="\n")
    (veh / "race.pc").write_text(json.dumps({"format": 2, "model": "fbeam_demo_cage",
                                             "parts": {"fbeam_demo_cage": "fbeam_demo_cage"}, "vars": {}}, indent=2),
                                 encoding="utf-8", newline="\n")
    (out / "fbeam.json").write_text(json.dumps({"team": "Demo Team", "car_id": "FB-CAGE", "beamng_version": "TBD",
                                                "pc": "vehicles/fbeam_demo_cage/race.pc",
                                                "chassis_parts": ["fbeam_demo_cage"]}, indent=2), encoding="utf-8",
                                    newline="\n")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    option = sys.argv[sys.argv.index("--option") + 1] if "--option" in sys.argv else "recommended"
    rows, node_mass = design(OPTIONS[option])
    print_tables(rows, node_mass)
    if option == "recommended":
        write_submission(rows, node_mass, ROOT / "examples" / "demo_cage")
