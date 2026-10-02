"""Read a vanilla BeamNG vehicle from the local donor/ folder and report what FBeam needs.

    python tools/donor_report.py [vehicle] [config]      (defaults: covet rally_gravel)

The game files stay local (donor/ is git-ignored and must never be committed); this
tool only derives facts for the editor and the rules, and writes them to
donor/<vehicle>_<config>_report.json:

  * the active part tree of a configuration (slots / slots2, .pc selections, defaults)
  * per part: nodes it defines, their mass, and the nodes it borrows from other parts
    (the donor interface: what a spec part needs from the car it is fitted to)
  * powertrain devices (engine torque curve, rpm limits, gear ratios, final drive)
  * stiffness-to-mass of every beam, k / mu with mu the reduced mass of its two nodes,
    to calibrate the editor's stability cap (regulations 8.9.6)

The committed summary of a run is rules/donor_covet.json (regulations 3, 6). Node
positions are read without nodeOffset / nodeMove; masses in kg, k in N/m, k / mu
in 1/s^2. The 8.9.6 cap for comparison is 0.25 (2 / dt)^2 = 4.0e6 1/s^2 at
dt = 1/2000 s.
"""

import glob
import json
import math
import os
import sys
from collections import defaultdict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from fbeam_checker import jbeam  # noqa: E402

NODE_DEFAULT = 25.0     # kg, BeamNG default nodeWeight (same as submission.NODE_DEFAULT_WEIGHT)
# jbeam sections whose rows reference nodes, to find nodes a part borrows from other parts
LINK_SECTIONS = ("beams", "triangles", "quads", "hydros", "torsionbars", "rails", "slidenodes", "torsionHydros")


def load_parts(vehicle):
    """{part name: part dict} from every .jbeam under donor/vehicles/<vehicle>; _file holds its path."""
    parts = {}
    for f in sorted(glob.glob(os.path.join(ROOT, "donor", "vehicles", vehicle, "**", "*.jbeam"), recursive=True)):
        with open(f, encoding="utf-8-sig") as fh:
            doc = jbeam.parse(fh.read(), f)
        for name, part in doc.items():
            parts[name] = dict(part, _file=os.path.relpath(f, os.path.join(ROOT, "donor")))
    return parts


def slot_entries(part):
    """[(slot_name, allowed_types, default)] from slots (type-based) or slots2 (name-based)."""
    out = []
    if part.get("slots2"):
        for r in jbeam.expand_table(part["slots2"]):
            allow = r.get("allowTypes") or []
            out.append((r.get("name"), allow if isinstance(allow, list) else [allow], r.get("default", "")))
    for r in jbeam.expand_table(part.get("slots") or []):
        out.append((r.get("type"), [r.get("type")], r.get("default", "")))
    return out


def active_tree(parts, pc):
    """Walk from the main part, choosing each slot's part from the .pc or the slot default.

    Returns (main part name, order) with order a depth-first list of
    (depth, slot, part name); parts not in this vehicle's files are listed with a note.
    """
    sel = pc.get("parts", {})
    # main part: the one the .pc "model" names when that is a slotType main part, else the first main part
    mains = [n for n, p in parts.items() if p.get("slotType") == "main"]
    if not mains:
        raise ValueError("no part with slotType 'main' in this vehicle's files")
    main = pc.get("model") if pc.get("model") in mains else mains[0]
    order, seen = [], set()

    def walk(name, slot, depth):
        order.append((depth, slot, name))
        seen.add(name)
        for sname, allow, default in slot_entries(parts.get(name, {})):
            choice = sel.get(sname, default)
            if choice and choice in seen:            # a part reached twice (or a slot cycle): count it once
                order.append((depth + 1, sname, f"{choice}  [already in the tree]"))
            elif choice and choice in parts:
                walk(choice, sname, depth + 1)
            elif choice:
                order.append((depth + 1, sname, f"{choice}  [not in this vehicle's files: BeamNG common]"))
    walk(main, "main", 0)
    return main, order


def variables(parts, active, pc):
    """$variable values: defaults of the active parts, then the .pc "vars"."""
    v = {}
    for n in active:
        for r in jbeam.expand_table(parts[n].get("variables") or []):
            if "name" in r and r.get("default") is not None:
                v[r["name"]] = r["default"]
    v.update(pc.get("vars", {}))
    return v


def part_nodes(part, vars_):
    """{node id: {"pos": [x, y, z] in m, "w": kg}} of one part; nodes that cannot be evaluated are skipped."""
    out = {}
    for r in jbeam.expand_table(part.get("nodes") or []):
        try:
            pos = [jbeam.to_float(r[k], vars_) for k in ("posX", "posY", "posZ")]
            w = jbeam.to_float(r.get("nodeWeight"), vars_, NODE_DEFAULT)
        except Exception:
            continue
        out[r["id"]] = {"pos": pos, "w": w}
    return out


def referenced_nodes(part):
    """Node ids a part's beams, triangles, hydros, ... and pressureWheels refer to (header columns ending in ':')."""
    refs = set()
    for sec in LINK_SECTIONS:
        rows = part.get(sec) or []
        if not isinstance(rows, list) or not rows or not isinstance(rows[0], list):   # rails are a dict of named rails
            continue
        header = [str(h) for h in rows[0]]
        cols = [i for i, h in enumerate(header) if h.endswith(":")]
        for r in rows[1:]:
            if isinstance(r, list):
                for i in cols:
                    if i < len(r) and isinstance(r[i], str):
                        refs.add(r[i])
    for r in jbeam.expand_table(part.get("pressureWheels") or []):
        for k in ("node1", "node2", "nodeArm", "nodeS", "nodeArm:", "node1:", "node2:"):
            if isinstance(r.get(k), str):
                refs.add(r[k])
    return refs


def main():
    """Build the report for argv [vehicle] [config], write it to donor/ and return it."""
    vehicle = sys.argv[1] if len(sys.argv) > 1 else "covet"
    config = sys.argv[2] if len(sys.argv) > 2 else "rally_gravel"
    parts = load_parts(vehicle)
    with open(os.path.join(ROOT, "donor", "vehicles", vehicle, f"{config}.pc"), encoding="utf-8") as fh:
        pc = json.load(fh)
    main_part, order = active_tree(parts, pc)
    active = [n for _, _, n in order if n in parts]
    vars_ = variables(parts, active, pc)
    nodes_by_part = {n: part_nodes(parts[n], vars_) for n in active}
    owner = {}
    for n, nd in nodes_by_part.items():
        for nid in nd:
            owner[nid] = n
    report = {"vehicle": vehicle, "config": config, "main": main_part, "tree": [], "parts": {}, "powertrain": {}}
    for depth, slot, name in order:
        report["tree"].append({"depth": depth, "slot": slot, "part": name})
    total = 0.0
    for n in active:
        nd = nodes_by_part[n]
        mass = sum(x["w"] for x in nd.values())
        total += mass
        borrowed = sorted(r for r in referenced_nodes(parts[n]) if r not in nd)
        report["parts"][n] = {"file": parts[n]["_file"], "slotType": parts[n].get("slotType"), "nodes": len(nd),
                              "mass_kg": round(mass, 3), "borrowed_nodes": borrowed,
                              "borrowed_from": sorted({owner.get(b, "?") for b in borrowed})}
        for dev in jbeam.expand_table(parts[n].get("powertrain") or []):
            report["powertrain"].setdefault(n, []).append({k: dev.get(k) for k in dev if k in ("type", "name", "inputName", "inputIndex")})
        for key in ("mainEngine", "gearbox", "differential", "finalDrive"):
            if key in parts[n]:
                sec = parts[n][key]
                keep = {k: sec.get(k) for k in ("torque", "maxRPM", "idleRPM", "revLimiterRPM", "inertia", "friction",
                                                 "gearRatios", "gearRatio", "diffType", "lsdPreload", "lsdLockCoef", "uiName")
                        if k in sec}
                report["powertrain"].setdefault(n + "." + key, keep)
    report["total_node_mass_kg"] = round(total, 2)
    # stiffness-to-mass of every beam (k / reduced mass) in the active parts
    allnodes = {nid: x for nd in nodes_by_part.values() for nid, x in nd.items()}
    ratios = []
    for n in active:
        for r in jbeam.expand_table(parts[n].get("beams") or []):
            a, b = r.get("id1"), r.get("id2")
            if a not in allnodes or b not in allnodes:
                continue
            try:
                k = jbeam.to_float(r.get("beamSpring"), vars_, 4300000.0)    # BeamNG default beamSpring, N/m
            except Exception:
                continue
            if not k or math.isinf(k):
                continue
            ma, mb = allnodes[a]["w"], allnodes[b]["w"]
            mu = ma * mb / (ma + mb)
            ratios.append((k / mu, k, mu, n, a, b))
    ratios.sort()
    # nearest-rank quantile of k / mu
    q = lambda f: ratios[min(len(ratios) - 1, int(f * len(ratios)))][0] if ratios else None
    report["stiffness"] = {"beams": len(ratios), "k_over_mu_median": q(0.5), "k_over_mu_p90": q(0.9),
                           "k_over_mu_p99": q(0.99), "k_over_mu_max": ratios[-1][0] if ratios else None,
                           "top": [{"k_over_mu": r[0], "k": r[1], "mu": r[2], "part": r[3], "beam": f"{r[4]}-{r[5]}"} for r in ratios[-8:]]}
    out = os.path.join(ROOT, "donor", f"{vehicle}_{config}_report.json")
    with open(out, "w", encoding="utf-8") as fh:
        json.dump(report, fh, indent=1)
    print(f"wrote {out}")
    return report


if __name__ == "__main__":
    main()
