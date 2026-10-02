"""Offline technical checks. Rule numbers refer to FBeam Technical Regulations v1.0.

run_all(submission, rules) is the entry point: it assembles the car once and
runs each check_* function, which reports through add(level, rule, message).
rules is the parsed rules/fbeam_v1.json. Covered offline:

    2.1, 8.3        manifest keys; chassis parts exist and are selected
    2.2.1-2.2.5     parse errors, duplicate or re-declared parts, fixed or
                    near-massless nodes, forbidden sections, values that
                    cannot be evaluated offline
    7.5             only allowed controllers
    4, 5            Spec Part slots (only when enforce_required_parts is on)
    8.1.3, 8.3-8.5  every chassis member beam within its declared section, and
                    every chassis node carrying its share of the beam mass
    8.10            bending beams within the budget of the tubes they brace
    7.1.6           suspension pickups on 3D-braced frame joints, no brackets
    6.1, 6.2, 6.4, 6.6   wheelbase, length, width, overhangs (unladen geometry)
    4.2, 4.5, 9.1   sealed and open tuning variables in the .pc
    10.2            file hash and the offline jbeam digest

To add a check, write check_x(sub, car, rules, add) and call it from run_all;
put any new limit in the rules file, not in the code. Ride height, clearance,
Race Mass, wheel travel, brakes and the dynamic tests need the in-game harness.
"""

import hashlib
import json
import math
from collections import defaultdict
from dataclasses import dataclass

from . import bending as fbend
from . import buckling as fbuck
from . import jbeam
from . import pickups as fpick
from .sections import SectionError, load_materials, parse_section
from .submission import BEAM_DEFAULTS

ERROR, WARN, INFO = "ERROR", "WARN", "INFO"    # any ERROR fails the car; WARN is reported only


@dataclass
class Finding:
    """One result line: level (ERROR / WARN / INFO), the rule number, and a message."""

    level: str
    rule: str
    message: str


def run_all(sub, rules):
    """Run every offline check on a Submission against a rules dict.

    Returns (findings, stats): findings is a list of Finding; stats holds the
    chassis mass totals, pickup and dimension figures, the file hash and the
    jbeam digest, for the report and the passport.
    """
    findings = []
    stats = {}
    add = lambda level, rule, msg: findings.append(Finding(level, rule, msg))

    for err in sub.parse_errors:
        add(ERROR, "2.2.5", f"jbeam parse error: {err}")
    for name, first, second in sub.duplicate_parts:
        add(ERROR, "2.2.3", f"part {name!r} defined twice ({first}, {second})")

    check_manifest(sub, add)
    car = sub.build_car()
    for nid, first, second in car.duplicate_nodes:
        add(WARN, "2.2.3", f"node {nid!r} is defined in {first} and again in {second}; the checker uses the second")
    for msg in car.unresolved:
        add(ERROR, "2.2.5", f"cannot evaluate offline: {msg}")
    check_spec_parts(sub, rules, add)
    check_exploits(sub, car, rules, add)
    stats["chassis"] = check_chassis(sub, car, rules, add)
    stats["pickups"] = check_pickups(sub, car, add)
    stats["dimensions"] = check_dimensions(sub, car, rules, add)
    check_tuning(sub, car, rules, add)
    stats["file_hash"] = sub.file_hash()
    stats["jbeam_digest"] = jbeam_digest(sub)
    return findings, stats


def check_manifest(sub, add):
    """2.1 / 8.3: fbeam.json has its required keys, and every chassis part exists and is selected."""
    m = sub.manifest
    for key in ("team", "car_id", "pc", "chassis_parts"):
        if key not in m:
            add(ERROR, "2.1", f"fbeam.json is missing '{key}'")
    for name in m.get("chassis_parts", []):
        if name not in sub.part_defs:
            add(ERROR, "8.3", f"chassis part {name!r} listed in fbeam.json is not defined in the Submission")
        elif name not in sub.active_part_names():
            add(ERROR, "8.3", f"chassis part {name!r} is not selected in the .pc")


def check_spec_parts(sub, rules, add):
    """2.2.3: no Spec Part re-declared; sections 4 / 5: required slots hold their Spec Parts."""
    spec = rules["spec_parts"]
    for name in sub.part_defs:
        if name in spec["sealed_part_names"]:
            add(ERROR, "2.2.3", f"Submission re-declares Spec Part {name!r}")
    selected = sub.pc.get("parts", {})
    if not spec.get("enforce_required_parts", True):     # names verified, enforced from the Spec Kit on
        return
    for slot, part in spec["required_parts"].items():
        if part is None:
            continue
        if selected.get(slot) != part:
            add(ERROR, "4 / 5", f"slot {slot!r} must hold Spec Part {part!r}, .pc has {selected.get(slot)!r}")


def check_exploits(sub, car, rules, add):
    """2.2.1 / 2.2.2 / 7.5: forbidden jbeam sections, controllers off the allowed list, fixed or too-light nodes."""
    ex = rules["exploits"]
    for name, part in car.parts.items():
        for section in ex["forbidden_sections"]:
            if part.get(section):
                add(ERROR, "2.2.2", f"part {name!r} uses forbidden section '{section}'")
        for rec in jbeam.expand_table(part.get("controller", [])):
            fn = rec.get("fileName")
            if fn and fn not in ex["allowed_controllers"]:
                add(ERROR, "7.5", f"part {name!r} loads controller {fn!r}, not on the allowed list")
    for node in car.nodes.values():
        if node.props.get("fixed") is True:
            add(ERROR, "2.2.1", f"node {node.id} ({node.part}) is fixed in space")
        if node.weight < ex["min_node_weight_kg"]:
            add(ERROR, "2.2.1", f"node {node.id} ({node.part}) weighs {node.weight:g} kg, under {ex['min_node_weight_kg']} kg")


def check_chassis(sub, car, rules, add):
    """Section 8: each chassis beam within its declared section; nodes carry the mass.

    Only beams of the parts listed in fbeam.json "chassis_parts" are checked,
    by fbeamRole:
      member (default)            8.3 limits of its fbeamSection at its rest length,
                                  within rules["chassis"]["tolerance"]; its mass
                                  rho*A*L goes half to each end node
      bending                     weightless, checked by check_bending (8.10)
      spring / mount / upright    not tubes, no mass share, must be finite (8.1.3)
    Each node must then weigh at least its share times (1 + joint_allowance)
    (8.3: +15 % for joints and welds); more than max_excess_per_node_kg above
    that must be a declared ballast or unsprung node (6.10).

    Returns the stats dict printed by the CLI (beam counts, masses in kg).
    """
    cr = rules["chassis"]
    tol = cr["tolerance"]
    materials = load_materials(rules)
    chassis = set(sub.manifest.get("chassis_parts", []))
    ballast_nodes = set(sub.manifest.get("ballast_nodes", [])) | set(sub.manifest.get("unsprung_nodes", []))
    share = defaultdict(float)
    n_beams = 0
    beam_mass = 0.0
    section_cache = {}
    members, bends = [], []

    # 8.9.7: a tube split into segments buckles over the whole unbraced run. Frame tubes only
    # (a suspension link ends on a suspension node and buckles over its own length).
    suspension = set(sub.manifest.get("suspension_nodes", []))
    frame_tubes = [(i, b.a, b.b) for i, b in enumerate(car.beams)
                   if b.part in chassis and str(b.props.get("fbeamRole", "member")).lower() == "member"
                   and b.a in car.nodes and b.b in car.nodes and b.a != b.b
                   and b.a not in suspension and b.b not in suspension]
    l_buckle = fbuck.buckling_lengths({n: nd.pos for n, nd in car.nodes.items()}, frame_tubes)

    for index, beam in enumerate(car.beams):
        if beam.part not in chassis:
            continue
        n_beams += 1
        where = f"beam {beam.a}-{beam.b} ({beam.part})"
        role = str(beam.props.get("fbeamRole", "member")).lower()
        if role == "bending":
            bends.append(beam)                       # weightless; checked against its budget below (8.10)
            continue
        if role in ("spring", "mount", "upright"):
            # suspension spring / damper, rigid upright: not tubes, no mass share, but finite (8.1.3).
            # 'mount' (pickup bracket) passes here but is an error in check_pickups (7.1.6).
            try:
                vals = {k: jbeam.to_float(beam.props.get(k), car.variables, BEAM_DEFAULTS[k]) for k in BEAM_DEFAULTS}
            except jbeam.UnresolvedValue as exc:
                add(ERROR, "2.2.5", f"{where}: cannot evaluate {exc}")
                continue
            if math.isinf(vals["beamStrength"]) or math.isinf(vals["beamDeform"]):
                add(ERROR, "8.1.3", f"{where}: {role} beams must break and deform at a finite load")
            continue
        btype = str(beam.props.get("beamType", "|NORMAL")).lstrip("|").upper()
        if btype not in cr["allowed_beam_types"]:
            add(ERROR, "8.3", f"{where}: beamType {btype} not allowed in the chassis")
            continue
        sec_text = beam.props.get("fbeamSection")
        if not sec_text:
            add(ERROR, "8.3", f"{where}: no fbeamSection declared")
            continue
        if sec_text not in section_cache:
            try:
                section_cache[sec_text] = parse_section(sec_text, materials, rules)
            except SectionError as exc:
                section_cache[sec_text] = exc
        sec = section_cache[sec_text]
        if isinstance(sec, SectionError):
            add(ERROR, "8.4", f"{where}: {sec}")
            continue
        na, nb = car.nodes.get(beam.a), car.nodes.get(beam.b)
        if na is None or nb is None:
            add(ERROR, "8.3", f"{where}: node not found in active parts")
            continue
        L = math.dist(na.pos, nb.pos)
        if L < 1e-4:
            add(ERROR, "2.2.1", f"{where}: zero-length beam")
            continue
        Lb = max(L, l_buckle.get(index, L))          # 8.9.7: the whole unbraced run of a split tube
        lim = sec.limits(L, Lb)
        try:
            vals = {k: jbeam.to_float(beam.props.get(k), car.variables, BEAM_DEFAULTS[k]) for k in BEAM_DEFAULTS}
        except jbeam.UnresolvedValue as exc:
            add(ERROR, "2.2.5", f"{where}: cannot evaluate {exc}")
            continue
        for key, rule in (("beamSpring", "8.3"), ("beamDeform", "8.3"), ("beamStrength", "8.1.3/8.3")):
            if vals[key] > lim[key] * (1 + tol):
                shown = "FLT_MAX (unbreakable)" if math.isinf(vals[key]) else f"{vals[key]:.4g}"
                run = f", buckling over {Lb:.3f} m (8.9.7)" if Lb > L + 1e-6 else ""
                add(ERROR, rule, f"{where}: {key} {shown} > {lim[key]:.4g} allowed for '{sec_text}' at L={L:.3f} m{run}")
        # damping ratio of the two-node oscillator: zeta = c / (2 sqrt(k mu)), mu = reduced mass (8.3, warning only)
        m_red = na.weight * nb.weight / (na.weight + nb.weight) if na.weight > 0 and nb.weight > 0 else 0.0
        zeta = vals["beamDamp"] / (2 * math.sqrt(vals["beamSpring"] * m_red)) if vals["beamSpring"] > 0 and m_red > 0 else 0
        if zeta > cr["damping_zeta_max"]:
            add(WARN, "8.3", f"{where}: damping ratio {zeta:.2f} > {cr['damping_zeta_max']}")
        share[beam.a] += lim["mass"] / 2
        share[beam.b] += lim["mass"] / 2
        beam_mass += lim["mass"]
        members.append({"id": str(beam.props.get("fbeamMember") or f"b{n_beams}"), "a": beam.a, "b": beam.b,
                        "sec": sec, "text": sec_text, "k": vals["beamSpring"], "EA_L": lim["beamSpring"]})

    n_bend = check_bending(car, members, bends, tol, add)

    joint = cr["joint_allowance"]             # 8.3: +15 % for joints, gussets and welds
    required_total = actual_total = excess_total = 0.0
    for node_id, s in sorted(share.items()):
        node = car.nodes[node_id]
        req = s * (1 + joint)
        required_total += req
        actual_total += node.weight
        if node.part not in chassis:
            add(WARN, "8.3", f"node {node_id} carries {req:.2f} kg of chassis beams but belongs to part {node.part!r}")
        if node.weight < req * (1 - tol):      # tolerance in the team's favour, as for the beam limits
            add(ERROR, "8.3", f"node {node_id}: {node.weight:.3f} kg < {req:.3f} kg required by its beams")
        excess = node.weight - req
        if excess > 0:
            excess_total += excess
            if excess > cr["max_excess_per_node_kg"] and node_id not in ballast_nodes:
                add(ERROR, "8.3 / 6.10", f"node {node_id}: {excess:.2f} kg above its beam share; declare it as a ballast node or remove it")
    return {"beams": n_beams, "bending_beams": n_bend, "beam_mass_kg": round(beam_mass, 3), "required_node_mass_kg": round(required_total, 3),
            "actual_node_mass_kg": round(actual_total, 3), "excess_as_ballast_kg": round(excess_total, 3)}


def check_bending(car, members, bends, tol, add):
    """8.10: bending beams only reproduce the bending of the welded tubes they name.

    Every budget is recomputed here from the member tubes (shared formula in
    fbeam_checker.bending); declared values are never trusted. members are the
    member-beam dicts collected by check_chassis (id, a, b, sec, text, k = the
    jbeam beamSpring, EA_L = E*A/L); only round tubes give a budget. bends are
    the Beams with fbeamRole "bending"; tol is the chassis tolerance.
    Returns the number of bending beams.
    """
    if not bends:
        return 0
    nodes = {n: car.nodes[n].pos for m in members for n in (m["a"], m["b"])}

    def section_of(m):
        # D and t back from the "tube DxT material" text, in m
        D, t = (float(x) / 1000 for x in m["text"].split()[1].lower().split("x"))
        return m["sec"].A, m["sec"].I, D, t

    def material_of(m):
        return m["sec"].material.E, m["sec"].material.sy

    budgets = fbend.budgets(nodes, [m for m in members if m["sec"].kind == "tube"], section_of, material_of)
    byid = defaultdict(list)                     # a split tube: several segments share one fbeamMember id
    for m in members:
        byid[m["id"]].append(m)
    for beam in bends:
        where = f"bending beam {beam.a}-{beam.b} ({beam.part})"
        e = budgets.get(tuple(sorted((beam.a, beam.b))))
        if e is None:
            add(ERROR, "8.10.0", f"{where}: its ends are not the far ends of two member tubes welded at one joint")
            continue
        named = [x.strip() for x in str(beam.props.get("fbeamMember", "")).split(",") if x.strip()]
        if not named or not set(named) <= set(e["members"]):
            add(ERROR, "8.10.2", f"{where}: fbeamMember must name the tubes it braces ({', '.join(e['members'])})")
        try:
            vals = {k: jbeam.to_float(beam.props.get(k), car.variables, BEAM_DEFAULTS[k]) for k in BEAM_DEFAULTS}
        except jbeam.UnresolvedValue as exc:
            add(ERROR, "2.2.5", f"{where}: cannot evaluate {exc}")
            continue
        scale = min(m["k"] / m["EA_L"] for i in e["members"] for m in byid[i])   # 8.10.5: same scale as its tubes
        # beamStrength <= STRENGTH_FACTOR (1.2) * beamDeform keeps the brace finite (8.10.3)
        limits = {"beamSpring": e["k"] * scale, "beamDeform": e["F"], "beamStrength": fbend.STRENGTH_FACTOR * e["F"]}
        for key, rule in (("beamSpring", "8.10.3/8.10.5"), ("beamDeform", "8.10.3"), ("beamStrength", "8.10.3")):
            if vals[key] > limits[key] * (1 + tol):
                shown = "FLT_MAX (unbreakable)" if math.isinf(vals[key]) else f"{vals[key]:.4g}"
                add(ERROR, rule, f"{where}: {key} {shown} > {limits[key]:.4g} allowed by the bending of {', '.join(e['members'])}")
    return len(bends)


def check_pickups(sub, car, add):
    """7.1.6: the suspension loads the frame only at joints braced in three dimensions; no brackets.

    Which part a beam or node is written in does not matter (splitting the frame over several
    parts, or writing suspension beams in the frame's part, changes nothing):
      suspension nodes  the nodes that move with the wheels, declared in fbeam.json
                        "suspension_nodes" (uprights, hubs, axle bodies, arm-mounted damper
                        bottoms, Watt's rockers)
      frame nodes       every other node of the chassis parts
      frame tubes       member beams (fbeamSection) with both ends on frame nodes
      loaded joint      a frame node at the end of any beam whose other end is a suspension node
                        (arm, link, tie rod, spring, damper; bending beams excluded)
    Each loaded joint needs at least three frame tubes, not all in one plane: the best
    |a . (b x c)| of their unit directions must reach pickups.MIN_SPAN (0.10).
    Errors as well:
      * any beam with fbeamRole "mount" (a weightless bracket);
      * a spring or upright beam with both ends on frame nodes. Those roles carry no section,
        mass or budget check, so between two frame joints they would be an invisible brace; it
        also catches suspension nodes left out of "suspension_nodes".
    Returns {"loaded_joints": n}.
    """
    chassis = set(sub.manifest.get("chassis_parts", []))
    suspension = set(sub.manifest.get("suspension_nodes", []))
    frame_nodes = {n for n, nd in car.nodes.items() if nd.part in chassis and n not in suspension}
    ends = defaultdict(list)
    loaded = defaultdict(set)
    for b in car.beams:
        role = str(b.props.get("fbeamRole", "member")).lower()
        where = f"beam {b.a}-{b.b} ({b.part})"
        if role == "mount":
            add(ERROR, "7.1.6", f"{where}: bracket ('mount') beams are not allowed; put the pickup on a frame joint")
        if role == "bending" or b.a not in car.nodes or b.b not in car.nodes:
            continue                                 # bending beams are checked in 8.10; spec-part nodes are not ours
        fa, fb = b.a in frame_nodes, b.b in frame_nodes
        if fa and fb:
            if role == "member":
                ends[b.a].append(car.nodes[b.b].pos)
                ends[b.b].append(car.nodes[b.a].pos)
            elif role in ("spring", "upright"):
                add(ERROR, "7.1.6", f"{where}: a {role} beam must join the frame to the suspension, but both ends "
                                    "are frame joints (list nodes that move with the wheels in fbeam.json suspension_nodes)")
        elif fa != fb:
            frame_end, other = (b.a, b.b) if fa else (b.b, b.a)
            if other in suspension:
                loaded[frame_end].add(f"{role} {other}")
    for n in sorted(loaded):
        count, span, ok = fpick.joint_bracing(n, car.nodes[n].pos, ends[n])
        if not ok:
            add(ERROR, "7.1.6", f"frame joint {n} carries the suspension ({', '.join(sorted(loaded[n])[:3])}) but its "
                                f"{count} frame tubes do not brace it in three dimensions (span {span:.2f} < {fpick.MIN_SPAN})")
    return {"loaded_joints": len(loaded)}


ARCH_HALF_LENGTH = 0.40   # m either side of an axle line counted as wheel arch (6.3 / 6.4): a 594 mm tyre plus clearance


def check_dimensions(sub, car, rules, add):
    """Section 6, from unladen jbeam geometry. Ride-height checks need the in-game harness.

    Uses every node of the active Team Parts (Spec Part wheels are not loaded).
    BeamNG axes: x lateral, y longitudinal with the front at -y, z up. Checks
    length (6.2), overall x span against the width over arches (6.4), and,
    when fbeam.json names two front and two rear hub nodes, the wheelbase
    (6.1, 2,450-2,550 mm around the Covet's 2,500 mm), overhangs (6.6) and the
    width away from the arches against the body width (6.3). Roof height (6.5)
    is the body shell's, which is not jbeam: it is checked in-game. Returns the
    measured figures in mm.
    """
    d = rules["dimensions"]
    if not car.nodes:
        return {}
    xs, ys, zs = zip(*(n.pos for n in car.nodes.values()))
    width, length, height = (max(v) - min(v) for v in (xs, ys, zs))
    out = {"length_mm": round(length * 1000), "width_mm": round(width * 1000), "height_mm": round(height * 1000)}
    if length * 1000 > d["max_length_mm"]:
        add(ERROR, "6.2", f"length {length * 1000:.0f} mm > {d['max_length_mm']} mm")
    if width * 1000 > d["max_width_over_arches_mm"]:
        add(ERROR, "6.4", f"width {width * 1000:.0f} mm > {d['max_width_over_arches_mm']} mm")
    axles = {}
    for key in ("front_axle_nodes", "rear_axle_nodes"):
        ids = sub.manifest.get(key, [])
        pts = [car.nodes[i].pos for i in ids if i in car.nodes]
        if len(pts) != 2:
            add(WARN, "6.1", f"fbeam.json '{key}' should name two nodes (left and right hub); wheelbase not checked")
            return out
        axles[key] = pts
    fy = sum(p[1] for p in axles["front_axle_nodes"]) / 2
    ry = sum(p[1] for p in axles["rear_axle_nodes"]) / 2
    wb = abs(fy - ry) * 1000
    out["wheelbase_mm"] = round(wb)
    lo, hi = d["wheelbase_mm"]
    if not lo <= wb <= hi:
        add(ERROR, "6.1", f"wheelbase {wb:.0f} mm outside {lo}-{hi} mm")
    front_y = min(ys)
    rear_y = max(ys)
    out["front_overhang_mm"] = round(abs(fy - front_y) * 1000)
    out["rear_overhang_mm"] = round(abs(rear_y - ry) * 1000)
    if out["front_overhang_mm"] > d["max_front_overhang_mm"]:
        add(ERROR, "6.6", f"front overhang {out['front_overhang_mm']} mm > {d['max_front_overhang_mm']} mm")
    if out["rear_overhang_mm"] > d["max_rear_overhang_mm"]:
        add(ERROR, "6.6", f"rear overhang {out['rear_overhang_mm']} mm > {d['max_rear_overhang_mm']} mm")
    # 6.3: away from the wheel arches (ARCH_HALF_LENGTH either side of each axle line) the car must fit
    # the body width; over the arches the wider 6.4 limit above applies
    body_x = [p[0] for p in (n.pos for n in car.nodes.values())
              if abs(p[1] - fy) > ARCH_HALF_LENGTH and abs(p[1] - ry) > ARCH_HALF_LENGTH]
    if body_x:
        bw = (max(body_x) - min(body_x)) * 1000
        out["body_width_mm"] = round(bw)
        if bw > d["max_body_width_mm"]:
            add(ERROR, "6.3", f"width {bw:.0f} mm away from the wheel arches > {d['max_body_width_mm']} mm")
    return out


def check_tuning(sub, car, rules, add):
    """9.1 / 10.3: every .pc variable within its allowed range; 4.2 / 4.5: sealed values exact.

    Spec Part variables are open only if listed in rules["tuning"]["open_spec_variables"]
    ([min, max]); sealed_spec_values must be set to exactly their value (required
    only once enforce_required_parts is on, because the donor defaults differ).
    Team-part variables must declare min and max in their part.
    """
    open_vars = rules["tuning"]["open_spec_variables"]
    sealed = rules["tuning"].get("sealed_spec_values", {})
    pc_vars = sub.pc.get("vars", {})
    if rules["spec_parts"].get("enforce_required_parts", True):
        for name in sealed:
            if name not in pc_vars:
                add(ERROR, "4.2 / 4.5", f"sealed variable {name} must be set to {sealed[name]} (the donor default differs)")
    for name, value in pc_vars.items():
        if name in sealed:
            if not isinstance(value, (int, float)) or abs(value - sealed[name]) > 1e-9:
                add(ERROR, "4.2 / 4.5", f"sealed variable {name} = {value!r}, must be {sealed[name]}")
            continue
        if name in open_vars:
            lo, hi = open_vars[name]
        elif name in car.variable_defs:
            d = car.variable_defs[name]
            lo, hi = d["min"], d["max"]
            if lo is None or hi is None:
                add(ERROR, "9.1", f"team variable {name} ({d['part']}) must declare min and max")
                continue
            try:
                lo, hi = float(lo), float(hi)
            except (TypeError, ValueError):
                add(ERROR, "9.1", f"team variable {name} ({d['part']}): min and max must be numbers")
                continue
        else:
            add(ERROR, "9.1", f"variable {name} is sealed or unknown; it may not be set in the .pc")
            continue
        try:
            v = float(value)
        except (TypeError, ValueError):
            add(ERROR, "9.1", f"variable {name} = {value!r} is not a number")
            continue
        if not lo <= v <= hi:
            add(ERROR, "9.1", f"variable {name} = {v:g} outside {lo:g}-{hi:g}")


def jbeam_digest(sub):
    """Offline stand-in for the physics hash (10.2, 10.3).

    Canonical hash of the active Team Parts with every variable at its default,
    so changing an open tuning variable keeps the digest. The real physics hash
    is taken in-game from the fully assembled vehicle, Spec Parts included.
    Covers the .pc part selections, node positions (m) and weights (kg) rounded
    to 1e-6, every beam row, and the raw JSON of each active part ("sections"
    holds whole parts, so any change to an active part changes the digest).
    """
    car = sub.build_car(use_defaults=True)
    canon = {
        "parts": sorted((k, v) for k, v in sub.pc.get("parts", {}).items() if v),
        "nodes": sorted((n.id, [round(c, 6) for c in n.pos], round(n.weight, 6)) for n in car.nodes.values()),
        "beams": sorted((b.a, b.b, b.part, json.dumps(b.props, sort_keys=True, default=str)) for b in car.beams),
        "sections": {name: json.dumps(part, sort_keys=True, default=str)
                     for name, part in sorted(car.parts.items())},
    }
    return "sha256:" + hashlib.sha256(json.dumps(canon, sort_keys=True).encode()).hexdigest()
