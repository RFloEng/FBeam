"""Full-car jbeam export: suspension part, hubs, refNodes and info.json (UNVERIFIED IN-GAME).

Pure Python, standard library only. FBeam axes: X left, Y rear, Z up, metres; N, N/m, kg.
Called by fbeam_editor_core._write_submission with the model (schema in fbeam_editor_core),
a suspension() result and the chassis design rows; it returns node and beam dicts that the
core writes as jbeam text. Rules: 7.1.3 (arms are tubes under the section 8 mass-strength
rule), 7.1.6 (pickups on frame joints), 5 and 5.4 (spec wheel placed by the hub's nodeOffset).

What is generated for each independent corner (left, and mirrored right):
  pickups   one node per inner pickup; a chassis joint when the pickup is attached to one,
            otherwise a bracket node tied by three weightless "mount" beams to the nearest joints
            (exported so the car still spawns, but the checker fails it under 7.1.6)
  links     every arm, rod, tie rod and toe link of the layout (fbeam_suspension.TOPOLOGY) as
            member tubes (fbeamSection, rule 7.1.3); trailing arms (plain, semi-trailing, with
            lateral links) as tubes from each pivot to every upright node, a twist beam as two
            cross tubes between the hubs; a MacPherson strut only as its coilover beam
  upright   its outer points and two hub nodes on the spin axis, braced as one rigid body
  spring    one coilover beam between the damper's two points:
            beamSpring = coil rate, beamDamp = bump damping (fbeamRole "spring"); rebound
            damping and the anti-roll bar are not exported yet
Solid axles and de Dion axles (TOPOLOGY "axle": True) are one rigid body carrying both hubs,
with the links on both sides and single links (Panhard rod, Watt's linkage) once (_build_axle).
wheels    the spec wheel (Covet rally rim 14 x 5.5 + 165/70R14, radius 0.297 m, regulations 5)
            mounted as the Covet does: a wheel_F_4 / wheel_R_4 slot whose nodeOffset puts the
            rim's axle nodes on the wheel centre, then a wheeldata part with the pressureWheels
            rows naming FBeam's upright nodes (build_wheels)
Node names: <f|r><point>_L / _R (e.g. flbj_L, fhubi_R); single axle points have no suffix.
Not generated here: powertrain (spec engine part) and the steering hydro, which attach through
the donor interface (docs/editor-plan.md section 4).
"""

import json
import math
from pathlib import Path

from fbeam_checker.pickups import mirror_name

PART = "fbeam_editor_suspension"
ARM_TUBE_DEFAULT = "25x2"       # arm and link tube (mm) when the corner has no "arm_tube"
HUB_HALF_WIDTH = 0.06           # m: hub axis nodes either side of the wheel centre
MOUNT_STIFF_FACTOR = 1.0        # mount beams: stiffness of the host tubes' stability-capped springs
UPRIGHT_STIFFNESS = 3.0e6       # N/m: rigid upright and hub beams (stability-limited like the chassis)
UPRIGHT_STRENGTH = 150000.0     # N: beamDeform of upright, hub, wheel and mount beams (beamStrength 1.2 x)
WHEELDATA_PART = "fbeam_editor_wheeldata"
# corner -> (Covet hub slot, spec rim part, rim axle node prefix: <pre>l inner, <pre>ll outer on the left)
SPEC_RIM = {"front": ("wheel_F_4", "wheel_17a_14x5.5_F", "fw1"), "rear": ("wheel_R_4", "wheel_17a_14x5.5_R", "rw1")}
RIM_AXLE_X = (0.31, 0.57)       # m: the rim part's axle nodes (inner, outer), local to the slot nodeOffset
JOINT_ALLOWANCE = json.loads((Path(__file__).resolve().parent.parent / "rules" / "fbeam_v1.json")
                             .read_text(encoding="utf-8"))["chassis"]["joint_allowance"]   # 8.3: +15 % joints and welds
RIM_AXLE_NODE_MASS = 4.5        # kg: each of the spec rim's axle nodes (wheel_17a_14x5.5 nodeWeight)
SPEC_WHEEL_MASS = 25.2          # kg: rim axle nodes + generated hub and tread nodes (rules/fbeam_v1.json spec_wheel)


def _mirror(p):
    """A point mirrored to the right side (x negated)."""
    return [-p[0], p[1], p[2]]


def _side(name, s):
    """Node name for side s (+1 left, -1 right): name_L or name_R."""
    return f"{name}_{'L' if s > 0 else 'R'}"


# upright points BeamNG uses as the steering axis (top, bottom) for the steered layouts; for the
# multi-link this is the line through two outer ball joints, not its virtual steering axis
STEER_PAIR = {"macpherson": ("SB", "LBJ"), "double_wishbone": ("UBJ", "LBJ"), "multilink": ("UFO", "LFO")}


def build_suspension(model, sus_result, node_mass, member_rows, section_limits):
    """Nodes and beams of the suspension part, for any layout in fbeam_suspension.TOPOLOGY.

    sus_result is a fbeam_editor_core.suspension() result (resolved "points" and "mounts" per
    corner); node_mass and member_rows are the chassis joints and tubes from _design (only
    joints in node_mass are in the chassis jbeam). section_limits(tube, L) -> limits dict
    (fbeam_checker.sections Section.limits) for arm tubes.

    Returns (nodes, beams, notes, chassis_extra):
      nodes          {name: {"pos" [x, y, z] m, "mass" kg, "corner", "side" (+1 / -1, 0 on the
                     centre line), "upright" (True for unsprung nodes; the wheel data attaches there)}}
      beams          [{"a", "b", "role": "member" | "spring" | "upright" | "mount", "k" N/m,
                     "F" N, plus "tube", "Fu" N, "mass" kg for members, "damp" N s/m for springs}]
      notes          [str] for the editor
      chassis_extra  {chassis joint: kg} arm tube mass that the chassis part must carry
    Unsprung mass per corner (model["unsprung_mass_per_corner"] less the spec wheel's own
    nodes, at least 5 kg) is spread evenly over the upright nodes.
    """
    import fbeam_suspension as fsus
    nodes, beams, notes = {}, [], []
    chassis = {n: model["nodes"][n] for n in node_mass}
    # arm, mount and twist-beam springs are capped at the softest chassis spring (2 MN/m with no tubes)
    k_ref = min((r["k_set"] for r in member_rows), default=2.0e6) * MOUNT_STIFF_FACTOR
    unsprung = model.get("unsprung_mass_per_corner") or 0.0
    for cname, res in (sus_result.get("corners") or {}).items():
        spec = model["suspension"][cname]
        lay = fsus.TOPOLOGY[spec["type"]]
        pts = res["points"]
        arm_tube = spec.get("arm_tube", ARM_TUBE_DEFAULT)
        tag = "f" if cname == "front" else "r"
        if lay.get("axle"):
            _build_axle(model, cname, spec, lay, res, chassis, k_ref, unsprung, section_limits, nodes, beams, notes)
            continue
        for s in (1, -1):
            P = {k: (v if s > 0 else _mirror(v)) for k, v in pts.items()}
            nm = {}

            def node(key, pos, mass, upright=False):
                n = _side(f"{tag}{key.lower()}", s)
                nodes[n] = {"pos": [round(c, 5) for c in pos], "mass": mass, "corner": cname, "side": s,
                            "upright": upright}
                nm[key] = n
                return n

            # inner pickups: chassis joints when attached, else 0.5 kg bracket nodes with mount
            # beams to the three nearest joints (7.1.6 fails)
            for key in res["mounts"]:
                k = key["point"]
                ref = spec["points"][k]
                if isinstance(ref, dict) and (ref["node"] if s > 0 else _mirror_name(ref["node"])) in chassis:
                    nm[k] = ref["node"] if s > 0 else _mirror_name(ref["node"])
                    continue                           # (a joint with no tubes is not in the jbeam: bracket it)
                n = node(k, P[k], 0.5)
                near = sorted(chassis, key=lambda c: math.dist(chassis[c], P[k]))[:3]
                for c in near:
                    beams.append({"a": n, "b": c, "role": "mount", "k": k_ref, "F": UPRIGHT_STRENGTH})
            # the upright: its outer points, two hub nodes on the spin axis and the damper bottom when it
            # carries it, braced as one rigid body
            top, bottom, carrier = lay["damper"]
            up_keys = list(dict.fromkeys(o for _, _, o in lay["links"] if o))
            if carrier == "upright" and bottom not in up_keys:
                up_keys.append(bottom)
            spin = [1.0, 0.0, 0.0] if s > 0 else [-1.0, 0.0, 0.0]
            wc = P["WC"]
            per = max(unsprung - SPEC_WHEEL_MASS, 5.0) / (len(up_keys) + 2)   # the spec wheel brings its own nodes
            for k in up_keys:
                node(k, P[k], per, upright=True)
            h_in = node("HUBI", [wc[i] - spin[i] * HUB_HALF_WIDTH for i in range(3)], per, upright=True)
            h_out = node("HUBO", [wc[i] + spin[i] * HUB_HALF_WIDTH for i in range(3)], per, upright=True)
            up_nodes = [nm[k] for k in up_keys] + [h_in, h_out]
            for i in range(len(up_nodes)):
                for j in range(i + 1, len(up_nodes)):
                    beams.append({"a": up_nodes[i], "b": up_nodes[j], "role": "upright", "k": UPRIGHT_STIFFNESS,
                                  "F": UPRIGHT_STRENGTH})
            # links: member tubes (7.1.3); each link is (kind, inner pickups, outer point) as in TOPOLOGY
            body = ["HUBI", "HUBO"] + up_keys
            tubes = []
            for kind, ins, out in lay["links"]:
                if kind in ("arm", "rod"):
                    tubes += [(i, out) for i in ins]
                elif kind in ("pivot", "ball", "pivot_mirror"):   # the arm is rigid with the hub: tubes to each pivot
                    tubes += [(i, k) for i in ins for k in body]
            if carrier != "upright":       # damper bottom on an arm (double wishbone): carrier is the link index
                kind, ins, out = lay["links"][carrier]
                node(bottom, P[bottom], 0.5)
                tubes += [(i, bottom) for i in ins] + [(bottom, out)]
            for a, b in dict.fromkeys(tubes):
                if nm.get(a) == nm.get(b):
                    continue
                pa, pb = _pos(nodes, chassis, nm[a]), _pos(nodes, chassis, nm[b])
                lim = section_limits(arm_tube, math.dist(pa, pb))
                beams.append({"a": nm[a], "b": nm[b], "role": "member", "tube": arm_tube, "k": min(lim["beamSpring"], k_ref),
                              "F": lim["beamDeform"], "Fu": lim["beamStrength"], "mass": lim["mass"]})
            beams.append({"a": nm[top], "b": nm[bottom], "role": "spring",
                          "k": spec.get("spring_N_per_mm", 50) * 1000, "damp": spec.get("damper_bump_Ns_per_m", 3500)})
        kind0 = lay["links"][0][0]
        if kind0 == "strut":
            notes.append(f"{cname}: the strut is exported as its coilover beam; its sideways guidance is unverified in-game")
        if kind0 == "pivot_mirror":                               # the twist beam itself: hub to hub
            tube = spec.get("beam_tube", "50x2.5")
            for k in ("hubi", "hubo"):
                a, b = f"{tag}{k}_L", f"{tag}{k}_R"
                lim = section_limits(tube, math.dist(nodes[a]["pos"], nodes[b]["pos"]))
                beams.append({"a": a, "b": b, "role": "member", "tube": tube, "k": min(lim["beamSpring"], k_ref),
                              "F": lim["beamDeform"], "Fu": lim["beamStrength"], "mass": lim["mass"]})
            notes.append(f"{cname}: twist beam exported as two cross tubes between the hubs; its roll stiffness is not tuned")
    # arm tube mass goes to their end nodes (half each, + the 15 % joint allowance of
    # rules chassis.joint_allowance, hard-coded here), like the chassis rule;
    # ends that are chassis joints are returned separately so the chassis part can carry them
    chassis_extra = {}
    for b in beams:
        if b["role"] == "member":
            for n in (b["a"], b["b"]):
                share = b["mass"] / 2 * (1 + JOINT_ALLOWANCE)
                if n in nodes:
                    nodes[n]["mass"] += share
                else:
                    chassis_extra[n] = chassis_extra.get(n, 0.0) + share
    notes.append("suspension part generated from the editor model; not yet verified in BeamNG")
    return nodes, beams, notes, chassis_extra


def _build_axle(model, cname, spec, lay, res, chassis, k_ref, unsprung, section_limits, nodes, beams, notes):
    """A solid or de Dion axle as one rigid body carrying both hubs; links on both sides, single
    links (Panhard rod, Watt's linkage) once. Arguments as in build_suspension; appends to the
    shared nodes, beams and notes in place. The unsprung mass of both corners is spread evenly
    over the axle body nodes."""
    pts = res["points"]
    single = set(lay.get("single", []))
    arm_tube = spec.get("arm_tube", ARM_TUBE_DEFAULT)
    tag = "f" if cname == "front" else "r"
    nm = {}                                   # (key, side) -> node name; side 0 for single points

    def node(key, side, pos, mass, upright=False):
        n = f"{tag}{key.lower()}" if side == 0 else _side(f"{tag}{key.lower()}", side)
        nodes[n] = {"pos": [round(c, 5) for c in pos], "mass": mass, "corner": cname,
                    "side": side if side else (1 if pos[0] > 0 else -1 if pos[0] < 0 else 0), "upright": upright}
        nm[(key, side)] = n
        return n

    def pos(key, side):
        """Point for side +1 or 0 (as given) or -1 (mirrored)."""
        p = pts[key]
        return p if side >= 0 else _mirror(p)

    # pickups: joints when attached, else bracket nodes (7.1.6 fails)
    for m in res["mounts"]:
        k = m["point"]
        sides = (0,) if k in single else (1, -1)
        for s in sides:
            ref = spec["points"][k]
            if isinstance(ref, dict) and (ref["node"] if s >= 0 else _mirror_name(ref["node"])) in chassis:
                nm[(k, s)] = ref["node"] if s >= 0 else _mirror_name(ref["node"])
                continue
            n = node(k, s, pos(k, s), 0.5)
            near = sorted(chassis, key=lambda c: math.dist(chassis[c], nodes[n]["pos"]))[:3]
            for c in near:
                beams.append({"a": n, "b": c, "role": "mount", "k": k_ref, "F": UPRIGHT_STRENGTH})
    # the axle body: outer points on both sides, single axle points once, two hub nodes per side
    outer = list(dict.fromkeys(o for _, _, o in lay["links"] if o))
    if lay["damper"][1] not in outer:
        outer.append(lay["damper"][1])
    body_keys = [(k, s) for k in outer if k not in single for s in (1, -1)] + [(k, 0) for k in outer if k in single]
    per = 2 * max(unsprung - SPEC_WHEEL_MASS, 5.0) / (len(body_keys) + 4)
    for k, s in body_keys:
        node(k, s, pos(k, s), per, upright=True)
    for s in (1, -1):
        wc = pos("WC", s)
        for key, d in (("HUBI", -HUB_HALF_WIDTH), ("HUBO", HUB_HALF_WIDTH)):
            node(key, s, [wc[0] + s * d, wc[1], wc[2]], per, upright=True)
    watt = next((l for l in lay["links"] if l[0] == "watt"), None)
    if watt:                                  # hinge axis for the Watt rocker, fixed in the axle
        wp = pts[watt[2]]
        node("WPF", 0, [wp[0], wp[1] - 0.06, wp[2]], 0.5, upright=True)
    body = [n for (k, s), n in nm.items() if nodes.get(n, {}).get("upright")]
    for i in range(len(body)):
        for j in range(i + 1, len(body)):
            beams.append({"a": body[i], "b": body[j], "role": "upright", "k": UPRIGHT_STIFFNESS, "F": UPRIGHT_STRENGTH})

    def tube(a, b, t=arm_tube):
        pa = nodes[a]["pos"] if a in nodes else chassis[a]
        pb = nodes[b]["pos"] if b in nodes else chassis[b]
        lim = section_limits(t, math.dist(pa, pb))
        beams.append({"a": a, "b": b, "role": "member", "tube": t, "k": min(lim["beamSpring"], k_ref),
                      "F": lim["beamDeform"], "Fu": lim["beamStrength"], "mass": lim["mass"]})

    for kind, ins, out in lay["links"]:
        if kind == "rod":
            for i in ins:
                if i in single and out in single:
                    tube(nm[(i, 0)], nm[(out, 0)])
                else:
                    for s in (1, -1):
                        tube(nm[(i, s)], nm[(out, s)])
        elif kind == "watt":                  # rocker hinged on the axle at WP, rods to the chassis
            wl, wr = ins
            wp = pts[out]
            wt = node("WT", 0, [wp[0], wp[1], pts[wl][2]], 0.8)
            wb = node("WB", 0, [wp[0], wp[1], pts[wr][2]], 0.8)
            for a, b in ((wt, wb), (wt, nm[(out, 0)]), (wb, nm[(out, 0)]), (wt, nm[("WPF", 0)]), (wb, nm[("WPF", 0)])):
                beams.append({"a": a, "b": b, "role": "upright", "k": UPRIGHT_STIFFNESS, "F": UPRIGHT_STRENGTH})
            tube(nm[(wl, 0)], wt)
            tube(nm[(wr, 0)], wb)
    top, bottom, _ = lay["damper"]
    for s in (1, -1):
        beams.append({"a": nm[(top, s)], "b": nm[(bottom, s)], "role": "spring",
                      "k": spec.get("spring_N_per_mm", 50) * 1000, "damp": spec.get("damper_bump_Ns_per_m", 3500)})
    notes.append(f"{cname}: {'de Dion tube' if lay.get('halfshafts') else 'solid axle'} exported as one rigid body "
                 "carrying both hubs; unverified in-game")
    if lay.get("halfshafts"):
        notes.append(f"{cname}: de Dion: the differential is sprung (on the chassis); halfshafts attach through the donor interface")


def rim_axle_node_masses():
    """Masses (kg) of the spec rims' axle nodes, which the wheel part brings: for the stability cap of
    the beams tying them to the uprights (8.9.6)."""
    return {f"{pre}{side}{side * extra}": RIM_AXLE_NODE_MASS for _, _, pre in SPEC_RIM.values()
            for side in ("l", "r") for extra in (0, 1)}


def build_wheels(sus_result, snodes):
    """Spec wheel mounting (regulations 5, 5.4). snodes are the nodes from build_suspension.

    Returns (slots2 rows, pressureWheels rows, attachment beams, parts for the .pc). The rim's
    axle nodes are tied to both hub nodes and the first two upright nodes by rigid beams.
    pressureWheels row: [name, hubGroup, group, node1 (outer rim node), node2 (inner rim node),
    nodeS (9999 = none), nodeArm (inner hub node), wheelDir (-1 left, +1 right), extra]; front
    wheels of a steered layout also get steerAxisUp / steerAxisDown from STEER_PAIR.

    BeamNG mirrors a slot's nodeOffset x by the sign of each node's x, so one offset places both sides.
    """
    slots, rows, beams, pc = [], [], [], {}
    for cname, res in (sus_result.get("corners") or {}).items():
        slot, rim, pre = SPEC_RIM[cname]
        tag = "f" if cname == "front" else "r"
        wc = res["points"]["WC"]
        ox = wc[0] - sum(RIM_AXLE_X) / 2              # midpoint of the rim's axle nodes on the wheel centre
        slots.append([slot, [slot], [], rim, f"Spec wheels {cname} (rules 5)",
                      {"nodeOffset": {"x": round(ox, 4), "y": round(wc[1], 4), "z": round(wc[2], 4)}}])
        pc[slot] = rim
        for s, side in ((1, "l"), (-1, "r")):
            n = lambda key: _side(f"{tag}{key.lower()}", s)
            upright = [k for k, d in snodes.items() if d.get("upright") and d["corner"] == cname and d["side"] == s
                       and k not in (n("hubi"), n("hubo"))]
            inner, outer = f"{pre}{side}", f"{pre}{side}{side}"
            wheel = ("F" if cname == "front" else "R") + side.upper()
            arm1, arm2 = (upright + [n("hubo")] * 2)[:2]
            extra = {"torqueCoupling:": arm1, "torqueArm:": arm2, "torqueArm2:": outer}
            pair = STEER_PAIR.get(res.get("type"))
            if cname == "front" and pair:
                extra.update({"steerAxisUp:": n(pair[0]), "steerAxisDown:": n(pair[1])})
            rows.append([wheel, f"wheel_{wheel}", f"tire_{wheel}", outer, inner, 9999, n("hubi"), -s, extra])
            for a in (inner, outer):
                for b in [n("hubi"), n("hubo")] + upright[:2]:
                    beams.append({"a": a, "b": b, "role": "upright", "k": UPRIGHT_STIFFNESS, "F": UPRIGHT_STRENGTH})
    if slots:
        slots.append([WHEELDATA_PART, [WHEELDATA_PART], [], WHEELDATA_PART, "Wheel data (after the wheels)"])
        pc[WHEELDATA_PART] = WHEELDATA_PART
    return slots, rows, beams, pc


def _pos(nodes, chassis, n):
    """Position of a suspension node or a chassis joint."""
    return nodes[n]["pos"] if n in nodes else chassis[n]


def _mirror_name(n):
    """Name of the mirrored joint (fbeam_checker.pickups.mirror_name)."""
    return mirror_name(n)


def ref_nodes(chassis_nodes):
    """refNodes from the chassis geometry: a low right-side ref node; left, back and up from it.

    chassis_nodes is {joint: [x, y, z]}. BeamNG reads them as a frame: left = +X, back = +Y,
    up = +Z from ref, so each is chosen to lie mainly along its own axis from ref (and more
    than 0.2 m along it). Returns {"ref", "back", "left", "up"}: joint names.
    """
    pts = chassis_nodes
    zmin = min(p[2] for p in pts.values())
    right = [n for n, p in pts.items() if p[0] < -0.05 and p[2] < zmin + 0.10] or list(pts)
    ref = min(right, key=lambda n: abs(pts[n][1]))

    def best(axis, exclude):
        r = pts[ref]

        def score(n):
            d = [pts[n][i] - r[i] for i in range(3)]
            L = math.sqrt(sum(c * c for c in d)) or 1e-9
            return d[axis] / L if d[axis] > 0.2 else -1.0          # direction cosine along the wanted axis
        return max((n for n in pts if n not in exclude), key=score)

    left = best(0, {ref})
    back = best(1, {ref, left})
    up = best(2, {ref, left, back})
    return {"ref": ref, "back": back, "left": left, "up": up}


def info_json(model_name):
    """The vehicle's info.json (name, type, default .pc) for BeamNG's vehicle selector."""
    return {"Name": "FBeam tube-frame car", "Brand": "FBeam", "Type": "Car", "Body Style": "Silhouette",
            "Country": "International", "Derby Class": "Compact", "default_pc": "race",
            "description": "Generated by the FBeam editor. Spec engine, gearbox and tyres attach through the donor interface.",
            "Author": "FBeam editor", "model": model_name}
