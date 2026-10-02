"""SVJ (Standard Vehicle JSON) import and export for the FBeam editor.

SVJ: https://github.com/RFloEng/SVJ-standard-vehicle-json (spec v0.99.1).

Called from fbeam_editor_core (export_svj, export_svj_bundle, import_svj_bundle);
docs/svj.md has the field-by-field mapping. All lengths are metres, angles radians,
rates N/m and masses kg on the SVJ side (SVJ "units": "SI").

Frames
  FBeam / BeamNG: X left, Y rear, Z up; origin at mid-wheelbase on the ground.
  SVJ (SAE J670): X forward, Y right, Z down; origin at the front-axle centre on the ground.
  Both are right-handed:  X_svj = -(y - y_front),  Y_svj = -x,  Z_svj = -z.
  y_front is the FBeam Y of the front axle line (see _front_y); it is stored in
  x_fbeam.y_front_axle_fbeam so the round trip lands on the same FBeam origin.
  glTF meshes (SVJ §22) use a third frame, Y up / -Z forward; see fbeam_gltf.

Ownership
  An imported file is kept whole in model["svj_source"] and is the base of every
  export. FBeam then overwrites only what it designs: suspension hardpoints and
  links, spring and damper mounts and rates, alignment, wheelbase, track and chassis
  stiffness; plus, when the model has them, mass_total / CG / inertia / mass_bodies
  (components), the aerodynamics block (aero devices) and the steering block (rack).
  Everything else (tyres, powertrain, brakes, corners FBeam does not model, other x_
  extensions) passes through untouched.

x_fbeam
  SVJ allows "x_" extension keys that other tools ignore. The top-level x_fbeam holds
  the full FBeam design model (tube frame, joints, pickups) so FBeam -> SVJ -> FBeam is
  lossless; on import it wins over the SVJ fields, which only update the pickups that
  really moved. Per-corner x_fbeam keeps what SVJ has no field for (FBeam layout name,
  coil rate, stroke, which body carries the damper), and per-link x_fbeam keeps the
  FBeam point keys of each link so non-standard layouts can be read back.
"""

import copy
import json
import math

SVJ_VERSION = "0.99.1"
FRAME_MESH_ID = "fbeam_tube_frame"                     # assets.meshes id of the generated tube-frame .glb
DEFAULT_GLTF_AXES = {"up": "Y", "forward": "-Z"}      # SVJ §22.4, Blender export convention
CORNERS = {"front": ("FL", "FR"), "rear": ("RL", "RR")}   # FBeam axle -> SVJ (left, right) corner keys
# SVJ wheel block for every corner: spec tyre 165/70R14 on a 14 x 5.5 in rim (SPEC_TYRE in fbeam_suspension.py)
DEFAULT_WHEELBASE = 2.50     # m: the Covet's, used when a file gives none (rule 6.1: 2,450-2,550 mm)
RIM = {"rim_diameter": 14 * 0.0254, "rim_width": 5.5 * 0.0254,
       "tire_section_width": 0.165, "tire_aspect_ratio": 0.70}
RIM["tire_outer_diameter"] = 2 * 0.297                                  # as simulated (jbeam radius); nominal 0.587


# ---------------------------------------------------------------- frames

def _front_y(model):
    """FBeam Y (m) of the front axle line: front hubs, else the front axle joints, else 0."""
    hubs = model.get("hubs") or {}
    ys = [h[1] for k, h in hubs.items() if k.startswith("F")]
    if ys:                                                     # wheel centres are the axle line
        return sum(ys) / len(ys)
    ax = (model.get("axles") or {}).get("front")
    if ax and all(n in model["nodes"] for n in ax):
        return sum(model["nodes"][n][1] for n in ax) / len(ax)
    return 0.0


def to_sae(p, yf):
    """FBeam point [x, y, z] (m) -> SAE J670 [X, Y, Z] (m); yf is the FBeam Y of the front axle."""
    return [-(p[1] - yf), -p[0], -p[2]]


def from_sae(p, yf):
    """SAE J670 point [X, Y, Z] (m) -> FBeam [x, y, z] (m); inverse of to_sae."""
    return [-p[1], yf - p[0], -p[2]]


def _mirror(p):
    """Left <-> right in the FBeam frame (flip X)."""
    return [-p[0], p[1], p[2]]


def _r(v, n=5):                                     # round a number or a point; 5 decimals = 0.01 mm
    return [round(c, n) for c in v] if isinstance(v, list) else round(v, n)


# ---------------------------------------------------------------- export

def _corner_to_svj(name, side, spec, pts, analysis, yf):
    """One SVJ corner from the FBeam left-side definition (side = +1 left, -1 right).

    name: "front" or "rear"; spec: model["suspension"][name]; pts: the solved FBeam
    left-side points (m); analysis: that corner of fbeam_editor_core.suspension()
    (its "static" block gives motion ratio and alignment), or None.
    MacPherson and double wishbone map to SVJ's own hardpoint names; every other
    layout goes through _generic_corner_to_svj. Returns the SVJ corner dict.

    Rates: SVJ spring.rate is the wheel rate = coil rate x motion ratio^2 (N/m); the
    coil rate itself is kept in x_fbeam. Dampers are linear, written as a two-point
    curve to 0.5 m/s. Defaults (50 N/mm coil, 160 mm stroke) match fbeam_suspension;
    the 3500 / 6000 N·s/m bump / rebound are placeholders, flagged damping_is_estimate.
    """
    P = {k: to_sae(v if side > 0 else _mirror(v), yf) for k, v in pts.items()}
    st = analysis["static"] if analysis else {}
    if spec["type"] not in ("macpherson", "double_wishbone"):
        return _generic_corner_to_svj(name, side, spec, P, st, pts, yf)
    steered = name == "front"
    link_tie = {"name": "steering_tie_rod" if steered else "toe_link", "type": "rod",
                "inboard_points": [_r(P["TRI"])],
                "outboard_ref": "hardpoints.steering_tie_rod_end" if steered else "hardpoints.toe_link_outer"}
    hp = {"lower_ball_joint": _r(P["LBJ"]), "wheel_center": _r(P["WC"])}
    hp["steering_tie_rod_end" if steered else "toe_link_outer"] = _r(P["TRO"])
    if spec["type"] == "macpherson":
        system = "macpherson"
        hp["strut_top"] = _r(P["ST"])           # SVJ convention: coincides with the turret at design height
        hp["strut_lower"] = _r(P["SB"])         # custom hardpoint: fixes the strut axis in the upright
        links = [{"name": "strut", "type": "strut", "inboard_points": [_r(P["ST"])], "outboard_ref": "hardpoints.strut_top"},
                 {"name": "lower_control_arm", "type": "arm", "inboard_points": [_r(P["LIF"]), _r(P["LIR"])],
                  "outboard_ref": "hardpoints.lower_ball_joint"}, link_tie]
        mount_in, mount_out = P["ST"], P["SB"]
    else:
        system = "double_wishbone"
        hp["upper_ball_joint"] = _r(P["UBJ"])
        hp["damper_outboard"] = _r(P["DB"])     # FBeam mounts it on the lower wishbone (see x_fbeam)
        links = [{"name": "upper_wishbone", "type": "arm", "inboard_points": [_r(P["UIF"]), _r(P["UIR"])],
                  "outboard_ref": "hardpoints.upper_ball_joint"},
                 {"name": "lower_wishbone", "type": "arm", "inboard_points": [_r(P["LIF"]), _r(P["LIR"])],
                  "outboard_ref": "hardpoints.lower_ball_joint"}, link_tie]
        mount_in, mount_out = P["DT"], P["DB"]
    mr = st.get("motion_ratio") or 1.0
    coil = spec.get("spring_N_per_mm", 50.0) * 1000
    cb, cr = spec.get("damper_bump_Ns_per_m", 3500.0), spec.get("damper_rebound_Ns_per_m", 6000.0)
    corner = {
        "topology": {"system_type": system,
                     "upright": {"id": "upright_" + CORNERS[name][0 if side > 0 else 1],
                                 "hardpoints": hp},
                     "links": links},
        "wheel": dict((k, round(v, 5)) for k, v in RIM.items()),
        "spring": {"type": "coilover", "rate": round(coil * mr * mr, 1), "motion_ratio": round(mr, 4),
                   "inboard_mount": _r(mount_in), "outboard_mount": _r(mount_out)},
        "damper": {"type": "twin_tube", "bump_curve": [[0.0, 0.0], [0.5, cb * 0.5]],
                   "rebound_curve": [[0.0, 0.0], [0.5, cr * 0.5]], "motion_ratio": round(mr, 4),
                   "inboard_mount": _r(mount_in), "outboard_mount": _r(mount_out)},
        "x_fbeam": {"coil_rate_N_per_m": coil, "damper_stroke_m": spec.get("damper_stroke_mm", 160) / 1000,
                    "damper_outboard_body": "lower_wishbone" if system == "double_wishbone" else "upright",
                    "damping_is_estimate": "damper_bump_Ns_per_m" not in spec},
    }
    if st:
        corner["alignment"] = {"camber": 0.0, "toe": 0.0, "caster": math.radians(st["caster_deg"]),
                               "kpi": math.radians(st["kpi_deg"]), "scrub_radius": st["scrub_radius_mm"] / 1000,
                               "caster_trail": st["trail_mm"] / 1000}
        corner["bump_stop"] = {"gap": round(st["bump_mm"] / 1000, 4)}
        corner["rebound_stop"] = {"gap": round(st["rebound_mm"] / 1000, 4)}
    return corner


# FBeam layouts without a dedicated SVJ mapping (keys of fbeam_suspension.TOPOLOGY) -> SVJ
# system_type. Several FBeam layouts share one SVJ type, so the FBeam name also goes in
# the corner's x_fbeam.layout and wins on import.
SVJ_SYSTEM = {"multilink": "multi_link", "trailing_arm": "trailing_arm", "semi_trailing_arm": "semi_trailing_arm",
              "trailing_arm_lateral": "multi_link", "twist_beam": "torsion_beam", "solid_axle_panhard": "solid_axle",
              "solid_axle_watt": "solid_axle", "solid_axle_triangulated": "solid_axle",
              "de_dion_watt": "de_dion", "de_dion_panhard": "de_dion"}
_SVJ_LINK = {"rod": "rod", "arm": "arm", "watt": "rod", "strut": "strut"}   # TOPOLOGY link kind -> SVJ link type
# FBeam point keys with a standard SVJ hardpoint name; other keys are named from their label
_HP_NAME = {"WC": "wheel_center", "LBJ": "lower_ball_joint", "UBJ": "upper_ball_joint", "DB": "damper_outboard"}


def _hp_name(key, steered):
    """SVJ hardpoint name for an FBeam point key (TRO depends on whether the corner steers)."""
    if key == "TRO":
        return "steering_tie_rod_end" if steered else "toe_link_outer"
    from fbeam_suspension import LABEL
    return _HP_NAME.get(key) or LABEL[key].lower().replace(",", "").replace("(", "").replace(")", "").replace(" ", "_")


def _generic_corner_to_svj(name, side, spec, P, st, pts, yf):
    """Any FBeam layout as SVJ links (rod / arm / pivot), with the FBeam point keys kept per link.

    Multi-link, trailing arms, twist beam, solid axles and de Dion. P: the corner's
    points already in SAE (mirrored for the right side); pts: the FBeam left-side
    points, used for the "single" points (Panhard rod, Watt's linkage), which exist
    once and are written unmirrored on both corners. Same rate rules as _corner_to_svj.
    """
    from fbeam_suspension import TOPOLOGY, mirror as smirror
    lay = TOPOLOGY[spec["type"]]
    P = dict(P)
    for k in lay.get("single", []):           # Panhard rod, Watt's linkage: one of them, not mirrored
        P[k] = to_sae(pts[k], yf)
    steered = name == "front" and lay["toe"] is not None
    hp, links = {"wheel_center": _r(P["WC"])}, []
    top, bottom, carrier = lay["damper"]
    for kind, ins, out in lay["links"]:
        if kind == "pivot_mirror":                       # twist beam: the axis runs through both bushes
            other = to_sae(smirror(from_sae(P[ins[0]], 0.0)), 0.0)
            links.append({"name": "twist_beam_arm", "type": "pivot", "inboard_points": [_r(P[ins[0]]), _r(other)],
                          "outboard_ref": "hardpoints.wheel_center", "x_fbeam": {"inner": list(ins), "outer": "WC"}})
            continue
        if kind in ("pivot", "ball"):
            links.append({"name": "trailing_arm", "type": "pivot" if kind == "pivot" else "arm",
                          "inboard_points": [_r(P[i]) for i in ins], "outboard_ref": "hardpoints.wheel_center",
                          "x_fbeam": {"inner": list(ins), "outer": "WC", "kind": kind}})
            continue
        hn = _hp_name(out, steered)
        hp[hn] = _r(P[out])
        for i in (ins if kind == "rod" else [ins]):
            keys = [i] if kind == "rod" else list(i)
            links.append({"name": ("steering_tie_rod" if steered else "toe_link") if out == "TRO" else
                          "watts_linkage" if kind == "watt" else f"{_hp_name(keys[0], steered).replace('_inner', '')}",
                          "type": _SVJ_LINK[kind], "inboard_points": [_r(P[k]) for k in keys], "outboard_ref": "hardpoints." + hn,
                          "x_fbeam": {"inner": keys, "outer": out}})
    hp["damper_outboard"] = _r(P[bottom])
    mr = st.get("motion_ratio") or 1.0
    coil = spec.get("spring_N_per_mm", 50.0) * 1000
    cb, cr = spec.get("damper_bump_Ns_per_m", 3500.0), spec.get("damper_rebound_Ns_per_m", 6000.0)
    corner = {
        "topology": {"system_type": SVJ_SYSTEM[spec["type"]],
                     "upright": {"id": "upright_" + CORNERS[name][0 if side > 0 else 1], "hardpoints": hp},
                     "links": links},
        "wheel": dict((k, round(v, 5)) for k, v in RIM.items()),
        "spring": {"type": "coilover", "rate": round(coil * mr * mr, 1), "motion_ratio": round(mr, 4),
                   "inboard_mount": _r(P[top]), "outboard_mount": _r(P[bottom])},
        "damper": {"type": "twin_tube", "bump_curve": [[0.0, 0.0], [0.5, cb * 0.5]],
                   "rebound_curve": [[0.0, 0.0], [0.5, cr * 0.5]], "motion_ratio": round(mr, 4),
                   "inboard_mount": _r(P[top]), "outboard_mount": _r(P[bottom])},
        "x_fbeam": {"layout": spec["type"], "coil_rate_N_per_m": coil,
                    **({"halfshaft_inner": _r(P["DIFF"])} if "DIFF" in P else {}),
                    "damper_stroke_m": spec.get("damper_stroke_mm", 160) / 1000,
                    "damper_outboard_body": "upright" if carrier == "upright" else "arm",
                    "damping_is_estimate": "damper_bump_Ns_per_m" not in spec},
    }
    if st:
        al = {"camber": 0.0, "toe": 0.0}
        if st.get("kpi_deg") is not None:
            al.update({"caster": math.radians(st["caster_deg"]), "kpi": math.radians(st["kpi_deg"]),
                       "scrub_radius": st["scrub_radius_mm"] / 1000, "caster_trail": st["trail_mm"] / 1000})
        corner["alignment"] = al
        corner["bump_stop"] = {"gap": round(st["bump_mm"] / 1000, 4)}
        corner["rebound_stop"] = {"gap": round(st["rebound_mm"] / 1000, 4)}
    return corner


def _generic_corner_from_svj(corner, yf, name, layout):
    """Points of an FBeam layout back from the links it wrote (external edits are picked up).

    Reads each link's x_fbeam inner / outer keys, then the damper mounts and the
    halfshaft inner joint. Returns (points in FBeam m, []) or (None, [note]) when a
    point the layout needs (fbeam_suspension.REQUIRED) is missing.
    """
    from fbeam_suspension import TOPOLOGY, REQUIRED
    B = lambda p: from_sae(p, yf)
    pts = {"WC": B(_hp(corner, "wheel_center"))}
    for link in corner["topology"].get("links", []):
        xl = link.get("x_fbeam") or {}
        for k, p in zip(xl.get("inner", []), link["inboard_points"]):
            pts.setdefault(k, B(p))
        out = xl.get("outer")
        if out and out != "WC":
            pts[out] = B(_hp(corner, link["outboard_ref"].split(".")[-1]))
    top, bottom, _ = TOPOLOGY[layout]["damper"]
    damper = corner.get("damper", {})
    if damper.get("inboard_mount"):
        pts[top] = B(damper["inboard_mount"])
    if damper.get("outboard_mount") or _hp(corner, "damper_outboard"):
        pts[bottom] = B(damper.get("outboard_mount") or _hp(corner, "damper_outboard"))
    if (corner.get("x_fbeam") or {}).get("halfshaft_inner"):
        pts["DIFF"] = B(corner["x_fbeam"]["halfshaft_inner"])
    missing = [k for k in REQUIRED[layout] if k not in pts]
    if missing:
        return None, [f"{name}: SVJ corner is missing {', '.join(missing)} for the {layout} layout"]
    return pts, []


def to_svj(model, suspension_result=None, rigidity=None, totals=None, cog=None, frame_mesh_uri=None):
    """Build an SVJ document (dict) from an FBeam model (and optional analysis results).

    model: the FBeam design model; an imported file in model["svj_source"] is the base.
    suspension_result: fbeam_editor_core.suspension() output (corners, static values);
      without it no suspension, wheelbase or track is written.
    rigidity: fbeam_editor_core.rigidity() output (N·m/deg, N/mm) for chassis_stiffness.
    totals, cog: from fbeam_editor_core.evaluate() (structure node mass in kg, CoG in
      FBeam m); with components in the model they also drive the full-car masses.
    frame_mesh_uri: relative path of a generated .glb of the tube frame (SVJ §22). It is
      bound to the chassis, unless the file already binds a body mesh there; then it is
      listed as an asset with a helper node and referenced from x_fbeam.
    The model may carry results the core attached before the call: "_node_mass",
    "_aero" and "_steering".
    """
    base = copy.deepcopy(model.get("svj_source") or {})
    yf = _front_y(model)
    svj = base
    meta = svj.setdefault("_metadata", {})
    meta.update({"specification": "SVJ", "version": SVJ_VERSION, "coordinate_system": "SAE_J670", "units": "SI",
                 "author": meta.get("author", "FBeam editor"), "source_format": "FBeam",
                 "alignment_convention": "relative_to_centerline"})
    meta.setdefault("description", "FBeam tube-frame rallycross car, exported by the FBeam editor.")
    meta.setdefault("data_origin", {"type": "calculated", "detail": "FBeam design geometry; masses are estimates",
                                    "confidence": "low"})
    svj.setdefault("vehicle_info", {"make": "FBeam", "model": "Tube-frame rallycross car", "drive_type": "FWD",
                                    "vehicle_class": "race_car"})
    if (model.get("driveline") or {}).get("layout"):
        svj["vehicle_info"]["drive_type"] = model["driveline"]["layout"]
    ch = svj.setdefault("chassis", {})
    corners = (suspension_result or {}).get("corners", {})
    susp = model.get("suspension") or {}
    if corners.get("front") and corners.get("rear"):
        wf, wr = corners["front"]["points"]["WC"], corners["rear"]["points"]["WC"]
        ch["wheelbase"] = round(abs(wr[1] - wf[1]), 4)
        ch["track_front"] = round(corners["front"]["static"]["track_mm"] / 1000, 4)
        ch["track_rear"] = round(corners["rear"]["static"]["track_mm"] / 1000, 4)
    structure = None
    if totals and cog:
        structure = {"id": "fbeam_tube_frame", "description": "FBeam tube frame (tubes + joint allowance)",
                     "category": "structural", "mass": round(totals["node_mass"], 3),
                     "position": _r(to_sae([cog["x"], cog["y"], cog["z"]], yf))}
    if "mass_total" not in ch:                        # no better data: declare the minimum Race Mass (rules 6.11, 1,100 kg)
        ch["mass_total"] = 1100.0
        ch["_est"] = True
        ch["_est_note"] = "FBeam minimum Race Mass (rule 6.11) with driver; not a weighed mass."
    if "center_of_gravity" not in ch:
        # fallback: mid-wheelbase of the Covet-size car (2.50 m, rule 6.1), 0.5 m up
        ch["center_of_gravity"] = structure["position"] if structure else [-DEFAULT_WHEELBASE / 2, 0.0, -0.5]
        ch["center_of_gravity_note"] = "Structure only (FBeam tube frame); full-car CG needs engine, driver and body."
    if model.get("components") and totals is not None:   # full-car masses from fbeam_mass: FBeam owns them
        import fbeam_mass
        node_mass = model.get("_node_mass") or {}
        fbeam_mass.derive_ballast(model)
        res = fbeam_mass.analyse(model, node_mass, 0.0) if node_mass else None
        if res:
            m = fbeam_mass.to_svj_masses(model, res, lambda p: to_sae(p, yf))
            ch["mass_total"] = round(res["mass_total"], 2)
            ch["center_of_gravity"] = m["center_of_gravity"]
            ch["inertia"] = m["inertia"]
            ch["mass_bodies"] = ([{"id": "fbeam_tube_frame", "description": "FBeam tube frame (jbeam node masses)",
                                   "category": "structural", "mass": round(res["structure_mass"], 3),
                                   "position": _r(to_sae([cog["x"], cog["y"], cog["z"]], yf)) if cog else [0, 0, 0]}]
                                 + m["mass_bodies"])
            if model.get("unsprung_mass_per_corner"):
                ch["mass_unsprung_per_corner"] = {k: model["unsprung_mass_per_corner"] for k in ("FL", "FR", "RL", "RR")}
            est = any(c.get("estimate") for c in model["components"])
            ch.pop("center_of_gravity_note", None)
            if est:
                ch["_est"] = True
                ch["_est_note"] = "Component masses are FBeam estimates; weigh the car to confirm."
            else:
                ch.pop("_est", None), ch.pop("_est_note", None)
    if model.get("aero") and model.get("_aero"):          # FBeam designs the aero devices: it owns this block
        import fbeam_aero
        svj["aerodynamics"] = fbeam_aero.to_svj(model, model["_aero"], lambda p: to_sae(p, yf), ch.get("wheelbase", DEFAULT_WHEELBASE))
    if rigidity:
        ch["chassis_stiffness"] = {"torsional_stiffness": round(rigidity["frame"]["torsion_Nm_per_deg"], 1),
                                   "bending_stiffness": round(rigidity["frame"]["bending_N_per_mm"], 1),
                                   "measurement_method": "calculated"}
    if corners:
        svj.setdefault("suspension", {})               # corners FBeam does not model are passed through
        for name, (L, R) in CORNERS.items():
            if name not in corners:
                continue
            res, spec = corners[name], susp[name]
            svj["suspension"][L] = _corner_to_svj(name, 1, spec, res["points"], res, yf)
            svj["suspension"][R] = _corner_to_svj(name, -1, spec, res["points"], res, yf)
    rack = None
    if "front" in corners:
        tri = corners["front"]["points"]["TRI"]
        rack = _r(to_sae([0.0, tri[1], tri[2]], yf))
        steer = model.get("_steering")
        if steer:                                     # FBeam designs the rack: write a complete steering block (§8)
            st = svj.setdefault("steering", {})
            st.update({"type": st.get("type", "rack_and_pinion"), "rack_position": rack,
                       "rack_travel": round(steer["rack_travel_total_m"], 4),
                       "overall_ratio": round(steer["overall_ratio"], 3),
                       "lock_to_lock": round(steer["lock_to_lock_turns"], 3)})
        elif "steering" in svj:                       # otherwise only update a complete block from an imported file
            svj["steering"]["rack_position"] = rack
    frame_node = None
    if frame_mesh_uri:                                 # bind the generated frame mesh (§22)
        assets = svj.setdefault("assets", {}).setdefault("meshes", [])
        assets[:] = [a for a in assets if a.get("id") != FRAME_MESH_ID]
        assets.append({"id": FRAME_MESH_ID, "uri": frame_mesh_uri, "description": "FBeam tube frame (generated)"})
        if "visual" in ch and ch["visual"].get("mesh_ref") != FRAME_MESH_ID:
            frame_node = "SVJ::helper::fbeam_tube_frame"
        else:
            frame_node = "SVJ::body::chassis"
            ch["visual"] = {"mesh_ref": FRAME_MESH_ID, "node": frame_node}
    # FBeam-only data (see the module docstring): the design model without svj_source, so it is not stored twice
    svj["x_fbeam"] = {"fbeam_model_version": 1, "y_front_axle_fbeam": yf,
                      "frame_mesh": {"mesh_ref": FRAME_MESH_ID, "node": frame_node} if frame_node else None,
                      "note": "FBeam design model (tube frame, joints, pickups). FBeam-only data; other tools may ignore it.",
                      "structure": structure,
                      "jbeam_torsion_Nm_per_deg": rigidity["jbeam"]["torsion_Nm_per_deg"] if rigidity else None,
                      "rack_position": rack,
                      "model": {k: v for k, v in model.items() if k != "svj_source"}}
    return svj


def visual_bindings(svj):
    """Every visual binding in an SVJ document: [{"path", "mesh_ref", "node", "uri"}].

    Looks at chassis, suspension corners, mass_bodies, aerodynamics components and the
    x_fbeam frame mesh (when bound to a helper node). path is the SVJ location, node the
    glTF node name; a binding without mesh_ref uses the only mesh asset if there is one.
    uri is None when the mesh_ref is not in assets.meshes.
    """
    uris = {a["id"]: a["uri"] for a in (svj.get("assets") or {}).get("meshes", [])}
    out = []

    def add(path, vis):
        if isinstance(vis, dict) and "node" in vis:
            ref = vis.get("mesh_ref") or (next(iter(uris)) if len(uris) == 1 else None)
            out.append({"path": path, "mesh_ref": ref, "node": vis["node"], "uri": uris.get(ref)})

    add("chassis", (svj.get("chassis") or {}).get("visual"))
    for name, c in (svj.get("suspension") or {}).items():
        add(f"suspension.{name}", c.get("visual"))
    for b in (svj.get("chassis") or {}).get("mass_bodies", []) or []:
        add(f"mass_bodies.{b.get('id')}", b.get("visual"))
    comps = (svj.get("aerodynamics") or {}).get("components")
    if isinstance(comps, dict):
        for k, a in comps.items():
            add(f"aerodynamics.{k}", a.get("visual") if isinstance(a, dict) else None)
    elif isinstance(comps, list):
        for a in comps:
            if isinstance(a, dict):
                add(f"aerodynamics.{a.get('id')}", a.get("visual"))
    fm = (svj.get("x_fbeam") or {}).get("frame_mesh")
    if fm and str(fm.get("node", "")).startswith("SVJ::helper::"):
        add("x_fbeam.frame_mesh", fm)
    return out


def gltf_axes(svj):
    """Axis convention of the glTF assets: _metadata.coordinate_system if it is an object (§22.4).

    Returns {"up": axis, "forward": axis} with axes like "Y" or "-Z"; the default is
    Y up, -Z forward. app.js gltfToThree turns this into the mesh placement matrix.
    """
    cs = (svj.get("_metadata") or {}).get("coordinate_system")
    return {"up": cs["up"], "forward": cs["forward"]} if isinstance(cs, dict) else dict(DEFAULT_GLTF_AXES)


# ---------------------------------------------------------------- import

def _find_link(links, *words):
    """First SVJ link whose name contains any of words, or None."""
    for l in links:
        if any(w in l["name"] for w in words):
            return l
    return None


def _hp(corner, key):
    """Upright hardpoint key of an SVJ corner (SAE m), or None."""
    return corner["topology"]["upright"]["hardpoints"].get(key)


def _corner_from_svj(corner, yf, name):
    """FBeam left-side corner points (coordinates) from an SVJ corner. Returns (spec, points, notes).

    corner: SVJ suspension corner; yf: FBeam Y of the front axle (m); name: "front" / "rear".
    A corner with x_fbeam.layout goes through _generic_corner_from_svj; otherwise only SVJ
    macpherson and double_wishbone are read, from SVJ hardpoint and link names. Missing
    points are guessed and noted. spec is None when the corner cannot be used.
    """
    topo = corner["topology"]
    kind, links, notes = topo["system_type"], topo.get("links", []), []
    layout = (corner.get("x_fbeam") or {}).get("layout")
    if layout and layout not in ("macpherson", "double_wishbone"):
        pts, notes = _generic_corner_from_svj(corner, yf, name, layout)
        if pts is None:
            return None, None, notes
        kind = layout
    elif kind not in ("macpherson", "double_wishbone"):
        return None, None, [f"{name}: SVJ system_type '{kind}' is not supported by the FBeam kinematics yet"]
    B = lambda p: from_sae(p, yf)
    if kind not in ("macpherson", "double_wishbone"):
        return _spec_from_svj(corner, kind), pts, notes
    lower = _find_link(links, "lower")
    tie = _find_link(links, "tie_rod", "toe")
    if not lower or not tie:
        return None, None, [f"{name}: needs a lower arm and a tie rod or toe link"]
    tie_out = tie["outboard_ref"].split(".")[-1]
    pts = {"LIF": B(lower["inboard_points"][0]), "LIR": B(lower["inboard_points"][1]),
           "LBJ": B(_hp(corner, "lower_ball_joint")), "TRI": B(tie["inboard_points"][0]),
           "TRO": B(_hp(corner, tie_out)), "WC": B(_hp(corner, "wheel_center"))}
    damper = corner.get("damper", {})
    if kind == "macpherson":
        strut = _find_link(links, "strut")
        pts["ST"] = B(strut["inboard_points"][0] if strut else _hp(corner, "strut_top"))
        lowpt = _hp(corner, "strut_lower") or damper.get("outboard_mount")
        if lowpt is None:
            lbj = pts["LBJ"]
            pts["SB"] = [lbj[i] + (pts["ST"][i] - lbj[i]) * 0.4 for i in range(3)]   # 40 % up the LBJ-strut top line
            notes.append(f"{name}: no strut lower point in the file; strut axis assumed through the lower ball joint")
        else:
            pts["SB"] = B(lowpt)
    else:
        upper = _find_link(links, "upper")
        pts.update({"UIF": B(upper["inboard_points"][0]), "UIR": B(upper["inboard_points"][1]),
                    "UBJ": B(_hp(corner, "upper_ball_joint"))})
        db = damper.get("outboard_mount") or _hp(corner, "damper_outboard")
        dt = damper.get("inboard_mount")
        pts["DB"] = B(db) if db else pts["LBJ"]
        pts["DT"] = B(dt) if dt else [pts["DB"][0], pts["DB"][1], pts["DB"][2] + 0.35]
        if not dt:
            notes.append(f"{name}: no damper inboard mount; placed 350 mm above the damper bottom")
    return _spec_from_svj(corner, kind), pts, notes


def _spec_from_svj(corner, kind):
    """FBeam suspension spec (rates and stroke, no points) from an SVJ corner.

    Coil rate from x_fbeam if present, else SVJ wheel rate / motion ratio^2 (N/mm); the
    damper coefficient is the slope of the first curve segment (N·s/m).
    """
    damper = corner.get("damper", {})
    spring = corner.get("spring", {})
    mr = spring.get("motion_ratio") or damper.get("motion_ratio") or 1.0
    xf = corner.get("x_fbeam", {})
    spec = {"type": kind,
            "spring_N_per_mm": round((xf.get("coil_rate_N_per_m") or spring.get("rate", 50000) / (mr * mr)) / 1000, 2),
            "damper_stroke_mm": round(xf.get("damper_stroke_m", 0.16) * 1000)}
    if not xf.get("coil_rate_N_per_m") and "rate" in spring:
        # SVJ stores the wheel rate: keep it, and let the caller re-derive the coil rate from FBeam's geometry
        spec["_svj_wheel_rate"], spec["_svj_motion_ratio"] = spring["rate"], mr
    for key, curve in (("damper_bump_Ns_per_m", damper.get("bump_curve")), ("damper_rebound_Ns_per_m", damper.get("rebound_curve"))):
        if curve and len(curve) > 1 and curve[1][0]:
            spec[key] = round(curve[1][1] / curve[1][0], 1)
    return spec


def from_svj(svj):
    """FBeam model from an SVJ document (dict). Returns (model, notes).

    Only the SAE_J670 frame is accepted. With x_fbeam.model the design is restored and
    the SVJ corners only update pickups that moved by more than 1 mm (points still on
    the same chassis joint stay attached). Without it a skeleton is built: one joint
    per inner pickup, axles, hubs, and mass_bodies as components. The SVJ file (minus
    x_fbeam) is kept in model["svj_source"] for pass-through on export; notes are
    plain-English messages for the issues panel.
    """
    notes = []
    meta = svj.get("_metadata", {})
    if meta.get("specification") != "SVJ":
        raise ValueError("not an SVJ file (_metadata.specification is not 'SVJ')")
    if meta.get("coordinate_system", "SAE_J670") != "SAE_J670":
        raise ValueError("only the SAE_J670 physics frame is supported")
    wb = svj.get("chassis", {}).get("wheelbase", DEFAULT_WHEELBASE)
    yf = -wb / 2
    xf = svj.get("x_fbeam", {})
    if xf.get("model"):                               # FBeam round trip: take the design model back as it was
        model = copy.deepcopy(xf["model"])
        yf = xf.get("y_front_axle_fbeam", yf)
        notes.append("FBeam design model restored from x_fbeam")
    else:
        model = {"nodes": {}, "members": []}
        notes.append("No FBeam data in the file: built a skeleton (joints at every inner pickup; draw the frame around them)")
    susp = svj.get("suspension", {})
    new = {}
    for name, (L, R) in CORNERS.items():
        if L not in susp:
            continue
        spec, pts, n = _corner_from_svj(susp[L], yf, name)
        notes += n
        if not spec:
            continue
        if R in susp and spec:
            _, ptsR, _ = _corner_from_svj(susp[R], yf, name)
            from fbeam_suspension import TOPOLOGY
            single = set(TOPOLOGY.get(spec["type"], {}).get("single", []))
            if ptsR and any(math.dist(_mirror(ptsR[k]), pts[k]) > 0.002 for k in pts if k in ptsR and k not in single):
                notes.append(f"{name}: {R} is not the mirror of {L}; FBeam uses {L} and mirrors it")
        new[name] = (spec, pts)
    if not xf.get("model"):
        from fbeam_suspension import INNER
        for name, (spec, pts) in new.items():
            for k in INNER[spec["type"]]:
                n = f"{name}_{k}"
                model["nodes"][n + "_L"] = [round(c, 5) for c in pts[k]]
                model["nodes"][n + "_R"] = [round(c, 5) for c in _mirror(pts[k])]
        if "front" in new and "rear" in new:
            ft = "ST" if new["front"][0]["type"] == "macpherson" else INNER[new["front"][0]["type"]][-1]
            rt = "ST" if new["rear"][0]["type"] == "macpherson" else INNER[new["rear"][0]["type"]][-1]
            model["axles"] = {"front": [f"front_{ft}_L", f"front_{ft}_R"], "rear": [f"rear_{rt}_L", f"rear_{rt}_R"]}
    if new:
        old = model.get("suspension") or {}
        model["suspension"] = {}
        for name, (spec, pts) in new.items():
            prev = old.get(name)
            points, changed = {}, []
            for k, p in pts.items():
                pv = prev["points"].get(k) if prev and prev.get("type") == spec["type"] else None
                pvpos = (model["nodes"].get(pv["node"]) if isinstance(pv, dict) else pv) if pv is not None else None
                if pvpos is not None and math.dist(pvpos, p) < 0.001:
                    points[k] = pv                        # unchanged: keep it (and its link to a chassis joint)
                    continue
                if prev:
                    changed.append(k)
                ref = next((n for n, q in model["nodes"].items() if math.dist(q, p) < 0.001), None)
                points[k] = {"node": ref} if ref and k != "WC" else [round(c, 5) for c in p]
            if prev and prev.get("type") == spec["type"]:
                for k, pv in prev["points"].items():      # points SVJ cannot hold (e.g. halfshaft inner joint)
                    points.setdefault(k, pv)
            if prev:
                merged = {k: v for k, v in prev.items() if k != "points"}
                for k, v in spec.items():                 # keep the FBeam value unless the SVJ one really differs
                    if k not in merged or (isinstance(v, (int, float)) and abs(v - merged[k]) > 1e-3 * max(1.0, abs(v))):
                        merged[k] = v
                spec = merged
                if changed:
                    notes.append(f"{name}: updated from the SVJ file: {', '.join(changed)}")
            model["suspension"][name] = dict(spec, points=points)
        model["hubs"] = {}
        for name, (spec, pts) in new.items():
            tag = "F" if name == "front" else "R"
            model["hubs"][tag + "L"] = [round(c, 5) for c in pts["WC"]]
            model["hubs"][tag + "R"] = [round(c, 5) for c in _mirror(pts["WC"])]
    ch = svj.get("chassis", {})
    if not xf.get("model") and ch.get("mass_bodies"):              # SVJ sprung bodies become FBeam components
        # SVJ mass_bodies category -> FBeam component kind (app.js KLABEL); a 75 kg payload is the driver ballast (6.7)
        cat = {"powertrain": "powertrain", "fluid": "fuel", "ballast": "ballast", "electrical": "battery",
               "structural": "body", "payload": "driver", "other": "other"}
        comps = []
        for b in ch["mass_bodies"]:
            kind = cat.get(b.get("category"), "other")
            if kind == "driver" and abs(b.get("mass", 0) - 75) > 0.5:
                kind = "other"
            I = b.get("inertia") or {}
            m = b.get("mass", 0.0)
            size = [0.4, 0.4, 0.4]
            if m > 0 and all(k in I for k in ("Ixx", "Iyy", "Izz")):     # the box with the same inertia
                ix, iy, iz = I["Iyy"], I["Ixx"], I["Izz"]                  # FBeam X = SAE pitch, FBeam Y = SAE roll
                sq = [max(6 / m * (iy + iz - ix), 1e-4), max(6 / m * (ix + iz - iy), 1e-4), max(6 / m * (ix + iy - iz), 1e-4)]
                size = [round(math.sqrt(v), 3) for v in sq]
            comps.append({"id": b.get("id", f"body_{len(comps)}"), "kind": kind, "mass": m,
                          "position": [round(v, 5) for v in from_sae(b.get("position", [0, 0, 0]), yf)],
                          "size": size, "estimate": False, "note": f"from SVJ ({b.get('category', 'other')})"})
        model["components"] = comps
        mu = ch.get("mass_unsprung_per_corner") or {}
        if mu:
            model["unsprung_mass_per_corner"] = round(sum(mu.values()) / len(mu), 3)
        notes.append(f"{len(comps)} mass bodies imported as components")
    model["svj_source"] = {k: v for k, v in svj.items() if k != "x_fbeam"}
    model["svj_info"] = {"vehicle": " ".join(str(svj.get("vehicle_info", {}).get(k, "")) for k in ("make", "model", "variant")).strip(),
                         "mass_total": ch.get("mass_total"), "center_of_gravity": ch.get("center_of_gravity"),
                         "wheelbase": ch.get("wheelbase"), "version": meta.get("version")}
    return model, notes
