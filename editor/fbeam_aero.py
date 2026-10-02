"""Aero devices for the FBeam editor (regulations 7.4): rear wing and front splitter.

Pure Python, standard library only (runs in Pyodide). FBeam axes: X left, Y rear, Z up,
metres; forces in N, speeds in km/h. Device positions are the device centre (wing: centre
of the chord).
First-order estimates, clearly labelled as such, until they are calibrated in-game:

  wing:     thin-airfoil slope 2 pi per rad with the finite-span correction
            CL = 2*pi*alpha * AR_e / (AR_e + 2), AR = span / chord, AR_e = AR * 1.9 with
            endplates (they block the tip vortex, raising the effective aspect ratio);
            stalled above STALL_DEG (CL x 0.6, CD + 0.15); |CL| capped at the rules
            maximum (7.4.5). CD = CD0 + CL^2 / (pi * e * AR_e) (induced drag).
  splitter: ground-effect plate, CL = SPLITTER_CL on its plan area
  body:     frontal area from the envelope, Cd and Cl of a silhouette body

Forces: F = q S C with q = 0.5 rho V^2; "cl_down" is positive for downforce. Downforce is
split between the axles by moments about each axle, including the pitch moment of drag
acting at its height. Checks: wing size and position 7.4.2, splitter 7.4.3, stall and the
CL cap 7.4 / 7.4.5.
"""

import math

RHO = 1.225                       # kg/m^3, ISA sea level
STALL_DEG = 14.0                  # typical single-element wing stall angle (estimate)
CD0_WING = 0.02                   # profile drag of a wing section (estimate)
OSWALD_E = 0.85                   # span efficiency (estimate)
ENDPLATE_FACTOR = 1.9             # effective aspect-ratio gain from endplates (estimate)
SPLITTER_CL = 0.8                 # downforce coefficient on plan area, near the ground (estimate)
BODY_CD, BODY_CL = 0.38, 0.10     # silhouette body, lift positive (estimate)
SPEEDS_KMH = (80, 120, 160)       # speeds the forces are reported at


def example_aero():
    """Example rear wing and splitter for the Covet-based example car (m, deg)."""
    return [
        {"id": "rear_wing", "kind": "wing", "span": 1.35, "chord": 0.22, "aoa_deg": 6.0, "endplates": True,
         "position": [0.0, 1.62, 1.31]},
        {"id": "splitter", "kind": "splitter", "width": 1.50, "depth": 0.12, "position": [0.0, -1.95, 0.16]},
    ]


def device_coefficients(d, cl_max):
    """Coefficients of one aero device on its own reference area (plan area: span x chord, width x depth).

    d: device dict (kind "wing" or "splitter"; other kinds give zeros). cl_max: rules CL cap.
    Returns {"area" m^2, "cl_down" (positive = downforce), "cd", "stalled", "capped"}.
    """
    if d["kind"] == "wing":
        ar = d["span"] / d["chord"]
        ar_e = ar * (ENDPLATE_FACTOR if d.get("endplates") else 1.0)
        a = math.radians(d["aoa_deg"])
        cl = 2 * math.pi * a * ar_e / (ar_e + 2)        # positive AoA = downforce (the wing is inverted)
        stalled = abs(d["aoa_deg"]) > STALL_DEG
        if stalled:
            cl *= 0.6                                    # separated flow: lift falls away
        capped = abs(cl) > cl_max
        cl = max(-cl_max, min(cl_max, cl))
        cd = CD0_WING + cl * cl / (math.pi * OSWALD_E * ar_e) + (0.15 if stalled else 0.0)
        return {"area": d["span"] * d["chord"], "cl_down": cl, "cd": cd, "stalled": stalled, "capped": capped}
    if d["kind"] == "splitter":
        return {"area": d["width"] * d["depth"], "cl_down": SPLITTER_CL, "cd": 0.02,   # cd: flat-plate estimate
                "stalled": False, "capped": False}
    return {"area": 0.0, "cl_down": 0.0, "cd": 0.0, "stalled": False, "capped": False}


def analyse(model, rules, axles_y, envelope):
    """Coefficients, forces at SPEEDS_KMH, aero balance and 7.4 checks.

    rules: the "aero" section of the rules (max_wing_cl). axles_y: (front, rear) axle Y, m.
    envelope: {"width": body width m, optional "body": (roof z, front bumper y, rear bumper y)};
    without a body the roof and bumpers come from the jbeam nodes.
    Returns {"devices", "speeds": [{kmh, downforce_front_N, downforce_rear_N, downforce_N,
    drag_N, balance_front, drag_power_kW}], "frontal_area_m2", "roof_m", "issues", "limits",
    "estimates"}.
    """
    nodes = model["nodes"]
    devices = model.get("aero") or []
    yf, yr = axles_y
    L = yr - yf
    zs = [p[2] for p in nodes.values()] or [0.0]
    ys = [p[1] for p in nodes.values()] or [0.0]
    roof = max(zs) + 0.05                            # body skin over the cage roof
    front_bumper, rear_bumper = min(ys), max(ys)
    if envelope.get("body"):                         # the body shell, when there is one
        roof, front_bumper, rear_bumper = envelope["body"]
    body_w = envelope["width"]
    frontal = 0.85 * body_w * max(roof - 0.10, 0.5)  # silhouette area estimate: 85 % of width x (roof - 0.10 m clearance)
    cl_max = rules.get("max_wing_cl", 1.4)           # fallback = rules/fbeam_v1.json value
    rows, issues = [], []
    for d in devices:
        co = device_coefficients(d, cl_max)
        p = d["position"]
        rows.append(dict(co, id=d["id"], kind=d["kind"], y=p[1], z=p[2]))
        if d["kind"] == "wing":
            half_c = d["chord"] / 2                  # position is mid-chord; the trailing-edge check ignores the AoA
            if d["span"] > body_w + 1e-9:
                issues.append({"level": "ERROR", "rule": "7.4.2", "message": f"{d['id']}: span {d['span'] * 1000:.0f} mm is wider than the body ({body_w * 1000:.0f} mm)"})
            if p[2] + half_c * math.sin(math.radians(abs(d["aoa_deg"]))) > roof + 1e-9:
                issues.append({"level": "ERROR", "rule": "7.4.2", "message": f"{d['id']}: top is above the roof ({roof * 1000:.0f} mm)"})
            if p[1] + half_c > rear_bumper + 1e-9:
                issues.append({"level": "ERROR", "rule": "7.4.2", "message": f"{d['id']}: trailing edge is behind the rear bumper"})
            if co["stalled"]:
                issues.append({"level": "WARN", "rule": "7.4", "message": f"{d['id']}: {d['aoa_deg']:.0f} deg is past stall ({STALL_DEG:.0f} deg); lift drops, drag rises"})
            if co["capped"]:
                issues.append({"level": "INFO", "rule": "7.4.5", "message": f"{d['id']}: lift coefficient capped at the rules maximum {cl_max}"})
        elif d["kind"] == "splitter":
            if p[1] - d["depth"] / 2 < front_bumper - 0.05 - 1e-9:
                issues.append({"level": "ERROR", "rule": "7.4.3", "message": f"{d['id']}: more than 50 mm ahead of the front bumper"})
            if d["width"] > body_w + 1e-9:
                issues.append({"level": "ERROR", "rule": "7.4.3", "message": f"{d['id']}: wider than the body"})
    # body forces act mid-wheelbase at half the roof height (estimate)
    body = {"area": frontal, "cl_down": -BODY_CL, "cd": BODY_CD, "y": (yf + yr) / 2, "z": 0.5 * roof}
    speeds = []
    for v_kmh in SPEEDS_KMH:
        q = 0.5 * RHO * (v_kmh / 3.6) ** 2
        down_f = down_r = drag = 0.0
        for r in rows + [dict(body, id="body")]:
            dn = q * r["area"] * r["cl_down"]
            dg = q * (frontal * r["cd"] if r.get("id") == "body" else r["area"] * r["cd"])
            down_f += dn * (yr - r["y"]) / L         # lever rule: moments about the rear / front axle
            down_r += dn * (r["y"] - yf) / L
            drag += dg
            # drag at height z pitches the car nose-up: load moves from the front to the rear axle
            down_f -= dg * r["z"] / L
            down_r += dg * r["z"] / L
        speeds.append({"kmh": v_kmh, "downforce_front_N": down_f, "downforce_rear_N": down_r,
                       "downforce_N": down_f + down_r, "drag_N": drag,
                       "balance_front": down_f / (down_f + down_r) if down_f > 0 and down_r > 0 else None,
                       "drag_power_kW": drag * v_kmh / 3.6 / 1000})
    return {"devices": rows, "speeds": speeds, "frontal_area_m2": frontal, "roof_m": roof, "issues": issues,
            "limits": {"body_width": body_w, "roof": roof, "rear_bumper": rear_bumper, "front_bumper": front_bumper},
            "estimates": {"body_cd": BODY_CD, "body_cl": BODY_CL, "splitter_cl": SPLITTER_CL}}


def to_svj(model, result, to_sae, wheelbase):
    """SVJ aerodynamics block (SVJ spec §13): reference, static coefficients, components.

    result: analyse() output. to_sae: FBeam point -> SAE J670 axes. wheelbase: m (reference length).
    Whole-car coefficients are on the frontal area, from the first speed (they do not change with
    speed here); SVJ Cl is positive for lift, so it is -cl_down.
    """
    A = result["frontal_area_m2"]
    s = result["speeds"][0]
    q = 0.5 * RHO * (s["kmh"] / 3.6) ** 2
    comps = []
    for d, r in zip(model.get("aero") or [], result["devices"]):
        comps.append({"name": d["id"], "type": d["kind"], "position": [round(v, 5) for v in to_sae(d["position"])],
                      "area": round(r["area"], 5), "Cd_contribution": round(r["cd"] * r["area"] / A, 5),
                      "Cl_contribution": round(-r["cl_down"] * r["area"] / A, 5),
                      "adjustable": d["kind"] == "wing",
                      **({"settings": [{"name": "design", "angle": round(math.radians(d["aoa_deg"]), 5),
                                        "Cl": round(-r["cl_down"], 5), "Cd": round(r["cd"], 5)}]} if d["kind"] == "wing" else {})})
    return {"reference": {"frontal_area": round(A, 4), "reference_length": wheelbase, "air_density": RHO},
            "coefficients": {"Cd": round(s["drag_N"] / (q * A), 4), "Cl": round(-s["downforce_N"] / (q * A), 4),
                             "Cl_front": round(-s["downforce_front_N"] / (q * A), 4),
                             "Cl_rear": round(-s["downforce_rear_N"] / (q * A), 4)},
            "components": comps,
            "x_fbeam": {"method": "first-order estimate (thin airfoil + body estimate); calibrate in-game",
                        "speeds": result["speeds"]}}
