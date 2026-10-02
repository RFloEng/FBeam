"""Full-car mass, centre of gravity and inertia for the FBeam editor (regulations 4.4, 4.7, 6.7, 6.10, 6.11).

Pure Python, standard library only (runs in Pyodide). FBeam axes: X left, Y rear, Z up,
metres, kg.

The car is the sum of:
  * the structure: every jbeam node as a point mass (exactly what BeamNG simulates);
  * components: rigid boxes (engine + gearbox, driver, fuel, cooling, battery, body, ballast ...)
    with {"id", "kind" (a KINDS key), "mass", "position" (box centre), "size" (x, y, z extents)};
  * unsprung masses: one lump per corner at the wheel centre (model["hubs"]).
Inertia is taken about the car's CG in FBeam axes: I = sum(I_own + m d^2) (parallel axis), with
I_own of a uniform box m (b^2 + c^2) / 12 and point masses having none.
Rules: engine block at most 200 mm behind the front axle 4.4, fuel at weigh-in 4.7, the 75 kg driver ballast (Race
Mass definition, 2.1), ballast 6.10, minimum Race Mass 6.11. The driver checks are tagged 6.7,
which in the current regulations is the closed-body rule.
"""

FUEL_DENSITY = 0.745            # kg/L, petrol
DRIVER_MASS = 75.0              # Race Mass definition (2.1), 6.11, 6.13
DRIVER_MIN_CG = 0.25            # m above ground: 6.13 (a seated driver's CG, so it cannot double as low ballast)
ENGINE_MAX_BEHIND_AXLE = 0.20   # m (4.4)
FUEL_AT_WEIGH_IN = 10.0         # L (4.7)
BALLAST_DENSITY = 7850.0        # kg/m^3: ballast is solid steel blocks (6.10), so its size follows its mass
TUBE_ALLOWANCE = 0.025          # m: floor surface above the lowest chassis node (tube radius)

# component kinds: label shown in the editor, and the SVJ mass_bodies category for the export
KINDS = {
    "powertrain": "Engine + gearbox (spec part)", "driver": "Driver ballast", "fuel": "Fuel + tank",
    "cooling": "Radiator + coolant", "exhaust": "Exhaust", "battery": "Battery", "seat": "Seat + harness",
    "steering": "Steering rack + column", "body": "Body panels", "ballast": "Ballast", "other": "Other",
}
SVJ_CATEGORY = {"powertrain": "powertrain", "driver": "payload", "fuel": "fluid", "cooling": "fluid",
                "exhaust": "powertrain", "battery": "electrical", "seat": "other", "steering": "other",
                "body": "structural", "ballast": "ballast", "other": "other"}


def example_components():
    """Example car components at the Covet positions (donor files). Flagged estimates unless from the donor.

    The body_panels entry is replaced by the body shell's derived mass once a body exists
    (fbeam_editor_core.derive), and the ballast is filled to 6.11 by fill_ballast.
    """
    c = lambda i, kind, m, pos, size, note="": {"id": i, "kind": kind, "mass": m, "position": pos, "size": size,
                                              "estimate": True, "note": note}
    return [
        dict(c("engine_gearbox", "powertrain", 184.0, [0.0, -1.42, 0.50], [0.75, 0.45, 0.46],
               "Covet donor node weights: 1.8 SOHC 142 + race 6MT 24 + race diff 18 kg (rules/donor_covet.json)"),
             estimate=False),
        c("cooling", "cooling", 12.0, [0.0, -1.70, 0.47], [0.60, 0.10, 0.40], "Covet radiator position"),
        c("exhaust", "exhaust", 10.0, [0.05, 0.20, 0.22], [0.20, 3.00, 0.10], "Covet race exhaust run"),
        c("fuel", "fuel", 5.0 + FUEL_AT_WEIGH_IN * FUEL_DENSITY, [-0.12, 1.00, 0.32], [0.75, 0.20, 0.10],
          "5 kg tank + 10 L at weigh-in (4.7)"),
        c("battery", "battery", 12.0, [-0.35, 0.60, 0.25], [0.25, 0.18, 0.20]),
        c("driver", "driver", DRIVER_MASS, [0.36, 0.08, 0.45], [0.45, 0.60, 0.80], "75 kg driver ballast (6.13)"),
        c("seat", "seat", 10.0, [0.36, 0.08, 0.35], [0.45, 0.50, 0.70]),
        c("steering", "steering", 6.0, [0.0, -1.09, 0.29], [0.78, 0.10, 0.10], "Covet rack line"),
        c("body_panels", "body", 60.0, [0.0, -0.04, 0.75], [1.68, 3.92, 1.10], "Silhouette panels, not modelled yet"),
        c("ballast", "ballast", 0.0, [0.0, 0.0, 0.22], [0.60, 0.50, 0.0], "Solid steel blocks on the cockpit floor (6.10)"),
    ]


EXAMPLE_UNSPRUNG = 45.0         # kg per corner: spec wheel 25 (jbeam), hub + upright + brake 15, shafts and arm ends 5 (6.12: 180 kg)


def floor_height(model):
    """Cockpit floor height (m): the lowest jbeam node plus TUBE_ALLOWANCE (0.0 + allowance without nodes)."""
    zs = [p[2] for p in model["nodes"].values()]
    return (min(zs) if zs else 0.0) + TUBE_ALLOWANCE


def derive_ballast(model):
    """Ballast by construction (6.10): steel density sets the block height, and it sits on the floor.

    Changes the model's ballast components in place; returns {id: {"height", "z"}} in metres.
    """
    floor, out = floor_height(model), {}
    for c in model.get("components") or []:
        if c["kind"] != "ballast":
            continue
        sx, sy = max(c["size"][0], 0.05), max(c["size"][1], 0.05)     # footprint at least 50 x 50 mm
        h = c["mass"] / (BALLAST_DENSITY * sx * sy)                   # h = m / (rho * footprint)
        c["size"] = [sx, sy, round(h, 5)]
        c["position"][2] = round(floor + h / 2, 5)
        out[c["id"]] = {"height": c["size"][2], "z": c["position"][2]}
    return out


def fill_ballast(model, node_mass, min_mass):
    """Add the ballast needed to reach the minimum Race Mass (6.11), on the first ballast block.

    node_mass: {node: kg} of the structure; min_mass: kg. Creates a ballast block if there is
    none, rounds its mass up to 0.1 kg and re-derives its size. Changes the model in place and
    returns the shortfall in kg (before rounding), 0.0 if none.
    """
    derive_ballast(model)
    res = analyse(model, node_mass, min_mass)
    need = res["ballast_needed"] if res else 0.0
    if need <= 0:
        return 0.0
    comps = model.setdefault("components", [])
    b = next((c for c in comps if c["kind"] == "ballast"), None)
    if b is None:
        b = {"id": "ballast", "kind": "ballast", "mass": 0.0, "position": [0.0, 0.0, 0.0], "size": [0.60, 0.50, 0.0],
             "estimate": False, "note": "Solid steel blocks on the cockpit floor (6.10)"}
        comps.append(b)
    b["mass"] = round(b["mass"] + need + 0.05, 1)        # round up to 0.1 kg
    derive_ballast(model)
    return need


def _box_inertia(m, size):
    """Principal inertias [Ixx, Iyy, Izz] (kg m^2) of a uniform box of size (a, b, c) about its centre."""
    a, b, c = size
    return [m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12]


def analyse(model, node_mass, min_mass):
    """Mass, CG, inertia and the mass-rule checks of the whole car.

    model: editor model (nodes, components, hubs, axles, unsprung_mass_per_corner).
    node_mass: {node: kg} of the structure. min_mass: minimum Race Mass, kg (6.11).
    Returns None for a massless model, else {"mass_total", "structure_mass", "components_mass",
    "unsprung_total", "by_kind", "cg" [x, y, z], "inertia" {"Ixx_roll", "Iyy_pitch", "Izz_yaw"}
    (about the CG, roll = about FBeam Y, pitch = about FBeam X), "min_mass", "ballast_needed",
    "issues"}, plus "front_share", "left_share" and "y_front" when the axles are known.
    """
    nodes = model["nodes"]
    parts = []
    for n, m in node_mass.items():
        parts.append({"id": "structure", "mass": m, "pos": nodes[n], "I": [0.0, 0.0, 0.0]})
    comps = model.get("components") or []
    for c in comps:
        parts.append({"id": c["id"], "mass": c["mass"], "pos": c["position"], "I": _box_inertia(c["mass"], c["size"])})
    unsprung = model.get("unsprung_mass_per_corner")
    hubs = model.get("hubs") or {}
    if unsprung:
        for h in hubs.values():
            parts.append({"id": "unsprung", "mass": unsprung, "pos": h, "I": [0.0, 0.0, 0.0]})
    M = sum(p["mass"] for p in parts)
    if M <= 0:
        return None
    cg = [sum(p["mass"] * p["pos"][i] for p in parts) / M for i in range(3)]
    I = [0.0, 0.0, 0.0]
    for p in parts:                                                 # parallel axis: own inertia + m d^2
        d = [p["pos"][i] - cg[i] for i in range(3)]
        I[0] += p["I"][0] + p["mass"] * (d[1] ** 2 + d[2] ** 2)
        I[1] += p["I"][1] + p["mass"] * (d[0] ** 2 + d[2] ** 2)
        I[2] += p["I"][2] + p["mass"] * (d[0] ** 2 + d[1] ** 2)
    structure = sum(node_mass.values())
    by_kind = {}
    for c in comps:
        by_kind[c["kind"]] = by_kind.get(c["kind"], 0.0) + c["mass"]
    out = {"mass_total": M, "structure_mass": structure, "components_mass": sum(c["mass"] for c in comps),
           "unsprung_total": (unsprung or 0) * len(hubs) if unsprung else 0.0, "by_kind": by_kind,
           "cg": cg, "inertia": {"Ixx_roll": I[1], "Iyy_pitch": I[0], "Izz_yaw": I[2]},
           "min_mass": min_mass, "ballast_needed": max(0.0, min_mass - M), "issues": []}
    ax = model.get("axles") or {}
    yf = yr = None
    half_track = 0.70                                               # fallback, m: about the Covet's half track
    if all(k in hubs for k in ("FL", "FR", "RL", "RR")):            # wheel centres are the axle lines
        yf, yr = (hubs["FL"][1] + hubs["FR"][1]) / 2, (hubs["RL"][1] + hubs["RR"][1]) / 2
        half_track = (abs(hubs["FL"][0] - hubs["FR"][0]) + abs(hubs["RL"][0] - hubs["RR"][0])) / 4
    elif all(n in nodes for n in ax.get("front", []) + ax.get("rear", [])) and ax.get("front") and ax.get("rear"):
        yf = sum(nodes[n][1] for n in ax["front"]) / len(ax["front"])
        yr = sum(nodes[n][1] for n in ax["rear"]) / len(ax["rear"])
    if yf is not None:                                              # static load shares by moments about the axles
        out["front_share"] = (yr - cg[1]) / (yr - yf)
        out["left_share"] = 0.5 + cg[0] / (2 * half_track)
        out["y_front"] = yf
    iss = out["issues"]
    if M < min_mass:
        iss.append({"level": "WARN", "rule": "6.11", "message":
                    f"estimated Race Mass {M:.0f} kg is {min_mass - M:.0f} kg under the {min_mass:.0f} kg minimum; "
                    f"that much ballast would be needed"})
    drivers = [c for c in comps if c["kind"] == "driver"]
    if not drivers:
        iss.append({"level": "ERROR", "rule": "6.13", "message": "no 75 kg driver ballast in the car"})
    for d in drivers:
        if abs(d["mass"] - DRIVER_MASS) > 0.5:
            iss.append({"level": "ERROR", "rule": "6.13", "message": f"driver ballast is {d['mass']:.1f} kg, must be 75 kg"})
        if d["position"][2] < DRIVER_MIN_CG:
            iss.append({"level": "ERROR", "rule": "6.13", "message":
                        f"driver ballast CG {d['position'][2] * 1000:.0f} mm, must be at least 250 mm above ground"})
    if "y_front" in out:
        for c in comps:
            if c["kind"] == "powertrain" and c["position"][1] - out["y_front"] > ENGINE_MAX_BEHIND_AXLE + 1e-9:
                iss.append({"level": "ERROR", "rule": "4.4", "message":
                            f"engine block centre is {(c['position'][1] - out['y_front']) * 1000:.0f} mm behind the "
                            f"front axle; the limit is 200 mm"})
    ys = [nodes[n][1] for n in (ax.get("front", []) + ax.get("rear", [])) if n in nodes]
    for c in comps:
        if c["kind"] != "ballast" or c["mass"] <= 0:
            continue
        if ys and not (min(ys) <= c["position"][1] - c["size"][1] / 2 and c["position"][1] + c["size"][1] / 2 <= max(ys)):
            iss.append({"level": "ERROR", "rule": "6.10", "message": f"ballast {c['id']} must lie between the axles, on the cockpit floor"})
        if c["size"][2] > 0.35:                                     # editor advice, not a rule limit
            iss.append({"level": "WARN", "rule": "6.10", "message":
                        f"ballast {c['id']} is a {c['size'][2] * 1000:.0f} mm tall steel stack; spread it over a larger footprint"})
    fuel = [c for c in comps if c["kind"] == "fuel"]
    if not fuel:
        iss.append({"level": "WARN", "rule": "4.7", "message": "no fuel in the car; weigh-in is with 10 L"})
    return out


def to_svj_masses(model, result, to_sae):
    """SVJ chassis mass fields (SVJ spec §7.1, §7.2, not the FBeam regulations): mass_bodies, CG and inertia.

    result: analyse() output. to_sae: function mapping an FBeam point to SAE J670 axes.
    Returns {"mass_bodies", "center_of_gravity", "inertia"}; the unsprung lumps are not listed here.
    """
    comps = model.get("components") or []
    bodies = []
    for c in comps:
        if c["mass"] <= 0:
            continue
        Ix, Iy, Iz = _box_inertia(c["mass"], c["size"])
        # FBeam (X left, Y rear) -> SAE (X forward, Y right): roll = FBeam Y axis, pitch = FBeam X axis
        body = {"id": c["id"], "description": KINDS.get(c["kind"], c["kind"]), "category": SVJ_CATEGORY.get(c["kind"], "other"),
                "mass": round(c["mass"], 3), "position": [round(v, 5) for v in to_sae(c["position"])],
                "inertia": {"Ixx": round(Iy, 4), "Iyy": round(Ix, 4), "Izz": round(Iz, 4)}}
        if c.get("visual"):
            body["visual"] = c["visual"]
        bodies.append(body)
    return {"mass_bodies": bodies,
            "center_of_gravity": [round(v, 5) for v in to_sae(result["cg"])],
            "inertia": {"Ixx": round(result["inertia"]["Ixx_roll"], 3), "Iyy": round(result["inertia"]["Iyy_pitch"], 3),
                        "Izz": round(result["inertia"]["Izz_yaw"], 3)}}
