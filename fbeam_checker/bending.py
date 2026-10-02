"""Bending beams: weightless jbeam beams that give pin joints back the bending of welded tubes (regulations 8.10).

Shared by the editor (which generates them) and the checker (which recomputes every
budget, so a declared value is never trusted).

For two member tubes J-A and J-C welded at joint J, the corner resists a change of
its angle through the bending of both tubes. Each tube, loaded at J with its far
end pinned, gives a rotational stiffness 3EI/L; the corner is the two in series:

    k_corner = 1 / (L_JA / 3EI_A + L_JC / 3EI_C)                 [N*m/rad]

A beam A-C at perpendicular distance h from J changes length by h * d(theta), so an
axial stiffness k gives the corner k * h^2. The bending beam therefore gets

    k_bend  <= k_corner / h^2                                      [N/m]
    F_bend  <= min(Mp_A, Mp_C) / h,   Mp = sigma_y * Z,  Z = (D^3 - d^3) / 6

and beamStrength <= 1.2 * F_bend (8.10.3). If several corners propose the same
beam A-C, their stiffness and strength add. Before export the stiffness is scaled
like the member tubes around it (8.10.5): times the smaller of their k_set / (E*A/L).

This corner model is the 8.10.3 budget, as the regulation states it. With the
corners of a closed frame braced it reproduces the real collapse load (for a
welded rectangle, the four-hinge 4 Mp / h) and errs soft on stiffness, because
each tube's far end is taken as pinned. Only round tubes are handled (Z above);
prism braces (fbeamBending: prism, 8.10.4) are not read yet.

Units: positions in m, E and sy in Pa, I in m^4, D and t in m; k in N/m, F in N.
"""

import math

MIN_ANGLE_DEG = 20.0          # corners flatter or sharper than this are not braced (collinear or folded back)
MIN_LEVER = 0.02              # m: a brace this close to the joint gives no useful lever (and k / h^2 blows up)
STRENGTH_FACTOR = 1.2         # beamStrength over beamDeform (8.10.3)


def plastic_modulus(D, t):
    """Plastic section modulus Z = (D^3 - d^3) / 6 of a round tube, in m^3 (D, t in m)."""
    d = D - 2 * t
    return (D ** 3 - d ** 3) / 6


def _lever(j, a, c):
    """Perpendicular distance from J to the line A-C."""
    ac = [c[i] - a[i] for i in range(3)]
    aj = [j[i] - a[i] for i in range(3)]
    L2 = sum(v * v for v in ac)
    if L2 <= 0:
        return 0.0
    t = sum(aj[i] * ac[i] for i in range(3)) / L2
    return math.dist(j, [a[i] + ac[i] * t for i in range(3)])


def corner_candidates(nodes, members):
    """Every braceable corner: [(joint, member_a, member_c, far_a, far_c)].

    nodes: {name: (x, y, z)}; members: dicts with "a", "b" (and "id" for budgets).
    A corner is two members sharing joint J whose far ends A, C are not already
    joined by a member and whose angle at J is within MIN_ANGLE_DEG of neither
    0 nor 180 degrees.
    """
    at = {}
    for m in members:
        at.setdefault(m["a"], []).append(m)
        at.setdefault(m["b"], []).append(m)
    tubes = {frozenset((m["a"], m["b"])) for m in members}
    out = []
    for j, ms in at.items():
        for i in range(len(ms)):
            for k in range(i + 1, len(ms)):
                ma, mc = ms[i], ms[k]
                a = ma["b"] if ma["a"] == j else ma["a"]
                c = mc["b"] if mc["a"] == j else mc["a"]
                if a == c or frozenset((a, c)) in tubes:
                    continue                                   # already a real triangle
                u = [nodes[a][q] - nodes[j][q] for q in range(3)]
                v = [nodes[c][q] - nodes[j][q] for q in range(3)]
                cos = sum(u[q] * v[q] for q in range(3)) / (math.hypot(*u) * math.hypot(*v))
                ang = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
                if MIN_ANGLE_DEG <= ang <= 180 - MIN_ANGLE_DEG:
                    out.append((j, ma, mc, a, c))
    return out


def budgets(nodes, members, section_of, material_of):
    """Budget of every bending beam: {(A, C): {"k", "F", "corners": [(J, id_a, id_c)], "members": [...]}}.

    section_of(member) -> (A, I, D, t) in m^2, m^4, m, m;  material_of(member) -> (E, sy) in Pa.
    Keys are the sorted far-end node names. k (N/m) is the real-stiffness budget,
    before the 8.10.5 scale; F (N) is the beamDeform budget; members are the
    sorted member ids around it. Corners with a lever under MIN_LEVER are skipped.
    """
    out = {}
    for j, ma, mc, a, c in corner_candidates(nodes, members):
        h = _lever(nodes[j], nodes[a], nodes[c])
        if h < MIN_LEVER:
            continue
        flex, Mp = 0.0, []
        for m, far in ((ma, a), (mc, c)):
            A, I, D, t = section_of(m)
            E, sy = material_of(m)
            flex += math.dist(nodes[j], nodes[far]) / (3 * E * I)      # rad per N*m, the two tubes in series
            Mp.append(sy * plastic_modulus(D, t))
        k_corner = 1.0 / flex
        key = tuple(sorted((a, c)))
        e = out.setdefault(key, {"k": 0.0, "F": 0.0, "corners": [], "members": set()})
        e["k"] += k_corner / (h * h)
        e["F"] += min(Mp) / h                                          # the weaker tube hinges first
        e["corners"].append((j, ma["id"], mc["id"]))
        e["members"].update((ma["id"], mc["id"]))
    for e in out.values():
        e["members"] = sorted(e["members"])
    return out
