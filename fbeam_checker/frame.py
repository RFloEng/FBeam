"""Small 3D structural solver for chassis rigidity (regulations 8.10.6, 9.4).

Pure Python, standard library only, so it runs in the browser through Pyodide.
Used by the editor (editor/fbeam_editor_core.py rigidity(), via editor/fbeam_frame.py)
to compare the welded tube frame with the pin-jointed jbeam model; the offline
checker does not call it yet (8.10.7). Linear statics, small displacements,
dense matrices: fine for a few hundred nodes.

Units: positions in m, E in Pa, A in m^2, I and J in m^4, k in N/m, forces in N.
DOF order per node: x, y, z translations, then (frame only) rotations about x, y, z.

Two element types:
  * frame: 3D Euler-Bernoulli beam, 6 DOF per node (welded tube frame, real bending)
  * truss: axial spring, 3 DOF per node (how jbeam behaves: pin joints)

Load cases use prescribed displacements, so no artificial supports are needed:
  * torsion: rear axle nodes held, front axle nodes moved +d / -d vertically
  * bending: axle nodes held vertically, a load at the two floor nodes nearest mid-wheelbase
"""

import math

G_OVER_E = 1 / 2.6   # G = E / (2 (1 + nu)) with nu = 0.3 (steel); also used for aluminium (nu 0.33)


# small 3-vector helpers on lists
def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _unit(a):
    n = math.sqrt(sum(x * x for x in a))
    return [x / n for x in a]


def _frame_element(p1, p2, E, A, I, J):
    """Global 12x12 stiffness of a 3D beam with a round section (Iy = Iz = I).

    Standard Euler-Bernoulli element (no shear deformation): axial EA/L, torsion
    GJ/L, bending 12EI/L^3, 6EI/L^2, 4EI/L, 2EI/L in each local plane. J is the
    polar moment (2I for a round tube). The local y axis is any normal to the
    beam, which is fine because the section is the same about every axis.
    """
    d = _sub(p2, p1)
    L = math.sqrt(sum(x * x for x in d))
    ex = _unit(d)
    ref = [0.0, 0.0, 1.0] if abs(ex[2]) < 0.95 else [1.0, 0.0, 0.0]   # any vector not parallel to the beam
    ey = _unit(_cross(ref, ex))
    ez = _cross(ex, ey)
    R = [ex, ey, ez]
    G = E * G_OVER_E
    k = [[0.0] * 12 for _ in range(12)]

    def put(i, j, v):
        k[i][j] += v
        if i != j:
            k[j][i] += v

    ea, gj = E * A / L, G * J / L
    b12, b6, b4, b2 = 12 * E * I / L ** 3, 6 * E * I / L ** 2, 4 * E * I / L, 2 * E * I / L
    put(0, 0, ea); put(6, 6, ea); put(0, 6, -ea)
    put(3, 3, gj); put(9, 9, gj); put(3, 9, -gj)
    # bending in the local x-y plane (v, theta_z) and x-z plane (w, theta_y)
    for v, t, s in ((1, 5, 1), (2, 4, -1)):
        put(v, v, b12); put(v + 6, v + 6, b12); put(v, v + 6, -b12)
        put(t, t, b4); put(t + 6, t + 6, b4); put(t, t + 6, b2)
        put(v, t, s * b6); put(v, t + 6, s * b6); put(v + 6, t, -s * b6); put(v + 6, t + 6, -s * b6)
    # rotate to global: K = T^T k T, T = blockdiag(R, R, R, R)
    T = [[0.0] * 12 for _ in range(12)]
    for blk in range(4):
        for i in range(3):
            for j in range(3):
                T[3 * blk + i][3 * blk + j] = R[i][j]
    kt = [[sum(k[i][m] * T[m][j] for m in range(12) if k[i][m]) for j in range(12)] for i in range(12)]
    return [[sum(T[m][i] * kt[m][j] for m in range(12) if T[m][i]) for j in range(12)] for i in range(12)]


def _truss_element(p1, p2, k_axial):
    """Global 6x6 stiffness of an axial spring k_axial (N/m): k e e^T blocks, e the unit direction."""
    e = _unit(_sub(p2, p1))
    blk = [[k_axial * e[i] * e[j] for j in range(3)] for i in range(3)]
    K = [[0.0] * 6 for _ in range(6)]
    for i in range(3):
        for j in range(3):
            K[i][j], K[i + 3][j + 3] = blk[i][j], blk[i][j]
            K[i][j + 3], K[i + 3][j] = -blk[i][j], -blk[i][j]
    return K


class Structure:
    """Assembled global stiffness matrix of a frame or truss model.

    nodes: {name: (x, y, z)} in m. members: list of dicts with a, b (node names)
    and either E, A, I, J for kind "frame" or k (N/m) for kind "truss". Only
    nodes used by a member get degrees of freedom; DOF i of node n is row
    dof * index[n] + i.
    """

    def __init__(self, nodes, members, kind):
        self.kind = kind
        self.dof = 6 if kind == "frame" else 3
        self.names = sorted({m["a"] for m in members} | {m["b"] for m in members})
        self.index = {n: i for i, n in enumerate(self.names)}
        n = self.dof * len(self.names)
        self.n = n
        self.K = [[0.0] * n for _ in range(n)]
        for m in members:
            pa, pb = nodes[m["a"]], nodes[m["b"]]
            ke = (_frame_element(pa, pb, m["E"], m["A"], m["I"], m["J"]) if kind == "frame"
                  else _truss_element(pa, pb, m["k"]))
            ids = [self.dof * self.index[m["a"]] + i for i in range(self.dof)] + \
                  [self.dof * self.index[m["b"]] + i for i in range(self.dof)]
            for i, gi in enumerate(ids):
                row = self.K[gi]
                for j, gj in enumerate(ids):
                    row[gj] += ke[i][j]
        self.nodes = nodes

    def solve(self, fixed, forces=None):
        """fixed: {(node, dof): displacement}; forces: {(node, dof): force}.

        Displacements in m (rad for rotations), forces in N (N*m for moments).
        Returns (displacements, reactions, mechanisms): u is the full DOF vector,
        reactions {global DOF: force} at the prescribed DOFs. Mechanisms (zero
        stiffness modes of a pin-jointed structure) are held by a tiny spring
        and returned as a list of (joint, axis) where the solver found them.

        Prescribed DOFs move to the right-hand side (K_ff u_f = F_f - K_fp u_p),
        then K_ff is solved by Cholesky; a pivot below 1e-9 of the largest
        diagonal counts as a mechanism and is replaced by that small value.
        """
        d = self.dof
        pres = {d * self.index[nm] + i: v for (nm, i), v in fixed.items() if nm in self.index}
        free = [i for i in range(self.n) if i not in pres]
        F = [0.0] * self.n
        for (nm, i), v in (forces or {}).items():
            if nm in self.index:
                F[d * self.index[nm] + i] += v
        rhs = [F[i] - sum(self.K[i][p] * v for p, v in pres.items()) for i in free]
        A = [[self.K[i][j] for j in free] for i in free]
        diag_max = max((A[i][i] for i in range(len(free))), default=1.0) or 1.0
        eps, mech = 1e-9 * diag_max, []
        # Cholesky in place: A = L L^T, L in the lower triangle
        nf = len(free)
        for j in range(nf):
            Aj = A[j]
            s = Aj[j] - sum(Aj[k] * Aj[k] for k in range(j))
            if s <= eps:
                mech.append((self.names[free[j] // d], "xyz"[free[j] % d] if free[j] % d < 3 else "r" + "xyz"[free[j] % d - 3]))
                s = eps
            Aj[j] = math.sqrt(s)
            inv = 1.0 / Aj[j]
            for i in range(j + 1, nf):
                Ai = A[i]
                Ai[j] = (Ai[j] - sum(Ai[k] * Aj[k] for k in range(j))) * inv
        # forward (L y = rhs) and back (L^T x = y) substitution
        y = [0.0] * nf
        for i in range(nf):
            y[i] = (rhs[i] - sum(A[i][k] * y[k] for k in range(i))) / A[i][i]
        x = [0.0] * nf
        for i in range(nf - 1, -1, -1):
            x[i] = (y[i] - sum(A[k][i] * x[k] for k in range(i + 1, nf))) / A[i][i]
        u = [0.0] * self.n
        for i, v in pres.items():
            u[i] = v
        for k, i in enumerate(free):
            u[i] = x[k]
        # reaction = K u - applied load at each prescribed DOF
        reac = {p: sum(self.K[p][j] * u[j] for j in range(self.n)) - F[p] for p in pres}
        return u, reac, mech


def torsion(struct, front, rear, d=0.001):
    """Torsional stiffness in N*m/deg: rear axle held, front axle twisted (as the 9.4 test).

    front, rear: (left, right) node names of the axle joints. The rear nodes are
    held in x, y and z; the front nodes are moved +d and -d in z (d in m, small).
    Torque = sum of vertical reactions times their x; twist angle = 2d / track
    (small angle). Returns (stiffness, mechanisms).
    """
    fl, fr = front
    fixed = {(n, i): 0.0 for n in rear for i in range(3)}
    fixed[(fl, 2)], fixed[(fr, 2)] = d, -d
    u, reac, mech = struct.solve(fixed)
    dof = struct.dof
    xl, xr = struct.nodes[fl][0], struct.nodes[fr][0]
    torque = reac[dof * struct.index[fl] + 2] * xl + reac[dof * struct.index[fr] + 2] * xr
    theta = 2 * d / abs(xl - xr)
    return abs(torque / math.degrees(theta)), mech


def bending(struct, front, rear, load_nodes, F=1000.0):
    """Vertical bending stiffness in N/mm: axles held vertically, load at mid-wheelbase.

    Rear nodes held in x, y, z; front nodes held in z, and front[0] also in y.
    F (N, default 1 kN) is shared over load_nodes (the editor picks the lowest
    node on each side nearest mid-wheelbase) and pushes down (-z). Stiffness =
    F / mean sag. Returns (stiffness, mechanisms); inf if the nodes do not sag.
    """
    fixed = {(n, i): 0.0 for n in rear for i in range(3)}
    for n in front:
        fixed[(n, 2)] = 0.0
    fixed[(front[0], 1)] = 0.0          # stop fore-aft sliding of the front axle
    if not load_nodes:
        raise ValueError("bending: no load nodes (needs floor joints off the centre line)")
    forces = {(n, 2): -F / len(load_nodes) for n in load_nodes}
    u, reac, mech = struct.solve(fixed, forces)
    dof = struct.dof
    sag = -sum(u[dof * struct.index[n] + 2] for n in load_nodes) / len(load_nodes)
    return (F / (sag * 1000) if sag > 0 else None), mech       # None: no sag (JSON has no infinity)


def point_load(struct, supports, load_nodes, axis, F=1000.0):
    """Stiffness in N/mm under a load at load_nodes along axis (0 = x sideways, 2 = z up; F < 0 pushes
    down), with every support node held in x, y and z: the sideways hoop load and the roof load of 8.10.6.

    F (N, default 1 kN) is shared over load_nodes. Stiffness = |F| / mean displacement along the
    load (mm). Returns (stiffness or None when the nodes do not move, mechanisms).
    """
    if not load_nodes:
        raise ValueError("point load: no load nodes")
    fixed = {(n, i): 0.0 for n in supports for i in range(3)}
    forces = {(n, axis): F / len(load_nodes) for n in load_nodes}
    u, reac, mech = struct.solve(fixed, forces)
    dof = struct.dof
    move = sum(u[dof * struct.index[n] + axis] for n in load_nodes) / len(load_nodes) * (1 if F > 0 else -1)
    return (abs(F) / (move * 1000) if move > 0 else None), mech
