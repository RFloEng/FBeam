"""Buckling length of split tubes (regulations 8.9.7), shared by the checker and the editor.

A tube can be written as several jbeam beams in a line. Each segment is shorter than
the tube, so on its own it would get a higher Euler buckling load (pi^2 E I / L^2),
and splitting a tube would make it stronger. It does not: a joint only braces a tube
if something else is attached there. So a run is followed through every joint where
exactly two tubes meet and the line carries on almost straight (it turns by at most
MAX_TURN_DEG, the chord of a bent tube included); its whole length is the buckling
length of every segment in it. A third tube at the joint, or a sharp corner, ends it.

Pure Python, standard library only. Lengths in metres.
"""

import math
from collections import defaultdict

MAX_TURN_DEG = 30.0     # a bend sharper than this ends the run (a corner, not a chord of a bent tube)


def buckling_lengths(pos, members, max_turn_deg=MAX_TURN_DEG):
    """Unbraced length of every member.

    pos: {node: (x, y, z)}. members: [(key, node_a, node_b)], the tubes that can brace each
    other (frame tubes only: not bending beams, springs or suspension links).
    Returns {key: buckling length (m)}, never shorter than the member itself.
    """
    at = defaultdict(list)
    for m in members:
        at[m[1]].append(m)
        at[m[2]].append(m)
    length = {m[0]: math.dist(pos[m[1]], pos[m[2]]) for m in members}
    cos_max = math.cos(math.radians(max_turn_deg))

    def other(m, node):
        return m[2] if m[1] == node else m[1]

    def carries_on(node, into, out_):
        """True if the line from tube `into` through `node` into tube `out_` turns by max_turn_deg or less."""
        u = [n - p for n, p in zip(pos[node], pos[other(into, node)])]
        v = [q - n for q, n in zip(pos[other(out_, node)], pos[node])]
        nu, nv = math.hypot(*u), math.hypot(*v)
        return nu > 0 and nv > 0 and sum(x * y for x, y in zip(u, v)) / (nu * nv) >= cos_max

    out = {}
    for m in members:
        total, seen = length[m[0]], {m[0]}
        for start in (m[1], m[2]):
            node, prev = start, m
            while len(at[node]) == 2:
                nxt = at[node][0] if at[node][1][0] == prev[0] else at[node][1]
                if nxt[0] in seen or not carries_on(node, prev, nxt):
                    break
                seen.add(nxt[0])
                total += length[nxt[0]]
                node, prev = other(nxt, node), nxt
        out[m[0]] = total
    return out
