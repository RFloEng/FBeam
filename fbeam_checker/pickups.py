"""Suspension pickups on the frame (regulations 7.1.6), shared by the checker and the editor.

jbeam joints are pins, so a load can only enter the frame at a joint, and the joint only
holds it in every direction if the tubes meeting there are not all in one plane. A pickup
part-way along a tube would need the tube to carry it in bending, which a pin-jointed
jbeam frame cannot do.

The test: for unit tube directions a, b, c at the joint, |a . (b x c)| is the
volume of the box they span (0 when coplanar, 1 when orthogonal); at least one
triple of tubes must reach MIN_SPAN. checks.check_pickups applies it to every
frame joint another part loads; the editor uses the same functions and
mirror_name for left / right joints.
"""

import math
import re

MIN_SPAN = 0.10          # |a . (b x c)| of three unit tube directions; 0 = coplanar, 1 = orthogonal (7.1.6)

_BEAMNG_SIDE = re.compile(r"^([a-z]+\d+)(ll|rr|l|r)$")      # BeamNG node names: f10ll / f10rr, fx2l / fx2r
_SWAP = {"ll": "rr", "rr": "ll", "l": "r", "r": "l"}


def mirror_name(n):
    """Name of the mirrored joint: FBeam's _L / _R, or the BeamNG l / r, ll / rr suffixes.

    Names with no side suffix (centreline joints) are returned unchanged.
    """
    if n.endswith("_L"):
        return n[:-2] + "_R"
    if n.endswith("_R"):
        return n[:-2] + "_L"
    m = _BEAMNG_SIDE.match(n)
    return m.group(1) + _SWAP[m.group(2)] if m else n


def spans_3d(directions, min_span=MIN_SPAN):
    """Best |a . (b x c)| over all triples of unit directions, and whether it reaches min_span.

    directions need not be unit length; near-zero vectors are ignored. Fewer
    than three directions give (0.0, False) unless min_span <= 0.
    """
    u = []
    for d in directions:
        n = math.sqrt(sum(c * c for c in d))
        if n > 1e-9:
            u.append([c / n for c in d])
    best = 0.0
    for i in range(len(u)):
        for j in range(i + 1, len(u)):
            a, b = u[i], u[j]
            for k in range(j + 1, len(u)):
                c = u[k]
                cr = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]]
                best = max(best, abs(sum(x * y for x, y in zip(a, cr))))
    return best, best >= min_span


def joint_bracing(node, pos, tube_ends):
    """7.1.6 test for one joint at pos (m).

    tube_ends: other-end positions of the frame member tubes meeting at node.
    Returns (count, span, ok): number of tubes, best triple product, and whether
    there are at least three tubes reaching MIN_SPAN. node (the joint name) is not used in the test.
    """
    dirs = [[q - p for q, p in zip(other, pos)] for other in tube_ends]
    span, ok = spans_3d(dirs)
    return len(dirs), span, ok and len(dirs) >= 3
