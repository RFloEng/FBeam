"""Draw docs/img/editor-mockup.svg from the real cage model in examples/cage.

    python docs/img/make_editor_mockup.py
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "examples" / "cage"))
sys.path.insert(0, str(ROOT))

import cage_calc as cc  # noqa: E402
from fbeam_checker.sections import parse_section  # noqa: E402

W, H = 1280, 760
INK, QUIET, LINE, PANEL, BG = "#1f2328", "#59636e", "#d0d7de", "#f6f8fa", "#ffffff"
YIELD, BUCKLE, SELECT, ENVELOPE = "#2f6fdf", "#e0782a", "#111111", "#8c959f"

rows, node_mass = cc.design(cc.OPTIONS["recommended"])
sel = next(r for r in rows if r["a"] == "APf_L" and r["b"] == "MHm_L")   # upper door bar, left
alt = parse_section("tube 38x2.5 steel_1018", cc.MATERIALS, cc.RULES).limits(sel["L"])

# viewport projection: yaw about Z, then pitch
VX, VY, VW, VH = 250, 64, 760, 470
YAW, PITCH, SCALE = math.radians(-35), math.radians(24), 200


def proj(p):
    x, y, z = p
    x1 = x * math.cos(YAW) - y * math.sin(YAW)
    y1 = x * math.sin(YAW) + y * math.cos(YAW)
    sx = VX + VW * 0.55 + SCALE * x1
    sy = VY + VH * 0.80 - SCALE * (z * math.cos(PITCH) + y1 * math.sin(PITCH))
    return sx, sy


out = []
a = out.append
a(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" '
  f'font-family="Segoe UI, Helvetica, Arial, sans-serif" font-size="13">')
a(f'<rect width="{W}" height="{H}" fill="{BG}"/>')

# top bar
a(f'<rect x="0" y="0" width="{W}" height="48" fill="{PANEL}" stroke="{LINE}"/>')
a(f'<text x="16" y="30" font-size="15" font-weight="600" fill="{INK}">FBeam car editor</text>')
a(f'<text x="166" y="30" fill="{QUIET}">demo_cage.fbeam.json</text>')
spaces = ["Chassis", "Body", "Suspension", "Powertrain", "Cockpit", "Aero", "Build"]
for i, t in enumerate(spaces):
    x = 360 + i * 92
    on = t == "Chassis"
    a(f'<rect x="{x}" y="10" width="86" height="28" rx="6" fill="{"#dbe7fb" if on else BG}" stroke="{YIELD if on else LINE}"/>')
    a(f'<text x="{x + 43}" y="29" text-anchor="middle" fill="{INK}" {"font-weight=\"600\"" if on else ""}>{t}</text>')
a(f'<rect x="1030" y="10" width="234" height="28" rx="6" fill="{YIELD}"/><text x="1147" y="29" text-anchor="middle" fill="#ffffff" font-weight="600">Build car + check + test in game</text>')

# left outline
a(f'<rect x="0" y="48" width="240" height="{H - 48 - 150}" fill="{PANEL}" stroke="{LINE}"/>')
a(f'<text x="16" y="76" font-weight="600" fill="{INK}">Car: FB-CAGE</text>')
car_tree = [("Chassis: safety cage", "18 tubes", INK), ("Body shell", "not started", QUIET),
            ("Suspension F / R", "not started", QUIET), ("Steering", "not started", QUIET),
            ("Wheels + tyres", "spec part", QUIET), ("Engine + gearbox", "spec part", QUIET),
            ("Brakes", "not started", QUIET), ("Cockpit, cameras", "not started", QUIET),
            ("Aero", "none", QUIET)]
y = 100
for name, status, col in car_tree:
    a(f'<text x="16" y="{y}" font-size="12" fill="{col}" {"font-weight=\"600\"" if col == INK else ""}>{name}</text>')
    a(f'<text x="226" y="{y}" font-size="11.5" text-anchor="end" fill="{QUIET}">{status}</text>')
    y += 21
a(f'<line x1="12" y1="{y - 8}" x2="228" y2="{y - 8}" stroke="{LINE}"/>')
a(f'<text x="16" y="{y + 12}" font-weight="600" fill="{INK}">Chassis members</text>')
groups = {}
for r in rows:
    groups.setdefault(r["group"], []).append(r)
y += 36
for g, rs in groups.items():
    tube = rs[0]["tube"].replace("x", " × ")
    bold = g == sel["group"]
    if bold:
        a(f'<rect x="8" y="{y - 15}" width="224" height="21" rx="4" fill="#dbe7fb"/>')
    a(f'<text x="16" y="{y}" font-size="12" fill="{INK}" {"font-weight=\"600\"" if bold else ""}>{g}{" ×" + str(len(rs)) if len(rs) > 1 else ""}</text>')
    a(f'<text x="226" y="{y}" font-size="11.5" text-anchor="end" fill="{QUIET}">{tube}</text>')
    y += 21
a(f'<text x="16" y="{y + 8}" font-size="12" fill="{BUCKLE}">Open bay: windscreen frame</text>')

# viewport
a(f'<rect x="{VX - 10}" y="{VY - 16}" width="{VW + 20}" height="{VH + 16}" fill="#fbfcfd" stroke="{LINE}"/>')
for gx in range(-8, 9):
    p1, p2 = proj((gx * 0.25, -1.4, 0.0)), proj((gx * 0.25, 1.6, 0.0))
    a(f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" stroke="#eef1f4"/>')
for gy in range(-7, 9):
    p1, p2 = proj((-2.0, gy * 0.2, 0.0)), proj((2.0, gy * 0.2, 0.0))
    a(f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" stroke="#eef1f4"/>')
# open bay: windscreen frame
N = cc.NODES
bay = [proj(N[k]) for k in ("APf_L", "APt_L", "APt_R", "APf_R")]
a('<polygon points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in bay) + f'" fill="{BUCKLE}" fill-opacity="0.10" stroke="{BUCKLE}" stroke-dasharray="5 4"/>')
# tubes, far ones first
order = sorted(rows, key=lambda r: -(proj(N[r["a"]])[1] + proj(N[r["b"]])[1]))
for r in sorted(rows, key=lambda r: (N[r["a"]][0] + N[r["b"]][0])):
    p, q = proj(N[r["a"]]), proj(N[r["b"]])
    w = float(r["tube"].split("x")[0]) / 6.5
    col = BUCKLE if r["governs"] == "buckling" else YIELD
    if r is sel:
        a(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{SELECT}" stroke-width="{w + 5:.1f}" stroke-linecap="round"/>')
    a(f'<line x1="{p[0]:.1f}" y1="{p[1]:.1f}" x2="{q[0]:.1f}" y2="{q[1]:.1f}" stroke="{col}" stroke-width="{w:.1f}" stroke-linecap="round"/>')
for n, pos in N.items():
    x, y = proj(pos)
    rad = 2.5 + 1.6 * math.sqrt(node_mass[n])
    a(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{rad:.1f}" fill="#ffffff" stroke="{INK}" stroke-width="1.2"/>')
sx, sy = proj(N["MHt_L"])
a(f'<text x="{sx + 10:.0f}" y="{sy - 10:.0f}" font-size="11.5" fill="{QUIET}">MHt_L  {node_mass["MHt_L"]:.2f} kg</text>')
# viewport legend
lx, ly = VX + 6, VY + 6
a(f'<text x="{lx}" y="{ly + 4}" font-size="12" font-weight="600" fill="{INK}">Colour: governing failure mode</text>')
a(f'<line x1="{lx}" y1="{ly + 22}" x2="{lx + 26}" y2="{ly + 22}" stroke="{YIELD}" stroke-width="6" stroke-linecap="round"/><text x="{lx + 34}" y="{ly + 26}" font-size="12" fill="{INK}">yield</text>')
a(f'<line x1="{lx + 90}" y1="{ly + 22}" x2="{lx + 116}" y2="{ly + 22}" stroke="{BUCKLE}" stroke-width="6" stroke-linecap="round"/><text x="{lx + 124}" y="{ly + 26}" font-size="12" fill="{INK}">buckling</text>')
a(f'<text x="{lx}" y="{ly + 46}" font-size="11.5" fill="{QUIET}">Tube width = diameter · node size = mass · dashed = open bay</text>')

# right inspector
IX = 1020
a(f'<rect x="{IX}" y="48" width="{W - IX}" height="{H - 48 - 150}" fill="{PANEL}" stroke="{LINE}"/>')
a(f'<text x="{IX + 16}" y="76" font-weight="600" fill="{INK}">Tube: {sel["group"]} (left)</text>')
a(f'<text x="{IX + 16}" y="100" font-size="12" fill="{QUIET}">Section</text>')
a(f'<rect x="{IX + 16}" y="108" width="228" height="28" rx="6" fill="{BG}" stroke="{LINE}"/>')
a(f'<text x="{IX + 26}" y="127" fill="{INK}">tube 40 × 2.0 steel_1018  ▾</text>')
fields = [
    ("Length", f'{sel["L"]:.2f} m'), ("Mass", f'{sel["mass"]:.2f} kg'),
    ("E·A/L", f'{sel["beamSpring"] / 1e7:.2f} × 10⁷ N/m'), ("Spring set (stability)", f'{sel["k_set"] / 1e6:.1f} × 10⁶ N/m'),
    ("Yield", f'{sel["yield_force"] / 1e3:.1f} kN'), ("Buckling", f'{sel["buckling_force"] / 1e3:.1f} kN'),
    ("beamDeform", f'{sel["beamDeform"] / 1e3:.1f} kN (buckling)'), ("beamStrength", f'{sel["beamStrength"] / 1e3:.1f} kN'),
]
y = 162
for k, v in fields:
    a(f'<text x="{IX + 16}" y="{y}" font-size="12" fill="{QUIET}">{k}</text><text x="{W - 16}" y="{y}" font-size="12" text-anchor="end" fill="{INK}">{v}</text>')
    y += 22
a(f'<text x="{IX + 16}" y="{y + 6}" font-size="11.5" fill="{QUIET}">All values derived; none editable.</text>')
dm = alt["mass"] - sel["mass"]
db = (alt["beamDeform"] - sel["beamDeform"]) / sel["beamDeform"] * 100
a(f'<rect x="{IX + 12}" y="{y + 20}" width="{W - IX - 24}" height="62" rx="6" fill="#fff8f0" stroke="{BUCKLE}"/>')
a(f'<text x="{IX + 22}" y="{y + 40}" font-size="12" font-weight="600" fill="{INK}">Try 38 × 2.5 instead</text>')
a(f'<text x="{IX + 22}" y="{y + 58}" font-size="12" fill="{INK}">{dm:+.2f} kg, buckling {db:+.0f} %</text>')
a(f'<text x="{IX + 22}" y="{y + 74}" font-size="11.5" fill="{QUIET}">Keep 40 × 2.0: same buckling for less mass</text>')

# bottom: budget bar and issues
BY = H - 150
a(f'<rect x="0" y="{BY}" width="{W}" height="150" fill="{PANEL}" stroke="{LINE}"/>')
tube_mass = sum(r["mass"] for r in rows)
nodes_total = sum(node_mass.values())
budget = [("Tube mass", f"{tube_mass:.1f} kg"), ("Node mass incl. joints", f"{nodes_total:.1f} kg"),
          ("Weakest member", f'{min(r["beamDeform"] for r in rows) / 1e3:.1f} kN'),
          ("Race Mass vs FIA min.", "needs spec parts / 1,100 kg")]
for i, (k, v) in enumerate(budget):
    x = 16 + i * 250
    a(f'<text x="{x}" y="{BY + 26}" font-size="11.5" fill="{QUIET}">{k}</text><text x="{x}" y="{BY + 48}" font-size="15" font-weight="600" fill="{INK}">{v}</text>')
a(f'<line x1="0" y1="{BY + 62}" x2="{W}" y2="{BY + 62}" stroke="{LINE}"/>')
a(f'<text x="16" y="{BY + 84}" font-weight="600" fill="{INK}">Issues</text>')
issues = [("WARN", "8.10", f"Windscreen frame is an open bay: add a bending beam (0 kg, capped) or a real 40 × 2.0 tube (+{math.dist(N['APf_L'], N['APt_R']) * sel['mass'] / sel['L']:.1f} kg)"),
          ("WARN", "6.1", "No hub hardpoints loaded: wheelbase not checked"),
          ("INFO", "8.9.3", f"{sum(1 for r in rows if r['governs'] == 'buckling')} of {len(rows)} tubes are buckling-governed; larger-diameter tubes save mass there")]
for i, (lvl, rule, msg) in enumerate(issues):
    yy = BY + 106 + i * 18
    col = BUCKLE if lvl == "WARN" else QUIET
    a(f'<text x="16" y="{yy}" font-size="12" font-weight="600" fill="{col}">{lvl}</text><text x="64" y="{yy}" font-size="12" fill="{QUIET}">[{rule}]</text><text x="118" y="{yy}" font-size="12" fill="{INK}">{msg}</text>')
a(f'<text x="{W - 16}" y="{H - 10}" font-size="11" text-anchor="end" fill="{QUIET}">Mock-up drawn from examples/cage (recommended option); numbers are the real calculation</text>')
a("</svg>")

path = Path(__file__).with_name("editor-mockup.svg")
path.write_text("\n".join(out), encoding="utf-8", newline="\n")
n_buckle = sum(1 for r in rows if r["governs"] == "buckling")
diag = math.dist(N["APf_L"], N["APt_R"]) * sel["mass"] / sel["L"]
print(f"wrote {path}; buckling-governed {n_buckle}/{len(rows)}; windscreen diagonal 40x2 mass {diag:.2f} kg; alt dm {dm:+.2f} db {db:+.1f}%")
