"""Declared structural sections and the limits they allow (regulations 8.2-8.5).

A section is declared on a beam property row as "fbeamSection", for example:
    "tube 45x2.5 steel_1018"    round tube, outer diameter 45 mm, wall 2.5 mm
    "sqtube 40x2 al_6061_t6"    square tube, side 40 mm, wall 2 mm
    "sheet 0.8x60 steel_1018"   sheet 0.8 mm thick, 60 mm tributary width (8.5)

Dimensions are written in mm; everything inside is SI (m, m^2, m^4, Pa, N, kg).
Materials and the minimum walls (8.4) come from the rules file. Section.limits()
gives the 8.3 maxima for one beam: beamSpring E*A/L, beamDeform min(sy*A, Euler
buckling), beamStrength su*A, and the mass rho*A*L its end nodes must carry.
Used by the checker and, through parse_section, by the editor and examples.
"""

import math
from dataclasses import dataclass


class SectionError(ValueError):
    """An fbeamSection string that is malformed or outside the 8.4 catalogue."""


@dataclass(frozen=True)
class Material:
    """One catalogue material (8.4), in SI units."""

    name: str
    E: float      # Pa
    sy: float     # Pa, yield
    su: float     # Pa, ultimate
    rho: float    # kg/m^3


@dataclass(frozen=True)
class Section:
    """A parsed fbeamSection: kind is "tube", "sqtube" or "sheet"; A and I in SI."""

    text: str
    kind: str
    material: Material
    A: float      # m^2
    I: float      # m^4, minimum second moment

    def limits(self, L, L_buckle=None):
        """Maximum jbeam values and mass for a beam of rest length L (m), regulations 8.3.

        L_buckle is the unbraced length for buckling (regulations 8.9.7); it
        defaults to L and is longer when a tube runs through unbraced joints.
        The checker always passes L alone (split tubes are not grouped yet).

        Returns beamSpring (N/m), beamDeform, beamStrength, yield_force and
        buckling_force (N), and mass (kg, the beam's rho*A*L).
        """
        m = self.material
        k_max = m.E * self.A / L
        f_yield = m.sy * self.A
        f_buckle = math.pi ** 2 * m.E * self.I / (L_buckle or L) ** 2     # Euler, pinned-pinned (K = 1): pi^2 E I / L^2
        return {
            "beamSpring": k_max,
            "beamDeform": min(f_yield, f_buckle),
            "beamStrength": m.su * self.A,
            "mass": m.rho * self.A * L,
            "yield_force": f_yield,
            "buckling_force": f_buckle,
        }


def load_materials(rules):
    """{name: Material} from rules["materials"] (E in GPa, stresses in MPa, rho in kg/m^3)."""
    out = {}
    for name, p in rules["materials"].items():
        out[name] = Material(name, p["E_GPa"] * 1e9, p["sy_MPa"] * 1e6, p["su_MPa"] * 1e6, p["rho_kg_m3"])
    return out


def parse_section(text, materials, rules):
    """Parse an fbeamSection string into a Section (A in m^2, I in m^4).

    materials comes from load_materials(); rules supplies the minimum walls and
    the maximum sheet tributary width. Raises SectionError if the string is
    malformed, names an unknown material, or breaks the 8.4 / 8.5 limits.
    """
    parts = text.split()
    if len(parts) != 3:
        raise SectionError(f"section {text!r}: expected '<kind> <dims> <material>'")
    kind, dims, mat_name = parts
    if mat_name not in materials:
        raise SectionError(f"section {text!r}: unknown material {mat_name!r}")
    try:
        a, b = (float(x) / 1000.0 for x in dims.lower().split("x"))
    except ValueError:
        raise SectionError(f"section {text!r}: dimensions must be like 45x2.5 (mm)")
    min_wall = rules["sections"]["min_wall_mm"]
    if kind == "tube":
        D, t = a, b                                 # 1e-9 below: float slack so a wall of exactly the minimum passes
        if t * 1000 < min_wall["tube"] - 1e-9 or 2 * t >= D:
            raise SectionError(f"section {text!r}: wall must be >= {min_wall['tube']} mm and < D/2")
        d = D - 2 * t
        A = math.pi / 4 * (D ** 2 - d ** 2)
        I = math.pi / 64 * (D ** 4 - d ** 4)        # hollow circle, same about every axis
    elif kind == "sqtube":
        B, t = a, b                                 # square tubes share the tube minimum wall
        if t * 1000 < min_wall["tube"] - 1e-9 or 2 * t >= B:
            raise SectionError(f"section {text!r}: wall must be >= {min_wall['tube']} mm and < B/2")
        bi = B - 2 * t
        A = B ** 2 - bi ** 2
        I = (B ** 4 - bi ** 4) / 12                 # hollow square about a side-parallel axis
    elif kind == "sheet":
        t, s = a, b
        if t * 1000 < min_wall["sheet"] - 1e-9:
            raise SectionError(f"section {text!r}: sheet must be >= {min_wall['sheet']} mm")
        if s * 1000 > rules["sections"]["max_sheet_tributary_mm"]:
            raise SectionError(f"section {text!r}: tributary width over {rules['sections']['max_sheet_tributary_mm']} mm")
        A = t * s                                   # 8.5: thickness times tributary width
        I = s * t ** 3 / 12                         # weak axis (out of the sheet plane), the minimum
    else:
        raise SectionError(f"section {text!r}: kind must be tube, sqtube or sheet")
    return Section(text, kind, materials[mat_name], A, I)
