import json
import math
import shutil
import tempfile
import unittest
import zipfile
from pathlib import Path

from fbeam_checker import jbeam
from fbeam_checker.checks import ERROR, run_all
from fbeam_checker.sections import load_materials, parse_section
from fbeam_checker.submission import Submission

ROOT = Path(__file__).resolve().parent.parent
DEMO = ROOT / "examples" / "demo_car"
JBEAM = "vehicles/fbeam_demo/fbeam_demo_chassis.jbeam"
RULES = json.loads((ROOT / "rules" / "fbeam_v1.json").read_text(encoding="utf-8"))


def errors(findings):
    return [f for f in findings if f.level == ERROR]


class TestJBeamParser(unittest.TestCase):
    def test_comments_and_optional_commas(self):
        doc = jbeam.parse('{"a": [1 2, 3,], /* c */ "b": {"x": "y",} // end\n "c": -1.5e3}')
        self.assertEqual(doc, {"a": [1, 2, 3], "b": {"x": "y"}, "c": -1500.0})

    def test_leading_plus_sign(self):
        self.assertEqual(jbeam.parse('[+0.5, +2, -1, 1e+3]'), [0.5, 2, -1, 1000.0])

    def test_hostile_and_non_numeric_values_are_unresolved(self):
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.resolve("$=9**9**9**9", {})                       # would hang eval()
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.to_float("abc", {})
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.resolve("$=$k*2", {"$k": "soft"})

    def test_table_properties_and_inline_override(self):
        rows = [["id", "posX"], {"nodeWeight": 2}, ["a", 0], ["b", 1, {"nodeWeight": 5}], ["c", 2]]
        recs = jbeam.expand_table(rows)
        self.assertEqual([r["nodeWeight"] for r in recs], [2, 5, 2])

    def test_variables(self):
        self.assertEqual(jbeam.resolve("$k", {"$k": 3}), 3)
        self.assertAlmostEqual(jbeam.resolve("$=$k*2+1", {"$k": 3}), 7.0)
        self.assertTrue(math.isinf(jbeam.to_float("FLT_MAX", {})))
        with self.assertRaises(jbeam.UnresolvedValue):
            jbeam.resolve("$=os.system('x')", {})


class TestSectionMaths(unittest.TestCase):
    def test_worked_example_from_regulations_8_6(self):
        sec = parse_section("tube 45x2.5 steel_1018", load_materials(RULES), RULES)
        lim = sec.limits(0.60)
        self.assertAlmostEqual(sec.A * 1e6, 334, delta=0.5)          # mm^2
        self.assertAlmostEqual(sec.I * 1e12, 75626, delta=5)         # mm^4
        self.assertAlmostEqual(lim["beamSpring"], 1.14e8, delta=0.01e8)
        self.assertAlmostEqual(lim["beamDeform"], 121.8e3, delta=0.2e3)   # yield governs
        self.assertAlmostEqual(lim["buckling_force"], 425e3, delta=1e3)
        self.assertAlmostEqual(lim["beamStrength"], 147e3, delta=0.5e3)
        self.assertAlmostEqual(lim["mass"], 1.572, delta=0.002)

    def test_thin_wall_rejected(self):
        with self.assertRaises(ValueError):
            parse_section("tube 30x1.0 steel_1018", load_materials(RULES), RULES)


class TestSubmission(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.car = self.tmp / "car"
        shutil.copytree(DEMO, self.car)

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def edit(self, old, new, path=JBEAM):
        f = self.car / path
        text = f.read_text(encoding="utf-8")
        self.assertIn(old, text)
        f.write_text(text.replace(old, new, 1), encoding="utf-8")

    def check(self):
        return run_all(Submission(self.car), RULES)

    def test_demo_passes(self):
        findings, stats = self.check()
        self.assertEqual(errors(findings), [])
        self.assertEqual(stats["dimensions"]["wheelbase_mm"], 2500)

    def test_unbreakable_beam_fails(self):
        self.edit('"beamStrength": 80000', '"beamStrength": "FLT_MAX"')
        self.assertTrue(any("unbreakable" in f.message for f in errors(self.check()[0])))

    def test_light_node_fails(self):
        self.edit('{"nodeWeight": 9.0}', '{"nodeWeight": 4.0}')
        self.assertTrue(any("node ml1" in f.message and "required" in f.message for f in errors(self.check()[0])))

    def test_hidden_ballast_fails(self):
        self.edit('{"nodeWeight": 3.8}', '{"nodeWeight": 30.0}')
        self.assertTrue(any("ballast" in f.message for f in errors(self.check()[0])))

    def test_zero_node_weight_is_reported_not_a_crash(self):
        self.edit('{"nodeWeight": 9.0}', '{"nodeWeight": 0}')
        self.assertTrue(any("node ml1" in f.message for f in errors(self.check()[0])))

    def test_non_numeric_beam_value_is_reported_not_a_crash(self):
        self.edit('"beamDeform": 30000', '"beamDeform": "soft"')
        self.assertTrue(any(f.rule == "2.2.5" for f in errors(self.check()[0])))

    def test_split_tube_buckles_over_its_whole_run(self):
        # a 1.5 m tube split at an unbraced, almost straight joint m: each half buckles over the whole run (8.9.7)
        from fbeam_checker.buckling import buckling_lengths
        pos = {"a": (0, 0, 0), "m": (0, 0.75, 0.01), "b": (0, 1.5, 0), "c": (1, 1.5, 0)}
        L = buckling_lengths(pos, [("s1", "a", "m"), ("s2", "m", "b"), ("x", "b", "c")])
        self.assertAlmostEqual(L["s1"], math.dist(pos["a"], pos["m"]) + math.dist(pos["m"], pos["b"]), places=9)
        self.assertAlmostEqual(L["x"], 1.0, places=9)                      # the 90 degree corner ends the run
        L2 = buckling_lengths(pos, [("s1", "a", "m"), ("s2", "m", "b"), ("y", "m", "c")])
        self.assertAlmostEqual(L2["s1"], math.dist(pos["a"], pos["m"]), places=9)   # a third tube braces the joint

    def test_undeclared_section_fails(self):
        self.edit('{"fbeamSection": "tube 40x2 steel_1018"},', '')
        self.assertTrue(any("no fbeamSection" in f.message for f in errors(self.check()[0])))

    def test_tuning_out_of_range_fails(self):
        self.edit('"$tirepressure_F": 26', '"$tirepressure_F": 40', "vehicles/fbeam_demo/race.pc")
        self.assertTrue(any("$tirepressure_F" in f.message for f in errors(self.check()[0])))

    def test_sealed_variable_fails(self):
        self.edit('"$tirepressure_R": 24', '"$tirepressure_R": 24, "$engine_torque_mult": 1.2',
                  "vehicles/fbeam_demo/race.pc")
        self.assertTrue(any("sealed or unknown" in f.message for f in errors(self.check()[0])))

    def test_sealed_gear_ratio(self):
        self.edit('"$tirepressure_R": 24', '"$tirepressure_R": 24, "$gear_1": 3.0', "vehicles/fbeam_demo/race.pc")
        self.assertEqual(errors(self.check()[0]), [])
        self.edit('"$gear_1": 3.0', '"$gear_1": 3.27', "vehicles/fbeam_demo/race.pc")
        self.assertTrue(any("$gear_1" in f.message and "must be 3.0" in f.message for f in errors(self.check()[0])))

    def test_spec_parts_once_enforced(self):
        rules = json.loads(json.dumps(RULES))
        rules["spec_parts"]["enforce_required_parts"] = True
        msgs = [f.message for f in errors(run_all(Submission(self.car), rules)[0])]
        self.assertTrue(any("'covet_engine' must hold Spec Part 'covet_engine_1.8_sohc'" in m for m in msgs))
        self.assertTrue(any("sealed variable $revLimiterRPM must be set" in m for m in msgs))

    def test_wide_body_away_from_the_arches_fails(self):
        self.edit('["ml1", 0.5,  0.0, 0.3], ["mr1", -0.5,  0.0, 0.3]',
                  '["ml1", 0.9,  0.0, 0.3], ["mr1", -0.9,  0.0, 0.3]')     # 1,800 mm at mid-wheelbase
        found = {f.rule for f in errors(self.check()[0])}
        self.assertIn("6.3", found)
        self.assertNotIn("6.4", found)                                       # still inside the 1,800 mm arch limit

    def test_long_car_fails(self):
        self.edit('["rl1", 0.5,  1.5, 0.3]', '["rl1", 0.5,  2.6, 0.3]')
        self.assertTrue(any(f.rule in ("6.2", "6.6") for f in errors(self.check()[0])))

    def test_file_hash_ignores_zip_packing(self):
        folder_hash = Submission(self.car).file_hash()
        for i, mode in enumerate((zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED)):
            z = self.tmp / f"car{i}.zip"
            with zipfile.ZipFile(z, "w", mode) as zf:
                files = sorted(p for p in self.car.rglob("*") if p.is_file())
                for p in (files if i == 0 else reversed(files)):
                    zf.write(p, p.relative_to(self.car).as_posix())
            self.assertEqual(Submission(z).file_hash(), folder_hash)

    def test_digest_keeps_open_tuning_but_catches_structure(self):
        before = self.check()[1]
        self.edit('"$demo_arb_F": 25000', '"$demo_arb_F": 30000', "vehicles/fbeam_demo/race.pc")
        after_tuning = self.check()[1]
        self.assertEqual(before["jbeam_digest"], after_tuning["jbeam_digest"])
        self.assertNotEqual(before["file_hash"], after_tuning["file_hash"])
        self.edit('"beamDeform": 30000', '"beamDeform": 29000')
        self.assertNotEqual(before["jbeam_digest"], self.check()[1]["jbeam_digest"])


class TestCornerBudget(unittest.TestCase):
    """The 8.10.3 worked example: the 8.9.4 main hoop welded to a floor tube, all 45 x 2.5."""

    def test_worked_example(self):
        from fbeam_checker import bending as fb
        sec = parse_section("tube 45x2.5 steel_1018", load_materials(RULES), RULES)
        E, sy = sec.material.E, sec.material.sy
        nodes = {"fL": (0.55, 0, 0), "tL": (0.55, 0, 1.0), "tR": (-0.55, 0, 1.0), "fR": (-0.55, 0, 0)}
        tubes = [{"id": i, "a": a, "b": b} for i, a, b in
                 (("legL", "fL", "tL"), ("top", "tL", "tR"), ("legR", "tR", "fR"), ("floor", "fR", "fL"))]
        b = fb.budgets(nodes, tubes, lambda m: (sec.A, sec.I, 0.045, 0.0025), lambda m: (E, sy))
        diag = b[("fL", "tR")]
        self.assertEqual(sorted(c[0] for c in diag["corners"]), ["fR", "tL"])     # one diagonal, two corners
        self.assertAlmostEqual(diag["k"], 8.09e4, delta=0.01e4)                    # N/m, before the 8.10.5 scale
        self.assertAlmostEqual(diag["F"], 4460, delta=5)                           # N: beamDeform budget
        mp = sy * fb.plastic_modulus(0.045, 0.0025)
        self.assertAlmostEqual(mp, 1650, delta=1)                                  # N*m
        cos = 1.10 / math.hypot(1.10, 1.0)
        self.assertAlmostEqual(2 * diag["F"] * cos, 4 * mp / 1.0, places=6)       # collapse = the four-hinge 4 Mp / h


class TestPickupRule(unittest.TestCase):
    """7.1.6 does not depend on which part a beam or node is written in."""

    def run_check(self, beams, chassis=("frame_a", "frame_b"), suspension=("S",)):
        from types import SimpleNamespace
        from fbeam_checker.checks import check_pickups
        from fbeam_checker.submission import Beam, Car, Node
        pos = {"J": (0.4, 1.0, 0.3), "A": (0.4, 0.5, 0.3), "B": (0.0, 1.0, 0.3), "C": (0.4, 1.0, 0.7),
               "S": (0.7, 1.2, 0.3)}
        part_of = {"J": "frame_a", "A": "frame_a", "B": "frame_a", "C": "frame_b", "S": "frame_a"}
        car = Car(parts={p: {} for p in chassis}, variables={}, variable_defs={})
        car.nodes = {n: Node(n, p, 5.0, part_of[n], {}) for n, p in pos.items()}
        car.beams = [Beam(a, b, part, props) for a, b, part, props in beams]
        sub = SimpleNamespace(manifest={"chassis_parts": list(chassis), "suspension_nodes": list(suspension)})
        out = []
        check_pickups(sub, car, lambda level, rule, msg: out.append((level, rule, msg)))
        return [m for lvl, r, m in out if lvl == ERROR and r == "7.1.6"]

    tube = {"fbeamSection": "tube 40x2 steel_1018"}
    spring = {"fbeamRole": "spring"}

    def test_suspension_beam_in_the_frames_part_still_loads_the_joint(self):
        # the spring J-S is written in the frame's own part: before the fix it was not seen as a load
        flat = [("J", "A", "frame_a", self.tube), ("J", "B", "frame_a", self.tube), ("J", "S", "frame_a", self.spring)]
        self.assertTrue(any("frame joint J" in m for m in self.run_check(flat)))

    def test_bracing_tube_in_another_chassis_part_counts(self):
        braced = [("J", "A", "frame_a", self.tube), ("J", "B", "frame_a", self.tube),
                  ("J", "C", "frame_b", self.tube), ("J", "S", "frame_a", self.spring)]
        self.assertEqual(self.run_check(braced), [])

    def test_spring_between_two_frame_joints_is_refused(self):
        # a spring has no section or mass check: between frame joints it would be an invisible brace
        self.assertTrue(any("both ends are frame joints" in m
                            for m in self.run_check([("A", "B", "frame_a", self.spring)])))

    def test_undeclared_suspension_nodes_fail(self):
        # S not declared: the arm-to-upright springs look like frame-to-frame springs and are refused
        braced = [("J", "A", "frame_a", self.tube), ("J", "B", "frame_a", self.tube),
                  ("J", "C", "frame_b", self.tube), ("J", "S", "frame_a", self.spring)]
        self.assertTrue(self.run_check(braced, suspension=()))


if __name__ == "__main__":
    unittest.main()
