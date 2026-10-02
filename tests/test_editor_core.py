import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "editor"))

import fbeam_editor_core as core  # noqa: E402


def ev(model):
    return json.loads(core.evaluate(json.dumps(model)))


def split(model, member_id, t, name):
    """Insert a joint on a tube, as the editor's Split tool does."""
    m = next(x for x in model["members"] if x["id"] == member_id)
    pa, pb = model["nodes"][m["a"]], model["nodes"][m["b"]]
    model["nodes"][name] = [a + (b - a) * t for a, b in zip(pa, pb)]
    new = dict(m, id=max(x["id"] for x in model["members"]) + 1, a=name)
    m["b"] = name
    new["b"] = pb and next(k for k, v in model["nodes"].items() if v == pb)
    model["members"].append(new)
    return model


class TestCageModel(unittest.TestCase):
    def test_matches_cage_calculator(self):
        import cage_calc as cc
        model = core.cage_model()
        self.assertEqual([(m["a"], m["b"]) for m in model["members"]], [(a, b) for _, a, b, _ in cc.MEMBERS])  # same 18 tubes
        res = ev(model)
        self.assertAlmostEqual(res["totals"]["tube_mass"], 39.46, delta=0.01)    # in the Covet cabin (the worked example: 38.56)
        self.assertAlmostEqual(res["totals"]["tube_mass"], ev(core.full_chassis_model())["totals"]["cage_mass"], places=6)
        self.assertEqual(res["totals"]["buckling_members"], 10)

    def test_undersized_cage_tube_is_refused(self):
        model = core.cage_model()
        model["members"][0]["tube"] = "38x2.5"   # a main hoop leg
        with self.assertRaises(ValueError):
            core.evaluate(json.dumps(model))

    def test_preview_matches_regulations_example(self):
        model = core.cage_model()
        door = next(m for m in model["members"] if m["a"] == "APf_L" and m["b"] == "MHm_L")
        p = json.loads(core.section_preview(json.dumps(model), door["id"], "38x2.5"))
        self.assertAlmostEqual(p["d_mass"], 0.39, delta=0.01)
        self.assertAlmostEqual(p["d_deform_pct"], 2.1, delta=0.1)


class TestFullChassis(unittest.TestCase):
    def test_builds_and_passes_checker(self):
        with tempfile.TemporaryDirectory() as tmp:
            res = json.loads(core.build(json.dumps(core.full_chassis_model()), str(Path(tmp) / "car")))
            self.assertTrue(res["passed"], res["findings"])
            self.assertEqual(res["dimensions"]["wheelbase_mm"], 2500)
            self.assertLessEqual(res["dimensions"]["length_mm"], 3870)
            self.assertTrue(Path(res["zip_path"]).stat().st_size > 1000)

    def test_frame_and_cage_masses_split(self):
        t = ev(core.full_chassis_model())["totals"]
        self.assertAlmostEqual(t["cage_mass"], 39.46, delta=0.01)
        self.assertGreater(t["frame_mass"], 20)
        self.assertAlmostEqual(t["tube_mass"], t["cage_mass"] + t["frame_mass"], places=6)

    def test_moving_a_joint_changes_the_numbers(self):
        model = core.full_chassis_model()
        before = ev(model)["totals"]["tube_mass"]
        model["nodes"]["MHt_L"][2] += 0.10
        model["nodes"]["MHt_R"][2] += 0.10
        self.assertGreater(ev(model)["totals"]["tube_mass"], before)


class TestDrawingRules(unittest.TestCase):
    def test_split_without_brace_keeps_full_buckling_length(self):
        model = core.cage_model()
        diag = next(m for m in model["members"] if m["group"] == "Main hoop diagonal")
        whole = next(m for m in ev(model)["members"] if m["id"] == diag["id"])
        model = split(model, diag["id"], 0.5, "J1")
        halves = [m for m in ev(model)["members"] if m["group"] == "Main hoop diagonal"]
        self.assertEqual(len(halves), 2)
        for h in halves:
            self.assertAlmostEqual(h["L_buckle"], whole["L"], places=6)
            self.assertAlmostEqual(h["beamDeform"], whole["beamDeform"], places=3)

    def test_braced_joint_shortens_buckling_length(self):
        model = split(core.cage_model(), next(m["id"] for m in core.cage_model()["members"]
                                              if m["group"] == "Main hoop diagonal"), 0.5, "J1")
        model["members"].append({"id": 999, "group": "Brace", "a": "J1", "b": "MHm_L",
                                 "role": "other", "tube": "40x2"})
        halves = [m for m in ev(model)["members"] if m["group"] == "Main hoop diagonal"]
        for h in halves:
            self.assertAlmostEqual(h["L_buckle"], h["L"], places=6)

    def test_loose_joint_is_reported_and_left_out(self):
        model = core.full_chassis_model()
        model["nodes"]["J9"] = [0.0, 0.0, 1.5]
        self.assertEqual(ev(model)["orphans"], ["J9"])
        with tempfile.TemporaryDirectory() as tmp:
            res = json.loads(core.build(json.dumps(model), str(Path(tmp) / "car")))
            self.assertTrue(res["passed"])
            self.assertTrue(any("J9" in f["message"] for f in res["findings"]))

    def test_duplicate_tube_is_refused(self):
        model = core.cage_model()
        m = dict(model["members"][0], id=999)
        model["members"].append(m)
        with self.assertRaises(ValueError):
            core.evaluate(json.dumps(model))


if __name__ == "__main__":
    unittest.main()


class TestFrameSolver(unittest.TestCase):
    def test_cantilever_axial_and_twist_match_theory(self):
        import fbeam_frame as f
        E, D, t = 205e9, 0.045, 0.0025
        d = D - 2 * t
        A, I = math.pi / 4 * (D ** 2 - d ** 2), math.pi / 64 * (D ** 4 - d ** 4)
        s = f.Structure({"a": [0, 0, 0], "b": [0.6, 0.8, 0]},
                        [{"a": "a", "b": "b", "E": E, "A": A, "I": I, "J": 2 * I}], "frame")
        fixed = {("a", i): 0.0 for i in range(6)}
        u, _, _ = s.solve(fixed, {("b", 2): 1000.0})
        self.assertAlmostEqual(1000 / u[8], 3 * E * I, delta=3 * E * I * 1e-9)
        u, _, _ = s.solve(fixed, {("b", 0): 600.0, ("b", 1): 800.0})
        self.assertAlmostEqual(1000 / (0.6 * u[6] + 0.8 * u[7]), E * A, delta=E * A * 1e-9)

    def test_pin_jointed_mechanisms_follow_maxwell(self):
        res = json.loads(core.rigidity(json.dumps(core.full_chassis_model())))
        t = ev(core.full_chassis_model())["totals"]
        self.assertEqual(res["frame"]["mechanisms"], 0)
        self.assertGreater(res["frame"]["torsion_Nm_per_deg"], 1000)
        self.assertEqual(res["pinned"]["mechanisms"], t["maxwell_needed"] - t["members"])

    def test_centre_of_gravity(self):
        cog = ev(core.full_chassis_model())["cog"]
        self.assertAlmostEqual(cog["x"], 0.0, delta=0.01)       # symmetric
        self.assertTrue(0.3 < cog["front_share"] < 0.7)
        self.assertTrue(0.2 < cog["z"] < 1.0)


class TestDriveline(unittest.TestCase):
    """Driven axles, differentials, halfshafts and propshaft (4.6)."""

    def setUp(self):
        self.model = core.full_chassis_model()

    def d(self, model):
        return json.loads(core.driveline(json.dumps(model)))

    def test_spec_fwd_has_unequal_front_halfshafts(self):
        r = self.d(self.model)
        self.assertEqual(r["layout"], "FWD")
        self.assertEqual(r["issues"], [])
        half = {s["id"]: s for s in r["shafts"]}
        self.assertEqual(sorted(half), ["half_front_L", "half_front_R"])
        L, R = half["half_front_L"], half["half_front_R"]
        self.assertGreater(math.dist(L["a"], L["b"]), math.dist(R["a"], R["b"]) + 0.2)   # Covet tra2 / tra3: long left, short right
        self.assertGreater(R["max_angle_deg"], L["max_angle_deg"])                        # the short shaft works at bigger angles

    def test_rwd_and_awd_are_for_testing(self):
        for layout, n in (("RWD", 3), ("AWD", 5)):
            m = json.loads(core.set_driveline(json.dumps(self.model), layout))
            r = self.d(m)
            self.assertEqual(len(r["shafts"]), n, layout)
            self.assertTrue(any(s["kind"] == "prop" for s in r["shafts"]), layout)
            self.assertTrue(any(i["rule"] == "4.6" and i["level"] == "WARN" for i in r["issues"]), layout)
            self.assertEqual(json.loads(core.export_svj(json.dumps(m)))["vehicle_info"]["drive_type"], layout)

    def test_live_axle_propshaft_follows_the_axle(self):
        m = json.loads(core.set_driveline(json.dumps(self.model), "RWD"))
        m = json.loads(core.suspension_layout(json.dumps(m), "rear", "solid_axle_panhard"))
        r = self.d(m)
        self.assertEqual([s["id"] for s in r["shafts"]], ["prop_rear"])         # no separate halfshafts: they are in the axle
        lo, hi = r["shafts"][0]["plunge_range_mm"]
        self.assertGreater(hi - lo, 5.0)                                         # the pinion moves with the axle

    def test_undriven_axle_has_no_halfshaft_curves(self):
        res = json.loads(core.suspension(json.dumps(self.model)))
        self.assertIn("halfshaft_deg", res["corners"]["front"]["curves"])
        self.assertNotIn("halfshaft_deg", res["corners"]["rear"]["curves"])


class TestLayouts(unittest.TestCase):
    """Every suspension layout (fbeam_suspension.TOPOLOGY): kinematics, export, SVJ."""

    def setUp(self):
        import fbeam_suspension
        self.fs = fbeam_suspension
        self.base = core.full_chassis_model()

    def switched(self, corner, layout):
        return json.loads(core.suspension_layout(json.dumps(self.base), corner, layout))

    def test_every_layout_assembles_with_sensible_start_geometry(self):
        for lay in self.fs.TOPOLOGY:
            res = json.loads(core.suspension(json.dumps(self.switched("rear", lay))))
            c = res["corners"]["rear"]
            near = [i for i, t in enumerate(c["curves"]["travel_mm"]) if abs(t) <= 60]
            self.assertGreaterEqual(c["static"]["usable_travel_mm"], 150, lay)
            self.assertLess(max(abs(c["curves"]["toe_deg"][i]) for i in near), 1.0, lay)
            self.assertTrue(all(not m["on_frame"] for m in c["mounts"]), lay)        # pickups start free
            self.assertTrue(any(i["rule"] == "7.1.6" for i in res["issues"]), lay)

    def test_pure_trailing_arm_keeps_camber_and_has_roll_centre_at_ground(self):
        c = json.loads(core.suspension(json.dumps(self.switched("rear", "trailing_arm"))))["corners"]["rear"]
        self.assertLess(max(abs(x) for x in c["curves"]["camber_deg"]), 1e-6)
        self.assertAlmostEqual(c["static"]["roll_centre_mm"], -9, delta=2)    # at the ground (z = -9 mm, Covet frame)
        semi = json.loads(core.suspension(json.dumps(self.switched("rear", "semi_trailing_arm"))))["corners"]["rear"]
        self.assertGreater(max(abs(x) for x in semi["curves"]["camber_deg"]), 2.0)      # swept axis: camber gain

    def test_front_must_steer(self):
        res = json.loads(core.suspension(json.dumps(self.switched("front", "trailing_arm"))))
        self.assertTrue(any(i["rule"] == "7.1" and "steer" in i["message"] for i in res["issues"]))

    def test_double_wishbone_kingpin_is_the_ball_joint_axis(self):
        m = self.switched("rear", "double_wishbone")
        res = json.loads(core.suspension(json.dumps(m)))["corners"]["rear"]
        p = res["points"]
        ax = [p["UBJ"][i] - p["LBJ"][i] for i in range(3)]
        self.assertAlmostEqual(res["static"]["kpi_deg"], math.degrees(math.atan2(-ax[0], ax[2])), places=3)

    def test_make_joints_then_brace_passes(self):
        m = json.loads(core.pickups_to_joints(json.dumps(self.switched("rear", "trailing_arm")), "rear"))
        pts = m["suspension"]["rear"]["points"]
        self.assertTrue(all(isinstance(pts[k], dict) for k in self.fs.INNER["trailing_arm"]))
        res = json.loads(core.suspension(json.dumps(m)))
        self.assertTrue(all(m_["on_frame"] and not m_["braced"] for m_ in res["corners"]["rear"]["mounts"]))

    def test_solid_axles(self):
        for lay, rc in (("solid_axle_panhard", "PHC"), ("solid_axle_watt", "WP"), ("solid_axle_triangulated", None)):
            c = json.loads(core.suspension(json.dumps(self.switched("rear", lay))))["corners"]["rear"]
            self.assertLess(max(abs(x) for x in c["curves"]["camber_deg"]), 1e-6, lay)    # a rigid axle stays upright in heave
            self.assertLess(max(abs(x) for x in c["curves"]["toe_deg"]), 1e-6, lay)
            shift = max(abs(x) for x in c["curves"]["axle_shift_mm"])
            if lay == "solid_axle_panhard":
                self.assertGreater(shift, 1.0)                   # the Panhard rod's arc moves the axle sideways
            else:
                self.assertLess(shift, 1e-3, lay)                # Watt's linkage and the upper triangle do not
        watt = json.loads(core.suspension(json.dumps(self.switched("rear", "solid_axle_watt"))))["corners"]["rear"]
        self.assertAlmostEqual(watt["static"]["roll_centre_mm"], watt["points"]["WP"][2] * 1000, places=3)

    def test_de_dion(self):
        for lay in ("de_dion_watt", "de_dion_panhard"):
            m = self.switched("rear", lay)
            self.assertIn("DIFF", m["suspension"]["rear"]["points"])          # chassis differential: required
            c = json.loads(core.suspension(json.dumps(m)))["corners"]["rear"]
            self.assertLess(max(abs(x) for x in c["curves"]["camber_deg"]), 1e-6, lay)
            self.assertIn("plunge_mm", c["curves"])                           # halfshafts from the sprung diff
            self.assertGreater(max(abs(x) for x in c["curves"]["plunge_mm"]), 1.0, lay)
            svj = json.loads(core.export_svj(json.dumps(m)))
            self.assertEqual(svj["suspension"]["RL"]["topology"]["system_type"], "de_dion")
            plain = json.loads(json.dumps(svj))
            plain["x_fbeam"].pop("model", None)
            back = json.loads(core.import_svj(json.dumps(plain)))["model"]["suspension"]["rear"]
            self.assertEqual(back["type"], lay)
            for a, b in zip(back["points"]["DIFF"], m["suspension"]["rear"]["points"]["DIFF"]):
                self.assertAlmostEqual(a, b, places=4)

    def test_braced_solid_axle_passes_the_checker(self):
        from fbeam_checker.pickups import mirror_name
        for lay in ("solid_axle_panhard", "solid_axle_watt", "de_dion_watt"):
            m = json.loads(core.pickups_to_joints(json.dumps(self.switched("rear", lay)), "rear"))
            frame, nid, done = list(self.base["nodes"]), max(x["id"] for x in m["members"]) + 1, set()
            for k in self.fs.INNER[lay]:
                j = m["suspension"]["rear"]["points"][k]["node"]
                for jj in {j, mirror_name(j)} - done:
                    if jj not in m["nodes"]:
                        continue
                    done.add(jj)
                    for n in sorted(frame, key=lambda n: math.dist(m["nodes"][n], m["nodes"][jj]))[:3]:
                        m["members"].append({"id": nid, "group": "Axle bracket", "a": jj, "b": n, "role": "frame", "tube": "30x2"})
                        nid += 1
            self.assertEqual(json.loads(core.suspension(json.dumps(m)))["issues"], [], lay)
            with tempfile.TemporaryDirectory() as tmp:
                r = json.loads(core.build(json.dumps(m), str(Path(tmp) / "car")))
            self.assertTrue(r["passed"], (lay, [f for f in r["findings"] if f["level"] == "ERROR"]))

    def test_every_layout_exports_jbeam_and_svj(self):
        from fbeam_checker import jbeam
        for lay in self.fs.TOPOLOGY:
            m = self.switched("rear", lay)
            with tempfile.TemporaryDirectory() as tmp:
                r = json.loads(core.build(json.dumps(m), str(Path(tmp) / "car")))
                other = [f for f in r["findings"] if f["level"] == "ERROR" and f["rule"] != "7.1.6"]
                self.assertEqual(other, [], lay)
                doc = jbeam.parse((Path(tmp) / "car" / "vehicles" / core.PART / "fbeam_editor_suspension.jbeam").read_text(encoding="utf-8"))
                self.assertEqual(len(jbeam.expand_table(doc["fbeam_editor_wheeldata"]["pressureWheels"])), 4, lay)
            svj = json.loads(core.export_svj(json.dumps(m)))
            plain = json.loads(json.dumps(svj))
            plain["x_fbeam"].pop("model", None)
            bm = json.loads(core.import_svj(json.dumps(plain)))["model"]
            back = bm["suspension"]["rear"]
            self.assertEqual(back["type"], lay)
            for k in self.fs.REQUIRED[lay]:
                got, want = core._point(bm, back["points"][k]), core._point(m, m["suspension"]["rear"]["points"][k])
                for a, b in zip(got, want):
                    self.assertAlmostEqual(a, b, places=4, msg=f"{lay} {k}")


class TestSuspension(unittest.TestCase):
    def setUp(self):
        self.res = json.loads(core.suspension(json.dumps(core.full_chassis_model())))

    def test_example_corners_pass_rules(self):
        self.assertEqual(self.res["issues"], [])
        for name, c in self.res["corners"].items():
            self.assertGreaterEqual(c["static"]["usable_travel_mm"], 150, name)
            self.assertTrue(all(m["on_frame"] and m["braced"] for m in c["mounts"]), name)

    def test_unbraced_pickup_joint_is_flagged(self):
        model = core.full_chassis_model()
        keep = {"RX_L", "RTI_L"}                         # leave the toe-link joint with two collinear rails only
        model["members"] = [m for m in model["members"]
                            if "RTI_L" not in (m["a"], m["b"]) or {m["a"], m["b"]} <= keep | {"RE_L"}]
        res = json.loads(core.suspension(json.dumps(model)))
        self.assertTrue(any(i["rule"] == "7.1.6" and "RTI_L" in i["message"] for i in res["issues"]))

    def test_off_frame_pickup_fails_the_checker(self):
        model = core.full_chassis_model()
        model["suspension"]["rear"]["points"]["UIF"] = [0.30, 1.143, 0.46]
        with tempfile.TemporaryDirectory() as tmp:
            res = json.loads(core.build(json.dumps(model), str(Path(tmp) / "car")))
        self.assertFalse(res["passed"])
        self.assertTrue(any(f["rule"] == "7.1.6" and "mount" in f["message"] for f in res["findings"]))

    def test_frame_carries_the_spec_engine_interface(self):
        donor = json.loads((Path(__file__).resolve().parent.parent / "rules" / "donor_covet.json").read_text(encoding="utf-8"))
        nodes = core.full_chassis_model()["nodes"]
        for n in donor["parts"]["covet_engine_1.8_sohc"]["interface_nodes"]:
            self.assertIn(n, nodes)                      # the sealed engine part bolts on unchanged

    def test_mirror_names(self):
        from fbeam_checker.pickups import mirror_name
        self.assertEqual([mirror_name(n) for n in ("FT_L", "f10ll", "fx2l", "f15r", "f11", "RK_R")],
                         ["FT_R", "f10rr", "fx2r", "f15l", "f11", "RK_L"])

    def test_static_alignment_and_bump_steer(self):
        for name, c in self.res["corners"].items():
            cur, tr = c["curves"], c["curves"]["travel_mm"]
            i0 = tr.index(0.0)
            self.assertAlmostEqual(cur["camber_deg"][i0], 0.0, places=6)
            near = [i for i, t in enumerate(tr) if abs(t) <= 60]
            self.assertLess(max(abs(cur["toe_deg"][i]) for i in near), 1.0, name)   # bump steer under 1 degree
            self.assertTrue(0.3 < c["static"]["motion_ratio"] < 1.2, name)

    def test_macpherson_kpi_and_moving_a_pickup(self):
        self.assertAlmostEqual(self.res["corners"]["front"]["static"]["kpi_deg"], 8.2, delta=0.1)
        model = core.full_chassis_model()
        model["suspension"]["front"]["points"]["TRO"] = [0.676, -1.121, 0.354]    # raise the tie-rod end 50 mm
        res = json.loads(core.suspension(json.dumps(model)))
        cur, tr = res["corners"]["front"]["curves"], res["corners"]["front"]["curves"]["travel_mm"]
        near = [i for i, t in enumerate(tr) if abs(t) <= 60]
        self.assertGreater(max(abs(cur["toe_deg"][i]) for i in near), 1.0)      # now it bump-steers

    def test_pickup_off_the_frame_is_flagged(self):
        model = core.full_chassis_model()
        model["suspension"]["rear"]["points"]["UIF"] = [0.30, 1.143, 0.46]
        res = json.loads(core.suspension(json.dumps(model)))
        self.assertTrue(any(i["rule"] == "7.1.6" and i["level"] == "ERROR" for i in res["issues"]))

    def test_exported_beam_line_matches_build(self):
        r = ev(core.full_chassis_model())["members"][0]
        self.assertTrue(r["jbeam"].startswith(f'["{r["a"]}", "{r["b"]}", {{"fbeamSection": "tube {r["tube"]}'))


class TestSVJ(unittest.TestCase):
    """SVJ (Standard Vehicle JSON) export and import. Set SVJ_SCHEMA to the official
    schema/svj.schema.json (and install jsonschema) to also validate against it."""

    def setUp(self):
        self.model = core.full_chassis_model()
        self.svj = json.loads(core.export_svj(json.dumps(self.model)))

    def test_frame_conversion_and_required_fields(self):
        d = self.svj
        self.assertEqual(d["_metadata"]["coordinate_system"], "SAE_J670")
        for k in ("mass_total", "wheelbase", "track_front", "track_rear", "center_of_gravity"):
            self.assertIn(k, d["chassis"])
        self.assertAlmostEqual(d["chassis"]["wheelbase"], 2.50, places=4)
        fl = d["suspension"]["FL"]["topology"]["upright"]["hardpoints"]["wheel_center"]
        rr = d["suspension"]["RR"]["topology"]["upright"]["hardpoints"]["wheel_center"]
        self.assertAlmostEqual(fl[0], 0.0, places=4)       # origin on the front axle
        self.assertLess(fl[1], 0)                          # left is -Y in SAE J670
        self.assertLess(fl[2], 0)                          # up is -Z
        self.assertAlmostEqual(rr[0], -2.50, places=4)     # rear axle behind (X forward)
        self.assertGreater(rr[1], 0)

    def test_spring_rate_is_wheel_rate(self):
        res = json.loads(core.suspension(json.dumps(self.model)))["corners"]["front"]["static"]
        self.assertAlmostEqual(self.svj["suspension"]["FL"]["spring"]["rate"] / 1000, res["wheel_rate_N_per_mm"], delta=0.05)

    def test_fbeam_round_trip_is_lossless(self):
        back = json.loads(core.import_svj(json.dumps(self.svj)))["model"]
        for k in ("nodes", "members", "axles", "hubs", "suspension"):
            self.assertEqual(back[k], self.model[k], k)

    def test_external_edit_is_picked_up(self):
        self.svj["suspension"]["FL"]["topology"]["upright"]["hardpoints"]["steering_tie_rod_end"][2] -= 0.02
        r = json.loads(core.import_svj(json.dumps(self.svj)))
        self.assertAlmostEqual(r["model"]["suspension"]["front"]["points"]["TRO"][2], 0.324, places=5)
        self.assertTrue(any("TRO" in n for n in r["notes"]))

    def test_skeleton_import_from_plain_svj(self):
        plain = {k: v for k, v in self.svj.items() if k != "x_fbeam"}
        for c in plain["suspension"].values():
            c.pop("x_fbeam", None)
        r = json.loads(core.import_svj(json.dumps(plain)))
        m = r["model"]
        self.assertEqual(m["members"], [])                               # no frame: a skeleton to draw on
        self.assertIn("front_LIF_L", m["nodes"])
        res = json.loads(core.suspension(json.dumps(m)))
        self.assertAlmostEqual(res["corners"]["front"]["static"]["kpi_deg"], 8.2, delta=0.1)
        self.assertAlmostEqual(res["corners"]["front"]["static"]["wheel_rate_N_per_mm"],
                               self.svj["suspension"]["FL"]["spring"]["rate"] / 1000, delta=0.05)

    def test_official_schema(self):
        import os
        path = os.environ.get("SVJ_SCHEMA")
        try:
            from jsonschema import Draft7Validator
        except ImportError:
            self.skipTest("jsonschema not installed")
        if not path:
            self.skipTest("set SVJ_SCHEMA to the official svj.schema.json")
        errors = list(Draft7Validator(json.load(open(path, encoding="utf-8"))).iter_errors(self.svj))
        self.assertEqual([e.message for e in errors], [])


class TestApplySVJ(unittest.TestCase):
    """Applying an SVJ to the current frame (core.apply_svj): joints follow pickups, layouts, meshes."""

    def setUp(self):
        self.base = core.full_chassis_model()

    def edited_svj(self, dz=0.010):
        svj = json.loads(core.export_svj(json.dumps(self.base)))
        link = next(l for l in svj["suspension"]["FL"]["topology"]["links"] if l["name"] == "lower_control_arm")
        link["inboard_points"][0][2] -= dz          # front lower arm, inner front: up by dz (SAE z is down)
        return svj

    def test_moved_pickup_drags_its_joint_and_mirror(self):
        r = json.loads(core.apply_svj(json.dumps(self.base), json.dumps(self.edited_svj())))
        m = r["model"]
        self.assertEqual(m["suspension"]["front"]["points"]["LIF"], {"node": "fx1l"})     # still attached
        self.assertAlmostEqual(m["nodes"]["fx1l"][2], self.base["nodes"]["fx1l"][2] + 0.010, places=6)
        self.assertEqual(m["nodes"]["fx1r"], [-m["nodes"]["fx1l"][0]] + m["nodes"]["fx1l"][1:])
        self.assertEqual(len(m["members"]), len(self.base["members"]))                     # the frame is kept
        self.assertEqual(json.loads(core.suspension(json.dumps(m)))["issues"], [])

    def test_option_off_detaches_instead(self):
        r = json.loads(core.apply_svj(json.dumps(self.base), json.dumps(self.edited_svj()),
                                      json.dumps({"move_joints": False})))
        m = r["model"]
        self.assertEqual(m["nodes"]["fx1l"], self.base["nodes"]["fx1l"])
        self.assertIsInstance(m["suspension"]["front"]["points"]["LIF"], list)              # free: 7.1.6 reports it

    def test_layout_is_taken_with_new_joints(self):
        src = json.loads(core.suspension_layout(json.dumps(self.base), "rear", "multilink"))
        svj = json.loads(core.export_svj(json.dumps(src)))
        r = json.loads(core.apply_svj(json.dumps(self.base), json.dumps(svj)))
        rear = r["model"]["suspension"]["rear"]
        self.assertEqual(rear["type"], "multilink")
        self.assertTrue(all(isinstance(rear["points"][k], dict) for k in ("UFI", "URI", "LFI", "LRI", "TRI", "DT")))
        self.assertTrue(r["new_joints"])
        off = json.loads(core.apply_svj(json.dumps(self.base), json.dumps(svj), json.dumps({"layout": False})))
        self.assertEqual(off["model"]["suspension"]["rear"]["type"], "double_wishbone")

    def test_unchanged_file_changes_nothing(self):
        svj = json.loads(core.export_svj(json.dumps(self.base)))
        r = json.loads(core.apply_svj(json.dumps(self.base), json.dumps(svj)))
        self.assertEqual((r["moved_joints"], r["new_joints"]), ([], []))
        self.assertEqual(r["model"]["nodes"], self.base["nodes"])

    def test_foreign_meshes_are_copied_and_exported(self):
        import zipfile
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            src = tmp / "src.zip"
            json.loads(core.export_svj_bundle(json.dumps(self.base), None, str(src), None))
            with zipfile.ZipFile(src) as z:                  # turn it into a "foreign" file with its own body mesh
                doc = json.loads(z.read("fbeam_car.svj.json"))
                glb = z.read("meshes/fbeam_body.glb")
            doc.pop("x_fbeam", None)
            doc["assets"]["meshes"].append({"id": "donor_body", "uri": "meshes/donor_body.glb"})
            doc["chassis"]["visual"] = {"mesh_ref": "donor_body", "node": "SVJ::body::body_panels"}
            foreign = tmp / "foreign.zip"
            with zipfile.ZipFile(foreign, "w") as z:
                z.writestr("donor.svj.json", json.dumps(doc))
                z.writestr("meshes/donor_body.glb", glb)
            res = json.loads(core.import_svj_bundle(str(foreign), str(tmp / "in"), json.dumps(self.base), None))
            m = res["model"]
            self.assertEqual([a["id"] for a in m["svj_source"]["assets"]["meshes"]], ["donor_body"])
            self.assertEqual(len(m["members"]), len(self.base["members"]))
            out = json.loads(core.export_svj_bundle(json.dumps(m), None, str(tmp / "out.zip"), res["base_dir"]))
            self.assertIn("meshes/donor_body.glb", out["files"])
            no = json.loads(core.import_svj_bundle(str(foreign), str(tmp / "in2"), json.dumps(self.base),
                                                   json.dumps({"meshes": False})))
            self.assertNotIn("svj_source", no["model"])
            self.assertEqual(no["meshes"], [])


class TestSVJMeshes(unittest.TestCase):
    """glTF visual binding (SVJ §22): bundles of .svj.json + .glb meshes."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def read_zip(self, path, name):
        with zipfile_open(path) as z:
            return z.read(name)

    def test_bundle_has_frame_mesh_bound_to_chassis(self):
        import fbeam_gltf as g
        r = json.loads(core.export_svj_bundle(json.dumps(core.full_chassis_model()), None, str(self.tmp / "car.zip")))
        self.assertIn("meshes/fbeam_tube_frame.glb", r["files"])
        doc = json.loads(self.read_zip(r["zip_path"], "fbeam_car.svj.json"))
        self.assertEqual(doc["chassis"]["visual"], {"mesh_ref": "fbeam_tube_frame", "node": "SVJ::body::chassis"})
        glb = self.read_zip(r["zip_path"], "meshes/fbeam_tube_frame.glb")
        self.assertEqual(g.node_names(glb), ["SVJ::body::chassis"])
        lo, hi = g.glb_json(glb)["accessors"][0]["min"], g.glb_json(glb)["accessors"][0]["max"]
        self.assertGreater(hi[1], 1.1)                      # glTF +Y is up: the cage roof
        self.assertAlmostEqual(lo[0], -hi[0], places=3)     # symmetric about the centreline

    def test_bundle_round_trip(self):
        model = core.full_chassis_model()
        r = json.loads(core.export_svj_bundle(json.dumps(model), None, str(self.tmp / "car.zip")))
        back = json.loads(core.import_svj_bundle(r["zip_path"], str(self.tmp / "in")))
        for k in ("nodes", "members", "suspension"):
            self.assertEqual(back["model"][k], model[k])
        self.assertEqual(back["bindings"][0]["node"], "SVJ::body::chassis")
        self.assertEqual(back["axes"], {"up": "Y", "forward": "-Z"})

    def test_existing_body_mesh_is_kept_and_frame_becomes_helper(self):
        import fbeam_gltf as g
        model = core.full_chassis_model()
        doc = json.loads(core.export_svj(json.dumps(model)))
        doc["assets"] = {"meshes": [{"id": "body_shell", "uri": "meshes/body.glb"}]}
        doc["chassis"]["visual"] = {"mesh_ref": "body_shell", "node": "SVJ::body::chassis"}
        (self.tmp / "src" / "meshes").mkdir(parents=True)
        (self.tmp / "src" / "car.svj.json").write_text(json.dumps(doc), encoding="utf-8")
        (self.tmp / "src" / "meshes" / "body.glb").write_bytes(g.tubes_glb([([0, 0, 0], [-1, 0, 0], 0.5)]))
        back = json.loads(core.import_svj_bundle(str(self.tmp / "src" / "car.svj.json"), str(self.tmp / "w")))
        r = json.loads(core.export_svj_bundle(json.dumps(back["model"]), None, str(self.tmp / "out.zip"), back["base_dir"]))
        self.assertIn("meshes/body.glb", r["files"])
        self.assertEqual(r["frame_node"], "SVJ::helper::fbeam_tube_frame")
        out = json.loads(self.read_zip(r["zip_path"], "fbeam_car.svj.json"))
        self.assertEqual(out["chassis"]["visual"]["mesh_ref"], "body_shell")
        self.assertEqual({a["id"] for a in out["assets"]["meshes"]}, {"body_shell", "fbeam_tube_frame", "fbeam_body"})

    def test_missing_mesh_is_reported(self):
        doc = json.loads(core.export_svj(json.dumps(core.full_chassis_model())))
        doc["assets"] = {"meshes": [{"id": "body_shell", "uri": "meshes/body.glb"}]}
        doc["chassis"]["visual"] = {"mesh_ref": "body_shell", "node": "SVJ::body::chassis"}
        (self.tmp / "car.svj.json").write_text(json.dumps(doc), encoding="utf-8")
        back = json.loads(core.import_svj_bundle(str(self.tmp / "car.svj.json"), str(self.tmp / "w")))
        self.assertTrue(any("was not provided" in n for n in back["notes"]))

    def test_zip_slip_is_refused(self):
        import zipfile
        bad = self.tmp / "bad.zip"
        with zipfile.ZipFile(bad, "w") as z:
            z.writestr("../evil.svj.json", "{}")
        with self.assertRaises(ValueError):
            core.import_svj_bundle(str(bad), str(self.tmp / "w"))


def zipfile_open(path):
    import zipfile
    return zipfile.ZipFile(path)


class TestMasses(unittest.TestCase):
    def setUp(self):
        self.model = core.full_chassis_model()

    def m(self, model=None):
        return json.loads(core.masses(json.dumps(model or self.model)))

    def test_total_is_structure_plus_components_plus_unsprung(self):
        r = self.m()
        self.assertAlmostEqual(r["mass_total"], r["structure_mass"] + r["components_mass"] + r["unsprung_total"], places=6)
        import fbeam_mass
        self.assertAlmostEqual(r["unsprung_total"], 4 * fbeam_mass.EXAMPLE_UNSPRUNG)
        self.assertGreaterEqual(r["mass_total"], 1100)             # the example is ballasted to the minimum (6.11)
        self.assertEqual(r["ballast_needed"], 0.0)

    def test_point_mass_inertia_matches_hand_calculation(self):
        import fbeam_mass
        model = {"nodes": {"a": [1.0, 0.0, 0.0], "b": [-1.0, 0.0, 0.0]}, "members": []}
        r = fbeam_mass.analyse(model, {"a": 10.0, "b": 10.0}, 0.0)
        self.assertAlmostEqual(r["inertia"]["Izz_yaw"], 20.0)      # 2 x 10 kg at 1 m
        self.assertAlmostEqual(r["inertia"]["Ixx_roll"], 20.0)     # roll is about the longitudinal (Y) axis
        self.assertAlmostEqual(r["inertia"]["Iyy_pitch"], 0.0)

    def test_rules(self):
        model = self.model
        eng = next(c for c in model["components"] if c["kind"] == "powertrain")
        eng["position"][1] += 0.40                                  # 500 mm behind the front axle
        drv = next(c for c in model["components"] if c["kind"] == "driver")
        drv["position"][2] = 0.20                                   # driver CG too low
        next(c for c in model["components"] if c["kind"] == "ballast")["mass"] = 100.0   # well under 1,100 kg
        rules = {i["rule"] for i in self.m(model)["issues"]}
        self.assertTrue({"4.4", "6.13", "6.11"} <= rules)

    def test_ballast_is_steel_on_the_floor_and_fill_reaches_minimum(self):
        import fbeam_mass
        model = self.model
        b = next(c for c in model["components"] if c["kind"] == "ballast")
        r = self.m(model)
        d = r["derived_ballast"][b["id"]]
        self.assertAlmostEqual(d["height"], b["mass"] / (7850 * b["size"][0] * b["size"][1]), places=4)
        self.assertAlmostEqual(d["z"] - d["height"] / 2, fbeam_mass.floor_height(model), places=4)
        b["mass"] = 50.0
        filled = json.loads(core.fill_ballast(json.dumps(model)))
        self.assertGreater(filled["added_kg"], 300)
        self.assertEqual(self.m(filled["model"])["ballast_needed"], 0.0)

    def test_svj_carries_masses(self):
        d = json.loads(core.export_svj(json.dumps(self.model)))
        r = self.m()
        self.assertAlmostEqual(d["chassis"]["mass_total"], r["mass_total"], places=1)
        self.assertAlmostEqual(d["chassis"]["inertia"]["Izz"], r["inertia"]["Izz_yaw"], places=2)
        body_sum = sum(b["mass"] for b in d["chassis"]["mass_bodies"]) + sum(d["chassis"]["mass_unsprung_per_corner"].values())
        self.assertAlmostEqual(body_sum, d["chassis"]["mass_total"], delta=0.05)    # SVJ §7.2.2 consistency
        back = json.loads(core.import_svj(json.dumps(d)))["model"]
        self.assertEqual(back["components"], self.model["components"])


class TestBendingBeams(unittest.TestCase):
    """Regulations 8.10: generated weightless braces, and the checker recomputing their budgets."""

    def setUp(self):
        self.model = core.full_chassis_model()
        self.tmp = Path(tempfile.mkdtemp())

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def build_with(self, edit=None):
        out = self.tmp / "car"
        core.build(json.dumps(self.model), str(out))
        jb = out / "vehicles" / core.PART / f"{core.PART}.jbeam"
        if edit:
            jb.write_text(edit(jb.read_text(encoding="utf-8")), encoding="utf-8")
        from fbeam_checker.checks import run_all
        from fbeam_checker.submission import Submission
        return [f for f in run_all(Submission(out), core.cc.RULES)[0] if f.level == "ERROR"]

    def test_braces_remove_mechanisms_without_beating_the_welded_frame(self):
        r = json.loads(core.rigidity(json.dumps(self.model)))
        self.assertGreater(r["bending_beams"], 50)
        self.assertEqual(r["pinned"]["mechanisms"], 30)             # tubes alone fold
        self.assertEqual(r["pinned_bending"]["mechanisms"], 0)      # braces make it rigid
        self.assertTrue(r["check_8_10_6"]["passed"])                # but never stiffer than the welded frame
        self.assertLess(r["check_8_10_6"]["torsion_ratio"], 1.0)
        for k in ("bending_ratio", "lateral_ratio", "roof_ratio"):          # all four 8.10.6 load cases
            self.assertTrue(0 < r["check_8_10_6"][k] < 1.10, k)
        self.assertEqual(sorted(r["top_nodes"]), ["MHt_L", "MHt_R"])         # loaded at the main-hoop top

    def test_off_switch(self):
        self.model["bending_beams"] = "off"
        self.assertEqual(ev(self.model)["totals"]["bending_beams"], 0)

    def test_generated_car_passes_checker(self):
        self.assertEqual(self.build_with(), [])

    def test_stiffer_brace_than_budget_is_refused(self):
        import re
        def boost(text):
            return re.sub(r'("fbeamRole": "bending", "fbeamMember": "[^"]+", "beamSpring": )(\d+)',
                          lambda m: m.group(1) + str(int(m.group(2)) * 3), text, count=1)
        errs = self.build_with(boost)
        self.assertTrue(any(e.rule.startswith("8.10.3") and "beamSpring" in e.message for e in errs), errs)

    def test_brace_that_is_not_a_corner_is_refused(self):
        def fake(text):
            return text.replace('    ]\n}\n}', '        ,["FE_L", "RE_R", {"fbeamRole": "bending", "fbeamMember": "m0", '
                                '"beamSpring": 1000, "beamDeform": 100, "beamStrength": 120}]\n    ]\n}\n}')
        errs = self.build_with(fake)
        self.assertTrue(any(e.rule == "8.10.0" for e in errs), errs)

    def test_unbreakable_brace_is_refused(self):
        import re
        def flt(text):
            return re.sub(r'("fbeamRole": "bending".*?"beamStrength": )(\d+)', r'\1"FLT_MAX"', text, count=1)
        errs = self.build_with(flt)
        self.assertTrue(any("FLT_MAX" in e.message for e in errs), errs)


class TestDynamics(unittest.TestCase):
    def setUp(self):
        self.model = core.full_chassis_model()
        self.d = json.loads(core.dynamics(json.dumps(self.model)))

    def test_steering_geometry(self):
        st = self.d["steering"]
        self.assertGreater(st["lock_inner_deg"], st["lock_outer_deg"])          # some Ackermann
        self.assertTrue(0 < st["ackermann_at_lock_pct"] < 100)
        # turning circle from the outer wheel angle: 2 L / sin(outer) + tyre width
        self.assertAlmostEqual(st["turning_circle_m"], 2 * 2.50 / math.sin(math.radians(st["lock_outer_deg"])) + 0.150, places=3)
        self.assertAlmostEqual(st["lock_to_lock_turns"], 140 / 65, places=3)

    def test_tie_rods_ahead_of_the_axle_still_steer(self):
        p = self.model["suspension"]["front"]["points"]
        wy, tri = p["WC"][1], core._point(self.model, p["TRI"])
        p["TRI"] = [tri[0], 2 * wy - tri[1], tri[2]]                  # mirror the steering fore-aft about the axle
        p["TRO"] = [p["TRO"][0], 2 * wy - p["TRO"][1], p["TRO"][2]]
        st = json.loads(core.dynamics(json.dumps(self.model)))["steering"]
        self.assertGreater(st["lock_outer_deg"], 20)                  # the rack is driven the other way
        self.assertLess(st["ackermann_at_lock_pct"], 0)               # mirrored geometry: anti-Ackermann, reported as such

    def test_quicker_rack_lowers_the_ratio(self):
        self.model["suspension"]["front"]["rack_mm_per_rev"] = 90
        d = json.loads(core.dynamics(json.dumps(self.model)))
        self.assertLess(d["steering"]["overall_ratio"], self.d["steering"]["overall_ratio"])

    def test_front_arb_moves_load_transfer_forward(self):
        base = self.d["roll"]
        self.model["suspension"]["front"]["arb_Nm_per_deg"] = 500
        d = json.loads(core.dynamics(json.dumps(self.model)))["roll"]
        self.assertGreater(d["lltd_front"], base["lltd_front"])
        self.assertLess(d["roll_gradient_deg_per_g"], base["roll_gradient_deg_per_g"])

    def test_total_lateral_transfer_matches_statics(self):
        # sum of axle transfers x track = total overturning moment M g h (rigid-body identity, per g)
        r, m = self.d["roll"], json.loads(core.masses(json.dumps(self.model)))
        moment = r["load_transfer_front_N_per_g"] * 1.36 + r["load_transfer_rear_N_per_g"] * 1.36
        self.assertAlmostEqual(moment / (m["mass_total"] * 9.81 * m["cg"][2]), 1.0, delta=0.02)


class TestAero(unittest.TestCase):
    def setUp(self):
        self.model = core.full_chassis_model()

    def a(self, model=None):
        return json.loads(core.aero(json.dumps(model or self.model)))

    def test_example_is_legal_and_rear_biased(self):
        r = self.a()
        self.assertEqual(r["issues"], [])
        s = next(x for x in r["speeds"] if x["kmh"] == 120)
        self.assertGreater(s["downforce_rear_N"], 0)
        self.assertGreater(s["drag_N"], 0)

    def test_forces_scale_with_speed_squared(self):
        s = {x["kmh"]: x for x in self.a()["speeds"]}
        self.assertAlmostEqual(s[160]["drag_N"] / s[80]["drag_N"], 4.0, places=6)

    def test_thin_airfoil_lift_and_rules(self):
        wing = next(d for d in self.model["aero"] if d["kind"] == "wing")
        ar_e = wing["span"] / wing["chord"] * 1.9
        cl = 2 * math.pi * math.radians(wing["aoa_deg"]) * ar_e / (ar_e + 2)
        r = self.a()
        self.assertAlmostEqual(next(d for d in r["devices"] if d["id"] == wing["id"])["cl_down"], cl, places=6)
        wing["span"], wing["aoa_deg"] = 1.7, 20                    # too wide, and stalled
        rules = {i["rule"] for i in self.a()["issues"]}
        self.assertIn("7.4.2", rules)
        split = next(d for d in self.model["aero"] if d["kind"] == "splitter")
        split["position"][1] -= 0.20                               # 200 mm further forward
        self.assertIn("7.4.3", {i["rule"] for i in self.a()["issues"]})

    def test_svj_aero_block(self):
        d = json.loads(core.export_svj(json.dumps(self.model)))["aerodynamics"]
        self.assertGreater(d["reference"]["frontal_area"], 1.0)
        self.assertEqual({c["type"] for c in d["components"]}, {"wing", "splitter"})
        self.assertLess(next(c for c in d["components"] if c["type"] == "wing")["Cl_contribution"], 0)   # SVJ: negative = downforce


class TestFullCarExport(unittest.TestCase):
    """Full-car jbeam export (suspension part, refNodes, info.json). Structure only: unverified in-game."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.res = json.loads(core.build(json.dumps(core.full_chassis_model()), str(self.tmp / "car")))
        self.veh = self.tmp / "car" / "vehicles" / core.PART

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_passes_checker_with_suspension(self):
        self.assertTrue(self.res["passed"], [f for f in self.res["findings"] if f["level"] == "ERROR"])
        self.assertEqual(sorted(p.name for p in self.veh.iterdir()),
                         ["fbeam_editor_chassis.jbeam", "fbeam_editor_suspension.jbeam", "info.json", "race.pc"])

    def test_suspension_part_contents(self):
        from fbeam_checker import jbeam
        doc = jbeam.parse((self.veh / "fbeam_editor_suspension.jbeam").read_text(encoding="utf-8"))
        part = doc["fbeam_editor_suspension"]
        beams = jbeam.expand_table(part["beams"])
        roles = [b.get("fbeamRole", "member") for b in beams]
        self.assertEqual(roles.count("spring"), 4)                        # one coilover per corner
        springs = [b for b in beams if b.get("fbeamRole") == "spring"]
        self.assertEqual(sorted(b["beamSpring"] for b in springs), [25500, 25500, 68500, 68500])   # Covet rally rates
        self.assertTrue(all("fbeamSection" in b for b in beams if b.get("fbeamRole", "member") == "member"))

    def test_spec_wheels_mounted_like_the_covet(self):
        import fbeam_vehicle as fveh
        from fbeam_checker import jbeam
        doc = jbeam.parse((self.veh / "fbeam_editor_suspension.jbeam").read_text(encoding="utf-8"))
        slots = {r["name"]: r for r in jbeam.expand_table(doc[fveh.PART]["slots2"])}
        self.assertEqual(slots["wheel_F_4"]["default"], "wheel_17a_14x5.5_F")
        names = [r["name"] for r in jbeam.expand_table(doc[fveh.PART]["slots2"])]
        self.assertEqual(names[-1], fveh.WHEELDATA_PART)                # wheel data after the wheels
        off = slots["wheel_F_4"]["nodeOffset"]
        hub = core.full_chassis_model()["hubs"]["FL"]
        for local in fveh.RIM_AXLE_X:                                    # rim axle nodes straddle the wheel centre
            self.assertLess(abs(local + off["x"] - hub[0]), 0.14)
        self.assertEqual([off["y"], off["z"]], [hub[1], hub[2]])
        rows = jbeam.expand_table(doc[fveh.WHEELDATA_PART]["pressureWheels"])
        self.assertEqual(sorted(r["name"] for r in rows), ["FL", "FR", "RL", "RR"])
        fl = next(r for r in rows if r["name"] == "FL")
        self.assertEqual((fl["node1"], fl["node2"], fl["wheelDir"]), ("fw1ll", "fw1l", -1))
        pc = json.loads((self.veh / "race.pc").read_text(encoding="utf-8"))
        self.assertEqual(pc["parts"]["wheel_R_4"], "wheel_17a_14x5.5_R")

    def test_ref_nodes_form_a_frame(self):
        from fbeam_checker import jbeam
        doc = jbeam.parse((self.veh / "fbeam_editor_chassis.jbeam").read_text(encoding="utf-8"))
        ref = jbeam.expand_table(doc[core.PART]["refNodes"])[0]
        nodes = core.full_chassis_model()["nodes"]
        r = nodes[ref["ref"]]
        self.assertGreater(nodes[ref["left"]][0] - r[0], 0.2)          # left is +X
        self.assertGreater(nodes[ref["back"]][1] - r[1], 0.2)          # back is +Y
        self.assertGreater(nodes[ref["up"]][2] - r[2], 0.2)            # up is +Z


class TestBody(unittest.TestCase):
    def setUp(self):
        self.model = core.full_chassis_model()

    def b(self, model=None):
        return json.loads(core.body(json.dumps(model or self.model)))

    def test_body_mass_is_skin_area_times_material(self):
        r = self.b()
        self.assertAlmostEqual(r["mass"], r["skin_area_m2"] * 7850 * 0.0008, places=6)
        comp = next(c for c in self.model["components"] if c["kind"] == "body")
        self.assertAlmostEqual(comp["mass"], r["mass"], places=2)          # the component follows the shell
        self.assertEqual(r["issues"], [])

    def test_lighter_panels_mean_more_ballast(self):
        base = next(c for c in self.model["components"] if c["kind"] == "ballast")["mass"]
        self.model["body"]["material"] = "composite_2.0mm"
        filled = json.loads(core.fill_ballast(json.dumps(self.model)))["model"]
        self.assertGreater(next(c for c in filled["components"] if c["kind"] == "ballast")["mass"], base)
        self.assertEqual(json.loads(core.masses(json.dumps(filled)))["ballast_needed"], 0.0)

    def test_rules(self):
        self.model["body"].update(width=1.80, height=1.25, ground_clearance=0.08, closed_roof=False)
        rules = {i["rule"] for i in self.b()["issues"]}
        self.assertTrue({"6.3", "6.5", "6.7", "6.9"} <= rules)


class TestBodyMesh(unittest.TestCase):
    def test_side_triangulation_covers_the_profile_exactly(self):
        import fbeam_body as fb
        r = json.loads(core.body(json.dumps(core.full_chassis_model())))
        tris = fb._ear_clip(r["profile"])
        area = sum(abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2 for a, b, c in tris)
        self.assertAlmostEqual(area, r["side_area_m2"], places=9)
        self.assertEqual(len(tris), len(r["profile"]) - 2)
