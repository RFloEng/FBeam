"""Command line for the offline checker (regulations 9.1, 9.2, 10).

    python -m fbeam_checker check <submission> [--rules FILE] [--json FILE] [--passport FILE]
    python -m fbeam_checker hash  <submission>

check runs every offline check (checks.run_all) against a rules file (default
rules/fbeam_v1.json), prints the findings, and optionally writes the full report
as JSON and, if the car passes, an unsigned car passport (10.4). hash prints the
Submission's file hash (10.2).

Exit codes: 0 pass (or hash printed), 1 at least one ERROR finding, 2 the
Submission could not be read.
"""

import argparse
import json
import sys
from datetime import date
from pathlib import Path

from .checks import ERROR, WARN, run_all
from .submission import Submission, SubmissionError

DEFAULT_RULES = Path(__file__).resolve().parent.parent / "rules" / "fbeam_v1.json"


def main(argv=None):
    """Parse argv (None means sys.argv[1:]), run the command, return the exit code."""
    ap =argparse.ArgumentParser(prog="fbeam_checker", description="FBeam offline technical checker")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check", help="check a Submission folder or zip")
    c.add_argument("submission")
    c.add_argument("--rules", default=str(DEFAULT_RULES))
    c.add_argument("--json", help="write the full report as JSON")
    c.add_argument("--passport", help="write an unsigned car passport if the car passes")
    h = sub.add_parser("hash", help="print the file hash of a Submission")
    h.add_argument("submission")
    args = ap.parse_args(argv)

    try:
        s = Submission(args.submission)
    except (SubmissionError, ValueError) as exc:
        print(f"ERROR  {exc}")
        return 2

    if args.cmd == "hash":
        print(s.file_hash())
        return 0

    try:
        rules = json.loads(Path(args.rules).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        print(f"ERROR  cannot read rules file {args.rules}: {exc}")
        return 2
    findings, stats = run_all(s, rules)
    errors = [f for f in findings if f.level == ERROR]
    warns = [f for f in findings if f.level == WARN]
    passed = not errors

    m = s.manifest
    print(f"FBeam offline check  rules {rules['version']}  car {m.get('car_id')}  team {m.get('team')}")
    for f in findings:
        print(f"{f.level:5}  [{f.rule}] {f.message}")
    ch = stats.get("chassis", {})
    if ch:
        print(f"chassis: {ch['beams']} beams, beam mass {ch['beam_mass_kg']} kg, "
              f"required node mass {ch['required_node_mass_kg']} kg, actual {ch['actual_node_mass_kg']} kg, "
              f"excess counted as ballast {ch['excess_as_ballast_kg']} kg")
    dims = stats.get("dimensions", {})
    if dims:
        print("geometry (unladen jbeam): " + ", ".join(f"{k} {v}" for k, v in dims.items()))
    print(f"file hash    {stats['file_hash']}")
    print(f"jbeam digest {stats['jbeam_digest']}")
    print(f"RESULT: {'PASS' if passed else 'FAIL'}  ({len(errors)} errors, {len(warns)} warnings)")
    print("Not checked offline: ride height, ground clearance, Race Mass, wheel travel, brakes, impact and torsion tests (in-game harness).")

    report = {"car_id": m.get("car_id"), "team": m.get("team"), "rules_version": rules["version"], "passed": passed,
              "findings": [f.__dict__ for f in findings], "stats": stats}
    if args.json:
        Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")
    if args.passport:
        if passed:
            # Offline half of the passport (10.4); the harness fills the in-game fields and the organiser signs.
            passport = {"car_id": m.get("car_id"), "team": m.get("team"), "rules_version": rules["version"],
                        "beamng_version": m.get("beamng_version"), "file_hash": stats["file_hash"],
                        "jbeam_digest": stats["jbeam_digest"], "physics_hash": None, "race_mass_kg": None,
                        "tuning_ranges": rules["tuning"]["open_spec_variables"],
                        "offline_check": date.today().isoformat(), "signature": None,
                        "_note": "physics_hash and race_mass_kg come from the in-game harness; the organiser signs."}
            Path(args.passport).write_text(json.dumps(passport, indent=2), encoding="utf-8")
        else:
            print("No passport written: the car did not pass.")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
