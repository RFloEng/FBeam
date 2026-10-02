"""Load a team Submission (folder or zip) and assemble the active car.

A Submission (regulations 2.1) holds fbeam.json at its root (the manifest:
team, car_id, the .pc path, chassis_parts, axle and ballast nodes), the team's
jbeam files and one part config (.pc). Submission reads every file into memory,
parses every .jbeam, and computes the file hash (10.2). build_car() assembles
the nodes and beams of the parts the .pc selects, with $variables resolved, as
a Car for the checks. Positions are in metres, node weights in kg.
"""

import hashlib
import json
import math
import zipfile
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath

from . import jbeam

MANIFEST = "fbeam.json"

NODE_DEFAULT_WEIGHT = 25.0  # kg, BeamNG default nodeWeight
# BeamNG's defaults for a beam that does not set these (N/m, N*s/m, N, N); inf = never breaks
BEAM_DEFAULTS = {"beamSpring": 4300000.0, "beamDamp": 580.0, "beamDeform": 220000.0, "beamStrength": math.inf}


class SubmissionError(ValueError):
    """The Submission cannot be read: no manifest, missing .pc, or not a folder or zip."""


@dataclass
class Node:
    """A jbeam node: pos (x, y, z) in m after nodeOffset / nodeMove, weight in kg, props = its table row."""

    id: str
    pos: tuple
    weight: float
    part: str
    props: dict


@dataclass
class Beam:
    """A jbeam beam between node ids a and b; props = its table row with property rows applied."""

    a: str
    b: str
    part: str
    props: dict


@dataclass
class Car:
    """The active Team Parts as assembled by Submission.build_car()."""

    parts: dict                      # active part name -> part dict
    variables: dict                  # "$name" -> value
    variable_defs: dict              # "$name" -> {"default", "min", "max", "part"}
    nodes: dict = field(default_factory=dict)        # node id -> Node (a later part overwrites an earlier id)
    beams: list = field(default_factory=list)        # [Beam]
    unresolved: list = field(default_factory=list)   # messages for nodes that could not be evaluated offline
    duplicate_nodes: list = field(default_factory=list)   # [(node id, first part, second part)]; the later wins


class Submission:
    """A team Submission read from a folder or zip.

    Attributes: files {posix path: bytes}, manifest (fbeam.json), part_defs
    {part name: (file path, part dict)} over every .jbeam, parse_errors [str],
    duplicate_parts [(name, first file, second file)] (the later file wins in
    part_defs), and pc (the part config named by manifest["pc"]).
    Raises SubmissionError if the manifest or the .pc is missing; a malformed
    fbeam.json or .pc raises json.JSONDecodeError (a ValueError).
    """

    def __init__(self, path):
        self.path = Path(path)
        self.files = self._read_files(self.path)
        if MANIFEST not in self.files:
            raise SubmissionError(f"{MANIFEST} missing at the root of the Submission")
        self.manifest = json.loads(self.files[MANIFEST].decode("utf-8"))
        self.part_defs = {}          # part name -> (file path, part dict)
        self.parse_errors = []
        self.duplicate_parts = []
        for p, data in sorted(self.files.items()):
            if not p.lower().endswith(".jbeam"):
                continue
            try:
                doc = jbeam.parse(data.decode("utf-8-sig"), p)
            except jbeam.JBeamError as exc:
                self.parse_errors.append(str(exc))
                continue
            if not isinstance(doc, dict):
                self.parse_errors.append(f"{p}: top level must be an object of parts")
                continue
            for name, part in doc.items():
                if name in self.part_defs:
                    self.duplicate_parts.append((name, self.part_defs[name][0], p))
                self.part_defs[name] = (p, part)
        pc_path = self.manifest.get("pc")
        if not pc_path or pc_path not in self.files:
            raise SubmissionError(f"part config {pc_path!r} named in {MANIFEST} is not in the Submission")
        self.pc = json.loads(self.files[pc_path].decode("utf-8"))

    @staticmethod
    def _read_files(path):
        """{relative posix path: bytes} of every file in the folder or zip."""
        files = {}
        if path.is_dir():
            for f in sorted(path.rglob("*")):
                if f.is_file():
                    files[f.relative_to(path).as_posix()] = f.read_bytes()
        elif zipfile.is_zipfile(path):
            with zipfile.ZipFile(path) as z:
                for info in z.infolist():
                    if not info.is_dir():
                        files[PurePosixPath(info.filename).as_posix()] = z.read(info)
        else:
            raise SubmissionError(f"{path} is neither a folder nor a zip file")
        return files

    def file_hash(self):
        """SHA-256 over sorted 'path NUL sha256(bytes)' lines (regulations 10.2).

        Independent of zip timestamps, compression and file order.
        """
        h = hashlib.sha256()
        for p in sorted(self.files):
            h.update(p.encode("utf-8") + b"\0" + hashlib.sha256(self.files[p]).hexdigest().encode() + b"\n")
        return "sha256:" + h.hexdigest()

    def active_part_names(self):
        """Sorted names of the parts selected in the .pc, plus any part with slotType "main".

        Only parts defined in the Submission count; Spec Parts from the game or
        the Spec Kit are not loaded. The slot tree is not walked, so a part
        selected in the .pc is active even if no active part offers its slot.
        """
        selected = set(v for v in self.pc.get("parts", {}).values() if v)
        names = [n for n, (_, part) in self.part_defs.items()
                 if n in selected or (isinstance(part, dict) and part.get("slotType") == "main")]
        return sorted(names)

    def build_car(self, use_defaults=False):
        """Assemble nodes and beams of the active Team Parts into a Car.

        Variables start at the defaults the active parts declare, then the .pc
        "vars" override them. use_defaults=True ignores the .pc variable values
        (used for the digest, so changing an open tuning variable does not
        change it). Nodes that cannot be evaluated are listed in car.unresolved
        instead of raising; beam values are left raw for the checks to resolve.
        """
        parts = {n: self.part_defs[n][1] for n in self.active_part_names()}
        defs = {}
        for name, part in parts.items():
            for rec in jbeam.expand_table(part.get("variables", [])):
                if "name" in rec:
                    defs[rec["name"]] = {"default": rec.get("default"), "min": rec.get("min"),
                                         "max": rec.get("max"), "part": name}
        variables = {k: v["default"] for k, v in defs.items() if v["default"] is not None}
        if not use_defaults:
            variables.update(self.pc.get("vars", {}))
        car = Car(parts, variables, defs)
        for name, part in parts.items():
            for rec in jbeam.expand_table(part.get("nodes", [])):
                if rec.get("id") in car.nodes:
                    car.duplicate_nodes.append((rec["id"], car.nodes[rec["id"]].part, name))
                try:
                    car.nodes[rec["id"]] = Node(rec["id"], _node_pos(rec, variables),
                                                jbeam.to_float(rec.get("nodeWeight"), variables, NODE_DEFAULT_WEIGHT),
                                                name, rec)
                except (jbeam.UnresolvedValue, KeyError, TypeError, ValueError) as exc:
                    car.unresolved.append(f"{name}: node {rec.get('id')}: {exc}")
            # the first two header columns (normally "id1:", "id2:") name the end nodes
            header = part.get("beams", [[]])[0] if part.get("beams") else []
            for rec in jbeam.expand_table(part.get("beams", [])):
                cols = [str(h).rstrip(":") for h in header]
                if len(cols) < 2 or cols[0] not in rec or cols[1] not in rec:
                    continue
                car.beams.append(Beam(rec[cols[0]], rec[cols[1]], name, rec))
        return car


def _node_pos(rec, variables):
    """Node position in m: posX/Y/Z plus nodeOffset (x mirrored to the node's side, as BeamNG does) and nodeMove."""
    x, y, z = (jbeam.to_float(rec[k], variables) for k in ("posX", "posY", "posZ"))
    off = rec.get("nodeOffset")
    if isinstance(off, dict):
        ox = jbeam.to_float(off.get("x", 0), variables)
        x += math.copysign(ox, x) if x != 0 else ox
        y += jbeam.to_float(off.get("y", 0), variables)
        z += jbeam.to_float(off.get("z", 0), variables)
    mv = rec.get("nodeMove")
    if isinstance(mv, dict):
        x += jbeam.to_float(mv.get("x", 0), variables)
        y += jbeam.to_float(mv.get("y", 0), variables)
        z += jbeam.to_float(mv.get("z", 0), variables)
    return (x, y, z)
