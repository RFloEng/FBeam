"""FBeam offline technical checker for BeamNG.drive rallycross Submissions.

Reads a team's Submission (folder or zip with fbeam.json, jbeam files and one
.pc) and checks what can be checked without the game, against the limits in
rules/fbeam_v1.json (FBeam Technical Regulations v1.0, docs/regulations.md):

    jbeam.py       lenient jbeam parser, table expansion, $variable resolution
    submission.py  loads the Submission, assembles the active car, file hash (10.2)
    sections.py    declared tube / sheet sections and their beam limits (8.3, 8.4)
    checks.py      every offline check and the jbeam digest; run_all() is the entry
    bending.py     budgets for weightless bending beams (8.10), shared with the editor
    pickups.py     3D bracing of suspension pickup joints (7.1.6), shared with the editor
    frame.py       small 3D frame / truss solver for chassis rigidity (8.10.6), used by the editor
    cli.py         command line (python -m fbeam_checker check|hash ...)

Standard library only, Python >= 3.10, so the editor can run it in Pyodide.
"""

__version__ = "0.1.0"
