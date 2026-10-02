"""Editor import shim for the 3D frame / truss solver (regulations 8.10.6, 9.4).

The solver moved to fbeam_checker.frame so the checker and the editor share one
implementation; fbeam_editor_core imports it under this name (Structure, torsion,
bending, G_OVER_E). Edit fbeam_checker/frame.py, not this file.
"""

from fbeam_checker.frame import *  # noqa: F401,F403
from fbeam_checker.frame import G_OVER_E, Structure, bending, torsion  # noqa: F401
