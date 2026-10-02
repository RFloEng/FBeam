"""Entry point for python -m fbeam_checker; see cli.main for commands and exit codes."""

import sys

from .cli import main

sys.exit(main())
