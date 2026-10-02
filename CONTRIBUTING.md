# Contributing to FBeam

## Rule changes

- Open an issue first, naming the regulation number (for example 8.10.3) and the problem.
- Give the engineering reason and, where possible, a number: a load case, a mass, a lap-time effect.
- If a limit changes, update `docs/regulations.md` and `rules/fbeam_v1.json` in the same pull request, and bump `version` in the rules file.

## Tool changes

- Standard library only for `fbeam_checker`, so it runs anywhere without installs.
- Add or update a test in `tests/` for every check. A new check needs a test that makes the demo car fail it.
- Run before pushing:

```
python -m unittest discover -s tests -t .
python -m fbeam_checker check examples/demo_car
python examples/cage/cage_calc.py
```

If `cage_calc.py` changes `examples/demo_cage`, commit the regenerated files.

## Facts and sources

Numbers taken from FIA, SAE or BeamNG sources must cite the page in the regulations' Sources section. BeamNG values from the community wiki are marked "to be confirmed in-game" until someone checks them in the game files.
