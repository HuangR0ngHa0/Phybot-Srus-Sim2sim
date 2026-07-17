## [ERR-20260716-001] navside_system_python_missing_runtime_deps

**Logged**: 2026-07-16T16:11:13+08:00
**Priority**: medium
**Status**: pending
**Area**: workflow

### Summary
NavSide short-run validation failed because system `/usr/bin/python3` lacks required runtime dependencies.

### Error
`ModuleNotFoundError: No module named 'numpy'`

`ModuleNotFoundError: No module named 'mujoco'`

### Context
- Commands attempted: `python3 -B scripts/run_nav.py --sim-control --config config/nav.yaml` and `python3 tools/check_mujoco_xml.py ...`
- Environment detail: `python3` resolved to `/usr/bin/python3`
- Static checks passed, and the failure happened before NavSide entered the interactive runtime.

### Suggested Fix
Locate the intended project Python or conda environment before running NavSide interaction tests; do not treat this as a code regression.

### Metadata
- Reproducible: yes
- Related Files: `scripts/run_nav.py`, `tools/check_mujoco_xml.py`

---
