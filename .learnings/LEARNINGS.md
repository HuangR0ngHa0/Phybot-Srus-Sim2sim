## [LRN-20260717-001] navside_camera_visualization_env

**Logged**: 2026-07-17T10:38:25+08:00
**Priority**: low
**Status**: pending
**Area**: workflow

### Summary
NavSide runtime environment has OpenCV available, but the default shell has no `python` command.

### Details
`python -c "import cv2"` failed because `python` was not found. `/home/ubuntu/miniconda3/envs/env_sru_ort/bin/python -c "import cv2; print(cv2.__version__)"` succeeded and reported `4.11.0`. NavSide GUI/debug commands should use the explicit `env_sru_ort` Python path.

### Suggested Action
Use `/home/ubuntu/miniconda3/envs/env_sru_ort/bin/python` for NavSide visualization and validation commands unless the user activates an equivalent conda environment.

### Metadata
- Source: investigation
- Related Files: NavSide/navside/sim.py, NavSide/navside/runtime.py
- Tags: navside, opencv, python-env, visualization

---
