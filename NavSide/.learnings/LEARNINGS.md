## [LRN-20260716-001] navside_env_sru_ort_interaction_validation

**Logged**: 2026-07-16T17:21:04+08:00
**Priority**: medium
**Status**: pending
**Area**: workflow

### Summary
Use `env_sru_ort` for NavSide short-run validation; the interaction state machine reached the MuJoCo main loop and passed basic mode checks.

### Details
`/home/ubuntu/miniconda3/envs/env_sru_ort/bin/python` can import the required NavSide runtime dependencies and load the MuJoCo XML. Short-run validation observed STANDBY, LOW_SPEED, EMERGENCY, MEDIUM_SPEED, and G-to-STANDBY behavior through the ANSI panel. Emergency forced `final_cmd` to zero while `policy_cmd` continued changing.

### Suggested Action
Prefer `env_sru_ort` for NavSide validation commands. Use robot-side logs or UDP capture for packet-count proof of A/G/F zero bursts.

### Metadata
- Source: investigation
- Related Files: `navside/sim.py`, `navside/runtime.py`, `navside/mode.py`
- Tags: navside, mujoco, udp, validation

---
