## [ERR-20260629-001] robotside_c2_merge_link_error

**Logged**: 2026-06-29T15:28:29+08:00
**Priority**: low
**Status**: pending
**Area**: workflow

### Summary
Merging Robot_C2 `RL_deploy_cpg` control code can fail at link time if C2 debug logging declarations/calls are restored without the `dataLog` function definition.

### Error
`undefined reference to rl_deploy_cpg::dataLog(Eigen::VectorXd&, std::ofstream&)`

### Context
- Command attempted: `cmake --build . --target main -j2` in `PhybotSofware/build`
- Environment: `WHICH_ENV=mujoco_sim_mini`
- Learned: C2 `SetDataToPackage()` writes `c2_policy_sim_walk.txt` via `dataLog`; current nav-modified branch had removed the helper.

### Suggested Fix
When reusing C2 `RL_deploy_cpg` logging fields, also restore `bool rl_deploy_cpg::dataLog(Eigen::VectorXd&, std::ofstream&)` in `RL_deploy_cpg/src/rl_deploy.cpp`.

### Metadata
- Reproducible: yes
- Related Files: `PhybotSofware/RL_deploy_cpg/src/rl_deploy.cpp`

---
