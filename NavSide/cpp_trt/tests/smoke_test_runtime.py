#!/usr/bin/env python3
"""Runtime smoke test for the NavSide TensorRT extension."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def main() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    build_dir = repo_root / "NavSide" / "cpp_trt" / "build"
    if str(build_dir) not in sys.path:
        sys.path.insert(0, str(build_dir))

    import navside_trt  # noqa: WPS433

    assert "onnxruntime" not in sys.modules, "onnxruntime must not be imported"

    runner = navside_trt.NavSideTRTRunner(
        "/home/amov/nav_arm_mujoco/NavSide/asset/models/vae_pretrain_new.plan",
        "/home/amov/nav_arm_mujoco/NavSide/asset/models/policy_1.plan",
    )

    depth = np.random.default_rng(0).standard_normal((1, 1, 40, 64), dtype=np.float32)
    obs = np.random.default_rng(1).standard_normal((1, 2576), dtype=np.float32)
    h_in = np.random.default_rng(2).standard_normal((1, 1, 512), dtype=np.float32)
    c_in = np.random.default_rng(3).standard_normal((1, 1, 512), dtype=np.float32)

    depth_features = runner.encode_depth(depth)
    actions, h_out, c_out = runner.run_policy(obs, h_in, c_in)

    print("encode_depth output:", depth_features.shape, depth_features.dtype)
    print("run_policy outputs:", actions.shape, h_out.shape, c_out.shape)

    assert depth_features.shape == (1, 64, 5, 8)
    assert actions.shape == (1, 3)
    assert h_out.shape == (1, 1, 512)
    assert c_out.shape == (1, 1, 512)


if __name__ == "__main__":
    main()
