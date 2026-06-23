from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

import yaml


@dataclass
class NavSideConfig:
    encoder_path: str
    policy_path: str
    dry_run_hz: float = 5.0
    vx_max: float = 0.45
    wz_max: float = 0.4
    walk_threshold: float = 0.3
    goal_pos_tolerance: float = 0.08
    min_depth: float = 0.25
    max_depth: float = 10.0
    verbose_sru: bool = True
    default_goal_w: tuple = (5.0, 1.0, 0.0)


def load_nav_config(config_path: str) -> NavSideConfig:
    data: Dict[str, Any] = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    model_cfg = data.get("models", {})
    control_cfg = data.get("control", {})
    depth_cfg = data.get("depth", {})
    log_cfg = data.get("logging", {})
    goal_cfg = data.get("goal", {})
    base_dir = Path(config_path).resolve().parent
    return NavSideConfig(
        encoder_path=str(base_dir / model_cfg.get("encoder_path", "../models/vae_encoder.onnx")),
        policy_path=str(base_dir / model_cfg.get("policy_path", "../models/nav_policy.onnx")),
        dry_run_hz=float(control_cfg.get("dry_run_hz", 5.0)),
        vx_max=float(control_cfg.get("vx_max", 0.45)),
        wz_max=float(control_cfg.get("wz_max", 0.4)),
        walk_threshold=float(control_cfg.get("walk_threshold", 0.3)),
        goal_pos_tolerance=float(control_cfg.get("goal_pos_tolerance", 0.08)),
        min_depth=float(depth_cfg.get("min_depth", 0.25)),
        max_depth=float(depth_cfg.get("max_depth", 10.0)),
        verbose_sru=bool(log_cfg.get("verbose_sru", True)),
        default_goal_w=tuple(goal_cfg.get("default_goal_w", [5.0, 1.0, 0.0])),
    )
