import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from .adapter import SruNavAdapter
from .state import SruRobotState
from .timing import timing_log


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "nav.yaml"


@dataclass
class NavSideConfig:
    encoder_path: str
    policy_path: str
    encoder_engine_path: str
    policy_engine_path: str
    inference_backend: str = "tensorrt"
    dry_run_hz: float = 8.0
    vx_max: float = 1
    wz_max: float = 1.5
    walk_threshold: float = 0.3
    goal_pos_tolerance: float = 0.08
    min_depth: float = 0.25
    max_depth: float = 10.0
    verbose_sru: bool = True
    default_goal_w: tuple = (5.0, 1.0, 0.0)


def load_nav_config(config_path: str) -> NavSideConfig:
    t0 = time.perf_counter()
    data = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
    model_cfg = data.get("models", {})
    control_cfg = data.get("control", {})
    depth_cfg = data.get("depth", {})
    log_cfg = data.get("logging", {})
    goal_cfg = data.get("goal", {})
    base_dir = Path(config_path).resolve().parent
    config = NavSideConfig(
        encoder_path=str(base_dir / model_cfg.get("encoder_path", "../asset/models/vae_pretrain_new.onnx")),
        policy_path=str(base_dir / model_cfg.get("policy_path", "../asset/models/policy_1.onnx")),
        encoder_engine_path=str(base_dir / model_cfg.get("encoder_engine_path", "../asset/models/vae_pretrain_new.plan")),
        policy_engine_path=str(base_dir / model_cfg.get("policy_engine_path", "../asset/models/policy_1.plan")),
        inference_backend=str(model_cfg.get("inference_backend", "tensorrt")),
        dry_run_hz=float(control_cfg.get("dry_run_hz", 8.0)),
        vx_max=float(control_cfg.get("vx_max", 0.45)),
        wz_max=float(control_cfg.get("wz_max", 0.4)),
        walk_threshold=float(control_cfg.get("walk_threshold", 0.3)),
        goal_pos_tolerance=float(control_cfg.get("goal_pos_tolerance", 0.08)),
        min_depth=float(depth_cfg.get("min_depth", 0.25)),
        max_depth=float(depth_cfg.get("max_depth", 10.0)),
        verbose_sru=bool(log_cfg.get("verbose_sru", True)),
        default_goal_w=tuple(goal_cfg.get("default_goal_w", [5.0, 1.0, 0.0])),
    )
    timing_log("runtime_load_nav_config", time.perf_counter() - t0)
    return config


class NavSideApp:
    def __init__(self, config: NavSideConfig):
        t0 = time.perf_counter()
        self.config = config
        adapter_t0 = time.perf_counter()
        self.adapter = SruNavAdapter(
            encoder_path=config.encoder_path,
            policy_path=config.policy_path,
            dry_run_hz=config.dry_run_hz,
            min_depth=config.min_depth,
            max_depth=config.max_depth,
            verbose=config.verbose_sru,
            inference_backend=config.inference_backend,
            encoder_engine_path=config.encoder_engine_path,
            policy_engine_path=config.policy_engine_path,
        )
        timing_log("runtime_adapter_construct", time.perf_counter() - adapter_t0)
        self.last_final_cmd = np.zeros(3, dtype=np.float32)
        timing_log("runtime_app_init_total", time.perf_counter() - t0)

    @classmethod
    def from_config(cls, config_path: str):
        return cls(load_nav_config(config_path))

    def default_goal(self) -> np.ndarray:
        return np.asarray(self.config.default_goal_w, dtype=np.float32)

    def default_state(self) -> SruRobotState:
        return SruRobotState(
            linear_vel_b=np.zeros(3, dtype=np.float32),
            angular_vel_b=np.zeros(3, dtype=np.float32),
            projected_gravity_b=np.array([0.0, 0.0, -1.0], dtype=np.float32),
            robot_pos_w=np.array([0.0, 0.0, 0.695], dtype=np.float32),
            robot_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        )

    def step(
        self,
        depth_img: np.ndarray,
        state: SruRobotState,
        target_pos_w: np.ndarray,
        timestamp: float | None = None,
    ):
        diag = self.adapter.step(
            depth_img=depth_img,
            state=state,
            target_pos_w=target_pos_w,
            timestamp=timestamp,
        )
        if diag is None:
            return None

        control_info = self.adapter.build_control_command(
            diag,
            vx_max=self.config.vx_max,
            wz_max=self.config.wz_max,
            walk_threshold=self.config.walk_threshold,
        )

        goal_dist = float(np.linalg.norm(np.asarray(diag["target_vec_b"], dtype=np.float32)))
        if goal_dist <= self.config.goal_pos_tolerance:
            control_info["final_cmd"] = np.zeros(3, dtype=np.float32)
            control_info["should_send"] = True
            control_info["zero_reason"] = "goal_reached"
            control_info["above_walk_threshold"] = False

        self.last_final_cmd = np.asarray(control_info["final_cmd"], dtype=np.float32).copy()
        print(
            "[NavSide] control raw={} final={} walk_threshold={} above_walk_threshold={} zero_reason={} goal_dist={:.4f}".format(
                np.array2string(control_info["raw_cmd"], precision=4),
                np.array2string(control_info["final_cmd"], precision=4),
                control_info["walk_threshold"],
                control_info["above_walk_threshold"],
                control_info["zero_reason"],
                goal_dist,
            )
        )

        return {
            "diag": diag,
            "control": control_info,
            "goal_dist": goal_dist,
        }


def parse_goal(goal_text):
    if goal_text is None:
        return None
    parts = [p.strip() for p in goal_text.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError('--goal must be "x,y,z"')
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float32)


def build_parser():
    parser = argparse.ArgumentParser(description="SRU-only NavSide launcher")
    parser.add_argument(
        "--sim-control",
        action="store_true",
        required=True,
        help="Run the MuJoCo SRU loop and send UDP commands to robotside.",
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Path to NavSide config YAML.")
    parser.add_argument("--goal", default=None, help='Optional goal override as "x,y,z".')
    parser.add_argument("--spawn", default=None, help='Optional robot spawn override as "x,y,z".')
    parser.add_argument("--spawn-yaw", type=float, default=None, help="Optional robot spawn yaw override in radians.")
    parser.add_argument(
        "--ignore-udp-state",
        action="store_true",
        help="Do not mirror incoming robot-side UDP state into the NavSide MuJoCo viewer.",
    )
    parser.add_argument("--mujoco-xml", default=None, help="Optional MuJoCo XML override for sim mode.")
    parser.add_argument("--camera-name", default=None, help="Optional MuJoCo camera name override for sim mode.")
    parser.add_argument("--verbose-sru", action="store_true", help="Print full SRU per-tick diagnostics in sim mode.")
    parser.add_argument("--summary-hz", type=float, default=None, help="Summary log frequency in sim mode.")
    return parser


def main():
    t0 = time.perf_counter()
    args = build_parser().parse_args()
    if not args.sim_control:
        raise SystemExit("--sim-control is required.")

    from .sim import run as run_mujoco_sim

    timing_log("runtime_main_pre_sim", time.perf_counter() - t0)
    run_mujoco_sim(args)
