import argparse
import json
from pathlib import Path

import numpy as np

from .app import NavSideApp
from .sim import run as run_mujoco_sim
from .state import SruRobotState


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "nav.yaml"


def parse_goal(goal_text):
    if goal_text is None:
        return None
    parts = [p.strip() for p in goal_text.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError('--goal must be "x,y,z"')
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float32)


def load_depth(depth_path):
    if depth_path is None:
        return np.full((480, 848), 1.0, dtype=np.float32)
    path = Path(depth_path)
    if path.suffix.lower() == ".npy":
        return np.load(path).astype(np.float32)
    raise ValueError("Only .npy depth input is supported in the NavSide smoke entry.")


def load_state(state_path):
    if state_path is None:
        return None
    data = json.loads(Path(state_path).read_text(encoding="utf-8"))
    return SruRobotState(
        linear_vel_b=np.asarray(data.get("linear_vel_b", [0.0, 0.0, 0.0]), dtype=np.float32),
        angular_vel_b=np.asarray(data.get("angular_vel_b", [0.0, 0.0, 0.0]), dtype=np.float32),
        projected_gravity_b=np.asarray(data.get("projected_gravity_b", [0.0, 0.0, -1.0]), dtype=np.float32),
        robot_pos_w=np.asarray(data.get("robot_pos_w", [0.0, 0.0, 0.695]), dtype=np.float32),
        robot_quat_wxyz=np.asarray(data.get("robot_quat_wxyz", [1.0, 0.0, 0.0, 0.0]), dtype=np.float32),
    )


def build_parser():
    parser = argparse.ArgumentParser(description="SRU-only NavSide launcher")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true", help="Load models and print one SRU diagnostic tick without sending commands.")
    mode.add_argument("--control", action="store_true", help="Load models and emit the clamped command for one SRU tick.")
    mode.add_argument("--sim-dry-run", action="store_true", help="Run the VIPlanner-compatible MuJoCo SRU loop without sending UDP commands.")
    mode.add_argument("--sim-control", action="store_true", help="Run the VIPlanner-compatible MuJoCo SRU loop and send UDP commands to robotside.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Path to NavSide config YAML.")
    parser.add_argument("--depth-npy", default=None, help="Optional .npy depth input for local smoke checks.")
    parser.add_argument("--state-json", default=None, help="Optional JSON state input for local smoke checks.")
    parser.add_argument("--goal", default=None, help='Optional goal override as "x,y,z".')
    parser.add_argument("--mujoco-xml", default=None, help="Optional MuJoCo XML override for sim mode.")
    parser.add_argument("--camera-name", default=None, help="Optional MuJoCo camera name override for sim mode.")
    parser.add_argument("--verbose-sru", action="store_true", help="Print full SRU per-tick diagnostics in sim mode.")
    parser.add_argument("--summary-hz", type=float, default=None, help="Summary log frequency in sim mode.")
    parser.add_argument("--once", action="store_true", help="Run one tick and exit (default behavior for smoke use).")
    return parser


def main():
    args = build_parser().parse_args()
    if args.sim_dry_run or args.sim_control:
        run_mujoco_sim(args)
        return

    app = NavSideApp.from_config(args.config)
    depth = load_depth(args.depth_npy)
    state = load_state(args.state_json)
    goal = parse_goal(args.goal)
    if state is None:
        state = app.default_state()
    if goal is None:
        goal = app.default_goal()

    result = app.step(depth_img=depth, state=state, target_pos_w=goal, timestamp=0.0, control=args.control)
    if result is not None:
        control_info = result["control"]
        print("[NavSide] final_cmd =", np.array2string(control_info["final_cmd"], precision=4))
        print("[NavSide] zero_reason =", control_info["zero_reason"])
        print("[NavSide] above_walk_threshold =", control_info["above_walk_threshold"])
