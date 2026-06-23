import json
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from .adapter import MujocoStateEstimator, SruNavAdapter
from .config import NavSideConfig, load_nav_config
from .state import SruRobotState


class NavSideApp:
    def __init__(
        self,
        config: NavSideConfig,
        command_sink: Optional[Callable[[np.ndarray], None]] = None,
    ):
        self.config = config
        self.command_sink = command_sink
        self.adapter = SruNavAdapter(
            encoder_path=config.encoder_path,
            policy_path=config.policy_path,
            dry_run_hz=config.dry_run_hz,
            min_depth=config.min_depth,
            max_depth=config.max_depth,
            verbose=config.verbose_sru,
        )
        self.state_estimator = MujocoStateEstimator()
        self.last_final_cmd = np.zeros(3, dtype=np.float32)

    @classmethod
    def from_config(
        cls,
        config_path: str,
        command_sink: Optional[Callable[[np.ndarray], None]] = None,
    ):
        return cls(load_nav_config(config_path), command_sink=command_sink)

    def default_state(self) -> SruRobotState:
        return SruRobotState(
            linear_vel_b=np.zeros(3, dtype=np.float32),
            angular_vel_b=np.zeros(3, dtype=np.float32),
            projected_gravity_b=np.array([0.0, 0.0, -1.0], dtype=np.float32),
            robot_pos_w=np.array([0.0, 0.0, 0.695], dtype=np.float32),
            robot_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        )

    def default_goal(self) -> np.ndarray:
        return np.asarray(self.config.default_goal_w, dtype=np.float32)

    def build_state_from_json(self, payload: str) -> SruRobotState:
        data = json.loads(payload)
        return SruRobotState(
            linear_vel_b=np.asarray(data.get("linear_vel_b", [0.0, 0.0, 0.0]), dtype=np.float32),
            angular_vel_b=np.asarray(data.get("angular_vel_b", [0.0, 0.0, 0.0]), dtype=np.float32),
            projected_gravity_b=np.asarray(
                data.get("projected_gravity_b", [0.0, 0.0, -1.0]), dtype=np.float32
            ),
            robot_pos_w=np.asarray(data.get("robot_pos_w", [0.0, 0.0, 0.695]), dtype=np.float32),
            robot_quat_wxyz=np.asarray(
                data.get("robot_quat_wxyz", [1.0, 0.0, 0.0, 0.0]), dtype=np.float32
            ),
        )

    def step(
        self,
        depth_img: np.ndarray,
        state: SruRobotState,
        target_pos_w: np.ndarray,
        timestamp: Optional[float] = None,
        control: bool = False,
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

        if control and self.command_sink is not None:
            if control_info["should_send"]:
                self.command_sink(control_info["final_cmd"])
            else:
                self.command_sink(np.zeros(3, dtype=np.float32))

        return {
            "diag": diag,
            "control": control_info,
            "goal_dist": goal_dist,
        }

    def demo_tick(self, control: bool = False):
        depth = np.full((480, 848), 1.0, dtype=np.float32)
        state = self.default_state()
        goal = self.default_goal()
        return self.step(depth_img=depth, state=state, target_pos_w=goal, timestamp=0.0, control=control)
