import os
import time
from typing import Dict, Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from .state import SruRobotState


STATE_DIM = 16
DEPTH_EMBEDDING_DIM = 2560
LSTM_HIDDEN_DIM = 512
DEFAULT_POLICY_SCALE = np.array([1.5, 1.0, 1.0], dtype=np.float32)
ZED_MINI_CROP_WIDTH = 1728
ZED_MINI_CROP_HEIGHT = 1080
ENCODER_INPUT_WIDTH = 64
ENCODER_INPUT_HEIGHT = 40


class SruNavAdapter:
    """Host-side SRU navigation adapter for NavSide."""

    def __init__(
        self,
        encoder_path: str,
        policy_path: str,
        dry_run_hz: float = 5.0,
        min_depth: float = 0.25,
        max_depth: float = 10.0,
        policy_scale: np.ndarray = DEFAULT_POLICY_SCALE,
        verbose: bool = True,
    ):
        import onnxruntime as ort

        self.encoder_path = encoder_path
        self.policy_path = policy_path
        self.dry_run_interval = 1.0 / dry_run_hz
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.policy_scale = np.asarray(policy_scale, dtype=np.float32)
        self.verbose = verbose
        self.last_policy_time: Optional[float] = None
        self.last_action = np.zeros(3, dtype=np.float32)
        self.h_state = np.zeros((1, 1, LSTM_HIDDEN_DIM), dtype=np.float32)
        self.c_state = np.zeros((1, 1, LSTM_HIDDEN_DIM), dtype=np.float32)
        self.last_depth_preprocess_info: Dict[str, np.ndarray] = {}

        self._check_model_path(self.encoder_path)
        self._check_model_path(self.policy_path)

        try:
            ort.preload_dlls(directory="")
        except Exception as e:
            print(f"[SRU] preload_dlls warning: {e}")

        available_providers = ort.get_available_providers()
        if "CUDAExecutionProvider" in available_providers:
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        else:
            providers = ["CPUExecutionProvider"]

        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess_options.intra_op_num_threads = 4

        self.encoder_session = ort.InferenceSession(
            self.encoder_path, sess_options=sess_options, providers=providers
        )
        self.policy_session = ort.InferenceSession(
            self.policy_path, sess_options=sess_options, providers=providers
        )

        self.encoder_input_name = self.encoder_session.get_inputs()[0].name
        self.encoder_output_name = self.encoder_session.get_outputs()[0].name
        self.policy_output_names = [out.name for out in self.policy_session.get_outputs()]

        print(
            "[SRU] adapter ready | providers={} encoder={} policy={}".format(
                providers, self.encoder_path, self.policy_path
            )
        )

    def _check_model_path(self, path: str) -> None:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"SRU model file not found: {path}")

    def should_tick(self, now: float) -> bool:
        if self.last_policy_time is None:
            return True
        return (now - self.last_policy_time) >= self.dry_run_interval

    def step(
        self,
        depth_img: np.ndarray,
        state: SruRobotState,
        target_pos_w: np.ndarray,
        timestamp: Optional[float] = None,
    ) -> Optional[Dict[str, np.ndarray]]:
        now = time.time() if timestamp is None else timestamp
        if not self.should_tick(now):
            return None

        dt = None if self.last_policy_time is None else now - self.last_policy_time
        self.last_policy_time = now

        depth_feature = self.depth_preprocess(depth_img)
        target_position, target_vec_b = self.build_target_position(
            target_pos_w, state.robot_pos_w, state.robot_quat_wxyz
        )

        state_input = np.concatenate(
            [
                state.linear_vel_b.astype(np.float32),
                state.angular_vel_b.astype(np.float32),
                state.projected_gravity_b.astype(np.float32),
                self.last_action.astype(np.float32),
                target_position.astype(np.float32),
            ]
        )
        obs = np.concatenate([state_input, depth_feature])[np.newaxis].astype(np.float32)

        outputs = self.policy_session.run(
            self.policy_output_names,
            {
                "obs": obs,
                "h_in": self.h_state,
                "c_in": self.c_state,
            },
        )
        raw_action, self.h_state, self.c_state = outputs
        raw_action = raw_action.squeeze(0).astype(np.float32)
        cmd_vel = (np.tanh(raw_action) * self.policy_scale).astype(np.float32)
        self.last_action = raw_action.copy()

        diag = {
            "timestamp": np.array([now], dtype=np.float64),
            "dt": np.array([-1.0 if dt is None else dt], dtype=np.float64),
            "depth_shape": np.array(depth_img.shape, dtype=np.int32),
            "depth_minmax": np.array([np.nanmin(depth_img), np.nanmax(depth_img)], dtype=np.float32),
            "depth_crop_shape": self.last_depth_preprocess_info.get(
                "crop_shape", np.array([-1, -1], dtype=np.int32)
            ),
            "depth_processed_shape": self.last_depth_preprocess_info.get(
                "processed_shape", np.array([-1, -1, -1, -1], dtype=np.int32)
            ),
            "depth_processed_minmax": self.last_depth_preprocess_info.get(
                "processed_minmax", np.array([np.nan, np.nan], dtype=np.float32)
            ),
            "depth_feature_shape": np.array(depth_feature.shape, dtype=np.int32),
            "linear_vel_b": state.linear_vel_b.astype(np.float32),
            "angular_vel_b": state.angular_vel_b.astype(np.float32),
            "projected_gravity_b": state.projected_gravity_b.astype(np.float32),
            "robot_pos_w": state.robot_pos_w.astype(np.float32),
            "robot_quat_wxyz": state.robot_quat_wxyz.astype(np.float32),
            "target_position": target_position.astype(np.float32),
            "target_vec_b": target_vec_b.astype(np.float32),
            "obs_shape": np.array(obs.shape, dtype=np.int32),
            "raw_action": raw_action,
            "cmd_vel": cmd_vel,
            "zero_reason": None,
        }
        if self.verbose:
            self.print_diagnostics(diag)
        return diag

    def build_control_command(
        self,
        diag: Dict[str, np.ndarray],
        vx_max: float = 0.45,
        wz_max: float = 0.4,
        walk_threshold: float = 0.3,
    ) -> Dict[str, np.ndarray]:
        zero_reason = diag.get("zero_reason")
        raw_cmd = diag.get("cmd_vel")

        if raw_cmd is None:
            zero_reason = "cmd_missing"
            raw_cmd = np.zeros(3, dtype=np.float32)
        else:
            raw_cmd = np.asarray(raw_cmd, dtype=np.float32).reshape(3)

        if zero_reason is None and not np.all(np.isfinite(raw_cmd)):
            zero_reason = "cmd_nan_or_inf"

        if zero_reason is None:
            final_cmd = np.array(
                [
                    np.clip(float(raw_cmd[0]), 0.0, vx_max),
                    0.0,
                    np.clip(float(raw_cmd[2]), -wz_max, wz_max),
                ],
                dtype=np.float32,
            )
            if not np.all(np.isfinite(final_cmd)):
                zero_reason = "final_cmd_nan_or_inf"
        else:
            final_cmd = np.zeros(3, dtype=np.float32)


        above_walk_threshold = bool(final_cmd[0] > walk_threshold)
        should_send = zero_reason is None

        return {
            "raw_cmd": raw_cmd.astype(np.float32),
            "final_cmd": final_cmd.astype(np.float32),
            "above_walk_threshold": above_walk_threshold,
            "should_send": should_send,
            "zero_reason": zero_reason or "",
            "walk_threshold": float(walk_threshold),
            "vx_max": float(vx_max),
            "wz_max": float(wz_max),
        }

    def depth_preprocess(self, depth_img: np.ndarray) -> np.ndarray:
        depth = np.asarray(depth_img, dtype=np.float32).copy()
        depth = np.nan_to_num(depth, nan=0.0, posinf=self.max_depth * 2.0, neginf=0.0)
        depth[depth > self.max_depth] = 0.0
        depth[depth < self.min_depth] = 0.0
        depth = self._center_crop_depth(depth, ZED_MINI_CROP_WIDTH, ZED_MINI_CROP_HEIGHT)
        depth_resized = cv2.resize(
            depth,
            (ENCODER_INPUT_WIDTH, ENCODER_INPUT_HEIGHT),
            interpolation=cv2.INTER_LINEAR,
        )
        depth_tensor = depth_resized[np.newaxis, np.newaxis, :, :].astype(np.float32)
        self.last_depth_preprocess_info = {
            "crop_shape": np.array(depth.shape, dtype=np.int32),
            "processed_shape": np.array(depth_tensor.shape, dtype=np.int32),
            "processed_minmax": np.array(
                [np.nanmin(depth_resized), np.nanmax(depth_resized)],
                dtype=np.float32,
            ),
        }
        vae_output = self.encoder_session.run(
            [self.encoder_output_name], {self.encoder_input_name: depth_tensor}
        )[0]
        return vae_output.flatten().astype(np.float32)

    def _center_crop_depth(self, depth: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
        height, width = depth.shape[:2]
        if width < target_width or height < target_height:
            raise ValueError(
                "Depth image is smaller than the strict ZED Mini crop: "
                f"got {width}x{height}, need at least {target_width}x{target_height}"
            )
        x0 = (width - target_width) // 2
        y0 = (height - target_height) // 2
        return depth[y0 : y0 + target_height, x0 : x0 + target_width]

    def build_target_position(
        self,
        target_pos_w: np.ndarray,
        robot_pos_w: np.ndarray,
        robot_quat_wxyz: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        target_pos_w = np.asarray(target_pos_w, dtype=np.float32).reshape(3)
        robot_pos_w = np.asarray(robot_pos_w, dtype=np.float32).reshape(3)
        robot_quat_wxyz = np.asarray(robot_quat_wxyz, dtype=np.float32).reshape(4)

        quat_xyzw = np.array(
            [robot_quat_wxyz[1], robot_quat_wxyz[2], robot_quat_wxyz[3], robot_quat_wxyz[0]],
            dtype=np.float32,
        )
        rot_wb = R.from_quat(quat_xyzw)
        target_vec_w = target_pos_w - robot_pos_w
        target_vec_b = rot_wb.inv().apply(target_vec_w).astype(np.float32)

        dist = float(np.linalg.norm(target_vec_b) + 1e-6)
        target_dir_b = target_vec_b / dist
        target_position = np.concatenate(
            [target_dir_b, np.array([np.log(dist + 1.0)], dtype=np.float32)]
        )
        return target_position.astype(np.float32), target_vec_b.astype(np.float32)

    def print_diagnostics(self, diag: Dict[str, np.ndarray]) -> None:
        dt = diag["dt"][0]
        dt_text = "first" if dt < 0 else f"{dt:.3f}s"
        print(
            "[SRU DRY] tick={:.3f} dt={} depth shape={} min/max={:.3f}/{:.3f} "
            "crop shape={} processed shape={} processed min/max={:.3f}/{:.3f} "
            "depth_feature shape={} obs shape={}".format(
                diag["timestamp"][0],
                dt_text,
                tuple(diag["depth_shape"].tolist()),
                diag["depth_minmax"][0],
                diag["depth_minmax"][1],
                tuple(diag["depth_crop_shape"].tolist()),
                tuple(diag["depth_processed_shape"].tolist()),
                diag["depth_processed_minmax"][0],
                diag["depth_processed_minmax"][1],
                tuple(diag["depth_feature_shape"].tolist()),
                tuple(diag["obs_shape"].tolist()),
            )
        )
        print(
            "[SRU DRY] linear_vel_b={} angular_vel_b={} projected_gravity_b={}".format(
                np.array2string(diag["linear_vel_b"], precision=4),
                np.array2string(diag["angular_vel_b"], precision=4),
                np.array2string(diag["projected_gravity_b"], precision=4),
            )
        )
        print(
            "[SRU DRY] robot_pos_w={} robot_quat_wxyz={} target_position={}".format(
                np.array2string(diag["robot_pos_w"], precision=4),
                np.array2string(diag["robot_quat_wxyz"], precision=4),
                np.array2string(diag["target_position"], precision=4),
            )
        )
        print(
            "[SRU DRY] raw_action={} postprocessed cmd_vel={}".format(
                np.array2string(diag["raw_action"], precision=4),
                np.array2string(diag["cmd_vel"], precision=4),
            )
        )
