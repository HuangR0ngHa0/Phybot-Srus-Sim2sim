import os
import time
import sys
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from .image import resize_image
from .state import SruRobotState
from .timing import RateMeter, timing_log


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
        dry_run_hz: float = 8.0,
        min_depth: float = 0.25,
        max_depth: float = 10.0,
        policy_scale: np.ndarray = DEFAULT_POLICY_SCALE,
        verbose: bool = True,
        inference_backend: str = "tensorrt",
        encoder_engine_path: Optional[str] = None,
        policy_engine_path: Optional[str] = None,
    ):
        init_t0 = time.perf_counter()
        self.encoder_path = encoder_path
        self.policy_path = policy_path
        self.inference_backend = (inference_backend or "tensorrt").strip().lower()
        self.encoder_engine_path = encoder_engine_path or encoder_path
        self.policy_engine_path = policy_engine_path or policy_path
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
        self._timing_logged_first_encode_depth = False
        self._timing_logged_first_run_policy = False
        self.encoder_rate_meter = RateMeter("encoder_trt")
        self.policy_rate_meter = RateMeter("policy_trt")

        backend_setup_t0 = time.perf_counter()
        if self.inference_backend == "tensorrt":
            if not encoder_engine_path or not policy_engine_path:
                raise ValueError(
                    "TensorRT backend requires encoder_engine_path and policy_engine_path"
                )
            self._setup_tensorrt_backend()
        elif self.inference_backend == "onnxruntime":
            self._setup_onnxruntime_backend()
        else:
            raise ValueError(f"Unsupported inference backend: {self.inference_backend}")
        timing_log("adapter_backend_setup_total", time.perf_counter() - backend_setup_t0)

        print(
            "[SRU] adapter ready | backend={} encoder={} policy={} encoder_engine={} policy_engine={}".format(
                self.inference_backend,
                self.encoder_path,
                self.policy_path,
                self.encoder_engine_path,
                self.policy_engine_path,
            )
        )
        timing_log("adapter_init_total", time.perf_counter() - init_t0)

    def _check_model_path(self, path: str) -> None:
        if not os.path.isfile(path):
            raise FileNotFoundError(f"SRU model file not found: {path}")

    def _cpp_trt_build_dir(self) -> Path:
        return Path(__file__).resolve().parents[1] / "cpp_trt" / "build"

    def _setup_tensorrt_backend(self) -> None:
        self._check_model_path(self.encoder_engine_path)
        self._check_model_path(self.policy_engine_path)

        build_dir = self._cpp_trt_build_dir()
        if str(build_dir) not in sys.path:
            sys.path.insert(0, str(build_dir))

        import_t0 = time.perf_counter()
        try:
            import navside_trt
        except Exception as exc:
            raise RuntimeError(
                "TensorRT backend init failed | "
                f"backend={self.inference_backend} "
                f"extension_path={build_dir / 'navside_trt.so'} "
                f"encoder_engine_path={self.encoder_engine_path} "
                f"policy_engine_path={self.policy_engine_path} "
                f"error={exc}"
            ) from exc
        timing_log("adapter_tensorrt_extension_import", time.perf_counter() - import_t0)

        runner_t0 = time.perf_counter()
        try:
            self.trt_runner = navside_trt.NavSideTRTRunner(
                self.encoder_engine_path,
                self.policy_engine_path,
            )
        except Exception as exc:
            raise RuntimeError(
                "TensorRT backend runner init failed | "
                f"backend={self.inference_backend} "
                f"extension_path={build_dir / 'navside_trt.so'} "
                f"encoder_engine_path={self.encoder_engine_path} "
                f"policy_engine_path={self.policy_engine_path} "
                f"error={exc}"
            ) from exc
        timing_log("adapter_tensorrt_runner_construct", time.perf_counter() - runner_t0)

        self.encoder_session = None
        self.policy_session = None
        self.encoder_input_name = None
        self.encoder_output_name = None
        self.policy_output_names = None

    def _setup_onnxruntime_backend(self) -> None:
        import onnxruntime as ort

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

    @staticmethod
    def _rotation_matrix_from_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
        w, x, y, z = np.asarray(quat_wxyz, dtype=np.float32).reshape(4)
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float32,
        )

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

        if self.inference_backend == "tensorrt":
            policy_t0 = time.perf_counter()
            raw_action, self.h_state, self.c_state = self.trt_runner.run_policy(
                np.ascontiguousarray(obs, dtype=np.float32),
                np.ascontiguousarray(self.h_state, dtype=np.float32),
                np.ascontiguousarray(self.c_state, dtype=np.float32),
            )
            if not self._timing_logged_first_run_policy:
                timing_log("adapter_first_run_policy", time.perf_counter() - policy_t0)
                self._timing_logged_first_run_policy = True
            self.policy_rate_meter.tick()
        else:
            policy_t0 = time.perf_counter()
            outputs = self.policy_session.run(
                self.policy_output_names,
                {
                    "obs": np.ascontiguousarray(obs, dtype=np.float32),
                    "h_in": np.ascontiguousarray(self.h_state, dtype=np.float32),
                    "c_in": np.ascontiguousarray(self.c_state, dtype=np.float32),
                },
            )
            raw_action, self.h_state, self.c_state = outputs
            if not self._timing_logged_first_run_policy:
                timing_log("adapter_first_run_policy", time.perf_counter() - policy_t0)
                self._timing_logged_first_run_policy = True
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
        depth_resized = resize_image(depth, ENCODER_INPUT_WIDTH, ENCODER_INPUT_HEIGHT)
        depth_tensor = depth_resized[np.newaxis, np.newaxis, :, :].astype(np.float32)
        self.last_depth_preprocess_info = {
            "crop_shape": np.array(depth.shape, dtype=np.int32),
            "processed_shape": np.array(depth_tensor.shape, dtype=np.int32),
            "processed_minmax": np.array(
                [np.nanmin(depth_resized), np.nanmax(depth_resized)],
                dtype=np.float32,
            ),
        }
        if self.inference_backend == "tensorrt":
            encode_t0 = time.perf_counter()
            vae_output = self.trt_runner.encode_depth(np.ascontiguousarray(depth_tensor, dtype=np.float32))
            if not self._timing_logged_first_encode_depth:
                timing_log("adapter_first_encode_depth", time.perf_counter() - encode_t0)
                self._timing_logged_first_encode_depth = True
            self.encoder_rate_meter.tick()
        else:
            encode_t0 = time.perf_counter()
            vae_output = self.encoder_session.run(
                [self.encoder_output_name],
                {self.encoder_input_name: np.ascontiguousarray(depth_tensor, dtype=np.float32)},
            )[0]
            if not self._timing_logged_first_encode_depth:
                timing_log("adapter_first_encode_depth", time.perf_counter() - encode_t0)
                self._timing_logged_first_encode_depth = True
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

        rot_wb = self._rotation_matrix_from_quat_wxyz(robot_quat_wxyz)
        target_vec_w = target_pos_w - robot_pos_w
        target_vec_b = rot_wb @ target_vec_w

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
