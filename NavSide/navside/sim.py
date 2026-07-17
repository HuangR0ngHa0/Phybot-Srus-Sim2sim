import time
import traceback
from pathlib import Path

import numpy as np
import yaml

from .mode import NavMode, NavModeController
from .runtime import NavSideApp
from .bridge import NavStatePacketV2, RobotComm
from .depth import get_camera_images
from .image import depth_to_rgb_uint8, resize_image, rgb_to_uint8
from .state import SruRobotState

try:
    import mujoco
    import mujoco.viewer
except Exception as exc:  # pragma: no cover - runtime dependency check
    raise RuntimeError(
        "NavSide sim requires the MuJoCo Python binding in the active Python environment."
    ) from exc


VIEWER_ONLY_MARKER_GROUP = 5
STRICT_ZED_MINI_RENDER_WIDTH = 1920
STRICT_ZED_MINI_RENDER_HEIGHT = 1200
CAMERA_VIEW_WIDTH = 640
CAMERA_VIEW_HEIGHT = 400
CAMERA_VIEW_HZ = 10.0


class CameraVisualizer:
    def __init__(self, enabled: bool):
        self.enabled = bool(enabled)
        self.cv2 = None
        self.window_name = "NavSide Camera: RGB | Depth"

    def _ensure_window(self):
        if not self.enabled or self.cv2 is not None:
            return
        try:
            import cv2

            self.cv2 = cv2
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        except Exception as exc:
            print(f"[NavSide] camera visualizer disabled: {exc}")
            self.enabled = False
            self.cv2 = None

    def update(self, rgb_img, depth_img):
        if not self.enabled:
            return
        self._ensure_window()
        if not self.enabled or self.cv2 is None:
            return

        rgb = resize_image(rgb_to_uint8(rgb_img), CAMERA_VIEW_WIDTH, CAMERA_VIEW_HEIGHT)
        depth = resize_image(depth_to_rgb_uint8(depth_img), CAMERA_VIEW_WIDTH, CAMERA_VIEW_HEIGHT)
        rgb = rgb.astype(np.uint8)
        depth = depth.astype(np.uint8)

        # MuJoCo renders RGB, while OpenCV imshow expects BGR.
        rgb_bgr = self.cv2.cvtColor(rgb, self.cv2.COLOR_RGB2BGR)
        depth_bgr = self.cv2.cvtColor(depth, self.cv2.COLOR_RGB2BGR)
        canvas = np.concatenate([rgb_bgr, depth_bgr], axis=1)
        self.cv2.imshow(self.window_name, canvas)
        key = self.cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            self.close()
            self.enabled = False

    def close(self):
        if self.cv2 is None:
            return
        try:
            self.cv2.destroyWindow(self.window_name)
        except Exception:
            pass


def _load_nav_config(config_path: str) -> dict:
    return yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))


def _resolve_path(config_path: str, raw_path: str) -> str:
    path = Path(raw_path)
    if path.is_absolute():
        return str(path)
    return str((Path(config_path).resolve().parent / path).resolve())


def _parse_goal(goal_text, default_goal):
    if goal_text is None:
        return np.asarray(default_goal, dtype=np.float32)
    parts = [p.strip() for p in goal_text.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError('--goal must be "x,y,z"')
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float32)


def _parse_vec3(vec_text, default_vec, arg_name):
    if vec_text is None:
        if default_vec is None:
            return None
        return np.asarray(default_vec, dtype=np.float32)
    parts = [p.strip() for p in vec_text.split(",") if p.strip()]
    if len(parts) != 3:
        raise ValueError(f'--{arg_name} must be "x,y,z"')
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float32)


def _yaw_to_quat_wxyz(yaw):
    half = 0.5 * float(yaw)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float64)


def _apply_spawn_to_mujoco(model, data, spawn_pos_w, spawn_yaw):
    if spawn_pos_w is None:
        return
    data.qpos[0:3] = np.asarray(spawn_pos_w, dtype=np.float64)
    if spawn_yaw is not None:
        data.qpos[3:7] = _yaw_to_quat_wxyz(spawn_yaw)
    mujoco.mj_forward(model, data)


def _mujoco_state_from_data(data):
    return SruRobotState(
        linear_vel_b=np.zeros(3, dtype=np.float32),
        angular_vel_b=np.zeros(3, dtype=np.float32),
        projected_gravity_b=np.array([0.0, 0.0, -1.0], dtype=np.float32),
        robot_pos_w=np.asarray(data.qpos[0:3], dtype=np.float32).copy(),
        robot_quat_wxyz=np.asarray(data.qpos[3:7], dtype=np.float32).copy(),
    )


def _set_viewer_only_marker(model, name, pos_w, rgba, size):
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id == -1:
        return False
    model.geom_pos[geom_id, 0:3] = np.asarray(pos_w, dtype=np.float64)
    model.geom_size[geom_id, 0] = float(size)
    model.geom_rgba[geom_id, 0:4] = np.asarray(rgba, dtype=np.float32)
    model.geom_group[geom_id] = VIEWER_ONLY_MARKER_GROUP
    model.geom_contype[geom_id] = 0
    model.geom_conaffinity[geom_id] = 0
    return True


def _configure_viewer_markers(model, spawn_pos_w, goal_pos_w):
    goal_marker_pos = np.asarray(goal_pos_w, dtype=np.float64).copy()
    goal_marker_pos[2] = max(float(goal_marker_pos[2]), 0.05)
    goal_ok = _set_viewer_only_marker(
        model,
        name="goal",
        pos_w=goal_marker_pos,
        rgba=[1.0, 0.0, 0.0, 1.0],
        size=0.12,
    )

    spawn_ok = False
    if spawn_pos_w is not None:
        spawn_marker_pos = np.asarray(spawn_pos_w, dtype=np.float64).copy()
        spawn_marker_pos[2] = max(float(spawn_marker_pos[2]), 0.08)
        spawn_ok = _set_viewer_only_marker(
            model,
            name="traj0",
            pos_w=spawn_marker_pos,
            rgba=[0.0, 0.35, 1.0, 1.0],
            size=0.12,
        )
    return spawn_ok, goal_ok


def _enable_viewer_marker_group(viewer):
    try:
        with viewer.lock():
            viewer.opt.geomgroup[VIEWER_ONLY_MARKER_GROUP] = 1
    except Exception:
        pass


def _validate_inputs(depth_img, state, target_pos_w):
    if depth_img is None:
        return False, "depth_missing"
    if state is None:
        return False, "state_missing"
    if target_pos_w is None:
        return False, "target_missing"

    depth_arr = np.asarray(depth_img, dtype=np.float32)
    if depth_arr.size == 0 or not np.all(np.isfinite(depth_arr)):
        return False, "depth_nan_or_inf"

    for name in (
        "linear_vel_b",
        "angular_vel_b",
        "projected_gravity_b",
        "robot_pos_w",
        "robot_quat_wxyz",
    ):
        arr = np.asarray(getattr(state, name), dtype=np.float32)
        if arr.size == 0 or not np.all(np.isfinite(arr)):
            return False, f"{name}_nan_or_inf"

    if np.linalg.norm(np.asarray(state.linear_vel_b, dtype=np.float32)) > 10.0:
        return False, "linear_vel_b_out_of_range"
    if np.linalg.norm(np.asarray(state.angular_vel_b, dtype=np.float32)) > 20.0:
        return False, "angular_vel_b_out_of_range"
    gravity_norm = np.linalg.norm(np.asarray(state.projected_gravity_b, dtype=np.float32))
    if gravity_norm < 0.5 or gravity_norm > 1.5:
        return False, "projected_gravity_b_out_of_range"
    quat_norm = np.linalg.norm(np.asarray(state.robot_quat_wxyz, dtype=np.float32))
    if quat_norm < 0.5 or quat_norm > 1.5:
        return False, "robot_quat_wxyz_out_of_range"

    target_arr = np.asarray(target_pos_w, dtype=np.float32)
    if target_arr.size == 0 or not np.all(np.isfinite(target_arr)):
        return False, "target_nan_or_inf"

    return True, ""


def _apply_udp_state_to_mujoco(data, state_packet):
    if state_packet is None:
        return
    pos = state_packet.robot_pos_w
    quat = state_packet.robot_quat_wxyz
    data.qpos[0] = pos[0]
    data.qpos[1] = pos[1]
    data.qpos[2] = pos[2]
    data.qpos[3] = quat[0]
    data.qpos[4] = quat[1]
    data.qpos[5] = quat[2]
    data.qpos[6] = quat[3]


def _state_source_name(state_packet):
    if state_packet is None:
        return "mujoco_mirror"
    return getattr(state_packet, "source", "nav_state_v2")


def _validate_strict_zed_mini_render_size(render_width, render_height, config_path):
    if (
        render_width < STRICT_ZED_MINI_RENDER_WIDTH
        or render_height < STRICT_ZED_MINI_RENDER_HEIGHT
    ):
        raise ValueError(
            "Strict ZED Mini reproduction requires MuJoCo renderer size at least "
            f"{STRICT_ZED_MINI_RENDER_WIDTH}x{STRICT_ZED_MINI_RENDER_HEIGHT}; "
            f"got {render_width}x{render_height}. config={config_path}"
        )


def run(args):
    config = _load_nav_config(args.config)
    sim_cfg = config.get("sim", {})
    udp_cfg = config.get("udp", {})
    goal_cfg = config.get("goal", {})

    xml_path = args.mujoco_xml or sim_cfg.get("mujoco_xml_path")
    if not xml_path:
        raise ValueError("Missing sim.mujoco_xml_path in config or --mujoco-xml.")
    xml_path = _resolve_path(args.config, xml_path)
    camera_name = args.camera_name or sim_cfg.get("camera_name", "head_camera")
    render_height = int(sim_cfg.get("render_height", STRICT_ZED_MINI_RENDER_HEIGHT))
    render_width = int(sim_cfg.get("render_width", STRICT_ZED_MINI_RENDER_WIDTH))
    _validate_strict_zed_mini_render_size(render_width, render_height, args.config)
    goal = _parse_goal(args.goal, goal_cfg.get("default_goal_w", [5.0, 1.0, 0.0]))
    spawn_pos = _parse_vec3(args.spawn, sim_cfg.get("spawn_w"), "spawn")
    spawn_yaw = args.spawn_yaw if args.spawn_yaw is not None else sim_cfg.get("spawn_yaw")
    spawn_yaw = None if spawn_yaw is None else float(spawn_yaw)
    ignore_udp_state = bool(args.ignore_udp_state or sim_cfg.get("ignore_udp_state", False))
    log_cfg = config.get("logging", {})
    summary_hz = args.summary_hz if args.summary_hz is not None else float(log_cfg.get("summary_hz", 1.0))
    summary_interval = 1.0 / max(summary_hz, 1e-6)

    app = NavSideApp.from_config(args.config)
    app.adapter.verbose = bool(args.verbose_sru or log_cfg.get("verbose_sru", False))
    app.adapter.reset_recurrent_state()
    mode_controller = NavModeController()

    comm = RobotComm(
        local_ip=udp_cfg.get("local_ip", "127.0.0.1"),
        local_port=int(udp_cfg.get("local_port", 8081)),
        remote_ip=udp_cfg.get("remote_ip", "127.0.0.1"),
        remote_port=int(udp_cfg.get("remote_port", 8080)),
    )
    comm.daemon = True
    comm.start()
    print("[NavSide] sending UDP handshake zeros")
    for _ in range(5):
        comm.send_zero()
        time.sleep(0.1)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    _apply_spawn_to_mujoco(model, data, spawn_pos, spawn_yaw)
    spawn_marker_ok, goal_marker_ok = _configure_viewer_markers(model, spawn_pos, goal)
    mujoco.mj_forward(model, data)
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id == -1:
        raise ValueError(f"Camera not found in MuJoCo XML: {camera_name}")
    renderer = mujoco.Renderer(model, height=render_height, width=render_width)

    print(f"[NavSide] MuJoCo XML: {xml_path}")
    print(
        f"[NavSide] camera={camera_name} size={render_width}x{render_height} "
        f"goal={goal.tolist()}"
    )
    print(
        "[NavSide] spawn={} spawn_yaw={} markers(spawn={}, goal={}) marker_group={}".format(
            spawn_pos.tolist() if spawn_pos is not None else None,
            spawn_yaw,
            spawn_marker_ok,
            goal_marker_ok,
            VIEWER_ONLY_MARKER_GROUP,
        )
    )
    print(f"[NavSide] ignore_udp_state={ignore_udp_state}")
    camera_visualizer = CameraVisualizer(enabled=getattr(args, "show_camera", False))
    if camera_visualizer.enabled:
        print("[NavSide] camera visualizer enabled. Press q or Esc in the camera window to close it.")

    last_safe_cmd = np.zeros(3, dtype=np.float32)
    policy_cmd = np.zeros(3, dtype=np.float32)
    zero_reason = "standby"
    goal_dist = None
    last_print_time = 0.0
    last_output_time = 0.0
    last_camera_view_time = 0.0
    output_interval = 1.0 / max(app.config.dry_run_hz, 1e-6)
    camera_view_interval = 1.0 / max(CAMERA_VIEW_HZ, 1e-6)

    try:
        mode_controller.start_input_thread()
        with mujoco.viewer.launch_passive(model, data) as viewer:
            _enable_viewer_marker_group(viewer)
            while viewer.is_running():
                now = time.time()
                state_packet = None if ignore_udp_state else comm.get_latest_state()
                if state_packet is not None:
                    _apply_udp_state_to_mujoco(data, state_packet)
                    mujoco.mj_forward(model, data)

                for event in mode_controller.poll_events():
                    if event.accepted and event.key in ("A", "G"):
                        app.adapter.reset_recurrent_state()
                        last_safe_cmd = np.zeros(3, dtype=np.float32)
                        policy_cmd = np.zeros(3, dtype=np.float32)
                        goal_dist = None
                        zero_reason = "quit_to_standby" if event.key == "G" else "standby"
                        mode_controller.update_status(
                            final_cmd=last_safe_cmd,
                            policy_cmd=policy_cmd,
                            goal_dist=goal_dist,
                            zero_reason=zero_reason,
                            state_source=_state_source_name(state_packet),
                            robot_xy=np.asarray(data.qpos[0:2], dtype=np.float32),
                        )
                        for _ in range(3):
                            comm.send_zero()
                        last_output_time = now
                    elif event.accepted and event.key == "F":
                        last_safe_cmd = np.zeros(3, dtype=np.float32)
                        zero_reason = "emergency"
                        for _ in range(3):
                            comm.send_zero()
                        last_output_time = now

                decision = mode_controller.get_decision()
                policy_tick = decision.run_policy and app.adapter.should_tick(now)

                if not decision.run_policy:
                    if now - last_output_time >= output_interval:
                        last_safe_cmd = np.zeros(3, dtype=np.float32)
                        policy_cmd = np.zeros(3, dtype=np.float32)
                        goal_dist = None
                        zero_reason = decision.zero_reason_override or "standby"
                        comm.send_command(0.0, 0.0, 0.0)
                        last_output_time = now
                    else:
                        last_safe_cmd = np.zeros(3, dtype=np.float32)
                        policy_cmd = np.zeros(3, dtype=np.float32)
                        zero_reason = decision.zero_reason_override or "standby"
                elif decision.mode == NavMode.EMERGENCY:
                    last_safe_cmd = np.zeros(3, dtype=np.float32)
                    zero_reason = decision.zero_reason_override or "emergency"
                    if not policy_tick and now - last_output_time >= output_interval:
                        comm.send_command(0.0, 0.0, 0.0)
                        last_output_time = now

                if (
                    camera_visualizer.enabled
                    and not policy_tick
                    and now - last_camera_view_time >= camera_view_interval
                ):
                    try:
                        rgb_img, depth_img = get_camera_images(
                            renderer,
                            data,
                            cam_id,
                            hidden_geom_groups=(VIEWER_ONLY_MARKER_GROUP,),
                        )
                        camera_visualizer.update(rgb_img, depth_img)
                        last_camera_view_time = now
                    except Exception:
                        traceback.print_exc()
                        camera_visualizer.close()
                        camera_visualizer.enabled = False

                if policy_tick:
                    try:
                        rgb_img, depth_img = get_camera_images(
                            renderer,
                            data,
                            cam_id,
                            hidden_geom_groups=(VIEWER_ONLY_MARKER_GROUP,),
                        )
                        camera_visualizer.update(rgb_img, depth_img)
                        last_camera_view_time = now
                        if isinstance(state_packet, NavStatePacketV2):
                            sru_state = state_packet.to_sru_robot_state()
                        else:
                            sru_state = _mujoco_state_from_data(data)
                        valid, invalid_reason = _validate_inputs(depth_img, sru_state, goal)
                        if not valid:
                            result = None
                            last_safe_cmd = np.zeros(3, dtype=np.float32)
                            policy_cmd = np.zeros(3, dtype=np.float32)
                            goal_dist = None
                            zero_reason = invalid_reason
                        else:
                            result = app.step(
                                depth_img=depth_img,
                                state=sru_state,
                                target_pos_w=goal,
                                timestamp=now,
                                vx_max=decision.vx_max,
                                wz_max=decision.wz_max,
                                print_control=False,
                            )
                            control_info = result["control"] if result else None
                            if control_info is None:
                                last_safe_cmd = np.zeros(3, dtype=np.float32)
                                policy_cmd = np.zeros(3, dtype=np.float32)
                                goal_dist = None
                                zero_reason = "policy_no_output"
                            else:
                                policy_cmd = np.asarray(control_info["raw_cmd"], dtype=np.float32)
                                goal_dist = result["goal_dist"]
                                if decision.mode == NavMode.EMERGENCY:
                                    last_safe_cmd = np.zeros(3, dtype=np.float32)
                                    zero_reason = decision.zero_reason_override or "emergency"
                                else:
                                    last_safe_cmd = np.asarray(control_info["final_cmd"], dtype=np.float32)
                                    zero_reason = control_info["zero_reason"]

                        if not np.all(np.isfinite(policy_cmd)):
                            policy_cmd = np.zeros(3, dtype=np.float32)
                        if not np.all(np.isfinite(last_safe_cmd)):
                            last_safe_cmd = np.zeros(3, dtype=np.float32)
                            if decision.mode == NavMode.EMERGENCY:
                                zero_reason = decision.zero_reason_override or "emergency"
                            else:
                                zero_reason = "command_nan_or_inf"

                    except Exception:
                        traceback.print_exc()
                        last_safe_cmd = np.zeros(3, dtype=np.float32)
                        policy_cmd = np.zeros(3, dtype=np.float32)
                        goal_dist = None
                        if decision.mode == NavMode.EMERGENCY:
                            zero_reason = decision.zero_reason_override or "emergency"
                        else:
                            zero_reason = "exception"

                    if decision.mode == NavMode.EMERGENCY:
                        last_safe_cmd = np.zeros(3, dtype=np.float32)
                        zero_reason = decision.zero_reason_override or "emergency"

                    comm.send_command(last_safe_cmd[0], last_safe_cmd[1], last_safe_cmd[2])
                    last_output_time = now

                mode_controller.update_status(
                    final_cmd=last_safe_cmd,
                    policy_cmd=policy_cmd,
                    goal_dist=goal_dist,
                    zero_reason=zero_reason,
                    state_source=_state_source_name(state_packet),
                    robot_xy=np.asarray(data.qpos[0:2], dtype=np.float32),
                )

                if now - last_print_time >= summary_interval:
                    print(mode_controller.render_panel(), end="", flush=True)
                    last_print_time = now

                _enable_viewer_marker_group(viewer)
                viewer.sync()

    except KeyboardInterrupt:
        print("[NavSide] interrupted by user.")
    finally:
        try:
            comm.send_zero()
        except Exception:
            pass
        try:
            mode_controller.stop_input_thread()
        except Exception:
            pass
        camera_visualizer.close()
        comm.stop()
