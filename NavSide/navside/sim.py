import time
import traceback
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from .app import NavSideApp
from .bridge import LegacyStatePacket, NavStatePacketV2, RobotComm
from .depth import get_camera_images, make_depth_viz


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
    if isinstance(state_packet, NavStatePacketV2):
        pos = state_packet.robot_pos_w
        quat = state_packet.robot_quat_wxyz
        data.qpos[0] = pos[0]
        data.qpos[1] = pos[1]
        data.qpos[2] = pos[2]
        data.qpos[3] = quat[0]
        data.qpos[4] = quat[1]
        data.qpos[5] = quat[2]
        data.qpos[6] = quat[3]
        return

    if isinstance(state_packet, LegacyStatePacket):
        data.qpos[0] = state_packet.x
        data.qpos[1] = state_packet.y
        quat_xyzw = R.from_euler("xyz", [0.0, 0.0, state_packet.yaw]).as_quat()
        data.qpos[3] = quat_xyzw[3]
        data.qpos[4] = quat_xyzw[0]
        data.qpos[5] = quat_xyzw[1]
        data.qpos[6] = quat_xyzw[2]


def _state_source_name(state_packet):
    if state_packet is None:
        return "mujoco_mirror"
    return getattr(state_packet, "source", "unknown")


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
    render_height = int(sim_cfg.get("render_height", 480))
    render_width = int(sim_cfg.get("render_width", 848))
    render_fps = float(sim_cfg.get("render_fps", 30.0))
    render_interval = 1.0 / max(render_fps, 1e-6)
    goal = _parse_goal(args.goal, goal_cfg.get("default_goal_w", [5.0, 1.0, 0.0]))
    log_cfg = config.get("logging", {})
    summary_hz = args.summary_hz if args.summary_hz is not None else float(log_cfg.get("summary_hz", 1.0))
    summary_interval = 1.0 / max(summary_hz, 1e-6)

    control_mode = bool(args.sim_control)
    app = NavSideApp.from_config(args.config)
    app.adapter.verbose = bool(args.verbose_sru or log_cfg.get("verbose_sru", False))

    comm = None
    if control_mode:
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
    else:
        print("[NavSide] sim dry-run: UDP command bridge disabled.")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id == -1:
        raise ValueError(f"Camera not found in MuJoCo XML: {camera_name}")
    renderer = mujoco.Renderer(model, height=render_height, width=render_width)

    print(f"[NavSide] MuJoCo XML: {xml_path}")
    print(f"[NavSide] camera={camera_name} size={render_width}x{render_height} goal={goal.tolist()}")

    latest_depth_viz = None
    last_safe_cmd = np.zeros(3, dtype=np.float32)
    last_print_time = 0.0
    last_render_time = 0.0

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                now = time.time()
                state_packet = comm.get_latest_state() if comm is not None else None
                if state_packet is not None:
                    _apply_udp_state_to_mujoco(data, state_packet)
                    mujoco.mj_forward(model, data)

                need_tick = app.adapter.should_tick(now)
                depth_img = None
                if need_tick:
                    _, depth_img = get_camera_images(renderer, data, cam_id)

                if need_tick:
                    try:
                        if isinstance(state_packet, NavStatePacketV2):
                            sru_state = state_packet.to_sru_robot_state()
                        else:
                            sru_state = app.state_estimator.extract(data, now)
                        valid, invalid_reason = _validate_inputs(depth_img, sru_state, goal)
                        if not valid:
                            result = None
                            last_safe_cmd = np.zeros(3, dtype=np.float32)
                            zero_reason = invalid_reason
                        else:
                            result = app.step(
                                depth_img=depth_img,
                                state=sru_state,
                                target_pos_w=goal,
                                timestamp=now,
                                control=False,
                            )
                            control_info = result["control"] if result else None
                            if control_info is None:
                                last_safe_cmd = np.zeros(3, dtype=np.float32)
                                zero_reason = "policy_no_output"
                            else:
                                last_safe_cmd = np.asarray(control_info["final_cmd"], dtype=np.float32)
                                zero_reason = control_info["zero_reason"]

                        if not np.all(np.isfinite(last_safe_cmd)):
                            last_safe_cmd = np.zeros(3, dtype=np.float32)
                            zero_reason = "command_nan_or_inf"

                        latest_depth_viz = make_depth_viz(depth_img) if depth_img is not None else latest_depth_viz
                    except Exception:
                        traceback.print_exc()
                        last_safe_cmd = np.zeros(3, dtype=np.float32)
                        zero_reason = "exception"

                    if control_mode and comm is not None:
                        comm.send_command(last_safe_cmd[0], last_safe_cmd[1], last_safe_cmd[2])

                    if now - last_print_time >= summary_interval:
                        print(
                            "[NavSide SIM] mode={} state_source={} cmd={} sent={} zero_reason={} robot_xy=({:.3f},{:.3f}) goal_dist={:.3f}".format(
                                "control" if control_mode else "dry-run",
                                _state_source_name(state_packet),
                                np.array2string(last_safe_cmd, precision=4),
                                bool(control_mode and comm is not None),
                                zero_reason,
                                float(data.qpos[0]),
                                float(data.qpos[1]),
                                float(result["goal_dist"]) if "result" in locals() and result else float("nan"),
                            )
                        )
                        last_print_time = now

                if now - last_render_time >= render_interval:
                    viewer.sync()
                    if latest_depth_viz is not None:
                        cv2.imshow("NavSide depth", latest_depth_viz)
                    cv2.waitKey(1)
                    last_render_time = now

    except KeyboardInterrupt:
        print("[NavSide] interrupted by user.")
    finally:
        if comm is not None:
            try:
                comm.send_zero()
            except Exception:
                pass
            comm.stop()
        cv2.destroyAllWindows()
