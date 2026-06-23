import cv2
import numpy as np


def get_camera_images(renderer, data, cam_id):
    renderer.update_scene(data, camera=cam_id)
    rgb_img = renderer.render()
    renderer.enable_depth_rendering()
    renderer.update_scene(data, camera=cam_id)
    depth_img = renderer.render()
    renderer.disable_depth_rendering()
    depth_img = np.asarray(depth_img, dtype=np.float32)
    depth_img = np.clip(depth_img, 0.01, 20.0)
    return rgb_img, depth_img


def make_depth_viz(depth_img: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth_img, dtype=np.float32)
    finite = np.isfinite(depth)
    if not np.any(finite):
        return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
    valid = depth[finite]
    dmin = float(valid.min())
    dmax = float(valid.max())
    if abs(dmax - dmin) < 1e-6:
        norm = np.zeros_like(depth, dtype=np.uint8)
    else:
        norm = np.clip((depth - dmin) / (dmax - dmin), 0.0, 1.0)
        norm = (norm * 255.0).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
