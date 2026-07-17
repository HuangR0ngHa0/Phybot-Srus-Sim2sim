import numpy as np


def get_camera_images(renderer, data, cam_id, hidden_geom_groups=()):
    old_groups = {}
    scene_option = getattr(renderer, "scene_option", None)
    if scene_option is not None:
        for group_id in hidden_geom_groups:
            old_groups[group_id] = int(scene_option.geomgroup[group_id])
            scene_option.geomgroup[group_id] = 0

    try:
        renderer.update_scene(data, camera=cam_id)
        rgb_img = renderer.render()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=cam_id)
        depth_img = renderer.render()
        renderer.disable_depth_rendering()
    finally:
        if scene_option is not None:
            for group_id, value in old_groups.items():
                scene_option.geomgroup[group_id] = value

    depth_img = np.asarray(depth_img, dtype=np.float32)
    depth_img = np.clip(depth_img, 0.01, 20.0)
    return rgb_img, depth_img
