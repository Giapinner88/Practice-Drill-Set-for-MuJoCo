"""Read pose, RGB pixels and depth values from a named MuJoCo camera."""

from pathlib import Path

import matplotlib.pyplot as plt
import mujoco
import numpy as np

LESSON_DIR = Path(__file__).resolve().parent
MODEL_PATH = LESSON_DIR / "model.xml"
OUTPUT_DIR = LESSON_DIR / "artifacts"

CAMERA_NAME = "fixed_cam"
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240


def capture(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    renderer = mujoco.Renderer(model, height=IMAGE_HEIGHT, width=IMAGE_WIDTH)
    try:
        renderer.update_scene(data, camera=CAMERA_NAME)
        rgb = renderer.render().copy()

        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera=CAMERA_NAME)
        depth = renderer.render().copy()
    finally:
        renderer.close()
    return rgb, depth


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    camera_id = model.camera(CAMERA_NAME).id
    rgb, depth = capture(model, data)

    center_y, center_x = IMAGE_HEIGHT // 2, IMAGE_WIDTH // 2
    print(f"camera={CAMERA_NAME!r} id={camera_id}")
    print(f"position_world_m={data.cam_xpos[camera_id]}")
    print(f"rotation_camera_to_world=\n{data.cam_xmat[camera_id].reshape(3, 3)}")
    print(f"rgb: shape={rgb.shape} dtype={rgb.dtype} center_pixel={rgb[center_y, center_x]}")
    print(
        f"depth: shape={depth.shape} dtype={depth.dtype} "
        f"center_pixel={depth[center_y, center_x]:.4f} m"
    )

    OUTPUT_DIR.mkdir(exist_ok=True)
    plt.imsave(OUTPUT_DIR / "fixed_cam_rgb.png", rgb)
    plt.imsave(OUTPUT_DIR / "fixed_cam_depth.png", depth, cmap="viridis")
    print(f"saved={OUTPUT_DIR}")


if __name__ == "__main__":
    main()
