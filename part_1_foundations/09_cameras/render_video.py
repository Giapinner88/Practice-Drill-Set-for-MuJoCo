"""Render a short MuJoCo trajectory and encode its RGB frames as MP4."""

import shutil
import subprocess
from pathlib import Path

import mujoco

from simulate import MODEL_PATH, target_position

OUTPUT_PATH = Path(__file__).resolve().parent / "artifacts" / "fixed_cam.mp4"

CAMERA_NAME = "fixed_cam"
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
VIDEO_FPS = 30
VIDEO_DURATION_S = 5.0


def render_frames(model: mujoco.MjModel, data: mujoco.MjData) -> list[bytes]:
    actuator_id = model.actuator("slide_position").id
    frame_period = 1.0 / VIDEO_FPS
    next_frame_time = 0.0
    frame_count = round(VIDEO_DURATION_S * VIDEO_FPS)
    frames: list[bytes] = []

    renderer = mujoco.Renderer(model, height=IMAGE_HEIGHT, width=IMAGE_WIDTH)
    try:
        while len(frames) < frame_count:
            if data.time + 1e-9 >= next_frame_time:
                renderer.update_scene(data, camera=CAMERA_NAME)
                frames.append(renderer.render().tobytes())
                next_frame_time += frame_period
            else:
                data.ctrl[actuator_id] = target_position(float(data.time))
                mujoco.mj_step(model, data)
    finally:
        renderer.close()
    return frames


def write_mp4(frames: list[bytes]) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to write MP4 video")

    OUTPUT_PATH.parent.mkdir(exist_ok=True)
    command = [
        "ffmpeg",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{IMAGE_WIDTH}x{IMAGE_HEIGHT}",
        "-framerate",
        str(VIDEO_FPS),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(OUTPUT_PATH),
    ]
    subprocess.run(command, input=b"".join(frames), check=True)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    frames = render_frames(model, data)
    write_mp4(frames)
    print(
        f"saved={OUTPUT_PATH} frames={len(frames)} fps={VIDEO_FPS} "
        f"duration={len(frames) / VIDEO_FPS:.3f} s"
    )


if __name__ == "__main__":
    main()
