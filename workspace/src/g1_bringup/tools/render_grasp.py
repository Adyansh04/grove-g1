#!/usr/bin/env python3
"""Draws grasps from g1_manipulation's grasp_eval snapshots, for looking at how a hand holds.

Each snapshot holds the joints and the objects (in the pinned pelvis's frame) at one moment of a
pick. The world is rebuilt from its scene with the hand model the snapshot's joints name, posed,
and drawn offscreen from three sides of the object, labelled with grasp_eval's score where its
results.json is beside the snapshots. A sheet of every grasp is written too.

Host-side tooling with MuJoCo's Python bindings (the worldgen venv), after a grasp_eval run:

    MUJOCO_GL=egl ~/ref/grove-worldgen/.venv/bin/python \\
        workspace/src/g1_bringup/tools/render_grasp.py data/grasp-eval/run1

Python because it is offline tooling around MuJoCo's own renderer; it runs nowhere in the stack.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco  # noqa: E402  (after MUJOCO_GL)
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

BRINGUP = pathlib.Path(__file__).resolve().parents[1]
MODEL_DIR = BRINGUP.parents[1] / "vendor" / "unitree_mujoco" / "unitree_robots" / "g1"
STAGED = "g1_render.staged.xml"
WIDTH, HEIGHT = 960, 720
# Views around the object: (azimuth, elevation) in degrees, distance in metres.
VIEWS = {"from the robot": (0.0, -35.0), "from the side": (90.0, -15.0), "from above": (0.0, -88.0)}
DISTANCE_M = 0.32
INSPIRE_SUBSTITUTIONS = (
    ('<include file="g1_29dof.xml"/>', '<include file="g1_29dof_inspire.xml"/>'),
    ('body1="left_hand_palm_link"', 'body1="L_hand_base_link"'),
    ('body1="right_hand_palm_link"', 'body1="R_hand_base_link"'),
)


def _font(size: int):
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default()


def stage(world: str, inspire: bool) -> pathlib.Path:
    """Writes the world's scene beside the robot model, as sim.launch.py stages it."""
    scene = (BRINGUP / "mjcf" / f"g1_{world}_scene.xml").read_text()
    if inspire:
        for old, new in INSPIRE_SUBSTITUTIONS:
            scene = scene.replace(old, new)
    path = MODEL_DIR / STAGED
    path.write_text(scene)
    return path


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def pose(model, data, snapshot: dict) -> None:
    """Puts the robot and the objects where the snapshot saw them."""
    data.qpos[:] = model.qpos0
    for name, value in snapshot["joints"].items():
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint >= 0:
            data.qpos[model.jnt_qposadr[joint]] = value
    # Joints coupled by an equality follow their driver, as the hand's linkages do.
    for eq in range(model.neq):
        if model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT and model.eq_obj2id[eq] >= 0:
            coef = model.eq_data[eq]
            driver = data.qpos[model.jnt_qposadr[model.eq_obj2id[eq]]]
            data.qpos[model.jnt_qposadr[model.eq_obj1id[eq]]] = coef[0] + coef[1] * driver
    mujoco.mj_forward(model, data)
    pelvis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    rotation = data.xmat[pelvis].reshape(3, 3)
    for name, obj in snapshot["objects"].items():
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body < 0 or model.body_jntnum[body] == 0:
            continue
        adr = model.jnt_qposadr[model.body_jntadr[body]]
        data.qpos[adr : adr + 3] = data.xpos[pelvis] + rotation @ np.array(obj["pos"])
        data.qpos[adr + 3 : adr + 7] = quat_mul(list(data.xquat[pelvis]), obj["quat"])
    mujoco.mj_forward(model, data)


def label(image: Image.Image, lines: list[str]) -> Image.Image:
    draw = ImageDraw.Draw(image)
    font = _font(26)
    draw.rectangle([0, 0, image.width, 14 + 32 * len(lines)], fill=(20, 24, 28))
    for i, line in enumerate(lines):
        draw.text((14, 8 + 32 * i), line, fill=(240, 240, 240), font=font)
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run", type=pathlib.Path, help="a grasp_eval --out folder")
    parser.add_argument("--world", default="grasp_eval")
    parser.add_argument("--phase", default="closed", help="closed or lifted")
    args = parser.parse_args()

    results = {}
    if (args.run / "results.json").is_file():
        results = {r["object"]: r for r in json.loads((args.run / "results.json").read_text())}
    snapshots = sorted((args.run / "snapshots").glob(f"*_{args.phase}.json"))
    if not snapshots:
        raise SystemExit(f"no *_{args.phase}.json under {args.run / 'snapshots'}")
    inspire = any(n.startswith("L_") for n in json.loads(snapshots[0].read_text())["joints"])
    path = stage(args.world, inspire)
    try:
        model = mujoco.MjModel.from_xml_path(str(path))
    finally:
        path.unlink()
    model.vis.global_.offwidth, model.vis.global_.offheight = WIDTH, HEIGHT
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, HEIGHT, WIDTH)
    out = args.run / "renders"
    out.mkdir(exist_ok=True)

    sheet_rows = []
    for snap_path in snapshots:
        snapshot = json.loads(snap_path.read_text())
        name = snapshot["object"]
        pose(model, data, snapshot)
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        row = results.get(name, {})
        grasp = row.get("grasp") or {}
        caption = [
            f"{name}, {snapshot['side']} hand, {args.phase}: {row.get('verdict', '?')}",
            f"table {row.get('table_peak_n', 0):.1f} N, grip off centre "
            f"{grasp.get('offset_mm', '-')} mm, digits {', '.join(grasp.get('digits', [])) or '-'}",
        ]
        images = []
        for view, (azimuth, elevation) in VIEWS.items():
            camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.lookat[:] = data.xpos[body]
            # Mirrored for the left hand, so the side view looks from outside the robot.
            side_sign = -1.0 if snapshot["side"] == "right" else 1.0
            camera.azimuth = azimuth * side_sign if view == "from the side" else azimuth
            camera.elevation = elevation
            camera.distance = DISTANCE_M
            renderer.update_scene(data, camera=camera)
            image = label(Image.fromarray(renderer.render()), [*caption, view])
            image.save(out / f"{name}_{args.phase}_{view.replace(' ', '_')}.png")
            images.append(image)
        sheet_rows.append(images)

    tile_w, tile_h = WIDTH // 2, HEIGHT // 2
    sheet = Image.new("RGB", (tile_w * len(VIEWS), tile_h * len(sheet_rows)), (255, 255, 255))
    for r, images in enumerate(sheet_rows):
        for c, image in enumerate(images):
            sheet.paste(image.resize((tile_w, tile_h), Image.LANCZOS), (c * tile_w, r * tile_h))
    sheet.save(out / f"sheet_{args.phase}.png")
    print(f"wrote {len(sheet_rows)} grasps to {out}")


if __name__ == "__main__":
    main()
