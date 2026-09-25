#!/usr/bin/env python3
"""Builds the apartment world from worlds/apartment.yaml and workspace/assets/manifest.yaml.

Writes, deterministically:
  mjcf/g1_apartment_scene.xml              the scene, asset paths under @GROVE_ASSETS@/
  worlds/apartment.truth.yaml              rooms, doorways and objects for scoring
  config/sim_sensors_apartment.yaml        sim_sensors.yaml tracking every prop
  g1_navigation/maps/apartment.{pgm,yaml}  the occupancy grid, rasterised from the scene

Host-side tooling in the worldgen venv, after tools/world_assets.py:

    ~/ref/grove-worldgen/.venv/bin/python workspace/src/g1_bringup/tools/build_world.py
"""

import argparse
import math
import pathlib
import re

import mujoco
import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

BRINGUP = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = BRINGUP.parents[1]
ASSETS = WORKSPACE / "assets"
MAPS = WORKSPACE / "src" / "g1_navigation" / "maps"

# sim.launch.py swaps this for the container's asset directory when it stages the scene.
PLACEHOLDER = "@GROVE_ASSETS@"

# The 2D scan keeps LiDAR returns in this band (g1_navigation's scan.yaml).
SCAN_BAND_M = (0.30, 1.50)
MAP_RESOLUTION_M = 0.05
MAP_MARGIN_M = 0.5
# map_server's trinary values, as map_saver_cli writes them.
OCCUPIED, FREE, UNKNOWN = 0, 254, 205

ROBOT_RADIUS_M = 0.45
# The D435i on the standing robot: optical centre height and how far it sits ahead of the pelvis.
CAMERA_HEIGHT_M = 1.21
CAMERA_FORWARD_M = 0.06
CAMERA_PITCH_DEG = 47.6
CAMERA_FOVY_DEG = 58.0

# The LiDAR sweep casts against group 2 only; the camera draws groups 0 to 2. Prop meshes are
# for the camera; their bounding-box proxies are what the LiDAR and physics see. Measured sweep
# cost is ~1.2x the facility's this way, against 2.3x to 3.7x with the meshes in group 2.
MESH_GROUP = 1
LIDAR_GROUP = 2

# Wall-face planes sit this far off the box, clear of depth-buffer fighting.
FACE_OFFSET_M = 0.002
# Lighting grid of the wall planes: the renderer lights per vertex.
FACE_GRID_M = 0.25
# Stands in for light bounced off the ceiling and floor, which MuJoCo does not model.
WALL_EMISSION = 0.2

BODY_NAME = re.compile(r"^([a-z][a-z_]*[a-z])_([0-9]+)$")
# sensor_frame.h's ObjectPoseRecord name field.
MAX_BODY_NAME = 31


def fmt(*values):
    """Shortest fixed-point form to 0.1 mm, so the output is stable and diffs stay readable."""
    out = []
    for value in values:
        text = f"{value:.4f}".rstrip("0").rstrip(".")
        out.append("0" if text in ("", "-0") else text)
    return " ".join(out)


def yaw_quat(yaw_deg):
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def rotate(xy, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    x, y = xy
    return (c * x - s * y, s * x + c * y)


# --- layout ---------------------------------------------------------------------------------


class Prop:
    """One static body: a single asset, or boxes and positioned asset meshes (the counter)."""

    def __init__(self, room, entry, manifest):
        self.room = room
        self.body = entry["body"]
        match = BODY_NAME.match(self.body)
        if not match or len(self.body) > MAX_BODY_NAME:
            raise ValueError(f"body {self.body!r} is not <class>_<k> within {MAX_BODY_NAME} chars")
        self.cls = match.group(1)
        self.asset = entry.get("asset")
        self.x, self.y, self.yaw = (float(v) for v in entry["pose"])
        # "on" would load as a boolean key under YAML 1.1.
        self.on = entry.get("support")
        self.z_override = entry.get("z")
        self.boxes = entry.get("boxes", [])
        if self.asset:
            self.meshes = [(self.asset, (0.0, 0.0, 0.0), 0.0)]
        else:
            self.meshes = [
                (m["asset"], tuple(m["pos"]), float(m.get("yaw", 0.0)))
                for m in entry.get("meshes", [])
            ]
        corners = []
        for box in self.boxes:
            corners += [
                np.subtract(box["pos"], np.divide(box["size"], 2)),
                np.add(box["pos"], np.divide(box["size"], 2)),
            ]
        for asset, pos, yaw in self.meshes:
            sx, sy, sz = manifest[asset]["bbox"]
            for dx in (-sx / 2, sx / 2):
                for dy in (-sy / 2, sy / 2):
                    x, y = rotate((dx, dy), yaw)
                    corners += [
                        (pos[0] + x, pos[1] + y, pos[2]),
                        (pos[0] + x, pos[1] + y, pos[2] + sz),
                    ]
        lo, hi = np.min(corners, 0), np.max(corners, 0)
        if abs(lo[0] + hi[0]) > 1e-3 or abs(lo[1] + hi[1]) > 1e-3 or abs(lo[2]) > 1e-3:
            raise ValueError(f"{self.body}: parts must be centred on the footprint, from z=0")
        self.size = [float(v) for v in hi - lo]
        self.z_base = 0.0

    @property
    def centre(self):
        return (self.x, self.y, self.z_base + self.size[2] / 2.0)


def wall_segments(plan):
    """Axis-aligned wall boxes with the doorways cut out, and a lintel over each doorway.

    A segment is (horizontal, fixed, start, end, jambs): `fixed` is the coordinate across the
    wall and `jambs` says which ends face a doorway, the only end faces left exposed.
    """
    (x0, y0), (x1, y1) = plan["outline"]
    lines = [
        [[x0, y0], [x1, y0]],
        [[x1, y0], [x1, y1]],
        [[x1, y1], [x0, y1]],
        [[x0, y1], [x0, y0]],
    ] + plan["walls"]
    thickness = plan["wall"]["thickness"]
    segments, lintels, used = [], [], set()
    for (ax, ay), (bx, by) in lines:
        horizontal = abs(ay - by) < 1e-9
        if not horizontal and abs(ax - bx) > 1e-9:
            raise ValueError(f"wall {(ax, ay)}-{(bx, by)} is not axis-aligned")
        fixed, lo, hi = (ay, *sorted((ax, bx))) if horizontal else (ax, *sorted((ay, by)))
        # Walls run half a thickness past their ends, so corners and junctions close.
        cuts = [(lo - thickness / 2.0, lo - thickness / 2.0, False)]
        for i, door in enumerate(plan["doorways"]):
            along, across = door["centre"] if horizontal else door["centre"][::-1]
            if abs(across - fixed) < 1e-9 and lo < along < hi and i not in used:
                cut = (along - door["width"] / 2.0, along + door["width"] / 2.0)
                cuts.append((*cut, True))
                lintels.append((horizontal, fixed, *cut, (False, False)))
                used.add(i)
        cuts.append((hi + thickness / 2.0, hi + thickness / 2.0, False))
        cuts.sort()
        for (_, start, door_before), (end, _, door_after) in zip(cuts, cuts[1:], strict=False):
            if end - start > 1e-6:
                segments.append((horizontal, fixed, start, end, (door_before, door_after)))
    if len(used) != len(plan["doorways"]):
        missing = [d["centre"] for i, d in enumerate(plan["doorways"]) if i not in used]
        raise ValueError(f"doorways not on any wall: {missing}")
    return segments, lintels


def wall_faces(segment, thickness, z_lo, z_hi):
    """Plane geoms over a wall box's exposed vertical faces: MuJoCo's box texturing smears a 2D
    texture across vertical faces, and a plane maps it by its own x-y."""
    horizontal, fixed, start, end, jambs = segment
    mid, half_len = (start + end) / 2, (end - start) / 2
    half_h, zc = (z_hi - z_lo) / 2, (z_hi + z_lo) / 2
    faces = []  # (centre x, centre y, outward normal x, y, half width)
    for side in (-1.0, 1.0):
        offset = fixed + side * (thickness / 2 + FACE_OFFSET_M)
        faces.append(
            (mid, offset, 0.0, side, half_len) if horizontal else (offset, mid, side, 0.0, half_len)
        )
    for jamb, at, side in ((jambs[0], start, -1.0), (jambs[1], end, 1.0)):
        if jamb:
            along = at + side * FACE_OFFSET_M
            faces.append(
                (along, fixed, side, 0.0, thickness / 2)
                if horizontal
                else (fixed, along, 0.0, side, thickness / 2)
            )
    # A plane faces its local +z; x = up x normal and y = up give that.
    return [
        f'pos="{fmt(x, y, zc)}" xyaxes="{fmt(-ny, nx, 0, 0, 0, 1)}" '
        f'size="{fmt(half_w, half_h, FACE_GRID_M)}"'
        for x, y, nx, ny, half_w in faces
    ]


# --- scene ----------------------------------------------------------------------------------


def asset_path(asset_id, file, root):
    return f"{root}/{asset_id}/{file}"


def mean_colour(texture_id):
    with Image.open(ASSETS / texture_id / "diffuse.png") as image:
        return (np.asarray(image.convert("RGB"), np.float64).mean((0, 1)) / 255.0).tolist()


def assets_xml(props, plan, manifest, root):
    """Textures, materials and meshes for these props, plus the building's when `plan` is given."""
    lines = []
    tiling = {b["texture"] for prop in props for b in prop.boxes if "texture" in b}
    if plan:
        tiling |= {plan["floor"], plan["walls_texture"]}
        tiling |= {p["texture"] for p in plan["floor_patches"]}
    for tex in sorted(tiling):
        tile_w, tile_h = manifest[tex]["tile_m"]
        emission = (
            f' emission="{fmt(WALL_EMISSION)}"' if plan and tex == plan["walls_texture"] else ""
        )
        lines.append(
            f'    <texture name="{tex}" type="2d" file="{asset_path(tex, "diffuse.png", root)}"/>'
        )
        lines.append(
            f'    <material name="{tex}" texture="{tex}" texuniform="true" '
            f'texrepeat="{fmt(1.0 / tile_w, 1.0 / tile_h)}" specular="0.1" '
            f'shininess="0.1"{emission}/>'
        )
    if plan:
        wall = mean_colour(plan["walls_texture"])
        lines.append(
            f'    <material name="wall_flat" rgba="{fmt(*wall, 1)}" specular="0.1" '
            f'shininess="0.1" emission="{fmt(WALL_EMISSION)}"/>'
        )
    for asset_id in sorted({asset for prop in props for asset, _, _ in prop.meshes}):
        written = set()
        for k, part in enumerate(manifest[asset_id]["parts"]):
            name = f"{asset_id}_{k}"
            lines.append(
                f'    <mesh name="{name}" file="{asset_path(asset_id, part["mesh"], root)}"/>'
            )
            material = (
                f'    <material name="{name}" rgba="{fmt(*part["rgba"])}" '
                f'specular="{fmt(part["specular"])}" shininess="{fmt(part["shininess"])}"'
            )
            if "texture" in part:
                tex_name = f"{asset_id}_{part['texture'].removesuffix('.png')}"
                if tex_name not in written:
                    lines.append(
                        f'    <texture name="{tex_name}" type="2d" '
                        f'file="{asset_path(asset_id, part["texture"], root)}"/>'
                    )
                    written.add(tex_name)
                material += f' texture="{tex_name}"'
            lines.append(material + "/>")
    return lines


def prop_xml(prop, manifest):
    half_h = prop.size[2] / 2.0
    quat = "" if prop.yaw % 360 == 0 else f' quat="{fmt(*yaw_quat(prop.yaw))}"'
    lines = [f'    <body name="{prop.body}" pos="{fmt(*prop.centre)}"{quat}>']
    for asset, (px, py, pz), yaw in prop.meshes:
        part_quat = "" if yaw % 360 == 0 else f' quat="{fmt(*yaw_quat(yaw))}"'
        for k in range(len(manifest[asset]["parts"])):
            name = f"{asset}_{k}"
            lines.append(
                f'      <geom class="apt_visual" mesh="{name}" material="{name}" '
                f'pos="{fmt(px, py, pz - half_h)}"{part_quat}/>'
            )
    for box in prop.boxes:
        pos = (box["pos"][0], box["pos"][1], box["pos"][2] - half_h)
        look = f'material="{box["texture"]}"' if "texture" in box else f'rgba="{fmt(*box["rgba"])}"'
        lines.append(
            f'      <geom class="apt_box" size="{fmt(*[s / 2 for s in box["size"]])}" '
            f'pos="{fmt(*pos)}" {look}/>'
        )
    # Boxes collide and size the ground truth themselves; mesh-only props get a bounding box.
    if not prop.boxes:
        lines.append(f'      <geom class="apt_proxy" size="{fmt(*[s / 2 for s in prop.size])}"/>')
    lines.append("    </body>")
    return lines


def scene_xml(plan, props, manifest, root):
    segments, lintels = wall_segments(plan)
    thickness, height = plan["wall"]["thickness"], plan["wall"]["height"]
    lintel_z = plan["wall"]["lintel"]
    (x0, y0), (x1, y1) = plan["outline"]
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    extent = 0.5 * math.hypot(x1 - x0, y1 - y0)

    out = [
        f'<mujoco model="g1_{plan["name"]}_scene">',
        f"  <!-- Generated by tools/build_world.py from worlds/{plan['name']}.yaml; edit that",
        "       and regenerate. Staged next to the vendored g1_29dof.xml at launch, which also",
        "       swaps @GROVE_ASSETS@ for the asset directory (workspace/assets, fetched by",
        "       tools/world_assets.py).",
        "       The LiDAR sweep sees group 2 only. Walls and floor are group 2 primitives, with",
        "       textured group 1 planes over the wall faces for the camera. Each prop is a static",
        "       body: textured meshes in group 1 for the camera, and a group 2 bounding box that",
        "       the LiDAR hits, the robot collides with and the ground truth is sized from. The",
        "       box is never drawn: mj_ray tests its material's alpha, which is 1, while the",
        "       renderer skips any geom whose own rgba alpha is 0. -->",
        "",
        '  <include file="g1_29dof.xml"/>',
        "",
        f'  <statistic center="{fmt(cx, cy, 1.0)}" extent="{fmt(round(extent, 1))}"/>',
        "",
        "  <visual>",
        '    <headlight diffuse="0.35 0.35 0.35" ambient="0.3 0.3 0.3" specular="0.1 0.1 0.1"/>',
        '    <rgba haze="0.15 0.2 0.25 1"/>',
        '    <global azimuth="120" elevation="-35" offwidth="1920" offheight="1080"/>',
        '    <quality shadowsize="4096"/>',
        "  </visual>",
        "",
        "  <default>",
        '    <default class="apt_visual">',
        f'      <geom type="mesh" group="{MESH_GROUP}" contype="0" conaffinity="0"/>',
        "    </default>",
        '    <default class="apt_proxy">',
        f'      <geom type="box" group="{LIDAR_GROUP}" contype="1" conaffinity="1" '
        'material="proxy" rgba="0.9 0.5 0.1 0"/>',
        "    </default>",
        '    <default class="apt_box">',
        f'      <geom type="box" group="{LIDAR_GROUP}" contype="1" conaffinity="1"/>',
        "    </default>",
        '    <default class="apt_wall">',
        f'      <geom type="box" group="{LIDAR_GROUP}" contype="1" conaffinity="1" '
        'material="wall_flat"/>',
        "    </default>",
        '    <default class="apt_wall_face">',
        f'      <geom type="plane" group="{MESH_GROUP}" contype="0" conaffinity="0" '
        f'material="{plan["walls_texture"]}"/>',
        "    </default>",
        "  </default>",
        "",
        "  <asset>",
        '    <texture type="skybox" builtin="gradient" rgb1="0.55 0.65 0.8" rgb2="0.15 0.2 0.3"',
        '             width="512" height="3072"/>',
        '    <material name="proxy" rgba="1 1 1 1"/>',
        *assets_xml(props, plan, manifest, root),
        "  </asset>",
        "",
        "  <worldbody>",
        f'    <geom name="floor" type="plane" group="{LIDAR_GROUP}" pos="{fmt(cx, cy, 0)}" '
        f'size="{fmt((x1 - x0 + thickness) / 2, (y1 - y0 + thickness) / 2, 0.1)}" '
        f'material="{plan["floor"]}"/>',
    ]
    # Visual only: the robot's feet and the LiDAR find the plain floor under them. Planes, not
    # boxes, so they are lit on the same grid as the floor and show no seam.
    for i, patch in enumerate(plan["floor_patches"]):
        (px0, py0), (px1, py1) = patch["rect"]
        out.append(
            f'    <geom name="floor_patch_{i}" type="plane" group="{MESH_GROUP}" contype="0" '
            f'conaffinity="0" pos="{fmt((px0 + px1) / 2, (py0 + py1) / 2, 0.002)}" '
            f'size="{fmt((px1 - px0) / 2, (py1 - py0) / 2, 0.1)}" '
            f'material="{patch["texture"]}"/>'
        )
    out.append("")
    for light in plan["lights"]:
        shadow = "true" if light.get("shadow") else "false"
        diffuse = fmt(*[light["diffuse"]] * 3)
        if light.get("directional"):
            out.append(
                f'    <light name="{light["name"]}" directional="true" '
                f'dir="{fmt(*light["dir"])}" pos="{fmt(cx, cy, 6)}" diffuse="{diffuse}" '
                f'specular="0.1 0.1 0.1" castshadow="{shadow}"/>'
            )
        else:
            out.append(
                f'    <light name="{light["name"]}" pos="{fmt(*light["pos"])}" dir="0 0 -1" '
                f'cutoff="75" exponent="1" diffuse="{diffuse}" specular="0.1 0.1 0.1" '
                f'castshadow="{shadow}"/>'
            )
    out.append("")
    walls = [(s, 0.0, height) for s in segments] + [(s, lintel_z, height) for s in lintels]
    for i, (segment, z_lo, z_hi) in enumerate(walls):
        horizontal, fixed, start, end, _ = segment
        mid, half_len = (start + end) / 2.0, (end - start) / 2.0
        pos = (mid, fixed) if horizontal else (fixed, mid)
        size = (half_len, thickness / 2) if horizontal else (thickness / 2, half_len)
        out.append(
            f'    <geom name="wall_{i}" class="apt_wall" pos="{fmt(*pos, (z_lo + z_hi) / 2)}" '
            f'size="{fmt(*size, (z_hi - z_lo) / 2)}"/>'
        )
        for face in wall_faces(segment, thickness, z_lo, z_hi):
            out.append(f'    <geom class="apt_wall_face" {face}/>')
    for room in plan["rooms"]:
        room_props = [p for p in props if p.room == room["id"]]
        if room_props:
            out += ["", f"    <!-- {room['name']} -->"]
            for prop in room_props:
                out += prop_xml(prop, manifest)
    out += [
        "",
        f'    <camera name="overview" pos="{fmt(cx, y0 - 9, 14)}" xyaxes="1 0 0 0 0.7 0.714" '
        'fovy="60"/>',
        "    <!-- D435i intrinsics only: the sensor sampler moves it to the torso mount every "
        "frame. -->",
        '    <camera name="d435i" mode="fixed" fovy="58" resolution="848 480" pos="0 0 1"/>',
        "  </worldbody>",
        "</mujoco>",
        "",
    ]
    return "\n".join(out)


def standalone(xml):
    """The scene without the robot, with host asset paths: what compiles on the host."""
    return xml.replace('<include file="g1_29dof.xml"/>', "").replace(PLACEHOLDER, str(ASSETS))


# --- placement ------------------------------------------------------------------------------


def footprint_samples(prop):
    """Centre, corners and edge midpoints of the footprint, 10% inside, in world x-y."""
    hx, hy = 0.45 * prop.size[0], 0.45 * prop.size[1]
    local = [(0, 0)] + [(sx * hx, sy * hy) for sx in (-1, 0, 1) for sy in (-1, 0, 1) if sx or sy]
    return [(prop.x + dx, prop.y + dy) for dx, dy in (rotate(p, prop.yaw) for p in local)]


def rest_on_supports(props, plan, manifest):
    """Sets each supported prop's base height to the highest support surface under it."""
    by_body = {p.body: p for p in props}
    for prop in props:
        if not prop.on:
            continue
        support = by_body[prop.on]
        if support.on:
            raise ValueError(f"{prop.body}: supports must stand on the floor")
        if prop.z_override is not None:
            prop.z_base = float(prop.z_override)
            continue
        # The support alone, what the camera sees of it: a table's proxy top is its highest point,
        # not the surface under the cup.
        xml = "\n".join(
            [
                "<mujoco>",
                "  <asset>",
                *assets_xml([support], None, manifest, str(ASSETS)),
                "  </asset>",
                "  <default>",
                f'<default class="apt_visual"><geom type="mesh" group="{MESH_GROUP}"/></default>',
                f'<default class="apt_box"><geom type="box" group="{LIDAR_GROUP}"/></default>',
                '<default class="apt_proxy"><geom type="box" group="5"/></default>',
                "  </default>",
                "  <worldbody>",
                *prop_xml(support, manifest),
                "  </worldbody>",
                "</mujoco>",
            ]
        )
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        group = np.zeros(mujoco.mjNGROUP, np.uint8)
        group[[MESH_GROUP, LIDAR_GROUP]] = 1
        top = support.size[2] + 0.1
        heights = []
        for x, y in footprint_samples(prop):
            geom = np.array([-1], np.int32)
            dist = mujoco.mj_ray(
                model, data, np.array([x, y, top]), np.array([0.0, 0.0, -1.0]), group, 1, -1, geom
            )
            if dist >= 0:
                heights.append(top - dist)
        if not heights:
            raise ValueError(f"{prop.body} is not over {support.body}")
        prop.z_base = max(heights) + 0.001


# --- occupancy map --------------------------------------------------------------------------


def occupancy(model, data, plan):
    """What the 2D scan can see: the footprint of every group 2 box reaching into the scan band.

    Walls, the counter and the prop proxies are all boxes, so this is exact; anything else in
    the LiDAR's group is refused rather than silently left off the map.
    """
    (x0, y0), (x1, y1) = plan["outline"]
    origin = (x0 - MAP_MARGIN_M, y0 - MAP_MARGIN_M)
    width = int(round((x1 - x0 + 2 * MAP_MARGIN_M) / MAP_RESOLUTION_M))
    height = int(round((y1 - y0 + 2 * MAP_MARGIN_M) / MAP_RESOLUTION_M))
    centres_x = origin[0] + (np.arange(width) + 0.5) * MAP_RESOLUTION_M
    centres_y = origin[1] + (np.arange(height) + 0.5) * MAP_RESOLUTION_M
    grid = np.stack(np.meshgrid(centres_x, centres_y), -1)  # row 0 is the lowest y
    occupied = np.zeros((height, width), bool)
    lo, hi = SCAN_BAND_M
    for g in range(model.ngeom):
        if model.geom_group[g] != LIDAR_GROUP or model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX:
            raise ValueError(f"geom {g} in the LiDAR group is not a box; rasterise it too")
        pos, mat, half = data.geom_xpos[g], data.geom_xmat[g].reshape(3, 3), model.geom_size[g]
        reach = np.abs(mat[2]) @ half  # vertical half-extent, rotated
        if pos[2] + reach < lo or pos[2] - reach > hi:
            continue
        local = (grid - pos[:2]) @ mat[:2, :2]
        occupied |= (np.abs(local[..., 0]) <= half[0]) & (np.abs(local[..., 1]) <= half[1])

    # Free is whatever the spawn point connects to; anything walled off stays unknown, as it
    # would in a scanned map.
    labels, _ = ndimage.label(~occupied)
    free = labels == labels[spawn_cell(origin)]
    return occupied, free, origin, (centres_x, centres_y)


def spawn_cell(origin):
    return (int(-origin[1] / MAP_RESOLUTION_M), int(-origin[0] / MAP_RESOLUTION_M))


def write_map(occupied, free, origin, name):
    image = np.full(occupied.shape, UNKNOWN, np.uint8)
    image[free] = FREE
    image[occupied] = OCCUPIED
    image = image[::-1]  # PGM rows run from the top, i.e. the highest y
    with open(MAPS / f"{name}.pgm", "wb") as out:
        out.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode())
        out.write(image.tobytes())
    (MAPS / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nmode: trinary\nresolution: {MAP_RESOLUTION_M}\n"
        f"origin: [{fmt(origin[0])}, {fmt(origin[1])}, 0]\nnegate: 0\n"
        # 205 is occupancy 0.196; a free threshold above that would serve unknown as free.
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )


# --- ground truth ---------------------------------------------------------------------------


def observable(props, occupied, free, origin, centres):
    """Whether the standing robot's camera can see any part of each prop from somewhere it can
    stand. The camera looks 47.6 deg down with a 58 deg vertical field, so its view's upper edge
    is 18.6 deg below the horizon: a point at height z shows within (1.21 - z) / tan(18.6 deg)."""
    clearance = ndimage.distance_transform_edt(~occupied) * MAP_RESOLUTION_M
    standable, _ = ndimage.label(free & (clearance >= ROBOT_RADIUS_M))
    cx, cy = centres
    spawn = standable[spawn_cell(origin)]
    if spawn == 0:
        raise ValueError("the robot cannot stand at the spawn point")
    rows, cols = np.nonzero(standable == spawn)
    stand = np.stack([cx[cols], cy[rows]], 1)
    upper_edge = math.tan(math.radians(CAMERA_PITCH_DEG - CAMERA_FOVY_DEG / 2.0))
    result = {}
    for prop in props:
        c, s = math.cos(math.radians(prop.yaw)), math.sin(math.radians(prop.yaw))
        rel = stand - (prop.x, prop.y)
        local = np.stack([c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]], 1)
        gap = np.maximum(np.abs(local) - np.array(prop.size[:2]) / 2.0, 0.0)
        nearest = float(np.hypot(gap[:, 0], gap[:, 1]).min()) - CAMERA_FORWARD_M
        reach = (CAMERA_HEIGHT_M - prop.z_base) / upper_edge
        result[prop.body] = prop.z_base < CAMERA_HEIGHT_M and nearest <= reach
    return result


def truth_yaml(plan, props, seen):
    rooms = []
    for room in plan["rooms"]:
        (x0, y0), (x1, y1) = room["rect"]
        rooms.append(
            {
                "id": room["id"],
                "name": room["name"],
                "type": room["type"],
                "polygon": [[x0, y0], [x1, y0], [x1, y1], [x0, y1]],
            }
        )
    objects = []
    for prop in props:
        objects.append(
            {
                "body": prop.body,
                "label": prop.cls.replace("_", " "),
                "synonyms": list(plan["synonyms"].get(prop.cls, [])),
                "room": prop.room,
                "centre": [round(v, 4) for v in prop.centre],
                "size": [round(v, 4) for v in prop.size],
                "yaw": prop.yaw,
                "on": prop.on,
                "observable_from_standing": bool(seen[prop.body]),
                "asset": prop.asset or "+".join(sorted({a for a, _, _ in prop.meshes} | {"boxes"})),
            }
        )
    header = (
        f"# Ground truth for the {plan['name']} world, for scoring the semantic map and\n"
        f"# exploration. Generated by tools/build_world.py from {plan['name']}.yaml; do not edit.\n"
        "# World frame, which the committed map shares: the robot spawns at the origin. centre\n"
        "# and size are the prop's bounding box, the same one the simulator publishes; yaw is in\n"
        "# degrees. observable_from_standing: some part of it enters the standing robot's camera\n"
        "# view from a pose it can reach; nothing above the camera ever does.\n"
    )
    body = {
        "world": plan["name"],
        "scene": f"g1_{plan['name']}_scene.xml",
        "camera": {
            "height_m": CAMERA_HEIGHT_M,
            "pitch_deg": CAMERA_PITCH_DEG,
            "fovy_deg": CAMERA_FOVY_DEG,
        },
        "rooms": rooms,
        "doorways": plan["doorways"],
        "objects": objects,
    }
    return header + yaml.safe_dump(body, sort_keys=False, default_flow_style=None, width=100)


def sensor_yaml(props, name):
    """sim_sensors.yaml with object_bodies swapped for this world's props; every other line kept."""
    base = (BRINGUP / "config" / "sim_sensors.yaml").read_text().splitlines()
    start = base.index("object_bodies:")
    end = start + 1
    while end < len(base) and base[end].startswith("  "):
        end += 1
    header = [
        f"# world:={name}: sim_sensors.yaml with every prop tracked. Generated by",
        "# tools/build_world.py; edit sim_sensors.yaml and regenerate.",
    ]
    listing = ["object_bodies:"] + [f"  - {p.body}" for p in props]
    return "\n".join(header + base[:start] + listing + base[end:]) + "\n"


# --- main -----------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--plan", type=pathlib.Path, default=BRINGUP / "worlds" / "apartment.yaml")
    args = parser.parse_args()

    plan = yaml.safe_load(args.plan.read_text())
    manifest = yaml.safe_load((ASSETS / "manifest.yaml").read_text())["assets"]
    name = plan["name"]
    props = [
        Prop(room, entry, manifest) for room, entries in plan["props"].items() for entry in entries
    ]
    bodies = [p.body for p in props]
    if len(set(bodies)) != len(bodies):
        raise ValueError("duplicate body names")
    rest_on_supports(props, plan, manifest)

    xml = scene_xml(plan, props, manifest, PLACEHOLDER)
    (BRINGUP / "mjcf" / f"g1_{name}_scene.xml").write_text(xml)

    model = mujoco.MjModel.from_xml_string(standalone(xml))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    occupied, free, origin, centres = occupancy(model, data, plan)
    write_map(occupied, free, origin, name)

    seen = observable(props, occupied, free, origin, centres)
    (BRINGUP / "worlds" / f"{name}.truth.yaml").write_text(truth_yaml(plan, props, seen))
    (BRINGUP / "config" / f"sim_sensors_{name}.yaml").write_text(sensor_yaml(props, name))

    print(
        f"{len(props)} props, {model.ngeom} geoms, {model.nmesh} meshes; map "
        f"{occupied.shape[1]}x{occupied.shape[0]}; unobservable: "
        f"{[b for b, s in seen.items() if not s]}"
    )


if __name__ == "__main__":
    main()
