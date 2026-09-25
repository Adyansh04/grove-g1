#!/usr/bin/env python3
"""Fetches the apartment world's third-party assets and converts them for MuJoCo 3.3.6.

Every model becomes one OBJ per material plus 1K PNG diffuse textures, in metres, Z up, with the
origin at the centre of its footprint on the floor and its front facing -y. Building textures
become a single diffuse PNG. manifest.yaml records each asset's source, author, licence and
bounding box; build_world.py reads it. Idempotent: a converted asset is skipped, and downloads
are cached under <out>/.downloads.

Host-side tooling, not a ROS node: it needs mujoco, trimesh and pillow, which the container lacks.

    ./scripts/setup-world-assets.sh
    python world_assets.py --only ph_sofa_02 --force
"""

import argparse
import json
import pathlib
import shutil
import sys
import time
import urllib.request
import zipfile

import mujoco
import numpy as np
import trimesh
import yaml
from PIL import Image
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

REPO = pathlib.Path(__file__).resolve().parents[4]
DEFAULT_OUT = REPO / "workspace" / "assets"

MAX_TRIANGLES = 20000
MAX_TEXTURE_PX = 1024

POLYHAVEN_API = "https://api.polyhaven.com"
GSO_RAW = "https://raw.githubusercontent.com/kevinzakka/mujoco_scanned_objects/main/models"
NVIDIA_REPO = "nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF"
NVIDIA_RAW = f"https://huggingface.co/datasets/{NVIDIA_REPO}/resolve/main/fixtures_lightwheel"

# Poly Haven asks API clients to identify themselves.
USER_AGENT = "grove-g1-world-assets/1.0"

SOURCES = {
    "polyhaven": {
        "source": "Poly Haven",
        "license": "CC0 1.0",
    },
    "gso": {
        "source": "Google Scanned Objects, MJCF conversion by Kevin Zakka",
        "author": "Google Research",
        "license": "CC BY 4.0",
    },
    "nvidia": {
        "source": "NVIDIA PhysicalAI Kitchen MJCF (Lightwheel fixtures)",
        "author": "NVIDIA",
        "license": "CC BY 4.0",
    },
}

# glTF is Y up; the world is Z up.
Y_UP_TO_Z_UP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])

# id -> source and conversion options. `height` rescales uniformly to that height in metres,
# `yaw` (degrees) turns a model whose front does not face -y.
MODELS = {
    "ph_sofa_02": {"class": "sofa", "polyhaven": "sofa_02"},
    "ph_modern_arm_chair_01": {"class": "armchair", "polyhaven": "modern_arm_chair_01"},
    "ph_armchair_01": {"class": "armchair", "polyhaven": "ArmChair_01"},
    "ph_modern_coffee_table_01": {
        "class": "coffee_table",
        "polyhaven": "modern_coffee_table_01",
        "yaw": 90,
    },
    "ph_modern_wooden_cabinet": {"class": "cabinet", "polyhaven": "modern_wooden_cabinet"},
    "ph_television_01": {"class": "television", "polyhaven": "Television_01"},
    "ph_wooden_bookshelf_worn": {"class": "bookshelf", "polyhaven": "wooden_bookshelf_worn"},
    "ph_wooden_display_shelves_01": {
        "class": "bookshelf",
        "polyhaven": "wooden_display_shelves_01",
        "yaw": 90,
    },
    "ph_potted_plant_01": {"class": "potted_plant", "polyhaven": "potted_plant_01"},
    "ph_potted_plant_02": {"class": "potted_plant", "polyhaven": "potted_plant_02"},
    "ph_dining_table": {"class": "dining_table", "polyhaven": "dining_table", "height": 0.76},
    "ph_dining_chair_02": {"class": "chair", "polyhaven": "dining_chair_02"},
    "ph_school_chair_01": {"class": "chair", "polyhaven": "SchoolChair_01"},
    "ph_gothic_bed_01": {"class": "bed", "polyhaven": "GothicBed_01"},
    "ph_vintage_day_bed": {"class": "bed", "polyhaven": "vintage_day_bed"},
    "ph_classic_nightstand_01": {"class": "nightstand", "polyhaven": "ClassicNightstand_01"},
    "ph_gothic_cabinet_01": {"class": "wardrobe", "polyhaven": "GothicCabinet_01"},
    "ph_metal_office_desk": {"class": "desk", "polyhaven": "metal_office_desk"},
    # Authored at ten times scale.
    "ph_steel_frame_shelves_01": {
        "class": "shelf",
        "polyhaven": "steel_frame_shelves_01",
        "height": 2.14,
    },
    "ph_steel_frame_shelves_03": {"class": "shelf", "polyhaven": "steel_frame_shelves_03"},
    "ph_cardboard_box_01": {"class": "cardboard_box", "polyhaven": "cardboard_box_01"},
    "ph_wooden_crate_02": {"class": "crate", "polyhaven": "wooden_crate_02", "yaw": 90},
    "ph_plastic_crate_02": {"class": "crate", "polyhaven": "plastic_crate_02"},
    "nv_refrigerator_040": {"class": "fridge", "nvidia": "fridges/Refrigerator040"},
    "nv_stove_066": {"class": "stove", "nvidia": "stoves/Stove066"},
    "nv_microwave_043": {"class": "microwave", "nvidia": "microwaves/Microwave043"},
    "nv_kettle_005": {"class": "kettle", "nvidia": "electric_kettles/ElectricKettle005"},
    # A door front for the procedural kitchen counter, which the dataset leaves to its generator.
    "nv_cabinet_door_002": {
        "class": "cabinet_door",
        "nvidia": "cabinets/cabinet_panels/CabinetDoorPanel002",
    },
    "gso_wastebasket": {
        "class": "dustbin",
        "gso": "Hefty_Waste_Basket_Decorative_Bronze_85_liter",
    },
    "gso_mug_ace": {"class": "mug", "gso": "ACE_Coffee_Mug_Kristen_16_oz_cup"},
    "gso_mug_room_essentials": {"class": "mug", "gso": "Room_Essentials_Mug_White_Yellow"},
    "gso_mug_cole": {"class": "mug", "gso": "Cole_Hardware_Mug_Classic_Blue"},
    "gso_bowl_threshold": {"class": "bowl", "gso": "Threshold_Bead_Cereal_Bowl_White"},
    "gso_bowl_room_essentials": {"class": "bowl", "gso": "Room_Essentials_Bowl_Turquiose"},
    "gso_book": {
        "class": "book",
        "gso": "Eat_to_Live_The_Amazing_NutrientRich_Program_for_Fast_and_Sustained_"
        "Weight_Loss_Revised_Edition_Book",
    },
    "gso_laptop": {"class": "laptop", "gso": "Travel_Mate_P_series_Notebook"},
    "gso_alarm_clock": {"class": "clock", "gso": "Crosley_Alarm_Clock_Vintage_Metal"},
}

# Tiling diffuse maps for floors, walls and the countertop, by Poly Haven id.
TEXTURES = {
    "tex_laminate_floor": "laminate_floor_02",
    "tex_floor_tiles": "floor_tiles_06",
    "tex_concrete_floor": "concrete_floor_worn_001",
    "tex_plaster_wall": "plastered_wall",
    "tex_marble": "marble_01",
}


# --- downloads ------------------------------------------------------------------------------


def fetch(url, dest):
    """Downloads url to dest once; a finished file is never fetched again."""
    if dest.exists():
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    partial = dest.with_name(dest.name + ".part")
    for attempt in range(4):
        try:
            request = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with (
                urllib.request.urlopen(request, timeout=120) as response,
                open(partial, "wb") as out,
            ):
                shutil.copyfileobj(response, out)
            partial.replace(dest)
            return dest
        except OSError as error:
            if attempt == 3:
                raise RuntimeError(f"download failed: {url}: {error}") from error
            time.sleep(2**attempt)


def fetch_json(url, dest):
    return json.loads(fetch(url, dest).read_text())


# --- mesh parts -----------------------------------------------------------------------------


class Part:
    """One material's worth of triangles: positions, faces, optional UVs and diffuse image."""

    def __init__(self, key, vertices, faces, uv, image, rgba, specular=0.2, shininess=0.2):
        self.key = key
        self.vertices = np.asarray(vertices, dtype=np.float64)
        self.faces = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
        self.uv = None if uv is None else np.asarray(uv, dtype=np.float64)
        self.image = image
        self.rgba = [round(float(c), 4) for c in rgba]
        self.specular = specular
        self.shininess = shininess


def merge_by_key(parts):
    """One part per material: MuJoCo allows one material per mesh, and fewer geoms cost the
    LiDAR sweep less."""
    merged = {}
    for part in parts:
        if part.key not in merged:
            merged[part.key] = part
            continue
        base = merged[part.key]
        if (base.uv is None) != (part.uv is None):
            raise ValueError(f"material {part.key} mixes textured and untextured primitives")
        offset = len(base.vertices)
        base.vertices = np.vstack([base.vertices, part.vertices])
        base.faces = np.vstack([base.faces, part.faces + offset])
        if base.uv is not None:
            base.uv = np.vstack([base.uv, part.uv])
    return [merged[key] for key in sorted(merged)]


def decimate(vertices, faces, uv, target):
    """Vertex clustering on a grid grown until at most `target` faces survive.

    Positions cluster by cell alone, so seams cannot crack open; UVs average only within one UV
    island (a component of the unwelded mesh), so no texture is smeared across a seam.
    """
    if len(faces) <= target:
        return vertices, faces, uv
    n = len(vertices)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])), shape=(n, n))
    _, island = connected_components(graph, directed=False)

    def ids(rows):
        return np.unique(rows, axis=0, return_inverse=True)[1].ravel()

    def cluster(cell):
        position_id = ids(np.floor(vertices / cell).astype(np.int64))
        vertex_id = ids(np.column_stack([island, position_id]))
        corner_pos = position_id[faces]
        keep = (
            (corner_pos[:, 0] != corner_pos[:, 1])
            & (corner_pos[:, 1] != corner_pos[:, 2])
            & (corner_pos[:, 0] != corner_pos[:, 2])
        )
        new_faces = np.unique(vertex_id[faces][keep], axis=0)
        return position_id, vertex_id, new_faces

    lo, hi = 0.0, float(np.ptp(vertices, axis=0).max())
    best = None
    for _ in range(30):
        cell = 0.5 * (lo + hi)
        result = cluster(cell)
        if len(result[2]) <= target:
            best, hi = result, cell
        else:
            lo = cell
    position_id, vertex_id, new_faces = best

    def mean_by(group, values):
        counts = np.bincount(group)[:, None]
        columns = [np.bincount(group, weights=values[:, k]) for k in range(values.shape[1])]
        return np.stack(columns, 1) / counts

    positions = mean_by(position_id, vertices)
    first = np.zeros(vertex_id.max() + 1, dtype=np.int64)
    first[vertex_id[::-1]] = np.arange(n)[::-1]
    new_vertices = positions[position_id[first]]
    new_uv = None if uv is None else mean_by(vertex_id, uv)

    used = np.unique(new_faces)
    remap = np.full(len(new_vertices), -1, dtype=np.int64)
    remap[used] = np.arange(len(used))
    return (
        new_vertices[used],
        remap[new_faces],
        None if new_uv is None else new_uv[used],
    )


# --- sources --------------------------------------------------------------------------------


def polyhaven_parts(ph_id, cache):
    files = fetch_json(
        f"{POLYHAVEN_API}/files/{ph_id}", cache / "polyhaven" / f"{ph_id}.files.json"
    )
    gltf = files["gltf"]["1k"]["gltf"]
    root = cache / "polyhaven" / ph_id
    path = fetch(gltf["url"], root / pathlib.Path(gltf["url"]).name)
    for relative, include in sorted(gltf["include"].items()):
        fetch(include["url"], root / relative)

    scene = trimesh.load_scene(path, process=False)
    parts = []
    for node in sorted(scene.graph.nodes_geometry):
        transform, name = scene.graph[node]
        mesh = scene.geometry[name]
        vertices = trimesh.transform_points(mesh.vertices, transform) @ Y_UP_TO_Z_UP.T
        visual = mesh.visual
        material = getattr(visual, "material", None)
        uv = getattr(visual, "uv", None)
        image = getattr(material, "baseColorTexture", None)
        factor = getattr(material, "baseColorFactor", None)
        rgba = [1.0, 1.0, 1.0, 1.0] if factor is None else np.asarray(factor) / 255.0
        if image is None or uv is None:
            image, uv = None, None
        key = getattr(material, "name", None) or name
        parts.append(Part(key, vertices, mesh.faces, uv, image, rgba))
    return parts


def polyhaven_credit(ph_id, cache):
    info = fetch_json(f"{POLYHAVEN_API}/info/{ph_id}", cache / "polyhaven" / f"{ph_id}.info.json")
    return {
        **SOURCES["polyhaven"],
        "name": info["name"],
        "author": ", ".join(sorted(info["authors"])),
        "url": f"https://polyhaven.com/a/{ph_id}",
    }, info


def gso_parts(name, cache):
    root = cache / "gso" / name
    obj = fetch(f"{GSO_RAW}/{name}/model.obj", root / "model.obj")
    png = fetch(f"{GSO_RAW}/{name}/texture.png", root / "texture.png")
    # The OBJ names a .mtl the repository does not ship; the texture is attached here instead.
    mesh = trimesh.load_mesh(obj, process=False, skip_materials=True)
    uv = getattr(mesh.visual, "uv", None)
    return [Part("texture", mesh.vertices, mesh.faces, uv, Image.open(png), [1, 1, 1, 1], 0.3, 0.3)]


def gso_credit(name):
    return {
        **SOURCES["gso"],
        "name": name.replace("_", " "),
        "url": f"https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/{name}",
    }


def nvidia_parts(ref, cache):
    category = ref.split("/")[0]
    archive = fetch(f"{NVIDIA_RAW}/{category}.zip", cache / "nvidia" / f"{category}.zip")
    xml = cache / "nvidia" / ref / "model.xml"
    if not xml.exists():
        with zipfile.ZipFile(archive) as zf:
            members = [n for n in zf.namelist() if n.startswith(f"{ref}/")]
            zf.extractall(cache / "nvidia", members)

    spec = mujoco.MjSpec.from_file(str(xml))
    m = spec.compile()
    d = mujoco.MjData(m)
    # Doors and drawers at qpos0, i.e. closed.
    mujoco.mj_forward(m, d)
    texture_files = [t.file for t in spec.textures]
    images = {}
    parts = []
    for g in range(m.ngeom):
        if (
            m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH
            or m.geom_contype[g]
            or m.geom_conaffinity[g]
        ):
            continue
        mat = m.geom_matid[g]
        rgba = m.mat_rgba[mat] if mat >= 0 else m.geom_rgba[g]
        # Glass shelves and clear plastic sit behind closed doors, so they are dropped; tinted
        # oven and microwave windows are made opaque, since transparent geoms stay out of the
        # depth image.
        if rgba[3] < 0.5:
            continue
        rgba = np.append(rgba[:3], 1.0)
        mesh = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mesh], m.mesh_vertnum[mesh]
        fa, fn = m.mesh_faceadr[mesh], m.mesh_facenum[mesh]
        vertices = m.mesh_vert[va : va + vn] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        faces = m.mesh_face[fa : fa + fn]
        texid = m.mat_texid[mat, mujoco.mjtTextureRole.mjTEXROLE_RGB] if mat >= 0 else -1
        ta = m.mesh_texcoordadr[mesh]
        uv = image = None
        if texid >= 0 and ta >= 0:
            texcoord = m.mesh_texcoord[ta : ta + m.mesh_texcoordnum[mesh]].astype(np.float64)
            # MuJoCo flips v when it reads an OBJ; flip back so the exported OBJ reads the same.
            texcoord[:, 1] = 1.0 - texcoord[:, 1]
            corners = np.stack([faces.ravel(), m.mesh_facetexcoord[fa : fa + fn].ravel()], 1)
            unique, inverse = np.unique(corners, axis=0, return_inverse=True)
            vertices, uv, faces = vertices[unique[:, 0]], texcoord[unique[:, 1]], inverse.ravel()
            file = texture_files[texid]
            if file not in images:
                images[file] = Image.open(xml.parent / file)
            image = images[file]
        key = f"{texture_files[texid] if texid >= 0 else 'flat'}|{np.round(rgba, 3).tolist()}"
        specular = float(m.mat_specular[mat]) if mat >= 0 else 0.5
        shininess = float(m.mat_shininess[mat]) if mat >= 0 else 0.5
        parts.append(Part(key, vertices, faces, uv, image, rgba, specular, shininess))
    return parts


def nvidia_credit(ref):
    category = ref.split("/")[0]
    return {
        **SOURCES["nvidia"],
        "name": ref.split("/")[-1],
        "url": f"https://huggingface.co/datasets/{NVIDIA_REPO}/blob/main/fixtures_lightwheel/"
        f"{category}.zip",
    }


# --- output ---------------------------------------------------------------------------------


def write_obj(path, vertices, faces, uv):
    with open(path, "w") as out:
        np.savetxt(out, vertices, fmt="v %.5f %.5f %.5f")
        if uv is not None:
            np.savetxt(out, uv, fmt="vt %.5f %.5f")
            corners = np.repeat(faces + 1, 2, axis=1)
            np.savetxt(out, corners, fmt="f %d/%d %d/%d %d/%d")
        else:
            np.savetxt(out, faces + 1, fmt="f %d %d %d")


def save_png(image, path):
    image = image.convert("RGB")
    image.thumbnail((MAX_TEXTURE_PX, MAX_TEXTURE_PX), Image.Resampling.LANCZOS)
    image.save(path)


def convert_model(asset_id, spec, out, cache):
    if "polyhaven" in spec:
        parts = polyhaven_parts(spec["polyhaven"], cache)
        credit, _ = polyhaven_credit(spec["polyhaven"], cache)
    elif "gso" in spec:
        parts, credit = gso_parts(spec["gso"], cache), gso_credit(spec["gso"])
    else:
        parts, credit = nvidia_parts(spec["nvidia"], cache), nvidia_credit(spec["nvidia"])

    parts = merge_by_key(parts)
    rotation = np.eye(3)
    if spec.get("yaw"):
        a = np.radians(spec["yaw"])
        rotation = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    for part in parts:
        part.vertices = part.vertices @ rotation.T
    everything = np.vstack([p.vertices for p in parts])
    lo, hi = everything.min(0), everything.max(0)
    scale = spec["height"] / (hi[2] - lo[2]) if "height" in spec else 1.0
    origin = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])

    folder = out / asset_id
    if folder.exists():
        shutil.rmtree(folder)
    folder.mkdir(parents=True)
    written_images = {}
    records = []
    for k, part in enumerate(parts):
        vertices = (part.vertices - origin) * scale
        vertices, faces, uv = decimate(vertices, part.faces, part.uv, MAX_TRIANGLES)
        mesh_file = f"mesh_{k}.obj"
        write_obj(folder / mesh_file, vertices, faces, uv)
        record = {
            "mesh": mesh_file,
            "rgba": part.rgba,
            "specular": round(part.specular, 3),
            "shininess": round(part.shininess, 3),
            "triangles": len(faces),
        }
        if part.image is not None:
            if id(part.image) not in written_images:
                texture_file = f"tex_{len(written_images)}.png"
                save_png(part.image, folder / texture_file)
                written_images[id(part.image)] = texture_file
            record["texture"] = written_images[id(part.image)]
        records.append(record)

    size = (hi - lo) * scale
    entry = {
        "kind": "model",
        "class": spec["class"],
        "label": spec["class"].replace("_", " "),
        **credit,
        "files": sorted(p.name for p in folder.iterdir()),
        "parts": records,
        "triangles": sum(r["triangles"] for r in records),
        "bbox": [round(float(s), 4) for s in size],
    }
    return entry


def convert_texture(asset_id, ph_id, out, cache):
    files = fetch_json(
        f"{POLYHAVEN_API}/files/{ph_id}", cache / "polyhaven" / f"{ph_id}.files.json"
    )
    diffuse = files["Diffuse"]["1k"]["png"]["url"]
    source = fetch(diffuse, cache / "polyhaven" / ph_id / pathlib.Path(diffuse).name)
    credit, info = polyhaven_credit(ph_id, cache)
    folder = out / asset_id
    folder.mkdir(parents=True, exist_ok=True)
    save_png(Image.open(source), folder / "diffuse.png")
    # Poly Haven gives the physical size of the tile in millimetres.
    width_mm, height_mm = info.get("dimensions", [2000, 2000])[:2]
    return {
        "kind": "texture",
        **credit,
        "files": ["diffuse.png"],
        "tile_m": [round(width_mm / 1000.0, 3), round(height_mm / 1000.0, 3)],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    parser.add_argument("--only", nargs="*", help="asset ids to (re)build; default all")
    parser.add_argument("--force", action="store_true", help="rebuild even if converted")
    args = parser.parse_args()

    out = args.out.resolve()
    cache = out / ".downloads"
    wanted = set(args.only) if args.only else set(MODELS) | set(TEXTURES)
    unknown = wanted - set(MODELS) - set(TEXTURES)
    if unknown:
        sys.exit(f"unknown asset ids: {sorted(unknown)}")

    for asset_id in sorted(wanted):
        marker = out / asset_id / "asset.yaml"
        if marker.exists() and not args.force:
            continue
        print(f"==> {asset_id}", flush=True)
        if asset_id in MODELS:
            entry = convert_model(asset_id, MODELS[asset_id], out, cache)
        else:
            entry = convert_texture(asset_id, TEXTURES[asset_id], out, cache)
        # Written last: its presence is what marks the asset as done.
        marker.write_text(yaml.safe_dump(entry, sort_keys=False, default_flow_style=None))

    manifest = {}
    for asset_id in sorted(set(MODELS) | set(TEXTURES)):
        marker = out / asset_id / "asset.yaml"
        if marker.exists():
            manifest[asset_id] = yaml.safe_load(marker.read_text())
    header = "# Generated by workspace/src/g1_bringup/tools/world_assets.py. Do not edit.\n"
    (out / "manifest.yaml").write_text(
        header + yaml.safe_dump({"assets": manifest}, sort_keys=False, default_flow_style=None)
    )
    missing = sorted((set(MODELS) | set(TEXTURES)) - set(manifest))
    print(
        f"{len(manifest)} assets in {out / 'manifest.yaml'}"
        + (f"; missing {missing}" if missing else "")
    )


if __name__ == "__main__":
    main()
