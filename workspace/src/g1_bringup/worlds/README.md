# worlds

Floor plans that `tools/build_world.py` turns into simulator worlds. One so far, `apartment`: five
furnished rooms and a hallway, about 20 by 12 m, for the semantic map and camera-coverage
exploration.

```bash
./scripts/setup-world-assets.sh    # on the host, once
ros2 launch g1_bringup bringup.launch.py world:=apartment mode:=localization nav:=true
```

## Files

| File | |
|---|---|
| `apartment.yaml` | The floor plan: rooms, walls, doorways, lights, and each prop's asset and pose. The one file to edit. |
| `apartment.truth.yaml` | Ground truth for scoring: room polygons and types, doorways, and for each object its label, synonyms, room, bounding box, support, and whether the standing robot's camera can see it from anywhere it can reach. |
| `../mjcf/g1_apartment_scene.xml` | The scene. Asset paths start with `@GROVE_ASSETS@/`, which `sim.launch.py` replaces with `$GROVE_ASSETS_DIR` (default `/root/workspace/assets`) when it stages the scene. |
| `../config/sim_sensors_apartment.yaml` | `sim_sensors.yaml` with every prop in `object_bodies`. `sim.launch.py` takes `sim_sensors_<world>.yaml` when one exists. |
| `g1_navigation`: `maps/apartment.{pgm,yaml}` | The occupancy grid, which localization picks for `world:=apartment`. |

All but the plan are generated. The meshes and textures are not committed: `setup-world-assets.sh`
fetches them into `workspace/assets/` (63 MB, plus a 265 MB download cache that can be deleted)
through `tools/world_assets.py`, in a host venv at `~/ref/grove-worldgen`, which it creates if
missing. Every model becomes one OBJ per material and 1K diffuse PNGs, in metres, Z up, origin at
the centre of its footprint on the floor, front facing -y, and at most 20k triangles per mesh.
`workspace/assets/manifest.yaml` records each one's source, licence and bounding box.

## Regenerating

After editing the plan or an asset:

```bash
~/ref/grove-worldgen/.venv/bin/python workspace/src/g1_bringup/tools/build_world.py
```

The output is deterministic, so an unchanged plan gives byte-identical files. Commit all five
together; the map is rasterised from the scene, so a stale map would be a map of another world.

## How the scene is built

- The LiDAR sweep sees geom group 2 only; the camera draws groups 0 to 2. Each prop is a static
  body with its textured meshes in group 1 and one bounding box in group 2. The box is what the
  LiDAR hits, the robot collides with and the simulator sizes ground truth from, and it is never
  drawn: `mj_ray` checks its material's alpha, which is 1, while the renderer skips a geom whose
  own rgba alpha is 0. A 360x32 sweep, timed on the host with a stand-in for the robot's geoms,
  takes 11 to 12 ms this way, 22 to 32 ms with the meshes in group 2, and 9 to 11 ms in the
  facility.
- MuJoCo 3.3.6 smears a 2D texture across the vertical faces of a box, so walls are plain group 2
  boxes dressed with textured group 1 planes, and the counter's carcass is untextured.
- Poses are quaternions: `g1_29dof.xml` sets `angle="radian"` for the whole model, so an `euler`
  in degrees would be read as radians.
- The renderer honours eight lights and the headlight takes one, so the plan has seven.
- `book_2` sits on a tier at 1.66 m, above the head camera, as a target that must be reported
  unobservable. `crate_2`, 0.25 m tall, is below the 2D scan band, so it is missing from the map.
- Doors are at least 1.2 m. The storage door was 1.0 m as a stress case, and Nav2 would not plan
  through it: the robot wedged in the frame and every later goal failed.

## Credits

Poly Haven assets are CC0; NVIDIA's kitchen set and Google Scanned Objects are CC BY 4.0.

| Asset | Author | Source | Licence |
|---|---|---|---|
| `ph_armchair_01` Arm Chair 01 | Kirill Sannikov | [Poly Haven](https://polyhaven.com/a/ArmChair_01) | CC0 1.0 |
| `ph_cardboard_box_01` Cardboard Box 01 | Rahul Chaudhary | [Poly Haven](https://polyhaven.com/a/cardboard_box_01) | CC0 1.0 |
| `ph_classic_nightstand_01` Classic Nightstand 01 | Kirill Sannikov | [Poly Haven](https://polyhaven.com/a/ClassicNightstand_01) | CC0 1.0 |
| `ph_dining_chair_02` Dining Chair 02 | James Ray Cock | [Poly Haven](https://polyhaven.com/a/dining_chair_02) | CC0 1.0 |
| `ph_dining_table` Dining Table | Aron Łyczek | [Poly Haven](https://polyhaven.com/a/dining_table) | CC0 1.0 |
| `ph_gothic_bed_01` Gothic Bed 01 | Kirill Sannikov | [Poly Haven](https://polyhaven.com/a/GothicBed_01) | CC0 1.0 |
| `ph_gothic_cabinet_01` Gothic Cabinet 01 | Kirill Sannikov | [Poly Haven](https://polyhaven.com/a/GothicCabinet_01) | CC0 1.0 |
| `ph_metal_office_desk` Metal Office Desk | Ulan Cabanilla | [Poly Haven](https://polyhaven.com/a/metal_office_desk) | CC0 1.0 |
| `ph_modern_arm_chair_01` Modern Arm Chair 01 | Vibrant Nordic | [Poly Haven](https://polyhaven.com/a/modern_arm_chair_01) | CC0 1.0 |
| `ph_modern_coffee_table_01` Modern Coffee Table 01 | Amin | [Poly Haven](https://polyhaven.com/a/modern_coffee_table_01) | CC0 1.0 |
| `ph_modern_wooden_cabinet` Modern Wooden Cabinet | Patrik Pangerl | [Poly Haven](https://polyhaven.com/a/modern_wooden_cabinet) | CC0 1.0 |
| `ph_plastic_crate_02` Plastic Crate 02 | Fabi_G | [Poly Haven](https://polyhaven.com/a/plastic_crate_02) | CC0 1.0 |
| `ph_potted_plant_01` Potted Plant 01 | Rico Cilliers | [Poly Haven](https://polyhaven.com/a/potted_plant_01) | CC0 1.0 |
| `ph_potted_plant_02` Potted Plant 02 | Rico Cilliers | [Poly Haven](https://polyhaven.com/a/potted_plant_02) | CC0 1.0 |
| `ph_school_chair_01` School Chair 01 | Ethan Place | [Poly Haven](https://polyhaven.com/a/SchoolChair_01) | CC0 1.0 |
| `ph_sofa_02` Sofa 02 | Kirill Sannikov | [Poly Haven](https://polyhaven.com/a/sofa_02) | CC0 1.0 |
| `ph_steel_frame_shelves_01` Steel Frame Shelves 01 | James Ray Cock | [Poly Haven](https://polyhaven.com/a/steel_frame_shelves_01) | CC0 1.0 |
| `ph_steel_frame_shelves_03` Steel Frame Shelves 03 | Ulan Cabanilla | [Poly Haven](https://polyhaven.com/a/steel_frame_shelves_03) | CC0 1.0 |
| `ph_television_01` Television 01 | Gabriel Radić | [Poly Haven](https://polyhaven.com/a/Television_01) | CC0 1.0 |
| `ph_vintage_day_bed` Vintage Day Bed | Aron Łyczek | [Poly Haven](https://polyhaven.com/a/vintage_day_bed) | CC0 1.0 |
| `ph_wooden_bookshelf_worn` Wooden Bookshelf Worn | Ulan Cabanilla | [Poly Haven](https://polyhaven.com/a/wooden_bookshelf_worn) | CC0 1.0 |
| `ph_wooden_crate_02` Wooden Crate 02 | James Ray Cock, Jurita Burger | [Poly Haven](https://polyhaven.com/a/wooden_crate_02) | CC0 1.0 |
| `ph_wooden_display_shelves_01` Wooden Display Shelves 01 | James Ray Cock | [Poly Haven](https://polyhaven.com/a/wooden_display_shelves_01) | CC0 1.0 |
| `tex_concrete_floor` Concrete Floor Worn 001 | Dimitrios Savva, Rico Cilliers | [Poly Haven](https://polyhaven.com/a/concrete_floor_worn_001) | CC0 1.0 |
| `tex_floor_tiles` Floor Tiles 06 | Rob Tuytel | [Poly Haven](https://polyhaven.com/a/floor_tiles_06) | CC0 1.0 |
| `tex_laminate_floor` Laminate Floor 02 | Charlotte Baglioni, Dario Barresi | [Poly Haven](https://polyhaven.com/a/laminate_floor_02) | CC0 1.0 |
| `tex_marble` Marble 01 | Rob Tuytel | [Poly Haven](https://polyhaven.com/a/marble_01) | CC0 1.0 |
| `tex_plaster_wall` Plastered Wall | Amal Kumar | [Poly Haven](https://polyhaven.com/a/plastered_wall) | CC0 1.0 |
| `nv_cabinet_door_002` CabinetDoorPanel002 | NVIDIA | [NVIDIA Kitchen MJCF](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF/blob/main/fixtures_lightwheel/cabinets.zip) | CC BY 4.0 |
| `nv_kettle_005` ElectricKettle005 | NVIDIA | [NVIDIA Kitchen MJCF](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF/blob/main/fixtures_lightwheel/electric_kettles.zip) | CC BY 4.0 |
| `nv_microwave_043` Microwave043 | NVIDIA | [NVIDIA Kitchen MJCF](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF/blob/main/fixtures_lightwheel/microwaves.zip) | CC BY 4.0 |
| `nv_refrigerator_040` Refrigerator040 | NVIDIA | [NVIDIA Kitchen MJCF](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF/blob/main/fixtures_lightwheel/fridges.zip) | CC BY 4.0 |
| `nv_stove_066` Stove066 | NVIDIA | [NVIDIA Kitchen MJCF](https://huggingface.co/datasets/nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF/blob/main/fixtures_lightwheel/stoves.zip) | CC BY 4.0 |
| `gso_alarm_clock` Crosley Alarm Clock Vintage Metal | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Crosley_Alarm_Clock_Vintage_Metal) | CC BY 4.0 |
| `gso_book` Eat to Live The Amazing NutrientRich... | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Eat_to_Live_The_Amazing_NutrientRich_Program_for_Fast_and_Sustained_Weight_Loss_Revised_Edition_Book) | CC BY 4.0 |
| `gso_bowl_room_essentials` Room Essentials Bowl Turquiose | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Room_Essentials_Bowl_Turquiose) | CC BY 4.0 |
| `gso_bowl_threshold` Threshold Bead Cereal Bowl White | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Threshold_Bead_Cereal_Bowl_White) | CC BY 4.0 |
| `gso_laptop` Travel Mate P series Notebook | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Travel_Mate_P_series_Notebook) | CC BY 4.0 |
| `gso_mug_ace` ACE Coffee Mug Kristen 16 oz cup | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/ACE_Coffee_Mug_Kristen_16_oz_cup) | CC BY 4.0 |
| `gso_mug_cole` Cole Hardware Mug Classic Blue | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Cole_Hardware_Mug_Classic_Blue) | CC BY 4.0 |
| `gso_mug_room_essentials` Room Essentials Mug White Yellow | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Room_Essentials_Mug_White_Yellow) | CC BY 4.0 |
| `gso_wastebasket` Hefty Waste Basket Decorative Bronze... | Google Research | [GSO, MJCF by Kevin Zakka](https://github.com/kevinzakka/mujoco_scanned_objects/tree/main/models/Hefty_Waste_Basket_Decorative_Bronze_85_liter) | CC BY 4.0 |
