import omni.usd
import omni.kit.app

from pxr import UsdGeom, Usd

import json
import os

OUTPUT_FILE = os.path.expanduser("~/isaac_scene.json")


OBJECTS = {
    "case": "/HDD/case",
    "lid": "/HDD/lid_1",

    "lid_screw_01": "/HDD/lid_screw_01",
    "lid_screw_02": "/HDD/lid_screw_02",
    "lid_screw_03": "/HDD/lid_screw_03",
    "lid_screw_04": "/HDD/lid_screw_04",
    "lid_screw_05": "/HDD/lid_screw_05",
    "lid_screw_06": "/HDD/lid_screw_06",
    "lid_screw_07": "/HDD/lid_screw_07",
    "lid_screw_08": "/HDD/lid_screw_08",
}

stage = omni.usd.get_context().get_stage()

if stage is None:
    print("No stage loaded.")
    raise RuntimeError("No USD stage loaded.")

object_cache = {}

for name, path in OBJECTS.items():

    prim = stage.GetPrimAtPath(path)

    if prim.IsValid():
        object_cache[name] = UsdGeom.Xformable(prim)
        print(f"Found {name}")
    else:
        print(f"Missing {path}")

print(f"\nWriting JSON to: {OUTPUT_FILE}")


xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())


frame_counter = 0

def write_json(event):

    global frame_counter

    frame_counter += 1
    if frame_counter < 10:
        return
    frame_counter = 0

    data = {}

    # Refresh cache each update
    xform_cache.Clear()

    for name, xformable in object_cache.items():

        try:
            matrix = xform_cache.GetLocalToWorldTransform(
                xformable.GetPrim()
            )

            t = matrix.ExtractTranslation()

            data[name] = {
                "xyz": [
                    float(t[0]),
                    float(t[1]),
                    float(t[2])
                ]
            }

        except Exception as e:
            print(f"Failed reading {name}: {e}")

    try:
        with open(OUTPUT_FILE, "w") as f:
            json.dump(data, f, indent=4)
    except Exception as e:
        print(f"Failed writing JSON: {e}")


stream = omni.kit.app.get_app().get_update_event_stream()

subscription = stream.create_subscription_to_pop(
    write_json,
    name="WriteSceneJSON"
)

print("JSON writer running.")
print("Output:", OUTPUT_FILE)
