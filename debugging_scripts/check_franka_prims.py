# check_franka_prims.py
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Usd

assets_root = get_assets_root_path()
franka_usd = assets_root + "/Isaac/Robots/Franka/franka.usd"
add_reference_to_stage(usd_path=franka_usd, prim_path="/World/Franka")

stage = omni.usd.get_context().get_stage()

# Force load all payloads on the stage
print("Loading all payloads...")
stage.Load()

print("Waiting for Franka asset to stream...")
for i in range(300):
    simulation_app.update()
    if i % 20 == 0:
        franka_prim = stage.GetPrimAtPath("/World/Franka")
        children = list(franka_prim.GetChildren())
        print(f"  tick {i}: {len(children)} children found")
        if len(children) > 0:
            print("  Asset populated! Stopping early.")
            break

franka_prim = stage.GetPrimAtPath("/World/Franka")
print(f"\nFranka prim valid: {franka_prim.IsValid()}")
print("Children of /World/Franka:")
for child in franka_prim.GetChildren():
    print(f"  {child.GetPath()}")
    for grandchild in child.GetChildren():
        print(f"    {grandchild.GetPath()}")

simulation_app.close()