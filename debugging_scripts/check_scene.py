# check_scene.py
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
import os
from pxr import Sdf, UsdLux
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import add_reference_to_stage

FRANKA_USD_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "assets", "franka.usd"
)

world = World()
stage = omni.usd.get_context().get_stage()

world.scene.add_default_ground_plane()
UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight")).CreateIntensityAttr(500)

pick_box = world.scene.add(DynamicCuboid(
    prim_path="/World/PickBox",
    name="pick_box",
    position=np.array([0.0, 0.0, 1.0]),  # high up so it's easy to see
    scale=np.array([0.2, 0.2, 0.2]),      # bigger so it's easy to see
    color=np.array([1.0, 0.0, 0.0]),
    mass=0.1,
))

world.reset()

for _ in range(10):
    world.step(render=True)

# Print actual position after reset
pos, _ = pick_box.get_world_pose()
print(f"PickBox actual position after reset: {pos}")

# Print all prims in stage
print("\nAll prims in stage:")
for prim in stage.Traverse():
    print(f"  {prim.GetPath()}")

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()