# check_franka_url.py
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.world import World
import omni.usd

assets_root = get_assets_root_path()
print(f"Assets root: {assets_root}")

# Try loading each path directly into the stage
paths_to_check = [
    "/Isaac/Robots/Franka/franka.usd",
    "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    "/Isaac/Robots/Franka/franka_alt_fingers.usd",
]

world = World()
world.reset()

for p in paths_to_check:
    full = assets_root + p
    print(f"\nTrying: {full}")
    try:
        add_reference_to_stage(usd_path=full, prim_path="/World/TestRobot")
        # tick a few frames to let it load
        for _ in range(10):
            simulation_app.update()
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath("/World/TestRobot")
        print(f"  Result: {'LOADED OK' if prim.IsValid() else 'INVALID PRIM'}")
        # clean up for next test
        stage.RemovePrim("/World/TestRobot")
    except Exception as e:
        print(f"  Result: FAILED — {e}")

simulation_app.close()