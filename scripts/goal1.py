from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

import numpy as np
import omni.usd
from pxr import Sdf, UsdLux
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.world import World

def addObjects():
    world = World()

    # ground plane
    world.scene.add_default_ground_plane()

    # light
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight")).CreateIntensityAttr(500)

    # table
    world.scene.add(FixedCuboid(
        prim_path="/World/Table",
        name="table",
        position=np.array([0.0, 0.0, 0.25]),
        scale=np.array([1.2, 0.6, 0.1]),
        color=np.array([0.4, 0.25, 0.1]),
    ))

    # box 1
    world.scene.add(DynamicCuboid(
        prim_path="/World/Box1",
        name="box1",
        position=np.array([-0.2, 0.0, 0.65]),
        scale=np.array([0.1, 0.1, 0.1]),
        color=np.array([1.0, 0.2, 0.2]),
        mass=0.5,
    ))

    return world


if __name__ == "__main__":
    world = addObjects()
    world.reset()

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()