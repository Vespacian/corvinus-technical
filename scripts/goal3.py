from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

import numpy as np
import omni.usd
from pxr import Sdf, UsdLux, UsdShade, UsdPhysics, Gf
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.world import World

def set_friction(stage, prim_path, friction=0.1):
    """Apply a physics material with low friction to a prim."""
    mat_path = prim_path + "_material"
    material = UsdShade.Material.Define(stage, mat_path)
    physics_mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_mat.CreateStaticFrictionAttr(friction)
    physics_mat.CreateDynamicFrictionAttr(friction)
    physics_mat.CreateRestitutionAttr(0.0)  # no bounciness

    # bind the material to the prim
    prim = stage.GetPrimAtPath(prim_path)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics"
    )

def addObjects():
    world = World()
    world.scene.add_default_ground_plane()

    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight")).CreateIntensityAttr(500)

    world.scene.add(FixedCuboid(
        prim_path="/World/Table",
        name="table",
        position=np.array([0.0, 0.0, 0.25]),
        scale=np.array([1.2, 0.6, 0.1]),
        color=np.array([0.4, 0.25, 0.1]),
    ))

    box1 = world.scene.add(DynamicCuboid(
        prim_path="/World/Box1",
        name="box1",
        position=np.array([-0.4, 0.0, 0.65]),
        scale=np.array([0.1, 0.1, 0.1]),
        color=np.array([1.0, 0.2, 0.2]),
        mass=0.5,
    ))

    box2 = world.scene.add(DynamicCuboid(
        prim_path="/World/Box2",
        name="box2",
        position=np.array([0.1, 0.0, 0.65]),
        scale=np.array([0.1, 0.1, 0.1]),
        color=np.array([0.2, 0.4, 1.0]),
        mass=0.5,
    ))

    return world, box1, box2, stage


if __name__ == "__main__":
    world, box1, box2, stage = addObjects()
    world.reset()

    # apply low friction to both boxes after reset
    set_friction(stage, "/World/Box1", friction=0.1)
    set_friction(stage, "/World/Box2", friction=0.1)

    # settle phase
    while True:
        world.step(render=True)
        vel = box1.get_linear_velocity()
        if abs(vel[2]) < 0.01:
            break

    CORNER_TARGET = np.array([0.55, 0.25])
    PUSH_VELOCITY = np.array([0.6, 0.0, 0.0])   # stronger push
    DONE_THRESHOLD = 0.08
    pushing = True

    while simulation_app.is_running():
        world.step(render=True)

        pos2, _ = box2.get_world_pose()

        if pushing:
            dist = np.linalg.norm(pos2[:2] - CORNER_TARGET)
            if dist > DONE_THRESHOLD:
                box1.set_linear_velocity(PUSH_VELOCITY)
            else:
                print(f"Box2 reached corner! Final position: {np.round(pos2, 3)}")
                box1.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
                pushing = False

    simulation_app.close()