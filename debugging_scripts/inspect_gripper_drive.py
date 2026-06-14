from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import os
from pxr import UsdPhysics, Usd
import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage

FRANKA_USD_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "assets", "franka.usd")
)

stage = omni.usd.get_context().get_stage()
add_reference_to_stage(usd_path=FRANKA_USD_PATH, prim_path="/World/Franka")

while not stage.GetPrimAtPath("/World/Franka/panda_link0").IsValid():
    simulation_app.update()

for prim in stage.Traverse():
    name = prim.GetName()
    if "finger_joint" in name:
        print(f"\nPrim: {prim.GetPath()}")
        print(f"  Type: {prim.GetTypeName()}")
        joint = UsdPhysics.Joint(prim)
        if joint:
            print(f"  Body0: {joint.GetBody0Rel().GetTargets()}")
            print(f"  Body1: {joint.GetBody1Rel().GetTargets()}")
        prismatic = UsdPhysics.PrismaticJoint(prim)
        if prismatic:
            print(f"  LowerLimit: {prismatic.GetLowerLimitAttr().Get()}")
            print(f"  UpperLimit: {prismatic.GetUpperLimitAttr().Get()}")
        if prim.HasAPI(UsdPhysics.DriveAPI, "linear"):
            drive = UsdPhysics.DriveAPI(prim, "linear")
            print(f"  Drive type: {drive.GetTypeAttr().Get()}")
            print(f"  Stiffness: {drive.GetStiffnessAttr().Get()}")
            print(f"  Damping: {drive.GetDampingAttr().Get()}")
            print(f"  MaxForce: {drive.GetMaxForceAttr().Get()}")
            print(f"  TargetPosition: {drive.GetTargetPositionAttr().Get()}")
        else:
            print("  No linear DriveAPI applied")
        for attr in prim.GetAttributes():
            if "mimic" in attr.GetName().lower():
                print(f"  {attr.GetName()} = {attr.Get()}")

simulation_app.close()
