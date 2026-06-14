import os
import time
import psutil

# Kill any leftover python instances of this script from previous runs so
# they don't hold onto the GPU/CUDA context before we start a new SimulationApp.
# Only target actual python processes (not the conda/launcher wrapper whose
# command line also happens to mention this script) and never our own ancestors.
_current_proc = psutil.Process(os.getpid())
_ancestor_pids = {_current_proc.pid}
_p = _current_proc
while _p.parent() is not None:
    _p = _p.parent()
    _ancestor_pids.add(_p.pid)

_script_name = os.path.basename(__file__)
_killed_stale = False
for _proc in psutil.process_iter(["pid", "name", "cmdline"]):
    if _proc.info["pid"] in _ancestor_pids:
        continue
    if "python" not in (_proc.info["name"] or "").lower():
        continue
    _cmdline = _proc.info["cmdline"] or []
    if any(_script_name in arg for arg in _cmdline):
        print(f"Killing stale process {_proc.info['pid']} from a previous run...")
        try:
            _proc.kill()
            _killed_stale = True
        except psutil.NoSuchProcess:
            pass
if _killed_stale:
    time.sleep(2)

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

import numpy as np
import omni.usd
from pxr import Sdf, UsdLux, UsdShade, UsdPhysics
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.kinematics_solver import KinematicsSolver

FRANKA_USD_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "assets", "franka.usd")
)


def setup_camera(stage):
    from pxr import Gf
    camera_prim = stage.GetPrimAtPath("/OmniverseKit_Persp")
    if camera_prim.IsValid():
        camera_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, -3.0, 2.0))
        camera_prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(55.0, 0.0, 0.0))


def set_friction(stage, prim_path, friction=0.1):
    mat_path = prim_path + "_material"
    material = UsdShade.Material.Define(stage, mat_path)
    physics_mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_mat.CreateStaticFrictionAttr(friction)
    physics_mat.CreateDynamicFrictionAttr(friction)
    physics_mat.CreateRestitutionAttr(0.0)
    prim = stage.GetPrimAtPath(prim_path)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics"
    )


def move_to_position(franka, ik_solver, target_pos, target_rot, steps, world):
    action, success = ik_solver.compute_inverse_kinematics(
        target_position=target_pos,
        target_orientation=target_rot,
    )

    if not success:
        print(f"  IK failed for target {np.round(target_pos, 3)} — skipping")
        return

    # Extract joint positions from ArticulationAction
    joint_positions = action.joint_positions
    print(f"  IK joint positions: {np.round(joint_positions, 3)}, shape: {joint_positions.shape}")

    current = franka.get_joint_positions()
    print(f"  Current joints shape: {current.shape}")

    arm_current = current[:7]
    arm_target = joint_positions[:7]

    for i in range(steps):
        alpha = (i + 1) / steps
        interp = arm_current + alpha * (arm_target - arm_current)
        franka.apply_action(ArticulationAction(joint_positions=interp, joint_indices=np.arange(7)))
        world.step(render=True)

    print(f"  Moved to {np.round(target_pos, 3)}")


def move_to_joint_positions(franka, target_joints, steps, world):
    """Drive the 7 arm joints toward a target joint configuration."""
    arm_current = franka.get_joint_positions()[:7]
    arm_target = target_joints

    for i in range(steps):
        alpha = (i + 1) / steps
        interp = arm_current + alpha * (arm_target - arm_current)
        franka.apply_action(ArticulationAction(joint_positions=interp, joint_indices=np.arange(7)))
        world.step(render=True)

    print(f"  Moved to joint positions: {np.round(target_joints, 3)}")


def set_gripper(franka, world, close, steps=80):
    """Close or open the gripper using its drive (applies real squeeze force)."""
    print(f"  Gripper joints before: {np.round(franka.get_joint_positions()[-2:], 4)}")
    print(f"  Gripper active dof indices: {franka.gripper.joint_dof_indicies}")
    print(f"  Gripper open/closed targets: {franka.gripper.joint_opened_positions} / {franka.gripper.joint_closed_positions}")
    if close:
        franka.gripper.close()
    else:
        franka.gripper.open()
    for i in range(steps):
        world.step(render=True)
        if i % 20 == 0:
            print(f"  Gripper joints @ step {i}: {np.round(franka.get_joint_positions()[-2:], 4)}")
    print(f"  Gripper joints after: {np.round(franka.get_joint_positions()[-2:], 4)}")


if __name__ == "__main__":
    world = World()
    stage = omni.usd.get_context().get_stage()

    world.scene.add_default_ground_plane()
    UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight")).CreateIntensityAttr(500)

    # Table
    world.scene.add(FixedCuboid(
        prim_path="/World/Table",
        name="table",
        position=np.array([0.0, 0.0, 0.25]),
        scale=np.array([2.4, 1.2, 0.1]),
        color=np.array([0.4, 0.25, 0.1]),
    ))

    # Barrier — 1/3 of Franka height above table top (top at 0.465 + 0.165 = 0.63)
    world.scene.add(FixedCuboid(
        prim_path="/World/Barrier",
        name="barrier",
        position=np.array([0.0, 0.0, 0.465]),
        scale=np.array([0.02, 1.2, 0.33]),
        color=np.array([0.8, 0.8, 0.2]),
    ))

    # Box on right side of barrier
    pick_box = world.scene.add(DynamicCuboid(
        prim_path="/World/PickBox",
        name="pick_box",
        position=np.array([0.35, 0.0, 0.43]),
        scale=np.array([0.05, 0.05, 0.05]),
        color=np.array([1.0, 0.2, 0.2]),
        mass=0.05,
    ))

    # Load Franka
    add_reference_to_stage(usd_path=FRANKA_USD_PATH, prim_path="/World/Franka")

    print("Waiting for Franka to load...")
    while not stage.GetPrimAtPath("/World/Franka/panda_link0").IsValid():
        simulation_app.update()
    print("Franka loaded!")

    # Robot on left side of barrier
    franka = world.scene.add(Franka(
        prim_path="/World/Franka",
        name="franka",
        position=np.array([-0.35, 0.0, 0.32]),
    ))

    world.reset()
    world.step(render=True)
    setup_camera(stage)

    # IK solver
    ik_solver = KinematicsSolver(franka)
    robot_pos, robot_orient = franka.get_world_pose()
    ik_solver.get_kinematics_solver().set_robot_base_pose(robot_pos, robot_orient)

    set_friction(stage, "/World/PickBox", friction=1.0)
    set_friction(stage, "/World/Franka/panda_leftfinger", friction=1.0)
    set_friction(stage, "/World/Franka/panda_rightfinger", friction=1.0)

    # Settle
    for _ in range(120):
        world.step(render=True)

    pos_box, _ = pick_box.get_world_pose()
    print(f"Box settled at: {np.round(pos_box, 3)}")

    # Remember starting joint configuration so the arm can return to it later
    initial_arm_joints = franka.get_joint_positions()[:7].copy()

    # Gripper pointing straight down (180 deg rotation about Y)
    gripper_down = np.array([0.0, 0.0, 1.0, 0.0])

    # Phase 1: pre-grasp above box
    print("Phase 1: pre-grasp above box...")
    move_to_position(franka, ik_solver,
        target_pos=np.array([pos_box[0], pos_box[1], pos_box[2] + 0.2]),
        target_rot=gripper_down, steps=200, world=world)

    # Phase 2: lower to grasp (gripper frame at box center height)
    print("Phase 2: lowering to grasp...")
    move_to_position(franka, ik_solver,
        target_pos=np.array([pos_box[0], pos_box[1], pos_box[2]]),
        target_rot=gripper_down, steps=200, world=world)

    # Phase 3: close gripper
    print("Phase 3: closing gripper...")
    set_gripper(franka, world, close=True)

    # Phase 4: lift clear of the barrier (top at z=0.63), staying within arm reach
    print("Phase 4: lifting above barrier...")
    move_to_position(franka, ik_solver,
        target_pos=np.array([pos_box[0], pos_box[1], 0.7]),
        target_rot=gripper_down, steps=200, world=world)
    box_pos_after_lift, _ = pick_box.get_world_pose()
    print(f"  Box height after lift: {np.round(box_pos_after_lift[2], 3)} (started at {np.round(pos_box[2], 3)})")

    # Phase 4b: arc toward barrier staying high
    print("Phase 4b: arcing toward barrier...")
    move_to_position(franka, ik_solver,
        target_pos=np.array([0.15, 0.0, 0.9]),
        target_rot=gripper_down, steps=150, world=world)

    # Phase 4c: cross over barrier to left side
    print("Phase 4c: crossing barrier...")
    move_to_position(franka, ik_solver,
        target_pos=np.array([-0.15, 0.0, 0.9]),
        target_rot=gripper_down, steps=150, world=world)

    # Phase 5: swing to robot's left side, above place spot (away from table center)
    print("Phase 5: above place spot (robot's left)...")
    move_to_position(franka, ik_solver,
        target_pos=np.array([-0.15, 0.5, 0.9]),
        target_rot=gripper_down, steps=150, world=world)

    # Phase 6: lower to place
    print("Phase 6: lowering to place...")
    move_to_position(franka, ik_solver,
        target_pos=np.array([-0.15, 0.5, pos_box[2]]),
        target_rot=gripper_down, steps=200, world=world)

    # Phase 7: open gripper
    print("Phase 7: opening gripper...")
    set_gripper(franka, world, close=False)

    # Phase 8: return arm to its original starting configuration
    print("Phase 8: returning to start position...")
    move_to_joint_positions(franka, initial_arm_joints, steps=200, world=world)

    print("Pick and place complete!")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()