"""Spawn OMX-F with the Sim2Real object scene.

This does not edit Assets/Sim2Real.usd. It composes it into the current
runtime stage as a sublayer, then spawns OMX-F at /World/OMX.

Examples:

    # Headless smoke check
    cd <YOUR_ISAACLAB_PATH>
    conda activate <YOUR_CONDA_ENV>
    source _isaac_sim/setup_conda_env.sh
    ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/run_omx_sim2real_scene.py --headless --steps 1

    # Visual inspection: close the Isaac Sim window when done
    ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/run_omx_sim2real_scene.py
"""

import argparse
import math
from pathlib import Path

from isaaclab.app import AppLauncher


REPO_ROOT = Path(__file__).resolve().parents[1]


parser = argparse.ArgumentParser(description="Compose Sim2Real.usd and spawn OMX-F in Isaac Lab.")
parser.add_argument(
    "--scene_usd",
    default=str(REPO_ROOT / "Assets" / "Sim2Real.usd"),
    help="Path to the Sim2Real scene USD.",
)
parser.add_argument(
    "--robot_usd",
    default=str(REPO_ROOT / "open_manipulator" / "open_manipulator_description" / "urdf" / "omx_f" / "omx_f" / "omx_f.usd"),
    help="Path to the OMX-F USD file.",
)
parser.add_argument("--steps", type=int, default=-1, help="Number of sim steps. Negative means run until window closes.")
parser.add_argument("--no_floor", action="store_true", help="Do not add the runtime floor collider.")
parser.add_argument("--no_light", action="store_true", help="Do not add the runtime dome light.")
parser.add_argument("--demo_joints", action="store_true", help="Move OMX joints through a small inspection motion.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Gf, Usd, UsdGeom, UsdPhysics

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationCfg, SimulationContext


OBJECT_SPECS = {
    # target_x_m and target_z_m come from the Daiso product dimensions.
    # The missing depth dimension is preserved from the 3D model by using a uniform scale.
    "/bear_v2_texture": {
        "target_x_m": 0.075,
        "target_z_m": 0.100,
        "center_xy": (0.20, -0.07),
        "mass_kg": 0.05,
    },
    "/chick_v2_texture": {
        "target_x_m": 0.080,
        "target_z_m": 0.055,
        "center_xy": (0.26, 0.00),
        "mass_kg": 0.04,
    },
    "/fish_v2_texture": {
        "target_x_m": 0.110,
        "target_z_m": 0.090,
        "center_xy": (0.32, 0.07),
        "mass_kg": 0.05,
    },
}

FLOOR_Z = 0.0


def make_omx_cfg() -> ArticulationCfg:
    return ArticulationCfg(
        prim_path="/World/OMX",
        spawn=sim_utils.UsdFileCfg(
            usd_path=args_cli.robot_usd,
            activate_contact_sensors=False,
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.0),
            joint_pos={
                "joint1": 0.0,
                "joint2": 0.0,
                "joint3": 0.0,
                "joint4": 0.0,
                "joint5": 0.0,
                "gripper_joint_1": 0.0,
                "gripper_joint_2": 0.0,
            },
        ),
        actuators={
            "all_joints": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
                effort_limit=1000.0,
                velocity_limit=4.8,
                stiffness=100.0,
                damping=10.0,
            )
        },
    )


def add_scene_sublayer(stage):
    root_layer = stage.GetRootLayer()
    if args_cli.scene_usd not in root_layer.subLayerPaths:
        root_layer.subLayerPaths.append(args_cli.scene_usd)
    print(f"[C1] Added scene sublayer: {args_cli.scene_usd}", flush=True)


def iter_mesh_prims(prim):
    for child in Usd.PrimRange(prim):
        if child.GetTypeName() == "Mesh":
            yield child


def add_floor(stage):
    floor_cfg = sim_utils.CuboidCfg(
        size=(0.8, 0.8, 0.01),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.015, 0.015, 0.015),
            roughness=0.95,
            metallic=0.0,
        ),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=1.0,
            dynamic_friction=0.8,
            restitution=0.0,
        ),
        collision_props=sim_utils.CollisionPropertiesCfg(),
    )
    floor_cfg.func("/World/Floor", floor_cfg, translation=(0.0, 0.0, -0.005))
    print("[C1] Added matte floor collider centered at world origin.", flush=True)


def configure_object_physics(stage, prim, mass_kg: float):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(mass_kg)

    mesh_count = 0
    for mesh_prim in iter_mesh_prims(prim):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
        mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
        mesh_collision.CreateApproximationAttr("convexHull")
        mesh_count += 1
    return mesh_count


def scale_and_place_objects(stage):
    bbox_cache = UsdGeom.BBoxCache(0.0, [UsdGeom.Tokens.default_])

    for path, spec in OBJECT_SPECS.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"[C1] object missing, cannot scale/place: {path}", flush=True)
            continue

        bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        bbox_min = bbox.GetMin()
        bbox_max = bbox.GetMax()
        bbox_size = bbox_max - bbox_min
        bbox_center = (bbox_min + bbox_max) * 0.5

        xform = UsdGeom.Xformable(prim)
        current_world = xform.ComputeLocalToWorldTransform(0.0)
        current_translation = current_world.ExtractTranslation()

        scale_x = spec["target_x_m"] / bbox_size[0]
        scale_z = spec["target_z_m"] / bbox_size[2]
        uniform_scale = (scale_x + scale_z) * 0.5

        local_center_x = bbox_center[0] - current_translation[0]
        local_center_y = bbox_center[1] - current_translation[1]
        local_min_z = bbox_min[2] - current_translation[2]

        target_x, target_y = spec["center_xy"]
        translate = Gf.Vec3d(
            target_x - local_center_x * uniform_scale,
            target_y - local_center_y * uniform_scale,
            FLOOR_Z - local_min_z * uniform_scale,
        )

        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(translate)
        xform.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0))
        xform.AddScaleOp().Set(Gf.Vec3f(uniform_scale, uniform_scale, uniform_scale))

        mesh_count = configure_object_physics(stage, prim, spec["mass_kg"])
        print(
            f"[C1] placed {path}: scale={uniform_scale:.8f}, translate={translate}, mass={spec['mass_kg']}, "
            f"collider_meshes={mesh_count}",
            flush=True,
        )


def print_object_summary(stage):
    bbox_cache = UsdGeom.BBoxCache(0.0, [UsdGeom.Tokens.default_])
    for path in OBJECT_SPECS:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"[C1] object missing: {path}", flush=True)
            continue
        schemas = list(prim.GetAppliedSchemas())
        local_to_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)
        translation = local_to_world.ExtractTranslation()
        bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        print(f"[C1] object: {path}", flush=True)
        print(f"[C1]   type={prim.GetTypeName()} schemas={schemas}", flush=True)
        print(f"[C1]   world_translation={translation}", flush=True)
        print(f"[C1]   world_bbox_min={bbox.GetMin()} max={bbox.GetMax()}", flush=True)


def get_demo_joint_target(robot: Articulation, sim, count: int):
    target = robot.data.default_joint_pos.clone()
    arm_joint_ids, _ = robot.find_joints(["joint[1-5]"], preserve_order=True)
    gripper_joint_ids, _ = robot.find_joints(["gripper_joint_.*"], preserve_order=True)
    phase = count * sim.get_physics_dt()
    values = torch.tensor(
        [
            0.20 * math.sin(phase * 0.8),
            0.35 + 0.18 * math.sin(phase * 1.1),
            -0.55 + 0.18 * math.sin(phase * 1.3),
            0.30 + 0.12 * math.sin(phase * 1.7),
            0.35 * math.sin(phase * 1.0),
        ],
        device=sim.device,
    )
    target[:, arm_joint_ids] = values
    if len(gripper_joint_ids) >= 2:
        gripper = 0.12 + 0.08 * math.sin(phase * 1.5)
        target[:, gripper_joint_ids[0]] = gripper
        target[:, gripper_joint_ids[1]] = -gripper
    return target


def main():
    sim = SimulationContext(SimulationCfg(dt=0.01, device=args_cli.device))
    stage = sim.stage

    add_scene_sublayer(stage)
    if not args_cli.no_floor:
        add_floor(stage)
    scale_and_place_objects(stage)

    if not args_cli.no_light:
        dome_cfg = sim_utils.DomeLightCfg(
            intensity=120.0,
            color=(0.85, 0.82, 0.78),
            visible_in_primary_ray=False,
        )
        dome_cfg.func("/World/SoftAmbient", dome_cfg)
        key_cfg = sim_utils.DistantLightCfg(
            intensity=450.0,
            color=(1.0, 0.88, 0.72),
            angle=8.0,
        )
        key_cfg.func("/World/WarmKeyLight", key_cfg, rotation=(math.radians(-45.0), math.radians(0.0), math.radians(35.0)))

    robot = Articulation(make_omx_cfg())
    sim.set_camera_view([0.55, -0.75, 0.48], [0.18, 0.0, 0.08])

    sim.reset()
    robot.reset()
    robot.update(sim.get_physics_dt())

    print("[C1] OMX + Sim2Real scene setup complete.", flush=True)
    print(f"[C1] robot_joints={robot.joint_names}", flush=True)
    print(f"[C1] robot_bodies={robot.body_names}", flush=True)
    print_object_summary(stage)
    print("[C1] NOTE: If object bbox positions look huge, object scale/units need B-task cleanup.", flush=True)

    count = 0
    while simulation_app.is_running():
        if args_cli.demo_joints:
            robot.set_joint_position_target(get_demo_joint_target(robot, sim, count))
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        count += 1
        if args_cli.steps >= 0 and count >= args_cli.steps:
            break


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close(skip_cleanup=True)
