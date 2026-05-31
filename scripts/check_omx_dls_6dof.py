"""Check DLS IK behavior for OMX-F around the Sim2Real objects.

This is the F-task smoke test. It answers:

* Can OMX-F reach object pregrasp positions?
* Does a 6-DOF pose command react to roll changes?
* How large are the final position/orientation errors?

Run:

    cd <YOUR_ISAACLAB_PATH>
    conda activate <YOUR_CONDA_ENV>
    source _isaac_sim/setup_conda_env.sh
    ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/check_omx_dls_6dof.py --headless
"""

import argparse
import math
from pathlib import Path

from isaaclab.app import AppLauncher


REPO_ROOT = Path(__file__).resolve().parents[1]


parser = argparse.ArgumentParser(description="Run OMX-F DLS IK roll sensitivity checks.")
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
parser.add_argument("--object", choices=["bear", "chick", "fish", "all"], default="all")
parser.add_argument("--iters", type=int, default=180, help="IK iterations per target.")
parser.add_argument("--settle_steps", type=int, default=20, help="Steps to settle after reset.")
parser.add_argument(
    "--ik_mode",
    choices=["full_pose", "position", "mask_no_wx", "mask_no_wy", "mask_no_wz", "all"],
    default="full_pose",
    help="IK mode to test. Use all to compare full 6D, position-only, and 5D masked DLS.",
)
parser.add_argument("--damping", type=float, default=0.05, help="Damping value for custom masked DLS modes.")
parser.add_argument("--step_scale", type=float, default=0.6, help="Joint update scale for custom masked DLS modes.")
parser.add_argument("--keep_open", action="store_true", help="Keep the GUI running after all DLS cases complete.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Gf, Usd, UsdGeom, UsdPhysics

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils.math import (
    compute_pose_error,
    quat_from_angle_axis,
    quat_mul,
    subtract_frame_transforms,
)


OBJECT_SPECS = {
    "bear": {
        "path": "/bear_v2_texture",
        "target_x_m": 0.075,
        "target_z_m": 0.100,
        "center_xy": (0.20, -0.07),
        "mass_kg": 0.05,
    },
    "chick": {
        "path": "/chick_v2_texture",
        "target_x_m": 0.080,
        "target_z_m": 0.055,
        "center_xy": (0.26, 0.00),
        "mass_kg": 0.04,
    },
    "fish": {
        "path": "/fish_v2_texture",
        "target_x_m": 0.110,
        "target_z_m": 0.090,
        "center_xy": (0.32, 0.07),
        "mass_kg": 0.05,
    },
}

FLOOR_Z = 0.0
ROLL_DEGREES = [0.0, 45.0, 90.0]
PREGRASP_CLEARANCE_M = 0.055
IK_MODES = ["full_pose", "position", "mask_no_wx", "mask_no_wy", "mask_no_wz"]
MASK_ROWS = {
    "position": [0, 1, 2],
    "mask_no_wx": [0, 1, 2, 4, 5],
    "mask_no_wy": [0, 1, 2, 3, 5],
    "mask_no_wz": [0, 1, 2, 3, 4],
}


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
                stiffness=120.0,
                damping=12.0,
            )
        },
    )


def iter_mesh_prims(prim):
    for child in Usd.PrimRange(prim):
        if child.GetTypeName() == "Mesh":
            yield child


def add_floor():
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


def configure_object_physics(prim, mass_kg: float):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass_kg)
    for mesh_prim in iter_mesh_prims(prim):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
        UsdPhysics.MeshCollisionAPI.Apply(mesh_prim).CreateApproximationAttr("convexHull")


def scale_and_place_objects(stage):
    bbox_cache = UsdGeom.BBoxCache(0.0, [UsdGeom.Tokens.default_])
    for spec in OBJECT_SPECS.values():
        prim = stage.GetPrimAtPath(spec["path"])
        if not prim.IsValid():
            raise RuntimeError(f"Missing object prim: {spec['path']}")

        bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        bbox_min = bbox.GetMin()
        bbox_max = bbox.GetMax()
        bbox_size = bbox_max - bbox_min
        bbox_center = (bbox_min + bbox_max) * 0.5

        xform = UsdGeom.Xformable(prim)
        current_translation = xform.ComputeLocalToWorldTransform(0.0).ExtractTranslation()
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
        configure_object_physics(prim, spec["mass_kg"])


def get_object_pregrasp_targets(stage):
    bbox_cache = UsdGeom.BBoxCache(0.0, [UsdGeom.Tokens.default_])
    targets = {}
    for name, spec in OBJECT_SPECS.items():
        if args_cli.object != "all" and args_cli.object != name:
            continue
        prim = stage.GetPrimAtPath(spec["path"])
        bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        mn = bbox.GetMin()
        mx = bbox.GetMax()
        center = (mn + mx) * 0.5
        targets[name] = torch.tensor(
            [[float(center[0]), float(center[1]), float(mx[2] + PREGRASP_CLEARANCE_M)]],
            device="cuda:0" if args_cli.device.startswith("cuda") else args_cli.device,
        )
    return targets


def reset_robot(robot: Articulation, sim: SimulationContext):
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
    robot.reset()
    for _ in range(args_cli.settle_steps):
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())


def dls_delta(jacobian: torch.Tensor, error: torch.Tensor, damping: float) -> torch.Tensor:
    jacobian_t = torch.transpose(jacobian, 1, 2)
    eye = torch.eye(jacobian.shape[1], device=jacobian.device).unsqueeze(0)
    lhs = jacobian @ jacobian_t + damping**2 * eye
    rhs = error.unsqueeze(-1)
    return (jacobian_t @ torch.linalg.solve(lhs, rhs)).squeeze(-1)


def run_case(
    sim: SimulationContext,
    robot: Articulation,
    ee_body_id: int,
    ee_jacobi_idx: int,
    joint_ids,
    target_pos,
    roll_deg,
    ik_mode,
):
    reset_robot(robot, sim)

    root_pose_w = robot.data.root_pose_w
    ee_pose_w = robot.data.body_pose_w[:, ee_body_id]
    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
    )

    roll_rad = torch.tensor([math.radians(roll_deg)], device=sim.device)
    roll_axis = torch.tensor([[1.0, 0.0, 0.0]], device=sim.device)
    roll_delta = quat_from_angle_axis(roll_rad, roll_axis)
    # Use the starting link5 orientation as the base pose. This isolates whether DLS responds to roll deltas.
    target_quat = quat_mul(ee_quat_b, roll_delta)

    controller = None
    if ik_mode == "full_pose":
        diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
        controller = DifferentialIKController(diff_ik_cfg, num_envs=1, device=sim.device)
        command = torch.cat((target_pos.to(sim.device), target_quat), dim=1)
        controller.reset()
        controller.set_command(command)

    joint_pos_des = robot.data.joint_pos[:, joint_ids].clone()
    for _ in range(args_cli.iters):
        jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, joint_ids]
        ee_pose_w = robot.data.body_pose_w[:, ee_body_id]
        root_pose_w = robot.data.root_pose_w
        joint_pos = robot.data.joint_pos[:, joint_ids]
        ee_pos_b, ee_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
        )
        if controller is not None:
            joint_pos_des = controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
        else:
            pos_error, axis_angle_error = compute_pose_error(
                ee_pos_b, ee_quat_b, target_pos.to(sim.device), target_quat, rot_error_type="axis_angle"
            )
            pose_error = torch.cat((pos_error, axis_angle_error), dim=1)
            row_ids = torch.tensor(MASK_ROWS[ik_mode], device=sim.device)
            masked_error = pose_error[:, row_ids]
            masked_jacobian = jacobian[:, row_ids, :]
            joint_pos_des = joint_pos + args_cli.step_scale * dls_delta(masked_jacobian, masked_error, args_cli.damping)
        robot.set_joint_position_target(joint_pos_des, joint_ids=joint_ids)
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())

    ee_pose_w = robot.data.body_pose_w[:, ee_body_id]
    root_pose_w = robot.data.root_pose_w
    final_pos_b, final_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
    )
    pos_error, axis_angle_error = compute_pose_error(
        final_pos_b, final_quat_b, target_pos.to(sim.device), target_quat, rot_error_type="axis_angle"
    )
    return {
        "target_pos": target_pos.detach().cpu()[0].tolist(),
        "target_quat": target_quat.detach().cpu()[0].tolist(),
        "final_pos": final_pos_b.detach().cpu()[0].tolist(),
        "final_quat": final_quat_b.detach().cpu()[0].tolist(),
        "pos_error_m": float(torch.linalg.norm(pos_error, dim=1).detach().cpu()[0]),
        "ori_error_deg": float(torch.linalg.norm(axis_angle_error, dim=1).detach().cpu()[0] * 180.0 / math.pi),
        "joint_pos": robot.data.joint_pos[:, joint_ids].detach().cpu()[0].tolist(),
    }


def main():
    print("[F] Creating simulation context...", flush=True)
    sim = SimulationContext(SimulationCfg(dt=0.01, device=args_cli.device))
    stage = sim.stage
    print("[F] Composing scene and objects...", flush=True)
    stage.GetRootLayer().subLayerPaths.append(args_cli.scene_usd)
    print("[F] Added scene sublayer.", flush=True)
    add_floor()
    print("[F] Added floor.", flush=True)
    scale_and_place_objects(stage)
    print("[F] Scaled and placed objects.", flush=True)

    print("[F] Creating robot articulation...", flush=True)
    robot = Articulation(make_omx_cfg())
    sim.set_camera_view([0.55, -0.75, 0.48], [0.18, 0.0, 0.08])
    print("[F] Calling sim.reset()...", flush=True)
    sim.reset()
    print("[F] Resetting robot...", flush=True)
    robot.reset()
    robot.update(sim.get_physics_dt())
    print("[F] Robot initialized.", flush=True)

    joint_ids, joint_names = robot.find_joints(["joint[1-5]"], preserve_order=True)
    body_ids, body_names = robot.find_bodies("link5", preserve_order=True)
    ee_body_id = body_ids[0]
    ee_jacobi_idx = ee_body_id - 1 if robot.is_fixed_base else ee_body_id
    targets = get_object_pregrasp_targets(stage)

    print("[F] OMX DLS / 6-DOF target check", flush=True)
    print(f"[F] controlled_joints={joint_names} ids={joint_ids}", flush=True)
    print(f"[F] ee_body={body_names[0]} body_id={ee_body_id} jacobian_id={ee_jacobi_idx}", flush=True)
    print("[F] link5 is used because end_effector_link is an Xform, not an Isaac Lab rigid body.", flush=True)
    ik_modes = IK_MODES if args_cli.ik_mode == "all" else [args_cli.ik_mode]
    print(f"[F] ik_modes={ik_modes}", flush=True)
    print("[F] object,ik_mode,roll_deg,pos_error_m,ori_error_deg,joint_pos", flush=True)

    for object_name, target_pos in targets.items():
        for ik_mode in ik_modes:
            for roll_deg in ROLL_DEGREES:
                result = run_case(sim, robot, ee_body_id, ee_jacobi_idx, joint_ids, target_pos, roll_deg, ik_mode)
                print(
                    f"[F] {object_name},{ik_mode},{roll_deg:.0f},{result['pos_error_m']:.5f},"
                    f"{result['ori_error_deg']:.2f},{[round(v, 4) for v in result['joint_pos']]}",
                    flush=True,
                )
                print(
                    f"[F_DETAIL] object={object_name} mode={ik_mode} roll={roll_deg:.0f} "
                    f"target_pos={[round(v, 4) for v in result['target_pos']]} "
                    f"final_pos={[round(v, 4) for v in result['final_pos']]} "
                    f"target_quat={[round(v, 4) for v in result['target_quat']]}",
                    flush=True,
                )

    if args_cli.keep_open:
        print("[F] Keeping GUI open on the final pose. Close the Isaac Sim window when inspection is done.", flush=True)
        while simulation_app.is_running():
            robot.write_data_to_sim()
            sim.step()
            robot.update(sim.get_physics_dt())


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close(skip_cleanup=True)
