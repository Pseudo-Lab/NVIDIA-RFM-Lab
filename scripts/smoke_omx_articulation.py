"""Smoke-test OMX-F USD as an Isaac Lab articulation.

Run from the Isaac Lab environment:

    cd <YOUR_ISAACLAB_PATH>
    conda activate <YOUR_CONDA_ENV>
    source _isaac_sim/setup_conda_env.sh
    ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/smoke_omx_articulation.py --headless
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher


REPO_ROOT = Path(__file__).resolve().parents[1]


parser = argparse.ArgumentParser(description="Spawn OMX-F and print Isaac Lab articulation metadata.")
parser.add_argument(
    "--usd_path",
    default=str(REPO_ROOT / "open_manipulator" / "open_manipulator_description" / "urdf" / "omx_f" / "omx_f" / "omx_f.usd"),
    help="Path to the OMX-F USD file.",
)
parser.add_argument(
    "--prim_path",
    default="/World/OMX",
    help="Prim path passed to Isaac Lab ArticulationCfg.",
)
parser.add_argument("--steps", type=int, default=10, help="Number of simulation steps to run before exiting.")
parser.add_argument("--keep_open", action="store_true", help="Keep the GUI running after the smoke steps complete.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch
from pxr import Usd

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.sim import SimulationCfg, SimulationContext

sys.path.insert(0, str(REPO_ROOT / "source"))

from sim2real_omx.assets import OMX_FOLLOWER_CFG


def print_usd_summary(usd_path: str):
    stage = Usd.Stage.Open(usd_path)
    default_prim = stage.GetDefaultPrim()
    print(f"[SMOKE] USD: {usd_path}", flush=True)
    print(f"[SMOKE] USD defaultPrim: {default_prim.GetPath() if default_prim else '<none>'}", flush=True)
    for prim in stage.Traverse():
        schemas = list(prim.GetAppliedSchemas())
        if "PhysicsArticulationRootAPI" in schemas or prim.GetTypeName().startswith("Physics"):
            print(f"[SMOKE] USD physics prim: {prim.GetPath()} type={prim.GetTypeName()} schemas={schemas}", flush=True)


def make_omx_cfg(usd_path: str, prim_path: str) -> ArticulationCfg:
    cfg = OMX_FOLLOWER_CFG.replace(prim_path=prim_path)
    cfg.spawn.usd_path = usd_path
    return cfg


def main():
    print_usd_summary(args_cli.usd_path)
    print(f"[SMOKE] ArticulationCfg prim_path: {args_cli.prim_path}", flush=True)

    print("[SMOKE] Creating SimulationContext...", flush=True)
    sim = SimulationContext(SimulationCfg(dt=0.01, device=args_cli.device))
    sim.set_camera_view([0.55, -0.75, 0.55], [0.12, 0.0, 0.18])

    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/defaultGroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=2500.0)
    light_cfg.func("/World/Light", light_cfg)

    print("[SMOKE] Creating Articulation object...", flush=True)
    robot = Articulation(make_omx_cfg(args_cli.usd_path, args_cli.prim_path))
    print("[SMOKE] Calling sim.reset()...", flush=True)

    sim.reset()
    print("[SMOKE] Calling robot.reset()...", flush=True)
    robot.reset()
    print("[SMOKE] Calling first robot.update()...", flush=True)
    robot.update(sim.get_physics_dt())

    print("[SMOKE] Isaac Lab setup complete.", flush=True)
    print(f"[SMOKE] num_instances: {robot.num_instances}", flush=True)
    print(f"[SMOKE] num_joints: {robot.num_joints}", flush=True)
    print(f"[SMOKE] joint_names: {robot.joint_names}", flush=True)
    print(f"[SMOKE] num_bodies: {robot.num_bodies}", flush=True)
    print(f"[SMOKE] body_names: {robot.body_names}", flush=True)

    arm_joint_ids, arm_joint_names = robot.find_joints(["joint[1-5]"], preserve_order=True)
    print(f"[SMOKE] arm_joint_ids: {arm_joint_ids}", flush=True)
    print(f"[SMOKE] arm_joint_names: {arm_joint_names}", flush=True)

    print(
        f"[SMOKE] end_effector_link in body_names: {'end_effector_link' in robot.body_names}",
        flush=True,
    )
    print("[SMOKE] NOTE: end_effector_link is present in USD as an Xform under link5, not a rigid body.", flush=True)
    for body_expr in ["link5", "link6", "link7"]:
        body_ids, body_names = robot.find_bodies(body_expr, preserve_order=True)
        print(f"[SMOKE] find_bodies({body_expr!r}): ids={body_ids}, names={body_names}", flush=True)

    default_joint_pos = robot.data.default_joint_pos.detach().cpu().tolist()
    default_joint_vel = robot.data.default_joint_vel.detach().cpu().tolist()
    print(f"[SMOKE] default_joint_pos: {default_joint_pos}", flush=True)
    print(f"[SMOKE] default_joint_vel: {default_joint_vel}", flush=True)

    target = robot.data.default_joint_pos.clone()
    if len(arm_joint_ids) == 5:
        target[:, arm_joint_ids] = torch.tensor([[0.0, 0.25, -0.35, 0.2, 0.1]], device=sim.device)
    robot.set_joint_position_target(target)

    for _ in range(args_cli.steps):
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())

    print(f"[SMOKE] final_joint_pos: {robot.data.joint_pos.detach().cpu().tolist()}", flush=True)
    print("[SMOKE] PASS: OMX-F spawned and Isaac Lab articulation metadata was read.", flush=True)

    if args_cli.keep_open:
        print("[SMOKE] Keeping GUI open. Close the Isaac Sim window when inspection is done.", flush=True)
        while simulation_app.is_running():
            robot.write_data_to_sim()
            sim.step()
            robot.update(sim.get_physics_dt())


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close(skip_cleanup=True)
