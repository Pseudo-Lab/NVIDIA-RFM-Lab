"""Keyboard teleoperation for OMX-F using reduced DLS.

This script loads the baked nominal scene and controls the OMX-F end-effector
with Isaac Lab's Se3Keyboard device. The 5-DOF arm uses masked DLS by default.

Run:

    cd <YOUR_ISAACLAB_PATH>
    conda activate <YOUR_CONDA_ENV>
    source _isaac_sim/setup_conda_env.sh
    ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/teleop_omx_keyboard.py
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher


REPO_ROOT = Path(__file__).resolve().parents[1]


parser = argparse.ArgumentParser(description="Teleoperate OMX-F with a keyboard and reduced DLS.")
parser.add_argument(
    "--scene_usd",
    default=str(REPO_ROOT / "Assets" / "scenes" / "Sim2Real_OMX_nominal.usd"),
    help="Baked nominal scene USD containing /World/OMX.",
)
parser.add_argument("--steps", type=int, default=-1, help="Simulation steps. Negative means run until window closes.")
parser.add_argument("--pos_sensitivity", type=float, default=0.001, help="Keyboard translation increment in meters.")
parser.add_argument("--rot_sensitivity", type=float, default=0.01, help="Keyboard rotation increment in radians.")
parser.add_argument("--damping", type=float, default=0.05, help="Damping value for reduced DLS.")
parser.add_argument("--step_scale", type=float, default=0.6, help="Scale applied to each DLS joint update.")
parser.add_argument(
    "--ik_mode",
    choices=["position", "mask_no_wx", "mask_no_wy", "mask_no_wz"],
    default="mask_no_wz",
    help="Reduced DLS task-space rows. mask_no_wz is the current OMX candidate.",
)
parser.add_argument("--gripper_open", type=float, default=0.0, help="Driven gripper joint target for the open state.")
parser.add_argument("--gripper_close", type=float, default=0.35, help="Driven gripper joint target for the closed state.")
parser.add_argument("--log_interval", type=int, default=120, help="Print target and error every N simulation steps.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

from isaaclab.assets import Articulation
from isaaclab.devices import Se3Keyboard, Se3KeyboardCfg
from isaaclab.sim import SimulationCfg, SimulationContext
from isaaclab.utils.math import apply_delta_pose, compute_pose_error, subtract_frame_transforms

sys.path.insert(0, str(REPO_ROOT / "source"))

from sim2real_omx.assets import OMX_FOLLOWER_CFG


MASK_ROWS = {
    "position": [0, 1, 2],
    "mask_no_wx": [0, 1, 2, 4, 5],
    "mask_no_wy": [0, 1, 2, 3, 5],
    "mask_no_wz": [0, 1, 2, 3, 4],
}


def dls_delta(jacobian: torch.Tensor, error: torch.Tensor, damping: float) -> torch.Tensor:
    jacobian_t = torch.transpose(jacobian, 1, 2)
    eye = torch.eye(jacobian.shape[1], device=jacobian.device).unsqueeze(0)
    lhs = jacobian @ jacobian_t + damping**2 * eye
    rhs = error.unsqueeze(-1)
    return (jacobian_t @ torch.linalg.solve(lhs, rhs)).squeeze(-1)


def get_ee_pose_b(robot: Articulation, ee_body_id: int):
    root_pose_w = robot.data.root_pose_w
    ee_pose_w = robot.data.body_pose_w[:, ee_body_id]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3],
        root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3],
        ee_pose_w[:, 3:7],
    )


def main():
    if args_cli.headless and args_cli.steps < 0:
        raise ValueError("Use a finite --steps value when running with --headless.")

    sim = SimulationContext(SimulationCfg(dt=0.01, device=args_cli.device))
    sim.stage.GetRootLayer().subLayerPaths.append(args_cli.scene_usd)
    sim.set_camera_view([0.55, -0.75, 0.48], [0.18, 0.0, 0.08])

    robot_cfg = OMX_FOLLOWER_CFG.replace(prim_path="/World/OMX", spawn=None)
    robot = Articulation(robot_cfg)

    sim.reset()
    robot.reset()
    robot.update(sim.get_physics_dt())

    arm_joint_ids, arm_joint_names = robot.find_joints(["joint[1-5]"], preserve_order=True)
    gripper_joint_ids, gripper_joint_names = robot.find_joints(["gripper_joint_1"], preserve_order=True)
    ee_body_ids, ee_body_names = robot.find_bodies("link5", preserve_order=True)
    ee_body_id = ee_body_ids[0]
    ee_jacobi_idx = ee_body_id - 1 if robot.is_fixed_base else ee_body_id
    row_ids = torch.tensor(MASK_ROWS[args_cli.ik_mode], device=sim.device)

    gripper_target = args_cli.gripper_open
    reset_requested = False
    target_pos_b, target_quat_b = get_ee_pose_b(robot, ee_body_id)

    keyboard = Se3Keyboard(
        Se3KeyboardCfg(
            pos_sensitivity=args_cli.pos_sensitivity,
            rot_sensitivity=args_cli.rot_sensitivity,
            sim_device=sim.device,
        )
    )

    def request_reset():
        nonlocal reset_requested
        reset_requested = True

    keyboard.add_callback("R", request_reset)
    keyboard.reset()

    print("[TELEOP] OMX keyboard teleoperation ready.", flush=True)
    print(f"[TELEOP] arm_joints={arm_joint_names}", flush=True)
    print(f"[TELEOP] gripper_joint={gripper_joint_names[0]}", flush=True)
    print(f"[TELEOP] ee_body={ee_body_names[0]} ik_mode={args_cli.ik_mode}", flush=True)
    print("[TELEOP] Keyboard deltas are accumulated into a Cartesian target.", flush=True)
    print(keyboard, flush=True)
    print("[TELEOP] Reset robot and Cartesian target: R", flush=True)
    if args_cli.ik_mode == "mask_no_wz":
        print("[TELEOP] NOTE: C/V yaw input is intentionally not enforced in mask_no_wz mode.", flush=True)

    count = 0
    while simulation_app.is_running():
        if sim.is_stopped():
            break
        if not sim.is_playing():
            sim.step()
            continue

        if reset_requested:
            robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
            robot.reset()
            robot.update(sim.get_physics_dt())
            keyboard.reset()
            gripper_target = args_cli.gripper_open
            target_pos_b, target_quat_b = get_ee_pose_b(robot, ee_body_id)
            reset_requested = False
            print("[TELEOP] Reset complete.", flush=True)

        command = keyboard.advance()
        delta_pose = command[0:6].unsqueeze(0)
        gripper_target = args_cli.gripper_open if command[6] > 0 else args_cli.gripper_close
        target_pos_b, target_quat_b = apply_delta_pose(target_pos_b, target_quat_b, delta_pose)
        ee_pos_b, ee_quat_b = get_ee_pose_b(robot, ee_body_id)

        jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, arm_joint_ids]
        pos_error, axis_angle_error = compute_pose_error(
            ee_pos_b,
            ee_quat_b,
            target_pos_b,
            target_quat_b,
            rot_error_type="axis_angle",
        )
        pose_error = torch.cat((pos_error, axis_angle_error), dim=1)
        arm_joint_pos = robot.data.joint_pos[:, arm_joint_ids]
        arm_joint_target = arm_joint_pos + args_cli.step_scale * dls_delta(
            jacobian[:, row_ids, :],
            pose_error[:, row_ids],
            args_cli.damping,
        )

        robot.set_joint_position_target(arm_joint_target, joint_ids=arm_joint_ids)
        robot.set_joint_position_target(
            torch.tensor([[gripper_target]], device=sim.device),
            joint_ids=gripper_joint_ids,
        )
        robot.write_data_to_sim()
        sim.step()
        robot.update(sim.get_physics_dt())
        count += 1

        if args_cli.log_interval > 0 and count % args_cli.log_interval == 0:
            print(
                f"[TELEOP] step={count} target_pos={[round(v, 4) for v in target_pos_b[0].tolist()]} "
                f"pos_error_m={torch.linalg.norm(pos_error, dim=1).item():.5f} "
                f"ori_error_deg={torch.linalg.norm(axis_angle_error, dim=1).item() * 180.0 / torch.pi:.2f} "
                f"gripper={gripper_target:.3f}",
                flush=True,
            )

        if args_cli.steps >= 0 and count >= args_cli.steps:
            break


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close(skip_cleanup=True)
