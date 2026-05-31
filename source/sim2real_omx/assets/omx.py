"""Reusable Isaac Lab asset configuration for the OMX follower."""

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


REPO_ROOT = Path(__file__).resolve().parents[3]
OMX_FOLLOWER_USD = (
    REPO_ROOT
    / "open_manipulator"
    / "open_manipulator_description"
    / "urdf"
    / "omx_f"
    / "omx_f"
    / "omx_f.usd"
)


OMX_FOLLOWER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(OMX_FOLLOWER_USD),
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=32,
            solver_velocity_iteration_count=1,
            fix_root_link=True,
        ),
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
        # Temporary values. Replace these after A1 physics sanity validation.
        "all_joints": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            effort_limit_sim=1000.0,
            velocity_limit_sim=4.8,
            stiffness=100.0,
            damping=10.0,
        )
    },
)
