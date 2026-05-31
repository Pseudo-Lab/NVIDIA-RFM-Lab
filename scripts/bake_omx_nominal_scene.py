"""Bake the nominal OMX Sim2Real scene into a standalone USD layer.

The source Assets/Sim2Real.usd is never modified. This script creates a new
scene layer containing references to the original object scene and OMX-F USD,
then authors the fixed nominal transforms, physics APIs, floor, and lights.

Run:

    cd <YOUR_ISAACLAB_PATH>
    conda activate <YOUR_CONDA_ENV>
    source _isaac_sim/setup_conda_env.sh
    python <YOUR_REPO_PATH>/scripts/bake_omx_nominal_scene.py
"""

import argparse
import os
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


REPO_ROOT = Path(__file__).resolve().parents[1]


parser = argparse.ArgumentParser(description="Bake a reusable nominal OMX Sim2Real scene USD.")
parser.add_argument(
    "--scene_usd",
    default=str(REPO_ROOT / "Assets" / "Sim2Real.usd"),
    help="Source object scene USD. This file is not modified.",
)
parser.add_argument(
    "--robot_usd",
    default=str(REPO_ROOT / "open_manipulator" / "open_manipulator_description" / "urdf" / "omx_f" / "omx_f" / "omx_f.usd"),
    help="OMX-F USD referenced by the baked nominal scene.",
)
parser.add_argument(
    "--output_usd",
    default=str(REPO_ROOT / "Assets" / "scenes" / "Sim2Real_OMX_nominal.usd"),
    help="Output USD path.",
)
parser.add_argument("--overwrite", action="store_true", help="Replace output_usd if it already exists.")
args_cli = parser.parse_args()


OBJECT_SPECS = {
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


def iter_mesh_prims(prim):
    for child in Usd.PrimRange(prim):
        if child.GetTypeName() == "Mesh":
            yield child


def configure_object_physics(prim, mass_kg: float):
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass_kg)
    for mesh_prim in iter_mesh_prims(prim):
        UsdPhysics.CollisionAPI.Apply(mesh_prim)
        UsdPhysics.MeshCollisionAPI.Apply(mesh_prim).CreateApproximationAttr("convexHull")


def scale_and_place_objects(stage):
    bbox_cache = UsdGeom.BBoxCache(0.0, [UsdGeom.Tokens.default_])
    for path, spec in OBJECT_SPECS.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Missing object prim: {path}")

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
        print(f"[BAKE] placed {path}: scale={uniform_scale:.8f}, translate={translate}", flush=True)


def add_floor(stage):
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.005))
    floor.AddScaleOp().Set(Gf.Vec3f(0.8, 0.8, 0.01))
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())

    material = UsdShade.Material.Define(stage, "/World/Looks/FloorMaterial")
    shader = UsdShade.Shader.Define(stage, "/World/Looks/FloorMaterial/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.015, 0.015, 0.015))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material)

    physics_material = UsdShade.Material.Define(stage, "/World/Looks/FloorPhysicsMaterial")
    physics_api = UsdPhysics.MaterialAPI.Apply(physics_material.GetPrim())
    physics_api.CreateStaticFrictionAttr(1.0)
    physics_api.CreateDynamicFrictionAttr(0.8)
    physics_api.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
        physics_material, materialPurpose="physics"
    )


def add_lights(stage):
    ambient = UsdLux.DomeLight.Define(stage, "/World/SoftAmbient")
    ambient.CreateIntensityAttr(120.0)
    ambient.CreateColorAttr(Gf.Vec3f(0.85, 0.82, 0.78))

    key = UsdLux.DistantLight.Define(stage, "/World/WarmKeyLight")
    key.CreateIntensityAttr(450.0)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.88, 0.72))
    key.CreateAngleAttr(8.0)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))


def make_relative_asset_path(asset_path: str, output_path: Path) -> str:
    return Path(os.path.relpath(Path(asset_path).resolve(), output_path.resolve().parent)).as_posix()


def add_robot_reference(stage, robot_usd: str):
    robot = stage.DefinePrim("/World/OMX", "Xform")
    robot.GetReferences().AddReference(robot_usd)


def bake():
    output_path = Path(args_cli.output_usd)
    if output_path.exists() and not args_cli.overwrite:
        raise FileExistsError(f"Output already exists: {output_path}. Use --overwrite to replace it.")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    scene_usd = make_relative_asset_path(args_cli.scene_usd, output_path)
    robot_usd = make_relative_asset_path(args_cli.robot_usd, output_path)
    stage.GetRootLayer().subLayerPaths.append(scene_usd)
    scale_and_place_objects(stage)
    add_floor(stage)
    add_lights(stage)
    add_robot_reference(stage, robot_usd)

    stage.GetRootLayer().Save()
    print(f"[BAKE] wrote nominal scene: {output_path}", flush=True)
    print("[BAKE] source scene was not modified.", flush=True)


if __name__ == "__main__":
    bake()
