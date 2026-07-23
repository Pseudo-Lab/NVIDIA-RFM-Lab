# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# omx teleop — "기존 Sim2Real.usd 를 손 안 대고 그대로 열고" 그 안의 /omx_f 를 SO-101 leader 로 teleop.
#
#   Isaac Lab manager-env(별도 /Robot 스폰 + 0.01 백드롭 스케일)를 쓰지 않는다. Sim2Real.usd 를
#   THE stage 로 열기 때문에 tray/오브젝트/plane/카메라/조명/로봇 배치가 "저작 그대로"(cm 단위) 나온다.
#   → 스케일 레이어링 없음 = 저작 화면과 1:1.
#
#   제어: leader(SO-101) 관절값 -> LeRobotOmxInterface 로 rad 매핑(sign/drive mask 그대로 재사용)
#         -> 씬 안 /omx_f 의 joint1~5 + gripper_joint_1 위치목표. gripper_joint_2 는 -1 미러(기어 종동).
#
#   실행:
#     ./isaaclab.sh -p .../scripts/omx_teleop_scene_direct.py \
#         --port /dev/ttyACM0 --robot_id my_awesome_leader_arm
import argparse
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Sim2Real.usd 원본을 열고 /omx_f 를 leader 로 teleop.")
parser.add_argument("--usd", type=str,
                    default=os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                         "../../../Assets/Sim2Real.usd")),
                    help="열 씬 USD (기본 = repo 의 Assets/Sim2Real.usd).")
parser.add_argument("--robot_prim", type=str, default="/omx_f", help="teleop 대상 articulation prim.")
parser.add_argument("--camera_prim", type=str, default="/workspace_cam", help="뷰포트로 쓸 씬 카메라.")
parser.add_argument("--port", type=str, default="/dev/ttyACM0", help="leader 포트.")
parser.add_argument("--robot_id", type=str, default="my_awesome_leader_arm", help="leader id(캘리브 파일명).")
parser.add_argument("--leader_type", type=str, default="omx", choices=["omx", "so101"],
                    help="leader 하드웨어: omx=실물 OMX-L(Dynamixel), so101=SO-101 leader(Feetech).")
parser.add_argument("--record_root", type=str,
                    default=os.path.expanduser("~/sim2real/datasets/omx_scene_direct"),
                    help="녹화 에피소드(frames PNG + jsonl) 저장 루트. S=시작/저장, C=취소.")
parser.add_argument("--capture_every", type=int, default=2,
                    help="sim 루프 n회당 1프레임 캡처 (렌더 ~60Hz 기준 2=~30fps).")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- 이하 SimulationApp 기동 후 import ---
import numpy as np
import torch

import omni.usd
from pxr import UsdPhysics, UsdGeom, Gf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

from sim_to_real_so101.utils.lerobot_omx_interface import LeRobotOmxInterface

# 씬 = cm 저작(metersPerUnit=0.01). Isaac 에 단위를 알려 물리/카메라가 저작대로 동작하게 한다.
STAGE_UNITS_M = 0.01
# 구동 joint 순서 = mapped_action(6) 순서와 1:1
DRIVEN_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint_1"]
MIRROR_SRC = "gripper_joint_1"   # 모터 구동 핑거
MIRROR_DST = "gripper_joint_2"   # 기어 종동 핑거(-1 미러)
MIRROR_MUL = -1.0


def main():
    # 1) 원본 씬을 THE stage 로 연다 (씬 지오메트리/배치/카메라는 손대지 않음)
    print(f"[SCENE] open stage: {args_cli.usd}", flush=True)
    open_stage(args_cli.usd)

    # 1-b) 그리퍼 종동 핑거(gripper_joint_2) 준비 — "이전 teleop 방식" 재현.
    #   raw Isaac Sim 은 USD PhysxMimicJoint 를 물리로 살려버려 명시 -1 미러와 충돌한다.
    #   → mimic 속성을 제거하고 DriveAPI(위치제어)를 걸어, 루프에서 joint_2 = -1 x joint_1 로
    #     직접 구동한다(Isaac Lab 의 MirroredGripperJointPositionAction 와 동일, g2/g1=-1.00 검증분).
    stage = omni.usd.get_context().get_stage()
    j2 = stage.GetPrimAtPath("/omx_f/joints/gripper_joint_2")
    if j2 and j2.IsValid():
        removed = 0
        for a in list(j2.GetAttributes()):
            if a.GetName().startswith("physxMimicJoint:"):
                j2.RemoveProperty(a.GetName()); removed += 1
        for r in list(j2.GetRelationships()):
            if r.GetName().startswith("physxMimicJoint:"):
                j2.RemoveProperty(r.GetName()); removed += 1
        # (mimic 파라미터 속성/관계를 지웠으므로 PhysX 는 mimic 을 구성하지 않는다 —
        #  gripper_joint_2 는 이제 일반 구동 joint 로 동작)
        drive = UsdPhysics.DriveAPI.Apply(j2, "angular")
        drive.CreateStiffnessAttr(17.8)
        drive.CreateDampingAttr(0.6)
        drive.CreateMaxForceAttr(1000.0)
        drive.CreateTypeAttr("force")
        print(f"[SCENE] gripper_joint_2: mimic 제거({removed}) + DriveAPI(angular) 적용", flush=True)
    else:
        print("[SCENE][WARN] gripper_joint_2 prim 못 찾음", flush=True)

    # 1-c) 집기대상(bear/chick/fish)을 현재의 70%로 축소 — 그리퍼 폭에 맞춰 집히도록.
    #   기존 xform 뒤에 named scale op 를 더해 위치는 유지, geometry/collision 만 0.7배.
    #   (원본 USD 는 안 건드리는 런타임 override — 값 확정되면 영구 반영)
    #   실측 반영: 가로·높이는 0.7 유지, "두께"만 실측 고정(fish 4 / bear 3.5 / chick 5 cm).
    #   toy 스택이 [..., scale, rotX90] 이라 shrink(a,b,c) -> world (X,Y,Z) = (a,c,b) 매핑
    #   (rotX90 이 로컬 Y<->Z 교환). 두께축: bear/fish = world Z(원본 4.008/4.714cm) -> b 성분,
    #   chick = world Y(원본 5.702cm) -> c 성분.
    TOY_SHRINK = {
        "/bear_v2_texture":  Gf.Vec3f(0.7, 3.5 / 4.008, 0.7),   # 두께 world Z = 3.5cm
        "/chick_v2_texture": Gf.Vec3f(0.7, 0.7, 5.0 / 5.702),   # 두께 world Y = 5.0cm
        "/fish_v2_texture":  Gf.Vec3f(0.7, 4.0 / 4.714, 0.7),   # 두께 world Z = 4.0cm
    }
    #   (idempotent: USD 에 shrink op 가 이미 저장돼 있으면 재사용해 절대값 Set — 중복 Add 크래시 방지)
    for toy, shrink in TOY_SHRINK.items():
        tp = stage.GetPrimAtPath(toy)
        if tp and tp.IsValid():
            xf = UsdGeom.Xformable(tp)
            op = next((o for o in xf.GetOrderedXformOps()
                       if o.GetOpName() == "xformOp:scale:shrink"), None)
            if op is None:
                op = xf.AddScaleOp(UsdGeom.XformOp.PrecisionFloat, "shrink")
            op.Set(shrink)
            print(f"[SCENE] {toy} shrink={tuple(shrink)} (가로·높이 0.7, 두께 실측)", flush=True)
        else:
            print(f"[SCENE][WARN] {toy} prim 못 찾음", flush=True)

    # 1-d) 지정 오브젝트를 부모좌표 Z축 +5 이동 (tray/카메라/라이트 제외).
    #   기존 xform op 앞에 named translate op 를 끼워 op 종류(matrix 등)와 무관하게 Z+5 를 준다.
    Z_SHIFT = Gf.Vec3d(0.0, 0.0, 5.0)
    for name in ("/chick_v2_texture", "/bear_v2_texture", "/fish_v2_texture",
                 "/Plane_02", "/omx_f", "/omx_l"):
        p = stage.GetPrimAtPath(name)
        if p and p.IsValid():
            xf = UsdGeom.Xformable(p)
            # idempotent: 이미 zshift 가 있으면(USD 에 저장된 경우) 값만 절대 Set — 이중 +5 방지
            shift = next((o for o in xf.GetOrderedXformOps()
                          if o.GetOpName() == "xformOp:translate:zshift"), None)
            if shift is None:
                existing = xf.GetOrderedXformOps()
                shift = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble, "zshift")
                xf.SetXformOpOrder([shift] + existing)  # 맨 앞 = 부모좌표계 이동
            shift.Set(Z_SHIFT)
            print(f"[SCENE] {name} translate Z+5", flush=True)
        else:
            print(f"[SCENE][WARN] {name} prim 못 찾음", flush=True)

    # 1-e) tray: 실측 footprint(24 x 9.5cm) scale 은 2026-07-12 에 Sim2Real.usd 에 영구 저장됨.
    #   이후 배치/위치 튜닝은 GUI 에서 하고 Ctrl+S 로 저장하는 워크플로우라, 여기서 강제하지 않는다
    #   (강제 Set 이 사용자의 GUI 조정을 되돌리는 충돌이 있었음).
    tray = stage.GetPrimAtPath("/tray")
    if tray and tray.IsValid():
        print(f"[SCENE] /tray scale={tuple(tray.GetAttribute('xformOp:scale').Get())} (USD 저장값 사용)",
              flush=True)

    # 2) 물리 컨텍스트 — 씬의 /physicsScene 을 그대로 쓰고 cm 단위 지정
    sim = SimulationContext(
        physics_dt=1.0 / 120.0,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=STAGE_UNITS_M,
    )

    # 3) 씬 안 /omx_f 를 articulation 으로 래핑
    robot = SingleArticulation(prim_path=args_cli.robot_prim, name="omx_f")

    # 4) 뷰포트 카메라 = 씬의 workspace_cam (perspective 대신 저작 뷰각)
    viewport_api = None
    try:
        from omni.kit.viewport.utility import get_active_viewport
        vp = get_active_viewport()
        if vp is not None:
            vp.camera_path = args_cli.camera_prim
            viewport_api = vp
            print(f"[SCENE] viewport -> {args_cli.camera_prim}", flush=True)
    except Exception as e:
        print(f"[SCENE] viewport 설정 실패: {e}", flush=True)

    # 5) 물리 초기화 (articulation 핸들 확보)
    sim.reset()
    robot.initialize()
    dof_names = list(robot.dof_names)
    print(f"[SCENE] /omx_f dof_names = {dof_names}", flush=True)

    # joint 이름 -> dof index
    def idx(name):
        return dof_names.index(name) if name in dof_names else None
    driven_idx = [idx(j) for j in DRIVEN_JOINTS]
    mirror_dst_idx = idx(MIRROR_DST)
    if any(i is None for i in driven_idx):
        print(f"[SCENE][WARN] 일부 구동 joint 를 못 찾음: {list(zip(DRIVEN_JOINTS, driven_idx))}", flush=True)

    # 6) leader 연결 (매핑 로직 재사용)
    robot_iface = LeRobotOmxInterface(
        device="cpu", port=args_cli.port, id=args_cli.robot_id,
        cameras={}, fps=30, kind="leader", leader_type=args_cli.leader_type,
    )
    print("[OMX-TELEOP] " + robot_iface.describe_mapping().replace("\n", "\n[OMX-TELEOP] "), flush=True)
    robot_iface.init_device()
    robot_iface.connect()
    print(f"[SCENE] leader 연결 완료: {args_cli.port} ({args_cli.robot_id})", flush=True)

    # 6-b) 키보드 컨트롤 (워크샵 keyboard.py 재사용) — R = reset, S = 녹화 시작/종료, C = 취소.
    #   UI text field 입력 중에는 S/C/R 가 오작동하지 않도록 차단(GuardedKeyboard).
    from sim_to_real_so101.utils.keyboard import KeyboardControl

    class GuardedKeyboard(KeyboardControl):
        def __init__(self, is_blocked):
            self._is_blocked = is_blocked
            self.zero_align = False   # Z = leader 현재 자세를 zero 로 재정렬
            self.screenshot = False   # P = 뷰포트 스크린샷(바탕화면 저장)
            self.save_layout = False  # L = 현재 UI 레이아웃 저장(다음 실행 시 자동 복원)
            super().__init__()

        def _on_keyboard_event(self, event, *args, **kwargs):
            if self._is_blocked():
                return False
            import carb
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                if event.input.name == "Z":
                    self.zero_align = True
                    return True
                if event.input.name == "P":
                    self.screenshot = True
                    return True
                if event.input.name == "L":
                    self.save_layout = True
                    return True
            return super()._on_keyboard_event(event, *args, **kwargs)

    # UI 레이아웃 저장/복원 — "켰을 때 바로 그 상태" (L 키로 저장, 시작 시 자동 복원)
    import json as _json
    LAYOUT_PATH = os.path.expanduser("~/sim2real/datasets/omx_layout.json")

    def save_layout():
        try:
            import omni.ui as _ui
            with open(LAYOUT_PATH, "w") as f:
                _json.dump(_ui.Workspace.dump_workspace(), f)
            print(f"[UI] 레이아웃 저장: {LAYOUT_PATH}", flush=True)
        except Exception as e:
            print(f"[UI] 레이아웃 저장 실패: {e}", flush=True)

    def restore_layout():
        if not os.path.exists(LAYOUT_PATH):
            return
        try:
            import omni.ui as _ui
            with open(LAYOUT_PATH) as f:
                _ui.Workspace.restore_workspace(_json.load(f))
            print(f"[UI] 레이아웃 복원: {LAYOUT_PATH}", flush=True)
        except Exception as e:
            print(f"[UI] 레이아웃 복원 실패: {e}", flush=True)

    # 스크린샷: 뷰포트 렌더(HUD/UI 미포함)를 바탕화면에 저장 — 우분투/윈도우 공통 ~/Desktop
    def take_screenshot():
        import datetime
        if viewport_api is None:
            print("[SCENE] viewport 없음 — 스크린샷 불가", flush=True)
            return
        from omni.kit.viewport.utility import capture_viewport_to_file
        path = os.path.join(os.path.expanduser("~/Desktop"),
                            f"omx_{datetime.datetime.now():%Y%m%d_%H%M%S}.png")
        capture_viewport_to_file(viewport_api, path)
        print(f"[SCENE] 스크린샷 저장: {path}", flush=True)

    # 6-c) 레코더 — viewport 캡처 + trajectory.jsonl 덤프 (LeRobotDataset 변환은 후처리 스크립트)
    from sim_to_real_so101.utils.scene_direct_recorder import SceneDirectRecorder
    recorder = None
    if viewport_api is not None:
        recorder = SceneDirectRecorder(
            root=args_cli.record_root, viewport_api=viewport_api,
            joint_names=dof_names, capture_every=args_cli.capture_every,
        )
        print(f"[SCENE] 레코더 준비: {args_cli.record_root} (다음 episode_{recorder.episode_idx:04d})",
              flush=True)
    else:
        print("[SCENE][WARN] viewport 없음 — 녹화 비활성", flush=True)

    # 6-d) 리플레이 + 상태/녹화/리플레이 UI 패널
    from sim_to_real_so101.utils.episode_replay import EpisodeReplay
    from sim_to_real_so101.utils.teleop_status_ui import TeleopStatusUI
    replay = EpisodeReplay()
    status_ui = (TeleopStatusUI(recorder=recorder, replay=replay, on_screenshot=take_screenshot)
                 if recorder is not None else None)
    keyboard = GuardedKeyboard(is_blocked=lambda: status_ui.editing if status_ui else False)
    print("[SCENE] 키보드: R=reset · S=녹화 · C=취소 · Z=zero · P=스크린샷 · L=레이아웃 저장", flush=True)
    restore_layout()  # 저장된 UI 배치가 있으면 그 상태로 시작
    # zero 기준: 저장된 json(Z 로 정렬한 것)을 로드해 시작 즉시 절대 매핑 — SO-101 캘리브와 동일 흐름.
    # json 이 없을 때만(최초 실행) 현재 자세를 부트스트랩 캡처+저장한다. Z = 재캘리브.
    if args_cli.leader_type == "omx" and robot_iface._zero_norm is None:
        with torch.inference_mode():
            _first = robot_iface.robot.get_action()
            robot_iface.capture_zero(robot_iface.get_raw_actions_tensor(_first), save=True)
        print("[SCENE] zero json 없음 — 현재 자세로 부트스트랩 저장 (필요 시 Z 로 재정렬)", flush=True)

    # 7) teleop 루프
    import time as _time
    fps_ema = 0.0
    t_prev = _time.perf_counter()
    while simulation_app.is_running():
        t_now = _time.perf_counter()
        dt = max(t_now - t_prev, 1e-6)
        t_prev = t_now
        fps_ema = 0.9 * fps_ema + 0.1 * (1.0 / dt) if fps_ema else 1.0 / dt

        if keyboard.reset_world:
            keyboard.reset_world = False
            print("[SCENE] world reset (R)", flush=True)
            # stop→play 로 모든 rigid body 를 stage 저작 상태(런타임 override 포함)로 복원.
            # articulation 핸들은 무효화되므로 다시 잡는다 (dof 순서는 불변 → driven_idx 재사용).
            replay.stop()
            sim.reset()
            robot.initialize()

        # 리플레이 재생 요청 → world reset 후 시작 (녹화 시작 상태와 배치를 맞춤)
        if replay.pending_start is not None:
            sim.reset()
            robot.initialize()
            replay.begin()

        with torch.inference_mode():
            real_action = robot_iface.robot.get_action()
            _raw, mapped = robot_iface.real_to_sim_obs_processor(real_action)  # (6,) rad
            if keyboard.zero_align:
                keyboard.zero_align = False
                robot_iface.capture_zero(_raw, save=True)
                _raw, mapped = robot_iface.real_to_sim_obs_processor(real_action)  # 재계산
            if keyboard.screenshot:
                keyboard.screenshot = False
                take_screenshot()
            if keyboard.save_layout:
                keyboard.save_layout = False
                save_layout()
            if replay.active:
                ra = replay.current_action()
                if ra is not None:
                    mapped_np = np.asarray(ra, dtype=np.float32)
                else:  # 재생 끝 → leader 로 복귀
                    mapped_np = mapped.detach().cpu().numpy().astype(np.float32)
            else:
                mapped_np = mapped.detach().cpu().numpy().astype(np.float32)

            # 구동 joint 목표각 — joint1~5 + gripper_joint_1 (mapped 6개)
            indices = [i for i in driven_idx if i is not None]
            values = [v for v, i in zip(mapped_np, driven_idx) if i is not None]

            # gripper_joint_2 = -1 x gripper_joint_1 명시 미러(기어 종동). 두 핑거가 정반대로 벌어진다.
            #   joint_2 의 물리 limit(-120~20°)이 joint_1(0~100°)보다 훨씬 loose 해서, clamp 없이
            #   -g1 을 그대로 주면 joint_1 이 상한 100°에서 멈춰도 joint_2 는 안 멈추고 대칭이 깨진다.
            #   → g1 을 joint_1 유효범위(0~100°)로 먼저 clip 해서 두 핑거가 대칭으로 정지하게 한다.
            if mirror_dst_idx is not None:
                g1 = float(np.clip(mapped_np[DRIVEN_JOINTS.index(MIRROR_SRC)],
                                   0.0, np.deg2rad(100.0)))
                indices.append(mirror_dst_idx)
                values.append(MIRROR_MUL * g1)

            robot.apply_action(
                ArticulationAction(
                    joint_positions=np.array(values, dtype=np.float32),
                    joint_indices=np.array(indices, dtype=np.int32),
                )
            )

            # 녹화 + 상태 UI (leader 정규화값 / sim 목표각 / 실제 관절각)
            # reset 직후 등 핸들 재초기화 틱에는 None 이 올 수 있음 — 해당 프레임 스킵 (크래시 방지)
            joint_pos = robot.get_joint_positions()
            if joint_pos is not None:
                if recorder is not None and not replay.active:
                    recorder.on_frame(_raw, mapped_np[:6], joint_pos)
                if status_ui is not None:
                    actual6 = [joint_pos[i] if i is not None else 0.0 for i in driven_idx]
                    status_ui.update(_raw, mapped_np[:6], actual6,
                                     fps=fps_ema,
                                     leader_connected=bool(getattr(robot_iface.robot, "is_connected", True)))

        sim.step(render=True)

    if recorder is not None:
        recorder.cleanup()
    robot_iface.robot.disconnect()
    simulation_app.close()


if __name__ == "__main__":
    main()
