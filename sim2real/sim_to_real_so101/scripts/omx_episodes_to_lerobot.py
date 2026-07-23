# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# omx_episodes_to_lerobot — scene-direct 녹화물(episode_NNNN/frames PNG + frames.jsonl)을
# 워크샵 표준 LeRobotDataset 으로 변환하는 후처리 스크립트. Isaac 불필요(순수 python).
#
#   ~/IsaacLab/_isaac_sim/python.sh .../omx_episodes_to_lerobot.py \
#       --episodes_root ~/sim2real/datasets/omx_scene_direct \
#       --repo_id pnltoen/omx_pick_place --task_name "Pick toys into tray"
#
# 스키마(워크샵 lerobot_recorder 와 동일):
#   action(6, float32)              = leader normalized (shoulder_pan..gripper)
#   observation.state(6, float32)   = 실제 sim 관절각을 leader normalized 단위로 역변환
#   observation.images.workspace    = viewport PNG (video)
import argparse
import json
import os

import numpy as np

JOINT_FEATURE_NAMES = [
    "shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos",
    "wrist_flex.pos", "wrist_roll.pos", "gripper.pos",
]


def rad_to_leader_norm(joint_pos_rad):
    """sim 관절각(rad, dof 순: joint1..5, gripper_1[, gripper_2]) -> leader normalized (6,).

    lerobot_omx_interface 의 정방향(leader n -> rad) 상수를 그대로 역적용한다.
    """
    from sim_to_real_so101.utils.lerobot_omx_interface import LeRobotOmxInterface as I

    deg = np.degrees(np.asarray(joint_pos_rad[:6], dtype=np.float64))
    out = np.empty(6, dtype=np.float32)
    offs = I.OMX_LEADER_JOINT_OFFSET_DEG
    out[:5] = (deg[:5] - np.asarray(offs)) / I.OMX_LEADER_JOINT_DEG_PER_UNIT
    out[5] = I.OMX_LEADER_GRIPPER_CLOSED_NORM + (
        np.degrees(float(joint_pos_rad[5])) / I.OMX_LEADER_GRIPPER_OPEN_DEG
    ) * (I.OMX_LEADER_GRIPPER_OPEN_NORM - I.OMX_LEADER_GRIPPER_CLOSED_NORM)
    return out


def load_episode(ep_dir):
    with open(os.path.join(ep_dir, "meta.json")) as f:
        meta = json.load(f)
    traj = os.path.join(ep_dir, "trajectory.jsonl")
    if not os.path.exists(traj):
        traj = os.path.join(ep_dir, "frames.jsonl")  # 구버전(2026-07-23) 호환
    frames = []
    with open(traj) as f:
        for line in f:
            frames.append(json.loads(line))
    return meta, frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes_root", type=str, required=True)
    parser.add_argument("--repo_id", type=str, required=True)
    parser.add_argument("--task_name", type=str, required=True)
    parser.add_argument("--dataset_root", type=str, default=None,
                        help="출력 LeRobotDataset 루트 (기본: <episodes_root>_lerobot)")
    parser.add_argument("--camera_name", type=str, default="workspace")
    args = parser.parse_args()

    import imageio.v2 as imageio
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset_root = args.dataset_root or args.episodes_root.rstrip("/") + "_lerobot"

    ep_dirs = sorted(
        os.path.join(args.episodes_root, d) for d in os.listdir(args.episodes_root)
        if d.startswith("episode_") and not d.endswith("__cancelled")
        and os.path.exists(os.path.join(args.episodes_root, d, "meta.json"))
    )
    if not ep_dirs:
        raise SystemExit(f"변환할 에피소드 없음: {args.episodes_root}")

    # fps/해상도는 첫 에피소드 기준 (에피소드 간 상이하면 경고)
    meta0, frames0 = load_episode(ep_dirs[0])
    fps = max(1, round(meta0["fps"]))
    first_png = os.path.join(ep_dirs[0], "frames", "000000.png")
    h, w = imageio.imread(first_png).shape[:2]

    features = {
        "action": {"dtype": "float32", "shape": (6,), "fps": fps, "names": JOINT_FEATURE_NAMES},
        "observation.state": {"dtype": "float32", "shape": (6,), "fps": fps,
                              "names": JOINT_FEATURE_NAMES},
        f"observation.images.{args.camera_name}": {
            "dtype": "video", "fps": fps, "shape": (h, w, 3),
            "names": ["height", "width", "channels"],
        },
    }

    if os.path.exists(dataset_root):
        dataset = LeRobotDataset(args.repo_id, root=dataset_root)
        print(f"[CONV] 기존 dataset 이어쓰기: {dataset_root} (episodes={dataset.meta.total_episodes})")
    else:
        dataset = LeRobotDataset.create(
            args.repo_id, fps=fps, features=features, root=dataset_root, robot_type="omx_follower",
        )
        print(f"[CONV] 새 dataset 생성: {dataset_root} (fps={fps}, {w}x{h})")

    for ep_dir in ep_dirs:
        meta, frames = load_episode(ep_dir)
        if abs(round(meta["fps"]) - fps) > 2:
            print(f"[CONV][WARN] {os.path.basename(ep_dir)} fps={meta['fps']} != dataset fps={fps}")
        for fr in frames:
            png = os.path.join(ep_dir, "frames", f"{fr['frame']:06d}.png")
            img = imageio.imread(png)[..., :3]  # RGBA 방어
            obs = rad_to_leader_norm(fr["joint_pos_rad"])
            dataset.add_frame({
                "action": np.asarray(fr["leader_norm"], dtype=np.float32),
                "observation.state": obs,
                f"observation.images.{args.camera_name}": img,
                "task": args.task_name,
            })
        dataset.save_episode()
        print(f"[CONV] {os.path.basename(ep_dir)} -> episode 저장 ({len(frames)}프레임)")

    print(f"[CONV] 완료 — 총 {dataset.meta.total_episodes} episodes @ {dataset_root}")


if __name__ == "__main__":
    main()
