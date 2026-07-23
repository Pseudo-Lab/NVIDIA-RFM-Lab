# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# SceneDirectRecorder — scene-direct teleop 용 경량 레코더.
#
# 설계(2026-07-23 합의): Isaac 안에서는 "픽셀 꺼내기 + 파일 덤프"까지만 하고,
# LeRobotDataset 조립은 후처리 스크립트(omx_episodes_to_lerobot.py)에서 한다.
#   - 프레임: viewport 캡처(PNG, capture_viewport_to_file — viewport 가 이미 물고 있는
#             render product 재사용, 반환이 future 라 파일 쓰기는 비동기)
#   - 상태: frames.jsonl 에 프레임별 {t, leader normalized, sim 목표각(rad), 실제 joint(rad)}
#   - 키: 워크샵 keyboard.py 의 kit 이벤트(S=start/stop→저장, C=cancel→폐기, R=stop→저장+리셋) 수신
#
# 에피소드 디렉터리 구조:
#   <root>/episode_NNNN/frames/000000.png ...
#   <root>/episode_NNNN/trajectory.jsonl
#   <root>/episode_NNNN/meta.json          (저장 확정 시 기록 — fps·프레임 수·녹화자·메모)
#   취소된 에피소드는 episode_NNNN__cancelled 로 rename (삭제하지 않음)
#
# 저장 흐름(2026-07-24): S(stop) 은 수집만 멈추고 'pending' 상태 → UI 다이얼로그에서
# 이름/메모 입력 후 commit(save=True/False) 으로 확정한다 (on_pending 콜백으로 UI 오픈).
import json
import os
import time

import omni.kit.app
from carb.eventdispatcher import get_eventdispatcher

from sim_to_real_so101.utils.keyboard import KeyboardControl


class SceneDirectRecorder:

    def __init__(self, root: str, viewport_api, joint_names, capture_every: int = 2):
        self.root = root
        self.viewport_api = viewport_api
        self.joint_names = list(joint_names)
        self.capture_every = max(1, capture_every)  # sim 루프 n회당 1프레임 (렌더 ~60Hz → 기본 30fps)

        os.makedirs(self.root, exist_ok=True)
        self.episode_idx = self._next_episode_idx()

        self.active = False
        self.pending = False       # stop 후 저장 확정 대기(UI 다이얼로그)
        self.on_pending = None     # 콜백: stop 시 UI 가 저장 다이얼로그를 열도록
        self.operator = ""         # 녹화자 이름 (UI 필드에서 갱신, REC 표시용)
        self.ep_dir = None
        self.jsonl = None
        self.frame_idx = 0
        self.loop_tick = 0
        self.t_start = None
        self._duration = 0.0

        ed = get_eventdispatcher()
        self._subs = [
            ed.observe_event(observer_name="sdr_start",
                             event_name=KeyboardControl.START_RECORDING_EVENT,
                             on_event=self._on_start),
            ed.observe_event(observer_name="sdr_stop",
                             event_name=KeyboardControl.STOP_RECORDING_EVENT,
                             on_event=self._on_stop),
            ed.observe_event(observer_name="sdr_cancel",
                             event_name=KeyboardControl.CANCEL_RECORDING_EVENT,
                             on_event=self._on_cancel),
        ]

    def _next_episode_idx(self):
        idx = 0
        for name in os.listdir(self.root):
            if name.startswith("episode_"):
                try:
                    idx = max(idx, int(name.split("_")[1]) + 1)
                except (IndexError, ValueError):
                    pass
        return idx

    # --- 키 이벤트 핸들러 ---
    def _on_start(self, event):
        if self.active:
            return
        if self.pending:
            print("[REC] 이전 에피소드 저장 확정 대기 중 — 다이얼로그를 먼저 처리", flush=True)
            return
        self.ep_dir = os.path.join(self.root, f"episode_{self.episode_idx:04d}")
        os.makedirs(os.path.join(self.ep_dir, "frames"), exist_ok=True)
        self.jsonl = open(os.path.join(self.ep_dir, "trajectory.jsonl"), "w")
        self.frame_idx = 0
        self.loop_tick = 0
        self.t_start = time.time()
        self.active = True
        print(f"[REC] episode_{self.episode_idx:04d} 녹화 시작 (by {self.operator or '?'})", flush=True)

    def _on_stop(self, event):
        if not self.active:
            return
        # 수집만 멈추고 저장 확정은 UI 다이얼로그(commit)로
        self.active = False
        self.pending = True
        self._duration = max(time.time() - self.t_start, 1e-6)
        self.jsonl.flush()
        print(f"[REC] episode_{self.episode_idx:04d} 수집 종료 — 저장 다이얼로그 대기", flush=True)
        if self.on_pending is not None:
            self.on_pending()

    def _on_cancel(self, event):
        if not self.active:
            return
        self.active = False
        self._duration = max(time.time() - self.t_start, 1e-6)
        self._discard()

    def commit(self, save: bool, operator: str = "", note: str = ""):
        """저장 다이얼로그 확정. save=False 면 폐기."""
        if not self.pending:
            return
        self.pending = False
        if save:
            self._save(operator=operator, note=note)
        else:
            self._discard()

    def _save(self, operator: str, note: str):
        self.jsonl.close()
        meta = {
            "episode": self.episode_idx,
            "recorded_by": operator,
            "note": note,
            "num_frames": self.frame_idx,
            "duration_s": self._duration,
            "fps": round(self.frame_idx / self._duration, 2),
            "joint_names": self.joint_names,
            "camera": "workspace_cam(viewport)",
        }
        with open(os.path.join(self.ep_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        print(f"[REC] episode_{self.episode_idx:04d} 저장 — {self.frame_idx}프레임, "
              f"{self._duration:.1f}s (~{meta['fps']}fps), by {operator or '?'}", flush=True)
        self.episode_idx += 1
        self.ep_dir = None
        self.jsonl = None

    def _discard(self):
        self.jsonl.close()
        cancelled = self.ep_dir + "__cancelled"
        os.rename(self.ep_dir, cancelled)
        print(f"[REC] episode 취소 → {os.path.basename(cancelled)} (삭제 안 함)", flush=True)
        self.ep_dir = None
        self.jsonl = None

    def list_saved_episodes(self):
        """저장 확정된(meta.json 있는) 에피소드 디렉터리 목록 (이름·녹화자 포함)."""
        out = []
        for name in sorted(os.listdir(self.root)):
            d = os.path.join(self.root, name)
            mp = os.path.join(d, "meta.json")
            if name.startswith("episode_") and not name.endswith("__cancelled") and os.path.exists(mp):
                try:
                    with open(mp) as f:
                        m = json.load(f)
                    out.append((d, f"{name} | {m.get('recorded_by') or '?'} | {m.get('num_frames', 0)}f"))
                except (json.JSONDecodeError, OSError):
                    out.append((d, name))
        return out

    # --- teleop 루프에서 매 iteration 호출 ---
    def on_frame(self, leader_norm, action_rad, joint_pos_rad):
        if not self.active:
            return
        self.loop_tick += 1
        if (self.loop_tick - 1) % self.capture_every != 0:
            return

        from omni.kit.viewport.utility import capture_viewport_to_file
        png = os.path.join(self.ep_dir, "frames", f"{self.frame_idx:06d}.png")
        capture_viewport_to_file(self.viewport_api, png)  # future 반환 — 비동기 저장

        self.jsonl.write(json.dumps({
            "frame": self.frame_idx,
            "t": time.time() - self.t_start,
            "leader_norm": [round(float(v), 4) for v in leader_norm],
            "action_rad": [round(float(v), 6) for v in action_rad],
            "joint_pos_rad": [round(float(v), 6) for v in joint_pos_rad],
        }) + "\n")
        self.frame_idx += 1

    @property
    def status_text(self):
        if self.active:
            who = f" | {self.operator}" if self.operator else ""
            return f"REC * episode_{self.episode_idx:04d}{who} | frame {self.frame_idx}"
        if self.pending:
            return f"PENDING episode_{self.episode_idx:04d} — enter name & save"
        return f"idle (next: episode_{self.episode_idx:04d})"

    def cleanup(self):
        if self.active:  # 앱 종료 시 미확정분은 저장으로 처리
            self._on_stop(None)
        if self.pending:
            self.commit(save=True, operator=self.operator, note="(자동 저장: 앱 종료)")
        self._subs.clear()
