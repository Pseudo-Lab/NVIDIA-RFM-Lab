# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# EpisodeReplay — scene-direct 녹화물(trajectory.jsonl)을 같은 씬에서 시간 기반 재생.
# 별도 리플레이 씬 없이 teleop 루프가 leader 대신 이 컨트롤러의 action 을 적용한다.
import json
import os
import time


class EpisodeReplay:

    def __init__(self):
        self.active = False
        self.pending_start = None  # 재생 요청된 에피소드 경로 (루프에서 reset 후 begin)
        self._frames = []
        self._t0 = None
        self._idx = 0
        self.episode_name = ""

    @staticmethod
    def trajectory_path(ep_dir):
        p = os.path.join(ep_dir, "trajectory.jsonl")
        if os.path.exists(p):
            return p
        legacy = os.path.join(ep_dir, "frames.jsonl")  # 구버전 호환
        return legacy if os.path.exists(legacy) else None

    def request(self, ep_dir):
        """UI 재생 버튼 → 루프가 world reset 후 begin() 하도록 예약."""
        if self.trajectory_path(ep_dir) is None:
            print(f"[REPLAY] trajectory 없음: {ep_dir}", flush=True)
            return False
        self.pending_start = ep_dir
        return True

    def begin(self):
        ep_dir = self.pending_start
        self.pending_start = None
        self._frames = []
        with open(self.trajectory_path(ep_dir)) as f:
            for line in f:
                fr = json.loads(line)
                self._frames.append((float(fr["t"]), fr["action_rad"]))
        self._idx = 0
        self._t0 = time.time()
        self.active = bool(self._frames)
        self.episode_name = os.path.basename(ep_dir)
        print(f"[REPLAY] {self.episode_name} 재생 시작 ({len(self._frames)}프레임)", flush=True)

    def current_action(self):
        """경과 시간에 해당하는 action(rad, 6). 끝나면 None."""
        if not self.active:
            return None
        elapsed = time.time() - self._t0
        while self._idx + 1 < len(self._frames) and self._frames[self._idx + 1][0] <= elapsed:
            self._idx += 1
        if elapsed > self._frames[-1][0] + 0.5:  # 마지막 프레임 후 0.5s 유지 뒤 종료
            self.stop()
            return None
        return self._frames[self._idx][1]

    def stop(self):
        if self.active:
            print(f"[REPLAY] {self.episode_name} 재생 종료", flush=True)
        self.active = False
        self.pending_start = None

    @property
    def status_text(self):
        return f"REPLAY > {self.episode_name}" if self.active else ""
