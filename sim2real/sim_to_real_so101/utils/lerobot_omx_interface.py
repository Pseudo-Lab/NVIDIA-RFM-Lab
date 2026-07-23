# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#
# LeRobotOmxInterface — SO-101 leader arm 으로 "가상 omx"(Lerobot-Omx-Teleop-Base)를 teleop 하기 위한
# interface. LeRobotSO101Interface 를 그대로 상속하고(= leader 연결/정규화/SO101_USD_MAPPING 재사용),
# "leader 정규화값 -> sim 목표각(rad)" 마지막 단계만 omx 에 맞춰 덮어쓴다.
#
# 배경: omx 와 SO-101 은 하드웨어가 달라 joint 매핑이 100% 일치하지 않는다(그리퍼 range·미러 기구,
#       zero/축 기준 차이). 이번 목표는 정밀 retargeting 이 아니라 "joint limit 이 호환되는 축은 leader
#       를 따라 움직이고, 아닌 축은 중립(0 rad)에서 정지" 하는 1차 teleop 데모다.
#
# 핵심 이점: leader 의 SO101_JOINT_ORDER 와 omx env 의 action 레이아웃(OMX_ARM_JOINTS =
#            [joint1..joint5, gripper_joint_1])이 "역할·순서가 1:1" 이라, 인덱스별로
#            (drive/freeze 게이트 + 방향 sign) 만 곱해주면 그대로 env.step 에 넣을 수 있다.
#
#   leader(SO101_JOINT_ORDER)        omx(OMX_ARM_JOINTS)      drive?
#   -------------------------        -------------------      ------
#   shoulder_pan.pos  (idx 0)   ->   joint1                    ✅
#   shoulder_lift.pos (idx 1)   ->   joint2                    ✅
#   elbow_flex.pos    (idx 2)   ->   joint3                    ✅
#   wrist_flex.pos    (idx 3)   ->   joint4                    ✅
#   wrist_roll.pos    (idx 4)   ->   joint5                    ✅
#   gripper.pos       (idx 5)   ->   gripper_joint_1           ⛔ (freeze @ 0)
import torch

from sim_to_real_so101.utils.lerobot_interface import LeRobotSO101Interface


class LeRobotOmxInterface(LeRobotSO101Interface):
    """SO-101 leader -> 가상 omx teleop interface.

    부모(LeRobotSO101Interface)와 동일하게 leader 를 읽고 SO101_USD_MAPPING 으로 rad 까지 계산한 뒤,
    ① drive mask 로 매칭되지 않는 축(그리퍼)을 0 rad(중립)으로 고정하고,
    ② sign 으로 축 방향(omx zero/축 기준 차이)을 보정한다.
    """

    # 인덱스 = SO101_JOINT_ORDER 순서 = omx OMX_ARM_JOINTS 순서(1:1 대응).
    # 1 = leader 따라 구동, 0 = 중립(0 rad)에서 고정.
    # 2026-07-10: 그리퍼 unfreeze — leader 0~100 -> omx deg -10~100° -> rad 가 omx
    #             gripper_joint_1 limit(0~100°)와 호환(닫힘쪽 -10°만 0 clamp)이라 remap 없이 구동.
    #             gripper_joint_2 는 USD PhysxMimicJointAPI(-1) 로 물리 추종.
    #                 j1 j2 j3 j4 j5 grip
    OMX_DRIVE_MASK = [1, 1, 1, 1, 1, 1]

    # 방향 보정 — leader 하드웨어별로 다르다.
    # so101: Feetech leader 는 joint1(베이스 yaw)이 omx 와 좌우 반전 → -1 (2026-07-01 육안 검증).
    # omx:   실물 OMX-L 은 sim omx_f 와 동일 규약이라 보정 불필요 → 전부 +1 (2026-07-23 육안 검증,
    #        so101 용 -1 이 남아 있으면 오히려 반대로 돈다).
    #                 j1  j2 j3 j4 j5 grip
    # omx j2: 2026-07-24 절대각 모드에서 재검증 — sign -1 이 반대로 확인되어 +1 원복.
    # (어제 "+1 반대" 관찰은 zero 오프셋 문제와 혼동. 절대각+깨끗한 homing frame 기준이 정답.)
    OMX_SIGN_BY_LEADER = {
        "so101": [-1, 1, 1, 1, 1, 1],
        "omx":   [ 1, 1, 1, 1, 1, 1],
    }

    # (참고) 그리퍼 freeze 를 해제해 remap 구동할 때 쓸 omx 그리퍼 가동범위(deg). 이번 범위 밖.
    OMX_GRIPPER_RANGE_DEG = (0.0, 65.0)

    # --- 실물 OMX-L(leader_type="omx") 직결 매핑 상수 ---
    # OmxLeader factory 캘리브는 homing 0 / range 0~4095(=360°) 고정이라:
    #   관절: normalized ±100 = ±180° → deg = n * 1.8. leader 가 omx 그 자체라 sim 관절과 각도 1:1.
    #   그리퍼: 트리거 가동 범위가 개체별로 다르다(이 개체는 ROBOTIS 기본 가정 open=37 과 무관).
    # SO101_USD_MAPPING(SO-101 limit 기준 재스케일)을 거치면 각도가 0.6배 축소 + 그리퍼 중간각
    # 오프셋이 생기므로 omx leader 에선 쓰지 않는다.
    # 2026-07-23 zero-pose homing: 6모터 EEPROM Homing_Offset 을 "sim 초기 자세 = 2048"로 영구
    # 기록(zero_home.py). 따라서 zero 자세가 normalized 0, 그리퍼 꽉 닫힘이 50.0 으로 읽힌다.
    # 주의: EXTENDED_POSITION 모드라 zero 기준 ±180° 밖까지 돌리면 normalized 가 ±100 에
    #       포화된다 → 자세 복귀 후 모터 reboot 으로 multi-turn 리셋 (reboot_motors.py).
    # 그리퍼 방향(2026-07-23 사용자 실기 확정): zero-pose homing 때 "꽉 닫은" 트리거가 2048(n=50)
    # 이므로 n=50 이 닫힘, 트리거를 열면 n 이 커져 55.3 부근이 열림 끝. (ROBOTIS 문서의
    # "놓으면 open" 가정으로 한 번 뒤집었다가 실기에서 완전 반전 확인 → 원위치)
    OMX_LEADER_JOINT_DEG_PER_UNIT = 1.8
    OMX_LEADER_GRIPPER_CLOSED_NORM = 50.0   # 트리거 꽉 닫힘(homing 기준 2048) → sim 0°(닫힘)
    OMX_LEADER_GRIPPER_OPEN_NORM = 55.3     # 트리거 연 끝(실측 arc 5.3) → sim 100°(활짝)
    OMX_LEADER_GRIPPER_OPEN_DEG = 100.0
    # 관절별 고정 오프셋(deg) — j5(wrist roll, USD joint5 axis=X)에 -90°: zero-pose 캡처 때
    # 손잡이 roll 이 sim zero 와 90° 어긋난 것을 보정 (2026-07-23 사용자 지시)
    #                                j1   j2   j3   j4   j5
    OMX_LEADER_JOINT_OFFSET_DEG = [0.0, 0.0, 0.0, 0.0, -90.0]

    def __init__(self, *args, leader_type: str = "omx", **kwargs):
        # leader_type: "omx" = 실물 OMX-L(Dynamixel, OpenRB-150 브리지) / "so101" = SO-101 leader(Feetech).
        # 두 leader 모두 정규화 출력이 동일(관절 -100~100, 그리퍼 0~100)해서 이후 매핑은 공유한다.
        self.leader_type = leader_type
        super().__init__(*args, **kwargs)
        # SO-101 캘리브와 동일 구조(2026-07-24 확정): Z 로 한 번 정렬한 기준(json)을 시작 시 로드해
        # 바로 절대 매핑. json 없으면 첫 실행 부트스트랩(스크립트에서 현재 자세 캡처+저장).
        self._zero_norm = None
        if self.leader_type == "omx":
            self.load_zero_norm()
        self._drive_mask = torch.tensor(
            self.OMX_DRIVE_MASK, dtype=torch.float32, device=self.device
        )
        self._sign = torch.tensor(
            self.OMX_SIGN_BY_LEADER[self.leader_type], dtype=torch.float32, device=self.device
        )

    # --- zero 정렬 캡처 (Z 키) ---
    # "sim 초기 자세에 leader 를 눈으로 맞추고 Z" 로 캡처한 leader normalized 값(6,)을
    # json 에 저장해 관절별 zero 오프셋으로 쓴다. EEPROM homing 의 눈대중 오차를 대체하며,
    # 존재하면 OMX_LEADER_JOINT_OFFSET_DEG(레거시 고정 오프셋)와 그리퍼 CLOSED 상수를 덮어쓴다.
    def _zero_json_path(self):
        import os
        base = os.environ.get("HF_LEROBOT_HOME",
                              os.path.expanduser("~/.cache/huggingface/lerobot"))
        return os.path.join(base, "calibration", "teleoperators", "omx_leader",
                            f"{self.id}_zero.json")

    def load_zero_norm(self):
        import json, os
        p = self._zero_json_path()
        if os.path.exists(p):
            with open(p) as f:
                self._zero_norm = json.load(f)["zero_norm"]
            print(f"[OMX-TELEOP] zero 정렬 로드: {p} -> {['%.2f' % v for v in self._zero_norm]}",
                  flush=True)
        else:
            self._zero_norm = None

    def capture_zero(self, raw_values, save: bool = False):
        """현재 leader normalized (6,) 를 zero 기준으로 캡처.

        teleop 시작 시 자동 호출(메모리만) — "sim 초기 자세 = 시작 순간 leader 자세".
        Z 키 재정렬 시 save=True 로 json 에도 남긴다(기록용).
        """
        import json, os
        self._zero_norm = [float(v) for v in raw_values]
        print(f"[OMX-TELEOP] zero 정렬 캡처 -> {['%.2f' % v for v in self._zero_norm]}", flush=True)
        if save:
            p = self._zero_json_path()
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w") as f:
                json.dump({"zero_norm": self._zero_norm}, f, indent=2)
            print(f"[OMX-TELEOP] zero 정렬 저장: {p}", flush=True)

    def make_cfg(self):
        if self.kind == "leader" and self.leader_type == "omx":
            from lerobot.teleoperators.omx_leader.config_omx_leader import OmxLeaderConfig

            # 캘리브 파일 없으면 factory default 로 자동 캘리브(대화형 프롬프트 없음).
            return OmxLeaderConfig(port=self.port, id=self.id)
        return super().make_cfg()

    def get_mapped_actions_vectorized(self, raw_values):
        if self.leader_type == "omx":
            # 실물 OMX-L: 관절은 각도 1:1 직결, 그리퍼는 실측 트리거 범위 remap (위 상수 주석 참조)
            deg = torch.empty_like(raw_values)
            zn = getattr(self, "_zero_norm", None)
            grip_span = (self.OMX_LEADER_GRIPPER_OPEN_NORM
                         - self.OMX_LEADER_GRIPPER_CLOSED_NORM)
            if zn is not None:
                # Z 캡처 기준(zero) + 고정 오프셋(j5 -90° 손목 방향)은 항상 위에 얹는다 —
                # 자동/수동 재정렬에 흡수되지 않고 유지돼야 하는 의도적 회전.
                z = torch.tensor(zn[:5], dtype=deg.dtype, device=deg.device)
                deg[:-1] = (raw_values[:-1] - z) * self.OMX_LEADER_JOINT_DEG_PER_UNIT + torch.tensor(
                    self.OMX_LEADER_JOINT_OFFSET_DEG, dtype=deg.dtype, device=deg.device
                )
                deg[-1] = ((raw_values[-1] - zn[5]) / grip_span) * self.OMX_LEADER_GRIPPER_OPEN_DEG
            else:
                deg[:-1] = raw_values[:-1] * self.OMX_LEADER_JOINT_DEG_PER_UNIT + torch.tensor(
                    self.OMX_LEADER_JOINT_OFFSET_DEG, dtype=deg.dtype, device=deg.device
                )
                deg[-1] = (
                    (raw_values[-1] - self.OMX_LEADER_GRIPPER_CLOSED_NORM) / grip_span
                ) * self.OMX_LEADER_GRIPPER_OPEN_DEG
            mapped = deg * torch.pi / 180
        else:
            # SO-101 leader: 정규화 -> SO101_USD_MAPPING(deg) -> rad
            mapped = super().get_mapped_actions_vectorized(raw_values)
        # 방향 보정(sign) 후 drive mask 적용 -> freeze 축(mask=0)은 목표각 0 rad(중립)
        return mapped * self._sign * self._drive_mask

    def describe_mapping(self) -> str:
        """시작 로그용 — leader -> omx drive/freeze 표를 문자열로 반환."""
        lines = [
            "leader(SO-101) -> 가상 omx joint 매핑 (drive=leader 추종 / freeze=0 rad 고정)",
            f"{'omx joint':<16}{'<- leader':<20}{'drive?':<10}sign",
            "-" * 54,
        ]
        omx_joints = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper_joint_1"]
        signs = self.OMX_SIGN_BY_LEADER[self.leader_type]
        for i, (omx_j, leader_j) in enumerate(zip(omx_joints, self.SO101_JOINT_ORDER)):
            drive = "DRIVE" if self.OMX_DRIVE_MASK[i] else "FREEZE(0)"
            lines.append(f"{omx_j:<16}{leader_j:<20}{drive:<10}{signs[i]:+d}")
        return "\n".join(lines)
