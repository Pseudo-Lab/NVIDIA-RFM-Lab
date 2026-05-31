# A2 OMX Leader USD / Mapping 검증

최종 갱신일: 2026-05-31

## 목표

OMX leader USD를 확인하고 teleoperation 및 real-data replay에 필요한
leader-to-follower joint mapping을 작성한다.

## 기준 USD

Leader:

```text
open_manipulator/open_manipulator_description/urdf/omx_l/omx_l/omx_l.usd
```

Follower:

```text
open_manipulator/open_manipulator_description/urdf/omx_f/omx_f/omx_f.usd
```

두 USD 모두 meter 단위를 사용하며 default prim을 가진다.

## 현재까지 진행한 내용

초기 USD 조사를 수행했다. Leader USD에는 다음 joint가 있다.

```text
joint1
joint2
joint3
joint4
joint5
gripper_joint_1
```

Follower USD에는 다음 joint가 있다.

```text
joint1
joint2
joint3
joint4
joint5
gripper_joint_1
gripper_joint_2
```

arm joint 이름이 일치하므로 기본 mapping 후보는 만들 수 있다. 단, 실제
control mapping 검증은 아직 완료되지 않았다.

## 임시 Mapping 표

| Leader joint | Follower joint | 현재 상태 |
| --- | --- | --- |
| `joint1` | `joint1` | 이름만 일치함 |
| `joint2` | `joint2` | 이름만 일치함 |
| `joint3` | `joint3` | 이름만 일치함 |
| `joint4` | `joint4` | 이름만 일치함 |
| `joint5` | `joint5` | 이름만 일치함 |
| `gripper_joint_1` | `gripper_joint_1`, 필요 시 `gripper_joint_2`에 mirror | 추가 검증 필요 |

## 원본 대비 차이

Leader USD와 follower USD는 수정하지 않았다. 아직 최종 mapping config도
추가하지 않았다. 현재 산출물은 초기 조사 결과와 검증 대상 목록이다.

## 현재 제한 사항

Leader USD에서도 follower와 유사한 visual reference 경고가 발생한다.

```text
Unresolved reference prim path ... configuration/omx_l_physics.usd@</visuals/world>
```

## 다음 실행 후보

기존 smoke script는 다른 USD 경로를 받을 수 있다.

```bash
cd <YOUR_ISAACLAB_PATH>
source <YOUR_CONDA_PATH>/etc/profile.d/conda.sh
conda activate <YOUR_CONDA_ENV>
source _isaac_sim/setup_conda_env.sh

./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/smoke_omx_articulation.py \
  --usd_path <YOUR_REPO_PATH>/open_manipulator/open_manipulator_description/urdf/omx_l/omx_l/omx_l.usd \
  --headless
```

주의: 현재 smoke script는 follower의 gripper joint 이름을 기준으로 초기화한다.
따라서 leader 전용 smoke test로 간주하려면 호환성 보완이 필요하다.

전체 argument 설명은 [스크립트 실행 가이드](script_execution_guide.md#1-smoke_omx_articulationpy)를
참조한다.

## 남은 작업 체크리스트

- [ ] smoke script가 OMX-L joint 구조도 처리하도록 보완한다.
- [ ] OMX-L articulation smoke test를 실행한다.
- [ ] arm joint order를 확인한다.
- [ ] leader-to-follower joint별 sign 방향을 확인한다.
- [ ] zero pose를 확인한다.
- [ ] ROS 데이터와 Isaac Lab command의 degree / radian 단위를 확인한다.
- [ ] joint limit을 확인한다.
- [ ] `gripper_joint_2` mirror 동작을 포함한 gripper mapping을 정의한다.
- [ ] 측정값이 포함된 최종 mapping 표를 작성한다.
- [ ] OMX-L visual reference 경고를 해결한다.
