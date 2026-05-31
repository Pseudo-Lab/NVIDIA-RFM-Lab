# A1 OMX Follower USD / Physics 검증

최종 갱신일: 2026-05-31

## 목표

OMX follower USD를 task env에 연결하기 전에 Isaac Lab articulation으로 사용할
수 있는지 확인한다.

## 기준 USD

Follower 원본:

```text
open_manipulator/open_manipulator_description/urdf/omx_f/omx_f/omx_f.usd
```

원본 USD는 meter 단위를 사용하며 default prim은 `/omx_f`이다.

## 런타임에서 추가한 내용

원본 USD 파일은 수정하지 않았다. `scripts/smoke_omx_articulation.py`가 실행
중에 Isaac Lab `ArticulationCfg`를 만들고 follower를 아래 경로에 spawn한다.

```text
/World/OMX
```

런타임 config에는 다음 내용이 포함된다.

- 모든 초기 joint position을 `0.0`으로 설정한다.
- 모든 joint에 smoke test용 implicit actuator를 적용한다.
- 임시 stiffness, damping, effort, velocity 값을 지정한다.
- 확인용 ground plane과 dome light를 추가한다.
- arm joint에 짧은 position target을 적용해 실제 움직임을 확인한다.

## 확인 결과

Follower spawn에 성공했으며 Isaac Lab에서 아래 metadata를 읽었다.

```text
num_joints: 7
joint_names:
  joint1
  joint2
  joint3
  joint4
  joint5
  gripper_joint_1
  gripper_joint_2

num_bodies: 9
body_names:
  world
  link0
  link1
  link2
  link3
  link4
  link5
  link6
  link7
```

USD의 `end_effector_link`는 `link5` 아래 Xform이지만 Isaac Lab rigid body로
노출되지 않는다. 현재 IK 검증에서는 `link5`를 end-effector body로 사용한다.

## 원본 대비 차이

| 항목 | 원본 follower USD | 런타임 smoke stage |
| --- | --- | --- |
| 로봇 prim 경로 | `/omx_f` | `/World/OMX` |
| actuator 설정 | 원본 USD에 기록된 값 | 스크립트의 임시 `ImplicitActuatorCfg` 사용 |
| ground | follower USD에 없음 | 확인용 ground plane 추가 |
| light | follower USD에 없음 | 확인용 dome light 추가 |
| joint target | 없음 | 움직임 확인용 arm target 적용 |
| 원본 USD 저장 변경 | 해당 없음 | 하지 않음 |

## 실행법

Headless smoke test:

```bash
cd <YOUR_ISAACLAB_PATH>
source <YOUR_CONDA_PATH>/etc/profile.d/conda.sh
conda activate <YOUR_CONDA_ENV>
source _isaac_sim/setup_conda_env.sh

./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/smoke_omx_articulation.py --headless
```

Smoke step 이후 GUI를 계속 유지:

```bash
./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/smoke_omx_articulation.py \
  --steps 100 \
  --keep_open
```

전체 argument 설명은 [스크립트 실행 가이드](script_execution_guide.md#1-smoke_omx_articulationpy)를
참조한다.

## 성공 기준

마지막 로그에 아래 문구가 출력되어야 한다.

```text
[SMOKE] PASS: OMX-F spawned and Isaac Lab articulation metadata was read.
```

## 남은 작업 체크리스트

- [ ] `configuration/omx_f_physics.usd@</visuals/world>` visual reference 경고를 해결한다.
- [ ] joint axis를 확인한다.
- [ ] joint sign을 확인한다.
- [ ] joint limit을 확인한다.
- [ ] mass와 inertia를 확인한다.
- [ ] friction, effort, velocity 값을 확인한다.
- [ ] drive stiffness와 damping을 확인한다.
- [ ] 임시 actuator 값을 최종 simulation 값으로 교체한다.
