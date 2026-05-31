# B Object / Scene USD 검증

최종 갱신일: 2026-05-31

## 목표

물체 3종의 visible texture를 유지하면서 Isaac Lab simulation에 필요한 크기,
배치, collider, mass, floor를 구성한다.

## 기준 Assets

원본:

```text
Assets/Sim2Real.usd
```

원본 scene의 root prim:

```text
/Render
/bear_v2_texture
/chick_v2_texture
/fish_v2_texture
```

원본에는 OMX-F와 runtime floor가 없다. stage 단위는 `0.01 m/unit`이다.

## 런타임 구성 및 Nominal USD 저장 방식

`scripts/run_omx_sim2real_scene.py`가 원본 scene을 sublayer로 합성하고,
simulation 상태를 런타임에 추가한다.

원본 `Assets/Sim2Real.usd` 파일은 수정하지 않는다.

검증한 런타임 구성을 아래 nominal scene으로 저장하는 bake script도 추가했다.

```text
scripts/bake_omx_nominal_scene.py
Assets/scenes/Sim2Real_OMX_nominal.usd
```

## 런타임에서 추가한 내용

### 물체 크기와 배치

| 물체 | scale 계산에 사용한 제품 크기 | 런타임 중심 XY |
| --- | --- | --- |
| bear | x `7.5 cm`, z `10 cm` | `(0.20, -0.07)` |
| chick | x `8 cm`, z `5.5 cm` | `(0.26, 0.00)` |
| fish | x `11 cm`, z `9 cm` | `(0.32, 0.07)` |

제품 정보에서 x와 높이만 확인할 수 있었으므로 uniform scale을 사용한다.
깊이는 원본 3D 모델 비율을 유지한다.

### 물체 physics

각 물체에 다음 항목을 추가한다.

- `UsdPhysics.RigidBodyAPI`
- 임시 mass
- `convexHull` mesh collision

현재 임시 mass:

| 물체 | 런타임 mass |
| --- | --- |
| bear | `0.05 kg` |
| chick | `0.04 kg` |
| fish | `0.05 kg` |

### Floor

런타임 stage에 `/World/Floor`를 추가한다.

```text
size: 0.8 x 0.8 x 0.01 m
static friction: 1.0
dynamic friction: 0.8
restitution: 0.0
```

### Robot과 조명

런타임 stage에 다음 항목도 추가한다.

- `/World/OMX` 경로의 OMX-F
- 약한 ambient dome light
- 따뜻한 색상의 distant key light

## 원본 대비 차이

| 항목 | 원본 `Assets/Sim2Real.usd` | 런타임 합성 stage |
| --- | --- | --- |
| 물체 texture | 있음 | 유지함 |
| 물체 scale | import 당시 값 | 제품 크기 기준으로 조정 |
| 물체 배치 | import 당시 위치 | OMX가 집기 쉬운 위치로 재배치 |
| 물체 rigid body | 사용하지 않음 | 추가 |
| 물체 collider | 사용하지 않음 | convex hull로 추가 |
| floor | 없음 | `/World/Floor` 추가 |
| OMX-F | 없음 | `/World/OMX` 추가 |
| 조명 | 원본 scene 상태 | ambient와 key light 추가 |
| 원본 USD 저장 변경 | 해당 없음 | 하지 않음 |

## Nominal USD 생성법

```bash
cd <YOUR_ISAACLAB_PATH>
source <YOUR_CONDA_PATH>/etc/profile.d/conda.sh
conda activate <YOUR_CONDA_ENV>
source _isaac_sim/setup_conda_env.sh

python <YOUR_REPO_PATH>/scripts/bake_omx_nominal_scene.py --overwrite
```

`--overwrite`를 생략하면 기존 nominal USD를 교체하지 않는다.

## 실행법

GUI를 계속 열어두고 관찰:

```bash
cd <YOUR_ISAACLAB_PATH>
source <YOUR_CONDA_PATH>/etc/profile.d/conda.sh
conda activate <YOUR_CONDA_ENV>
source _isaac_sim/setup_conda_env.sh

./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/run_omx_sim2real_scene.py
```

GUI에서 joint 움직임까지 계속 확인:

```bash
./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/run_omx_sim2real_scene.py --demo_joints
```

짧은 headless smoke check:

```bash
./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/run_omx_sim2real_scene.py \
  --headless \
  --steps 3
```

전체 argument 설명은 [스크립트 실행 가이드](script_execution_guide.md#2-run_omx_sim2real_scenepy)를
참조한다.

## 남은 작업 체크리스트

- [ ] 실제 물체 질량을 측정하여 반영한다.
- [x] runtime composition을 nominal scene USD로 저장하는 bake script를 작성한다.
- [x] `Assets/scenes/Sim2Real_OMX_nominal.usd`를 생성한다.
- [ ] 물체가 floor를 관통하지 않는지 확인한다.
- [ ] 물체 drop 동작을 확인한다.
- [ ] 로봇 gripper와 물체 collider의 접촉 동작을 확인한다.
- [ ] 실제 물체 mass를 측정할 필요가 있는지 결정한다.
- [ ] 실데이터 수집 팀과 camera pose, intrinsics, resolution, fps를 맞춘다.
- [ ] 실데이터 수집 팀과 조명 조건을 맞춘다.
