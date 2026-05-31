# Sim2Real OMX 스크립트 실행 가이드

최종 갱신일: 2026-05-31

이 문서는 현재 작성된 Python 스크립트 5개의 역할, 실행법, argument를 정리한다.

## 공통 환경 준비

새 터미널을 열었다면 먼저 실행한다.

```bash
cd /home/<your path>/IsaacLab
source _isaac_sim/setup_conda_env.sh
```

## 실행 명령의 기본 구조

Isaac Lab simulation을 실행하는 세 스크립트는 Isaac Lab wrapper를 통해
실행한다.

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/<SCRIPT_NAME>.py <ARGUMENTS>
```

여기서 `-p`는 뒤에 지정한 Python 파일을 Isaac Lab Python 환경으로
실행하라는 `isaaclab.sh` wrapper 옵션이다. Python 스크립트 자체의 argument는
파일 경로 뒤에 작성한다.

F 스크립트 실행 예시에 사용하는 `TERM=xterm`은 일부 비대화형 터미널에서
발생할 수 있는 `terminal type 'dumb' cannot reset tabs` 오류를 피하기 위한
환경 변수 지정이다.

## 1. `smoke_omx_articulation.py`

### 역할

OMX-F USD가 Isaac Lab articulation으로 정상 spawn되는지 빠르게 검사한다.
joint, rigid body, 초기 상태를 출력하고 짧은 joint target을 적용한다.

### 대표 실행

Headless smoke test:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/smoke_omx_articulation.py --headless
```

100 step 실행 후 GUI 유지:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/smoke_omx_articulation.py \
  --steps 100 \
  --keep_open
```

### 고유 argument

| Argument | 기본값 | 설명 |
| --- | --- | --- |
| `-h`, `--help` | 없음 | 도움말을 출력하고 종료한다. |
| `--usd_path USD_PATH` | OMX-F USD 경로 | 검사할 로봇 USD 경로를 지정한다. |
| `--prim_path PRIM_PATH` | `/World/OMX` | 로봇을 spawn할 stage prim 경로를 지정한다. |
| `--steps STEPS` | `10` | joint target 적용 후 실행할 simulation step 수를 지정한다. |
| `--keep_open` | 꺼짐 | smoke step 종료 후 GUI를 닫지 않고 계속 유지한다. |

## 2. `run_omx_sim2real_scene.py`

### 역할

`Assets/Sim2Real.usd`를 런타임에 합성하고 OMX-F, floor, 물체 physics,
조명을 추가한다. GUI에서 scene 크기, 배치, texture, 관절 움직임을 확인할 때
사용한다.

### 대표 실행

GUI를 계속 유지:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/run_omx_sim2real_scene.py
```

GUI에서 joint 움직임도 계속 확인:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/run_omx_sim2real_scene.py --demo_joints
```

Headless 3 step smoke test:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/run_omx_sim2real_scene.py \
  --headless \
  --steps 3
```

### 고유 argument

| Argument | 기본값 | 설명 |
| --- | --- | --- |
| `-h`, `--help` | 없음 | 도움말을 출력하고 종료한다. |
| `--scene_usd SCENE_USD` | `Assets/Sim2Real.usd` 절대 경로 | 런타임에 sublayer로 합성할 object scene USD 경로를 지정한다. |
| `--robot_usd ROBOT_USD` | OMX-F USD 절대 경로 | spawn할 OMX-F robot USD 경로를 지정한다. |
| `--steps STEPS` | `-1` | 실행할 simulation step 수를 지정한다. 음수이면 창을 직접 닫을 때까지 계속 실행한다. |
| `--no_floor` | 꺼짐 | 런타임 floor collider를 추가하지 않는다. |
| `--no_light` | 꺼짐 | 런타임 ambient 및 key light를 추가하지 않는다. 조명 비교에 사용할 수 있다. |
| `--demo_joints` | 꺼짐 | OMX arm과 gripper에 주기적인 target을 적용해 계속 움직인다. |

## 3. `check_omx_dls_6dof.py`

### 역할

5축 OMX arm에 적용할 DLS 전략을 비교한다. 물체 위쪽 pregrasp target에 대해
full 6D pose, position-only, angular axis mask 조합의 위치 및 방향 오차를
출력한다.

### 대표 실행

전체 비교:

```bash
TERM=xterm ./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/check_omx_dls_6dof.py \
  --headless \
  --object all \
  --iters 120 \
  --ik_mode all
```

Fish의 `mask_no_wz` 결과를 GUI에서 확인하고 마지막 pose 유지:

```bash
TERM=xterm ./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/check_omx_dls_6dof.py \
  --object fish \
  --iters 120 \
  --ik_mode mask_no_wz \
  --keep_open
```

### 고유 argument

| Argument | 기본값 | 설명 |
| --- | --- | --- |
| `-h`, `--help` | 없음 | 도움말을 출력하고 종료한다. |
| `--scene_usd SCENE_USD` | `Assets/Sim2Real.usd` 절대 경로 | DLS 검증에 사용할 object scene USD 경로를 지정한다. |
| `--robot_usd ROBOT_USD` | OMX-F USD 절대 경로 | 검증할 OMX-F robot USD 경로를 지정한다. |
| `--object {bear,chick,fish,all}` | `all` | 검사할 물체를 선택한다. `all`이면 세 물체를 순서대로 검사한다. |
| `--iters ITERS` | `180` | 각 target에 적용할 IK 반복 횟수를 지정한다. |
| `--settle_steps SETTLE_STEPS` | `20` | 각 case를 시작하기 전에 reset 상태를 안정화할 simulation step 수를 지정한다. |
| `--ik_mode {full_pose,position,mask_no_wx,mask_no_wy,mask_no_wz,all}` | `full_pose` | 비교할 IK 모드를 지정한다. `all`이면 모든 모드를 순서대로 검사한다. |
| `--damping DAMPING` | `0.05` | 로컬 masked DLS 계산에 사용할 damping 값을 지정한다. |
| `--step_scale STEP_SCALE` | `0.6` | 로컬 masked DLS에서 계산된 joint delta에 곱할 update scale을 지정한다. |
| `--keep_open` | 꺼짐 | 모든 DLS case 종료 후 마지막 pose에서 GUI를 계속 유지한다. |

### `--ik_mode` 값 설명

| 값 | 제어 성분 | 용도 |
| --- | --- | --- |
| `full_pose` | x, y, z, wx, wy, wz | Isaac Lab 기본 full 6D DLS 기준점 |
| `position` | x, y, z | 방향을 포기하고 위치 도달성만 확인 |
| `mask_no_wx` | x, y, z, wy, wz | x축 angular constraint 제외 |
| `mask_no_wy` | x, y, z, wx, wz | y축 angular constraint 제외 |
| `mask_no_wz` | x, y, z, wx, wy | z축 angular constraint 제외 |
| `all` | 위 모드 전체 | 비교 표 생성 |

## 4. `bake_omx_nominal_scene.py`

### 역할

검증한 고정 scene 구성을 nominal USD로 저장한다. 원본
`Assets/Sim2Real.usd`는 수정하지 않는다. 물체 scale, 위치, collider, mass,
floor, 조명, OMX-F reference를 아래 파일에 기록한다.

```text
Assets/scenes/Sim2Real_OMX_nominal.usd
```

### 대표 실행

처음 생성:

```bash
python /home/<your path>/NVIDIA-RFM-Lab/scripts/bake_omx_nominal_scene.py
```

기존 nominal USD를 명시적으로 교체:

```bash
python /home/<your path>/NVIDIA-RFM-Lab/scripts/bake_omx_nominal_scene.py --overwrite
```

### 고유 argument

| Argument | 기본값 | 설명 |
| --- | --- | --- |
| `-h`, `--help` | 없음 | 도움말을 출력하고 종료한다. |
| `--scene_usd SCENE_USD` | `Assets/Sim2Real.usd` 절대 경로 | 변경하지 않을 원본 object scene USD를 지정한다. |
| `--robot_usd ROBOT_USD` | OMX-F USD 절대 경로 | nominal scene에서 reference할 robot USD를 지정한다. |
| `--output_usd OUTPUT_USD` | `Assets/scenes/Sim2Real_OMX_nominal.usd` 절대 경로 | 생성할 nominal scene USD 경로를 지정한다. |
| `--overwrite` | 꺼짐 | 출력 USD가 이미 있을 때 교체를 허용한다. |

이 스크립트는 pure USD API를 사용하므로 `isaaclab.sh -p` wrapper와
`AppLauncher` 공통 argument를 사용하지 않는다.

## 5. `teleop_omx_keyboard.py`

### 역할

Bake된 nominal scene을 열고 Isaac Lab `Se3Keyboard` 입력을
`mask_no_wz` reduced DLS로 변환해 OMX-F를 조작한다.

현재 구현은 keyboard delta를 Cartesian target에 누적한다. 작업 공간 밖으로
target이 계속 증가할 수 있으므로 DLS teleop 입력 처리와 반응성 조정은 남은
작업이다.

### 대표 실행

```bash
TERM=xterm ./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/teleop_omx_keyboard.py
```

### 고유 argument

| Argument | 기본값 | 설명 |
| --- | --- | --- |
| `-h`, `--help` | 없음 | 도움말을 출력하고 종료한다. |
| `--scene_usd SCENE_USD` | `Assets/scenes/Sim2Real_OMX_nominal.usd` 절대 경로 | teleop에 사용할 bake된 nominal scene을 지정한다. |
| `--steps STEPS` | `-1` | 실행할 simulation step 수를 지정한다. 음수이면 창을 직접 닫을 때까지 실행한다. |
| `--pos_sensitivity VALUE` | `0.001` | translation 입력 증가량을 meter 단위로 지정한다. |
| `--rot_sensitivity VALUE` | `0.01` | rotation 입력 증가량을 radian 단위로 지정한다. |
| `--damping VALUE` | `0.05` | reduced DLS damping 값을 지정한다. |
| `--step_scale VALUE` | `0.6` | DLS joint delta update scale을 지정한다. |
| `--ik_mode MODE` | `mask_no_wz` | `position`, `mask_no_wx`, `mask_no_wy`, `mask_no_wz` 중 하나를 지정한다. |
| `--gripper_open VALUE` | `0.0` | gripper open 상태의 driven joint target을 지정한다. |
| `--gripper_close VALUE` | `0.35` | gripper close 상태의 driven joint target을 지정한다. |
| `--log_interval STEPS` | `120` | target pose와 오차 로그 출력 주기를 지정한다. |

상세 키 배치는 [D keyboard teleop 문서](D_keyboard_teleop_check.md)를 참조한다.

## 6. Simulation 스크립트 공통 Isaac Lab argument

Simulation 스크립트는 `AppLauncher.add_app_launcher_args()`를 호출하므로 아래
argument를 공통으로 사용할 수 있다.

| Argument | 설명 |
| --- | --- |
| `--headless` | GUI를 열지 않고 실행한다. 자동 검증에 사용한다. |
| `--livestream {0,1,2}` | Isaac Sim livestream 모드를 강제로 설정한다. 값은 `LIVESTREAM` 환경 변수 mapping을 따른다. |
| `--enable_cameras` | camera sensor와 관련 extension dependency를 활성화한다. |
| `--xr` | VR/AR application을 위한 XR mode를 활성화한다. |
| `--device DEVICE` | simulation device를 지정한다. 예: `cpu`, `cuda`, `cuda:0`. |
| `--verbose` | SimulationApp의 verbose log를 활성화한다. |
| `--info` | SimulationApp의 info log를 활성화한다. |
| `--experience EXPERIENCE` | 실행할 SimulationApp experience file을 지정한다. 상대 경로는 Isaac Sim과 Isaac Lab의 `apps` 폴더 기준으로 해석된다. |
| `--rendering_mode {quality,balanced,performance}` | 렌더링 preset을 선택한다. 시각 품질 확인은 `quality`, 빠른 실행은 `performance`를 사용할 수 있다. |
| `--kit_args KIT_ARGS` | Omniverse Kit argument를 공백으로 구분한 문자열로 전달한다. |
| `--anim_recording_enabled` | Isaac Lab PhysX simulation의 time-sampled USD animation 기록을 활성화한다. |
| `--anim_recording_start_time TIME` | animation 기록 시작 시간을 지정한다. |
| `--anim_recording_stop_time TIME` | animation 기록 종료 시간을 지정한다. |

## 도움말 확인

코드와 문서가 다르게 보이면 현재 설치 환경의 도움말을 우선 확인한다.

```bash
python /home/<your path>/NVIDIA-RFM-Lab/scripts/smoke_omx_articulation.py --help
python /home/<your path>/NVIDIA-RFM-Lab/scripts/run_omx_sim2real_scene.py --help
python /home/<your path>/NVIDIA-RFM-Lab/scripts/check_omx_dls_6dof.py --help
python /home/<your path>/NVIDIA-RFM-Lab/scripts/bake_omx_nominal_scene.py --help
python /home/<your path>/NVIDIA-RFM-Lab/scripts/teleop_omx_keyboard.py --help
```

## 남은 작업 체크리스트

- [ ] 새 argument가 추가되면 이 문서를 갱신한다.
- [ ] 최종 `OMX_CFG`와 task env 실행 명령을 추가한다.
- [ ] camera sensor를 연결한 뒤 `--enable_cameras` 사용 예시를 추가한다.
- [x] Keyboard teleop 실행 명령과 argument를 추가한다.
