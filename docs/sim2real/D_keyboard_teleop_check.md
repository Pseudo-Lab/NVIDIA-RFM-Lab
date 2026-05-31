# D Keyboard Teleop 입력 경로 검증

최종 갱신일: 2026-05-31

## 목표

Galaxy AR 입력을 연결하기 전에 keyboard를 사용해 Cartesian teleop 경로를
검증한다. Keyboard와 Galaxy AR은 모두 end-effector pose 변화량을 입력하는
경로로 연결할 수 있으므로 keyboard가 먼저 동작하면 reduced DLS controller의
기본 구조를 재사용할 수 있다.

## 구현 파일

```text
scripts/teleop_omx_keyboard.py
```

## 현재 구조

```text
Isaac Lab Se3Keyboard
→ Cartesian delta pose
→ 누적 Cartesian target
→ mask_no_wz reduced DLS
→ OMX arm joint1 ~ joint5 target
→ OMX follower articulation
```

Gripper는 keyboard의 binary open / close 입력을 사용한다.

```text
K
→ gripper open / close toggle
→ gripper_joint_1 target
→ gripper_joint_2 mimic joint
```

`gripper_joint_2`는 follower USD에서 `PhysxMimicJointAPI:rotZ`를 가진다.

## 사용 Scene

Keyboard teleop는 bake된 nominal scene을 기본값으로 로드한다.

```text
Assets/scenes/Sim2Real_OMX_nominal.usd
```

이 scene에는 OMX-F, floor, 조명, 물체 3종의 nominal 위치와 physics가 들어 있다.

## 실행법

```bash
cd <YOUR_ISAACLAB_PATH>
source <YOUR_CONDA_PATH>/etc/profile.d/conda.sh
conda activate <YOUR_CONDA_ENV>
source _isaac_sim/setup_conda_env.sh

TERM=xterm ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/teleop_omx_keyboard.py
```

GUI 창을 직접 닫을 때까지 실행된다.

## 키 배치

| 키 | 동작 |
| --- | --- |
| `W` / `S` | end-effector x축 이동 |
| `A` / `D` | end-effector y축 이동 |
| `Q` / `E` | end-effector z축 이동 |
| `Z` / `X` | x축 회전 |
| `T` / `G` | y축 회전 |
| `C` / `V` | z축 회전 입력 |
| `K` | gripper 열기 / 닫기 toggle |
| `R` | 로봇 joint와 Cartesian target reset |
| `L` | Isaac Lab `Se3Keyboard` 내부 입력 상태 reset |

기본 `mask_no_wz` 모드에서는 z축 angular constraint를 DLS에서 제외한다.
따라서 `C` / `V` yaw 입력은 target에 누적되지만 OMX arm에 강제하지 않는다.

## 주요 옵션

| Argument | 기본값 | 설명 |
| --- | --- | --- |
| `--scene_usd SCENE_USD` | `Assets/scenes/Sim2Real_OMX_nominal.usd` 절대 경로 | teleop에 사용할 bake된 nominal scene을 지정한다. |
| `--steps STEPS` | `-1` | 실행 step 수를 지정한다. 음수이면 GUI 창을 닫을 때까지 실행한다. |
| `--pos_sensitivity VALUE` | `0.001` | keyboard 이동 입력 1회당 translation 증가량을 meter 단위로 지정한다. |
| `--rot_sensitivity VALUE` | `0.01` | keyboard 회전 입력 1회당 rotation 증가량을 radian 단위로 지정한다. |
| `--damping VALUE` | `0.05` | reduced DLS damping 값을 지정한다. |
| `--step_scale VALUE` | `0.6` | DLS joint delta에 곱할 update scale을 지정한다. |
| `--ik_mode MODE` | `mask_no_wz` | `position`, `mask_no_wx`, `mask_no_wy`, `mask_no_wz` 중 하나를 선택한다. |
| `--gripper_open VALUE` | `0.0` | gripper open 상태의 driven joint target을 지정한다. |
| `--gripper_close VALUE` | `0.35` | gripper close 상태의 driven joint target을 지정한다. |
| `--log_interval STEPS` | `120` | target pose와 오차 로그를 출력할 주기를 지정한다. |

Isaac Lab 공통 argument는
[스크립트 실행 가이드](script_execution_guide.md#6-simulation-스크립트-공통-isaac-lab-argument)를
참조한다.

## 검증 결과

GUI 3-step smoke test에서 다음 항목을 확인했다.

- nominal scene 로드 성공
- `/World/OMX` articulation 연결 성공
- arm joint `joint1` ~ `joint5` 조회 성공
- gripper driven joint `gripper_joint_1` 조회 성공
- end-effector body `link5` 조회 성공
- Isaac Lab `Se3Keyboard` 생성 성공
- 기본 `mask_no_wz` reduced DLS loop 실행 성공
- Keyboard delta pose를 누적 Cartesian target으로 적용

실제 키 입력에서 목표 pose가 작업 공간 밖까지 누적되며 오차가 커지는 현상을
확인했다. 현재 코드는 초기 누적 target 방식을 유지한다. reduced DLS 제어 방식과
입력 처리 방식은 후속 작업에서 함께 조정해야 한다.

## 남은 작업 체크리스트

- [x] Keyboard Cartesian teleop script를 작성한다.
- [x] Nominal scene과 OMX follower articulation을 연결한다.
- [x] `mask_no_wz` reduced DLS를 연결한다.
- [x] Gripper mimic joint 구조를 확인한다.
- [ ] 누적 target이 작업 공간 밖으로 계속 증가하는 원인을 해결한다.
- [ ] reduced DLS가 keyboard teleop에 적합한지 다시 검증한다.
- [ ] gravity와 actuator 설정이 조작 반응성에 미치는 영향을 비교한다.
- [ ] GUI에서 이동 방향과 회전 방향을 직접 확인한다.
- [ ] GUI에서 gripper open / close 방향을 직접 확인한다.
- [ ] sensitivity 값을 조정한다.
- [ ] demo recorder를 연결한다.
- [ ] Galaxy AR 입력을 같은 Cartesian target 경로에 연결한다.
- [ ] 아직 오차가 있어서 정밀성이 떨어짐. F번 문제와 같이 해결해야함?
