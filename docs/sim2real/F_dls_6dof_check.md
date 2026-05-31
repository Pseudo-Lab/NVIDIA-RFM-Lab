# F DLS / 6-DOF Target 검증

최종 갱신일: 2026-05-31

## 목표

제어 대상 joint가 5개인 OMX arm에 일반적인 6D task-space pose target을
어떻게 적용할지 결정한다.

```text
x, y, z, wx, wy, wz
```

## 검증 스크립트

```text
scripts/check_omx_dls_6dof.py
```

스크립트는 object scene을 런타임에 합성하고 OMX-F를 spawn한 뒤, `link5`를
end-effector body로 사용한다. bear, chick, fish 위쪽의 pregrasp target을
시험한다.

각 target은 다음 roll 값으로 시험한다.

```text
0 deg
45 deg
90 deg
```

## 비교한 IK 모드

| 모드 | 제어하는 task-space 성분 |
| --- | --- |
| `full_pose` | x, y, z, wx, wy, wz |
| `position` | x, y, z |
| `mask_no_wx` | x, y, z, wy, wz |
| `mask_no_wy` | x, y, z, wx, wz |
| `mask_no_wz` | x, y, z, wx, wy |

`full_pose`는 Isaac Lab `DifferentialIKController`를 사용한다. 축을 줄인
모드는 원하는 task-space row를 제외할 수 있도록 로컬 DLS 계산을 사용한다.

## 1차 결과

현재는 `mask_no_wz`의 결과가 가장 좋다.

| 물체 | `full_pose` 위치 오차 | `mask_no_wz` 위치 오차 | `mask_no_wz` 방향 오차 |
| --- | --- | --- | --- |
| bear | 약 `61-67 mm` | 약 `3.5 mm` | 약 `2.6-2.9 deg` |
| chick | 약 `44-69 mm` | 약 `5-6 mm` | 약 `1.2-2.3 deg` |
| fish | 약 `61-69 mm` | 약 `33 mm` | 약 `2.8-4.5 deg` |

## 해석

5축 OMX arm이 임의의 full 6D pose target을 항상 정확하게 만족할 수 있다고
가정하면 안 된다. angular task-space constraint 하나를 제외하면 결과가
크게 개선된다.

현재 scene에서는 `wz`를 제외하는 방식이 가장 적합하다.

fish는 여전히 위치 오차가 크다. DLS 모드뿐 아니라 물체 배치, pregrasp
clearance, provisional `link5` end-effector frame을 함께 조정해야 한다.

## 실행법

전체 DLS 비교:

```bash
cd <YOUR_ISAACLAB_PATH>
source <YOUR_CONDA_PATH>/etc/profile.d/conda.sh
conda activate <YOUR_CONDA_ENV>
source _isaac_sim/setup_conda_env.sh

TERM=xterm ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/check_omx_dls_6dof.py \
  --headless \
  --object all \
  --iters 120 \
  --ik_mode all
```

GUI에서 후보 모드를 실행하고 마지막 pose 유지:

```bash
TERM=xterm ./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/check_omx_dls_6dof.py \
  --object fish \
  --iters 120 \
  --ik_mode mask_no_wz \
  --keep_open
```

전체 argument 설명은 [스크립트 실행 가이드](script_execution_guide.md#3-check_omx_dls_6dofpy)를
참조한다.

## 로그 형식

예시:

```text
[F] bear,mask_no_wz,45,0.00362,2.65,[...]
```

열 순서:

```text
object, ik_mode, roll_deg, pos_error_m, ori_error_deg, joint_pos
```

## 남은 작업 체크리스트

- [ ] fish target 위치를 조정한다.
- [ ] fish pregrasp 높이를 조정한다.
- [ ] `link5`만으로 충분한지 확인한다.
- [ ] 필요하면 tool-center-point offset을 추가한다.
- [ ] 선택한 reduced DLS 전략을 task env에 반영한다.
- [ ] 아직 keyboard나 AR기기 teleop을 적용할만큼 정교하지 못하다 (error가 있는듯함.)
