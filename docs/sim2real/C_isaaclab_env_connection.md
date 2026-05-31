# C OMX_CFG / Isaac Lab Env 연결 현황

최종 갱신일: 2026-05-31

## 목표

OMX-F를 재사용 가능한 robot config로 만들고 Isaac Lab task env에 연결한다.

## 현재까지 진행한 내용

재사용 가능한 follower config를 아래 파일에 1차 작성했다.

```text
source/sim2real_omx/assets/omx.py
```

이 파일의 `OMX_FOLLOWER_CFG`로 OMX-F를 아래 경로에 spawn할 수 있다.

```text
/World/OMX
```

현재 공용 config에서 확인한 내용:

- robot USD 경로
- zero-position 초기 상태
- joint 이름
- 임시 implicit actuator 설정
- Isaac Lab articulation spawn 성공

`scripts/smoke_omx_articulation.py`가 공용 `OMX_FOLLOWER_CFG`를 사용하도록
연결했으며 smoke test PASS를 확인했다.

## 아직 완료하지 않은 내용

기존 task env의 robot을 OMX-F로 교체하는 env config diff와 task env spawn
결과도 아직 없다.

## 기존 상태 대비 차이

| 항목 | 작업 전 | 현재 상태 |
| --- | --- | --- |
| OMX Isaac Lab config | 없음 | 공용 `OMX_FOLLOWER_CFG` 1차 작성 완료 |
| OMX articulation spawn | 미확인 | 확인 완료 |
| 공용 `OMX_CFG` 모듈 | 없음 | `source/sim2real_omx/assets/omx.py`에 추가 |
| task env 연결 | 없음 | 아직 없음 |
| teleop env spawn | 없음 | 아직 확인하지 않음 |

## 현재 실행 가능한 smoke test

```bash
cd <YOUR_ISAACLAB_PATH>
source <YOUR_CONDA_PATH>/etc/profile.d/conda.sh
conda activate <YOUR_CONDA_ENV>
source _isaac_sim/setup_conda_env.sh

./isaaclab.sh -p <YOUR_REPO_PATH>/scripts/smoke_omx_articulation.py --headless
```

전체 argument 설명은 [스크립트 실행 가이드](script_execution_guide.md)를
참조한다.

## 남은 작업 체크리스트

- [x] 재사용 가능한 `OMX_FOLLOWER_CFG` Python 모듈을 1차 작성한다.
- [ ] 합의된 actuator 값을 공용 config로 옮긴다.
- [ ] task env config의 robot을 `OMX_CFG`로 교체한다.
- [ ] provisional IK end-effector body로 `link5`를 연결한다.
- [ ] task env spawn에 성공하는지 확인하고 로그 또는 캡처를 남긴다.
- [ ] teleop env spawn을 확인한다.
- [ ] grasp 정확도에 따라 tool-center-point offset을 추가할지 결정한다.
