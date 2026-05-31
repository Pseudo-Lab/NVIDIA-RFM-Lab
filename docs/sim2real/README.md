# Sim2Real OMX 진행 현황 및 전달 문서

최종 갱신일: 2026-05-31

이 디렉터리는 `archive/sync.html`에 정의된 OMX Sim2Real 작업의 현재 진행
상태를 전달하기 위한 문서 모음이다. 완료된 검증, 런타임에서만 적용되는
변경 사항, 아직 구현하지 않은 작업을 구분해서 기록한다.

## 현재 상태 요약

| 작업 | 현재 상태 | 확인된 결과 | 남은 핵심 작업 |
| --- | --- | --- | --- |
| A1 follower USD / physics | 1차 smoke 검증 완료 | OMX-F가 Isaac Lab articulation으로 spawn됨. arm joint 5개와 gripper joint 2개를 확인함 | visual reference 경고 해결, joint 및 physics sanity 표 작성 |
| A2 leader USD / mapping | 초기 USD 조사 완료 | OMX-L USD가 존재하며 arm joint 이름이 follower와 대응함 | leader 전용 smoke 검증, sign, order, zero pose, unit, limit 검증 |
| B object / scene USD | 런타임 구성 1차 완료 | 물체 3종 scale, 위치, collider, mass와 어두운 floor collider를 런타임에 추가함 | 런타임 구성을 정리된 USD로 저장할지 결정, 충돌 동작 검증 |
| C OMX_CFG / env 연결 | spawn 가능성 검증 완료 | 스크립트 내부 `ArticulationCfg`로 OMX-F spawn 성공 | 재사용 가능한 `OMX_CFG` 모듈 작성, task env 연결 |
| F DLS / 6-DOF 검증 | 1차 비교 완료 | 5축 arm에 full 6D pose를 강제하면 과구속됨. 현재는 `mask_no_wz`가 가장 적합함 | fish 위치 및 pregrasp target 조정, env에 controller 반영 |

## 기준 Assets

변경하지 않은 기본 object scene:

```text
Assets/Sim2Real.usd
```

기본 scene의 root prim:

```text
/Render
/bear_v2_texture
/chick_v2_texture
/fish_v2_texture
```

기본 scene에는 OMX-F와 런타임 floor가 포함되어 있지 않다.

로봇 USD:

```text
open_manipulator/open_manipulator_description/urdf/omx_f/omx_f/omx_f.usd
open_manipulator/open_manipulator_description/urdf/omx_l/omx_l/omx_l.usd
```

## 공통 실행 환경

새 터미널을 열었다면 먼저 아래 명령을 실행한다.

```bash
cd /home/<your path>/IsaacLab
source /home/<your path>/miniconda3/etc/profile.d/conda.sh
conda activate isaaclab311
source _isaac_sim/setup_conda_env.sh
```

설치 상태를 간단히 확인하려면:

```bash
python -c "from pxr import Usd; import isaaclab; print('Isaac Lab OK')"
```

## 대표 실행 명령

OMX-F articulation smoke test:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/smoke_omx_articulation.py --headless
```

GUI를 열고 scene을 계속 관찰:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/run_omx_sim2real_scene.py
```

GUI에서 관절 움직임까지 계속 관찰:

```bash
./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/run_omx_sim2real_scene.py --demo_joints
```

DLS 전체 모드 비교:

```bash
TERM=xterm ./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/check_omx_dls_6dof.py \
  --headless \
  --object all \
  --iters 120 \
  --ik_mode all
```

선택한 DLS 모드를 GUI에서 실행하고 마지막 pose를 계속 관찰:

```bash
TERM=xterm ./isaaclab.sh -p /home/<your path>/NVIDIA-RFM-Lab/scripts/check_omx_dls_6dof.py \
  --object fish \
  --iters 120 \
  --ik_mode mask_no_wz \
  --keep_open
```

## 문서 목록

- [스크립트 실행 가이드 및 전체 argument 설명](script_execution_guide.md)
- [A1 follower USD 검증](A1_follower_usd_check.md)
- [A2 leader mapping 검증](A2_leader_mapping_check.md)
- [B scene collider 검증](B_scene_collider_check.md)
- [C Isaac Lab env 연결 현황](C_isaaclab_env_connection.md)
- [F DLS 6-DOF 검증](F_dls_6dof_check.md)

## 공통 제한 사항

OMX-F와 OMX-L은 현재 아래 visual reference 경고를 출력한다.

```text
Unresolved reference prim path ... configuration/omx_*_physics.usd@</visuals/world>
```

physics articulation 검증은 가능하지만, 로봇 visual과 material 표현은 완료
상태로 보지 않는다.

## 남은 작업 체크리스트

- [ ] A1 visual reference 경고를 해결한다.
- [ ] A1 joint 및 physics sanity 표를 작성한다.
- [ ] A2 leader-to-follower mapping을 실제 관절 움직임으로 검증한다.
- [ ] B collider drop 및 관통 테스트를 수행한다.
- [ ] B runtime 구성을 최종 USD로 저장할지 결정한다.
- [ ] C 재사용 가능한 `OMX_CFG`를 작성한다.
- [ ] C task env에 OMX-F를 연결한다.
- [ ] F fish target의 도달성을 조정한다.
- [ ] 실제 데이터 수집 팀과 카메라 및 조명 조건을 맞춘다.
- [ ] D 와 E 작업 수행한다.


### 요청 사항
- 그동안 작성하셨던 html 파일 업로드 가능한지
- screenshot 같은 것도 예전에 했던거 있으면 보여주실 수 있는지