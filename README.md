# NVIDIA-RFM-Lab

**ROBOTIS OpenMANIPULATOR(OMX)를 NVIDIA Isaac Sim에서 시뮬레이션하기 위한 Sim2Real 저장소입니다.**

가짜연구소(Pseudo Lab) Sim2Real 팀에서 진행하는, 실제 로봇 팔(OpenMANIPULATOR)과 커스텀 3D 오브젝트를
Isaac Sim 가상 환경에 재현하여 Sim-to-Real 학습·검증 파이프라인을 구축하는 것을 목표로 합니다.

> 📋 **[프로젝트 통합 보고서](PROJECT_REPORT.md)** — NVIDIA 로봇 파운데이션 모델 스터디의
> 두 트랙(Cosmos 증강 기반 GR00T 파인튜닝 + Isaac Sim Sim2Real 시뮬레이션) 전 과정과 결과,
> 멤버 명단, 공개 산출물([모델](https://huggingface.co/pseudolab/GR00T-N1.7-3B-OMX-PickupDolls)·
> [데이터셋](https://huggingface.co/datasets/pseudolab/omx_f_PickUpDoll)) 링크 포함

---

## 스터디 트랙과 RFM 결과 요약

본 저장소(Sim2Real 시뮬레이션)는 [NVIDIA 로봇 파운데이션 모델 스터디](https://pseudo-lab.com/projects/4f1a337f-2999-4dd1-a78e-50c937ac444e)의
두 트랙 중 하나이며, 다른 한 트랙인 **RFM 파인튜닝**에서는 같은 태스크
(OMX "인형 집어 옮기기")를 실기 데이터로 다뤘습니다:

- **데이터 증강**: 텔레오퍼레이션 75 에피소드를 SAM3 보존 마스크 +
  **Cosmos-Transfer2.5** 외관 재합성(배경·조명·재질 변형)으로 **525 에피소드**로 확장
  — 행동 라벨은 프레임 단위로 보존
- **파인튜닝**: **GR00T N1.7-3B**를 파인튜닝, 베이스라인 대조로 변형 외관에서
  **MSE 36% 개선** 확인, holdout 검증으로 최적 학습량(14k 스텝) 확정
- **공개 산출물**: 모델 [pseudolab/GR00T-N1.7-3B-OMX-PickupDolls](https://huggingface.co/pseudolab/GR00T-N1.7-3B-OMX-PickupDolls),
  데이터셋 [omx_f_PickUpDoll](https://huggingface.co/datasets/pseudolab/omx_f_PickUpDoll) ·
  [omx_f_PickUpDollWith2Cam](https://huggingface.co/datasets/pseudolab/omx_f_PickUpDollWith2Cam)
- 이 저장소의 3D 인형(bear·chick·fish)은 RFM 트랙 실기 데이터와 **같은 물체**로,
  파인튜닝된 정책의 시뮬레이션 검증과 분포 확장 데이터 생산이 후속 과제입니다

전 과정과 수치는 **[프로젝트 통합 보고서](PROJECT_REPORT.md)** 참고.

### 공개 산출물

| 구분 | 링크 | 내용 |
|---|---|---|
| 🤖 **모델** | [pseudolab/GR00T-N1.7-3B-OMX-PickupDolls](https://huggingface.co/pseudolab/GR00T-N1.7-3B-OMX-PickupDolls) | Cosmos 증강 데이터로 파인튜닝한 GR00T N1.7-3B 정책 (holdout 검증 최적 체크포인트, 모델 카드 포함) |
| 📊 **데이터셋** | [pseudolab/omx_f_PickUpDoll](https://huggingface.co/datasets/pseudolab/omx_f_PickUpDoll) | OMX 텔레오퍼레이션 원본 시연 (LeRobot v2.1, 탑다운 카메라) |
| 📊 **데이터셋** | [pseudolab/omx_f_PickUpDollWith2Cam](https://huggingface.co/datasets/pseudolab/omx_f_PickUpDollWith2Cam) | 2카메라 추가 실측 시연 94 eps (60fps) |
| 🧩 **시뮬레이션** | 본 저장소 `Assets/` | Isaac Sim 씬 + 2D→3D 에셋 파이프라인 결과물 |

---

## 구성

이 저장소는 크게 세 부분으로 나뉩니다.

| 디렉토리 | 내용 |
|---------|------|
| `Assets/` | Isaac Sim 시뮬레이션 에셋 (USD 씬, 3D 오브젝트) |
| `Assets/Image_to_3D/` | 2D 사진에서 시뮬레이션용 3D 오브젝트를 생성하는 파이프라인 |
| `open_manipulator/` | ROBOTIS OpenMANIPULATOR ROS 2 패키지 (로봇 모델·제어·MoveIt) |

```
NVIDIA-RFM-Lab/
├── Assets/
│   ├── Sim2Real.usd            # Isaac Sim 메인 씬 (로봇 + 오브젝트 배치)
│   └── Image_to_3D/            # 2D → 3D 에셋 생성 파이프라인
│       ├── Images/             #   입력 이미지 (원본 + 전처리본)
│       └── mesh/               #   생성된 3D 모델 (.fbx / .usd + 텍스처)
└── open_manipulator/           # ROBOTIS OpenMANIPULATOR ROS 2 패키지
    ├── open_manipulator_description/   # URDF/Xacro 로봇 모델, 메시
    ├── open_manipulator_bringup/       # 실기·Gazebo 구동 launch
    ├── open_manipulator_moveit_config/ # MoveIt 모션 플래닝 설정
    ├── open_manipulator_gui/           # 조인트 제어 GUI
    ├── open_manipulator_teleop/        # 텔레오퍼레이션
    ├── open_manipulator_playground/    # 예제 노드 (궤적 제어 등)
    ├── open_manipulator_collision/     # 자가 충돌 검사
    └── ros2_controller/                # 커스텀 ros2_control 플러그인
```

지원 로봇 모델: `omx_f`, `omx_l`(OpenMANIPULATOR-X), `omy_3m`, `omy_f3m`, `omy_l100`(OMY 시리즈).

---

## 1. Isaac Sim 씬 — `Assets/Sim2Real.usd`

Isaac Sim에서 여는 메인 USD 씬(USDC 바이너리)입니다. Sim2Real 실험의 시작점이 됩니다.

**씬 구성**

- **로봇**: **OpenMANIPULATOR-X (`omx_f`)** 매니퓰레이터가 씬의 조작 주체로 배치됩니다.
  5개 회전 관절(`joint1`~`joint5`)로 구성된 팔과 2지 평행 그리퍼(`gripper_joint_1`/`gripper_joint_2`)를 가지며,
  모델은 `open_manipulator/open_manipulator_description/urdf/omx_f/`의 URDF에서 유래합니다.
  로봇을 통해 아래 오브젝트를 집고 옮기는 pick-and-place형 Sim2Real 태스크를 구성합니다.
- **오브젝트**: `Image_to_3D` 파이프라인으로 만든 3종(**bear / chick / fish**)이 배치됩니다.
  메시를 씬에 임베드하지 않고 `./Image_to_3D/mesh/`의 에셋을 **레퍼런스/페이로드**로 참조하며,
  각 오브젝트는 `xformOp`(translate·rotateXYZ·scale)로 위치가 지정됩니다.
- **물리**: 각 오브젝트에 리지드 바디(`PhysicsRigidBodyAPI`)와 충돌(`PhysicsCollisionAPI` —
  Convex Hull / Triangle Mesh)이 부여되어 있어 집기·낙하 같은 물리 상호작용이 가능합니다.
- **렌더링/뷰**: Isaac Sim(Omniverse Kit) RTX 렌더 설정과 카메라 북마크(Front / Right / Top / Perspective)가 저장되어 있습니다.

> 씬은 에셋을 경로 참조하므로, 저장소 구조를 유지한 채(에셋과 상대 경로가 깨지지 않게) 열어야 합니다.

**여는 방법**
1. Isaac Sim 실행
2. `File > Open` → `Assets/Sim2Real.usd` 선택

> 대용량 바이너리(`.fbx`, 텍스처 `.png`)는 Git LFS로 관리됩니다.
> 클론 후 에셋이 비어 있다면 `git lfs install && git lfs pull`을 실행하세요.

---

## 2. 3D 에셋 파이프라인 — `Assets/Image_to_3D/`

다이소 인형(**bear / chick / fish**)을 실제 촬영한 사진에서 시뮬레이션용 3D 모델로 변환하는 파이프라인입니다.

```
원본 사진 → [ChatGPT Image 2.0] 전처리 → [Hunyuan 3D 1.1] 3D 생성 → [Meshy 6] Remesh → 최종 모델
```

오브젝트별 분기, 프롬프트, 결과물 비교 등 상세 내용은 **[`Assets/Image_to_3D/README.md`](Assets/Image_to_3D/README.md)** 를 참고하세요.

---

## 3. ROBOTIS OpenMANIPULATOR ROS 2 패키지 — `open_manipulator/`

실제 로봇 제어와 URDF 모델의 출처가 되는 ROBOTIS 공식 ROS 2 패키지입니다.
Isaac Sim 씬의 로봇 모델(URDF/메시)이 여기서 유래하며, 실기(Real) 측 제어에도 사용됩니다.

자세한 사용법과 공식 문서 링크는 **[`open_manipulator/README.md`](open_manipulator/README.md)** 를 참고하세요.

주요 참조:
- [OMY 문서](https://ai.robotis.com/omy/introduction_omy.html)
- [OpenMANIPULATOR-X 문서](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/)
- [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools)

---

## 사전 준비

- **NVIDIA Isaac Sim** — USD 씬 구동
- **ROS 2 Jazzy** + **MoveIt 2** — `open_manipulator` 패키지 빌드·실행용 (실기/Gazebo). Docker 이미지는 `open_manipulator/docker/` 참고
- **Git LFS** — 3D 메시·텍스처 에셋 다운로드용

---

## 라이선스

- 이 저장소: MIT License (© 2026 가짜연구소 / Pseudo Lab) — [`LICENSE`](LICENSE)
- `open_manipulator/`: ROBOTIS 원본 라이선스([`open_manipulator/LICENSE`](open_manipulator/LICENSE))를 따릅니다.
