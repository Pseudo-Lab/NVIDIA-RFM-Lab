# Image to 3D 파이프라인

2D 이미지에서 3D 모델을 생성하고 최적화하는 파이프라인.
대상 오브젝트: **bear**, **chick**, **fish** (모두 다이소 인형)

---

## 파이프라인 개요

```
원본 사진
    ↓
[1] ChatGPT Image 2.0  ─ 이미지 전처리 (배경/태그 제거, 조명 정규화)
    ↓
[2] Hunyuan 3D 1.1     ─ 3D 모델 생성 (~50,000 poly)
    ↓
[3] Meshy 6            ─ Remesh (폴리곤 수 최적화)
    ↓
최종 3D 모델
```

> **Meshy 멀티뷰 미사용**: 입력 이미지가 어느 뷰에 매핑되는지 알 수 없어 단일 이미지 베이스인 Hunyuan으로 우회.

### 오브젝트별 파이프라인 분기

| 오브젝트 | 적용 흐름 | 비고 |
|---------|----------|------|
| **bear**  | ChatGPT → Hunyuan → Meshy remesh | 표준 흐름, 한 번에 성공 |
| **chick** | ChatGPT → Hunyuan → **Meshy retexture** → remesh | Hunyuan 결과물 색상/고리 이슈 → retexture 보정 |
| **fish**  | ChatGPT(좌우 반전) → **Meshy 6 직접 생성** → remesh | Hunyuan에서 봉제선 문제 발생 → Meshy 단독 생성으로 우회 |

---

## 1단계: 이미지 전처리 (ChatGPT Image 2.0)

### bear / chick — 공통 프롬프트

```
image to 3d 할거라 너가 내가 지금 보낸 사진 편집좀 해줘
- 배경이 검은색
- 별도의 그림자 없음
- 위의 금색 체인은 없애되 인형 및 인형 고리는 건들지 말것
- 우측 하단의 made in china 태그 없애줘
- 별도의 빛이나 그림자 시뮬레이션하지 말아줘
만약 내가 보낸 사진이 이미 빛이 불규칙하게 받아있다면 완전 정면에서 빛은 받는 것처럼 모델을 다시 렌더링 해줘
위에 흰색(아이보리) 큰 체인 없애주고 인형에 있는 고리는 남겨줘
사진 약간 좌측에 있는 택 없애줘
```

### fish

원본 사진 1장을 좌우 반전해 앞면/뒷면 2장으로 활용 (대칭 구조).

---

## 2단계: 3D 모델 생성 (Hunyuan 3D 1.1)

전처리된 이미지를 Hunyuan에 입력 → 약 50,000 poly의 3D 모델 생성.
fish의 경우 봉제선 문제로 이 단계 스킵하고 Meshy 6로 직접 생성.

---

## 3단계: Remesh (Meshy 6)

Hunyuan/Meshy 결과물을 Meshy 6에서 remesh해 폴리곤 수 최적화.
chick은 remesh 전에 retexture 단계 추가.

---

## 폴더 구조

```
Image_to_3D/
├── Images/                # 입력 이미지
│   ├── raw_*.png          # 실제 촬영한 원본 사진
│   └── {obj}_N.png        # ChatGPT Image 2.0 전처리 완료 (1086x1448, RGB)
└── mesh/                  # 3D 모델
    ├── {obj}_v1.fbx       # remesh 전 원본
    └── {obj}_v2/          # Meshy remesh 완료본 (.fbx + 텍스처 PNG)
```

| 항목 | 의미 |
|------|------|
| `raw_*` | 실제 촬영 원본 |
| `{obj}_N.png` (Images/) | ChatGPT Image 2.0 편집본 |
| `{obj}_v1` | remesh 전 원본 mesh |
| `{obj}_v2` | Meshy remesh 완료본 |

> README 캡처 이미지는 `Simulation_Team/logs/media/` 에 있습니다.

---

## 오브젝트별 현황

| 오브젝트 | 원본 수 | 편집본 수 | 출처 |
|---------|--------|---------|------|
| bear  | 2 | 2 | [다이소몰](https://www.daisomall.co.kr/pd/pdr/SCR_PDR_0001?pdNo=1071420&recmYn=N) |
| chick | 2 | 2 | [다이소몰](https://www.daisomall.co.kr/pd/pdr/SCR_PDR_0001?pdNo=1045895&recmYn=Y) |
| fish  | 1 | 2 | [다이소몰](https://www.daisomall.co.kr/pd/pdr/SCR_PDR_0001?pdNo=10227271&recmYn=N) |

> fish는 좌우 반전으로 1장 → 2장 활용

---

## 결과물

### Bear

표준 흐름 (Hunyuan → Meshy remesh)으로 한 번에 성공.

| 단계 | Front | Back |
|------|-------|------|
| **Hunyuan 3D 1.1** ([Share Link](https://3d.hunyuanglobal.com/share?shareId=a303b45a-79a0-40c9-9f45-d8b20507a5a3)) | ![](../../logs/media/HY_bear_front.png) | ![](../../logs/media/HY_bear_back.png) |
| **Meshy 6 (remesh)** | ![](../../logs/media/Meshy6_bear.png) | — |

---

### Chick

Hunyuan 결과 이슈:
- 윗면에서 고리가 2개처럼 나옴
- 정면 눈/볼 색상이 원본과 다름

→ Meshy의 **retexture** 기능으로 보정 후 remesh.

| 단계 | View |
|------|------|
| **Hunyuan 3D 1.1** — 이슈 발생 ([Share Link](https://3d.hunyuanglobal.com/share?shareId=f8a46257-ad1b-48e1-86a8-cb1e6e830854)) | ![](../../logs/media/HY_chick_top-front.png) |
| **Meshy 6 v1** — retexture 보정 | ![](../../logs/media/Meshy6_chick_v1.png) |
| **Meshy 6 v2** — remesh 완료 | ![](../../logs/media/Meshy6_chick_v2.png) |

---

### Fish

Hunyuan 3D 1.1에서 정면/후면 봉제선이 크게 나타나는 문제 → 단일 이미지를 Meshy 6에 직접 입력해 생성 후 remesh.

| 단계 | View |
|------|------|
| **Hunyuan 3D 1.1** — 봉제선 문제로 미사용 ([Share Link](https://3d.hunyuanglobal.com/share?shareId=d5474aad-1575-4a72-b9a4-92648ccf2cc0)) | ![](../../logs/media/HY_fish_front.png) |
| **Meshy 6 v1** — 직접 생성 (200,000 poly) | ![](../../logs/media/Meshy6_fish_v1.png) |
| **Meshy 6 v2** — remesh 완료 | ![](../../logs/media/Meshy6_fish_v2.png) |
