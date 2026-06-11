# TSDF Pipeline 따라가기 — 수학 약해도 OK

> **이 문서가 누구를 위한 건가**: `backend/modules/pointcloud/tsdf_builder.py`의 `build_mesh()`가 무슨 짓을 하는지 *코드 한 줄 → 알고리즘 → 수학*까지 위에서 아래로 내려가며 이해하고 싶은 사람.
>
> 수학 표기 최소화. 직관 + 그림 + 구체적 숫자 → 그다음 수식. 매 챕터 끝에 `tsdf_builder.py`의 정확한 줄 번호.
>
> **읽는 법**: 위에서부터 순서대로. 한 챕터 다 이해 안 됐으면 다음으로 안 넘어가도 OK — 어차피 뒤가 앞에 의존함.
>
> **사전 지식**: 행렬 곱(A·B)이 뭔지 정도. "행렬은 무서운 것"이라는 인식만 깨면 나머지는 따라옴.

---

## 0. 전체 한눈에

`build_mesh(scans, ...)`는 *여러 자세에서 찍은 RGBD를 합쳐 mesh로 만드는 함수*. 8단계로 나뉘어 있음.

| # | 단계 이름 | 한 줄 요약 | 챕터 |
|---|----------|----------|------|
| 1 | **자세별 초기 변환** | "이 RGBD는 base 좌표계 기준으로 어디서 찍힌 거야?"를 FK로 계산 | §1 |
| 2 | **Depth 평활화** | 노이즈 줄이되 edge는 보존 | §2 (가볍게) |
| 3 | **RGBD → 점군 + normal** | (u, v, depth) 픽셀을 (X, Y, Z) 3D 점으로 펴고, 표면 방향 추정 | §3, §4 |
| 4 | **Pair-wise ICP** | 두 점군이 살짝 어긋나 있으면 한쪽을 조금 회전·이동시켜 맞춤 | §5 |
| 5 | **PoseGraph + global optimization** | 여러 ICP 결과를 그래프로 묶고, 누적 오차를 한 번에 풂 | §6 |
| 6 | **TSDF integrate** | 3D 공간을 voxel grid로 쪼개고 각 voxel에 "표면까지 거리"를 누적 | §7 |
| 7 | **Mesh 추출 + 정리** | voxel grid에서 표면(=distance가 0인 면)을 삼각형 mesh로 뽑음 | §8 |
| 8 | **PLY 저장** | 파일로 떨굼 | (생략) |

이 표를 한 번 머리에 박고 시작하자. 다음 챕터부터는 *수학 챕터 → 그게 코드 어디?* 패턴.

---

## 1. 좌표계와 변환행렬 — 모든 것의 문법

`build_mesh`의 첫 작업은 자세별로 이런 줄을 만드는 거:

```python
T_bc = T_base_ee @ T_ee_cam      # tsdf_builder.py:115
```

이 한 줄이 안 보이면 뒤가 전부 안 보임. 그래서 이 챕터는 길다. 천천히 가자.

### 1.1 좌표계는 "관찰자"다

같은 점이라도 누가 보냐에 따라 숫자가 달라짐.

- 책상 위 컵이 있다.
- **로봇 base에서 보면**: (0.3, 0.2, 0.1) m (앞 30cm, 옆 20cm, 위 10cm)
- **카메라에서 보면**: (0.05, -0.02, 0.4) m (카메라 정면 40cm, 약간 오른쪽 아래)

물리적 컵은 하나인데 좌표는 두 개. 좌표계가 "측정 기준"이라서 그래.

OMX_F에는 좌표계가 여러 개 있다:

```
base  ─→  shoulder  ─→  upper  ─→  fore  ─→  wrist  ─→  ee  ─→  cam
                    (조인트 1~5)                      (hand_eye)
```

- `base`: 로봇 받침
- `ee`: 그리퍼 끝
- `cam`: D405 카메라 광학 중심
- 사이의 조인트들은 FK가 자동으로 처리

### 1.2 한 점이 좌표계에 따라 다른 숫자

수학적으로 "좌표계 A에서 본 점 p"를 `p_A`로 적자. 같은 물리적 점이 좌표계 B에서는 `p_B`. 둘은 다름.

이 둘을 잇는 게 **변환행렬**:

```
p_A = T_A_B · p_B
```

읽는 법: "T_A_B는 B 좌표를 A 좌표로 바꿔주는 행렬." 첨자 순서가 `목적지_출발지`야. 이 컨벤션은 OMX_F 코드 전체에서 일관됨.

> **혼동 주의**: Open3D는 같은 변환을 `T_target←source`로 표기. 화살표가 반대 방향처럼 보이지만 의미는 같음. `T_target←source`는 "source 점을 target 좌표로 옮긴다"는 뜻이라 `T_target_source`와 동의어.

### 1.3 회전 + 이동 = SE(3)

두 좌표계가 다른 이유는 두 가지뿐:
1. **회전**: 축 방향이 다름 (카메라가 비스듬 등)
2. **평행이동**: 원점 위치가 다름

이 둘을 합친 게 SE(3) (3차원 강체 변환 그룹). 회전을 3×3 행렬 `R`, 이동을 3-벡터 `t`로 쓰면:

```
p_A = R · p_B + t
```

이게 핵심. 점을 회전시킨 후 옮긴다. 끝.

### 1.4 4×4 행렬: homogeneous transform

위 식을 매번 `R·p + t`라고 쓰면 합성이 귀찮음. 그래서 트릭:

```
        [ R₃ₓ₃   t₃ₓ₁ ]
T_A_B = [             ]
        [  0  0  0  1 ]
```

4×4 행렬. 그리고 점 `p`도 4-벡터로:

```
p_homogeneous = [pₓ, pᵧ, p_z, 1]ᵀ
```

이러면 변환이 그냥 행렬 곱:

```
[p_A]   [ R  t ] [p_B]
[ 1 ] = [ 0  1 ] [ 1 ]
```

마지막 1을 무시하면 정확히 `R·p + t`가 나옴. (직접 손으로 계산해 봐.)

**왜 4×4가 좋은가**: 변환의 합성이 그냥 행렬 곱이 돼서. 이게 다음 절의 핵심.

### 1.5 합성: T_base_cam = T_base_ee · T_ee_cam

코드의 그 한 줄:

```python
T_bc = T_base_ee @ T_ee_cam      # tsdf_builder.py:115
```

읽는 법:

- `T_ee_cam`: 카메라 좌표 → ee 좌표
- `T_base_ee`: ee 좌표 → base 좌표
- 곱하면: 카메라 좌표 → ee 좌표 → base 좌표 = **카메라 좌표 → base 좌표** = `T_base_cam`

**첨자가 인접한 위치에서 같아야 한다** — 이게 합성의 문법. `T_a_b · T_b_c = T_a_c`. 가운데 `b`가 소거되는 것처럼 외움. 안 맞으면 곱하면 안 됨(정확히는 곱해도 의미가 안 통함).

### 1.6 역행렬: 방향 뒤집기

변환을 거꾸로 가고 싶으면 `inv(T)`. `T_A_B`의 역은 `T_B_A`. 4×4의 inverse는 일반적으로 비싸지만, SE(3) 행렬은 다음 공식으로 *빨리* 계산됨:

```
T_A_B = [ R  t ]    →   T_B_A = [ Rᵀ  -Rᵀt ]
        [ 0  1 ]                [  0    1  ]
```

회전은 transpose, 이동은 -Rᵀ·t. (실제 코드는 그냥 `np.linalg.inv`를 쓰지만 머리속에선 위 식.)

코드에서 이게 나오는 곳:

```python
extrinsic = np.linalg.inv(T_base_cam_refined[i])    # tsdf_builder.py:244
```

`T_base_cam`의 역은 `T_cam_base`. Open3D의 `volume.integrate`가 `T_cam←base`를 요구해서 inv 한 번 박는 거임.

### 1.7 코드에서 어디?

[tsdf_builder.py:99-116](backend/modules/pointcloud/tsdf_builder.py#L99-L116) 자세히 보자:

```python
for idx, s in enumerate(scans):
    raw_positions = s["raw_motor_positions"]      # 모터 raw 값 (정수)
    scan_arm_ids = s["arm_motor_ids"]
    arm_rad: list[float] = []
    for raw, mid in zip(raw_positions, scan_arm_ids):
        cfg = cfg_by_id.get(int(mid))
        arm_rad.append(coords.motor_to_urdf(int(raw), cfg))   # raw → URDF rad

    R_be, t_be = solver.fk_to_matrix(arm_rad)     # FK: 조인트각 → ee pose
    T_base_ee = np.eye(4)
    T_base_ee[:3, :3] = np.asarray(R_be)          # R 채우고
    T_base_ee[:3, 3] = np.asarray(t_be)           # t 채우고 → 4×4 완성
    T_bc = T_base_ee @ T_ee_cam                   # § 1.5
    T_base_cam_init.append(T_bc)
```

흐름:
1. scan에 박혀 있는 raw motor 값(0~4095 정수)을 라디안으로 변환 → `arm_rad`
2. PybulletSolver의 FK가 조인트각을 받아 ee의 위치(t)와 회전(R)을 돌려줌
3. R과 t를 4×4 안에 채워 `T_base_ee` 완성
4. hand_eye 캘 결과 `T_ee_cam`을 곱해 `T_base_cam` 완성

이게 자세별로 한 번씩, 총 `n` 번 실행. 결과는 `T_base_cam_init` 리스트 — *각 자세의 카메라가 base 좌표계에서 어디에 있는지의 초기 추정*. "초기"인 이유는 §6의 ICP/PoseGraph가 이걸 더 정확하게 refining 하기 때문.

**Self-check**: `T_ee_cam`의 첨자가 왜 `ee_cam`이지 `cam_ee`가 아닌가? — 답: 카메라 좌표를 ee 좌표로 옮기는 변환이라 그래. hand_eye 캘이 푸는 것이 정확히 이거 (`T_cam2gripper` = `T_ee←cam`).

---

## 2. Depth 평활화 — 짧게

```python
depth_filtered = cv2.bilateralFilter(depth_f, ...)   # tsdf_builder.py:120-125
```

**bilateral filter**는 픽셀 평균을 내는데:
- **공간적으로 가까운** 픽셀에 가중치 (= 가우시안 블러)
- **값이 비슷한** 픽셀에만 가중치 (= edge 보존)

D405 stereo depth는 노이즈가 픽셀별로 흩어져 있어서 평균이 좋음. 그러나 단순 가우시안 블러를 쓰면 물체 경계가 뭉개짐. bilateral은 두 번째 조건 덕에 *경계는 살리면서* 노이즈만 줄임.

수학으로 들어가지 않아도 됨. "edge 보존 평활화"라고만 알면 OK. ICP/TSDF 정밀도에 1mm 단위 영향을 주는 자잘한 정제 단계.

**코드에서 어디?**: [tsdf_builder.py:118-125](backend/modules/pointcloud/tsdf_builder.py#L118-L125).

---

## 3. 카메라 모델 — 픽셀이 3D 점이 되는 마법

```python
rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(...)    # :128
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)  # :141
```

이 두 줄이 *2D 이미지를 3D 점군으로 펴는* 작업. 어떻게?

### 3.1 핀홀 모델: 3D → 2D

카메라는 빛이 작은 구멍을 통과해 뒤판에 맺히는 장치. 수학으로는:

```
u = fx · X/Z + cx
v = fy · Y/Z + cy
```

- `(X, Y, Z)`: 카메라 좌표계 기준 3D 점 (m 단위, Z는 광학축 앞쪽)
- `(u, v)`: 이미지 픽셀 좌표
- `fx, fy`: 초점거리 (focal length, 픽셀 단위)
- `cx, cy`: principal point (광학축이 이미지에 닿는 픽셀)

직관:
- Z가 멀수록 `X/Z`가 작아져서 화면 중앙에 가까워짐 (멀리 있으면 작게 보임)
- `fx, fy`가 크면 같은 X가 더 큰 픽셀 변화로 → 좁은 화각 (망원)

이 4개 숫자(`fx, fy, cx, cy`)를 **intrinsic**이라 부름. D405 같은 카메라는 공장 출하 시 측정되어 있고, 코드에서는 scan npz 안에 박혀 옴.

### 3.2 거꾸로: 픽셀 + depth → 3D 점

위 식을 Z에 대해 푼다고 생각하자. Z가 있으면 (depth 카메라가 주잖아):

```
X = (u - cx) · Z / fx
Y = (v - cy) · Z / fy
Z = depth_value · depth_scale          # uint16을 미터로
```

매 픽셀마다 이걸 돌리면 (H × W)개의 점이 나옴. 이게 **점군 (point cloud)** 의 정의. Open3D의 `create_from_rgbd_image`가 이 짓을 내부적으로 해주는 것.

색상은 같은 픽셀의 RGB가 그 점의 색이 됨.

### 3.3 결과: 카메라 좌표계의 점군

여기서 만든 점군은 **카메라 좌표계** 기준. 즉 카메라 광학 중심이 (0,0,0), Z축이 카메라가 보는 방향. 같은 물체를 다른 자세에서 찍었으면 점군 모양은 비슷하지만 좌표값은 완전 다름 — 각자 자기 카메라 기준이라서.

이 점군들을 같은 base 좌표계로 모으는 게 §6 TSDF의 일.

### 3.4 코드에서 어디?

[tsdf_builder.py:127-141](backend/modules/pointcloud/tsdf_builder.py#L127-L141):

```python
color_rgb = np.ascontiguousarray(s["color_bgr"][:, :, ::-1])   # BGR→RGB
rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d.geometry.Image(color_rgb),
    o3d.geometry.Image(depth_filtered),
    depth_scale=1.0 / s["depth_scale"],          # uint16 → meter
    depth_trunc=depth_trunc,                     # 0.5m 넘는 점 버림
    convert_rgb_to_intensity=False,
)
intrinsic = o3d.camera.PinholeCameraIntrinsic(
    s["width"], s["height"], s["fx"], s["fy"], s["cx"], s["cy"]
)
# ...
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
```

핵심:
- `intrinsic`이 fx/fy/cx/cy를 묶은 객체
- `RGBDImage`는 color + depth + intrinsic 메타를 묶은 컨테이너
- `create_from_rgbd_image`가 §3.2 식을 모든 픽셀에 적용

이 시점에서 `pcd`는 *카메라 좌표계의* 점군. base 좌표계로 옮기려면 §1의 `T_base_cam`을 곱하면 됨 — 그런데 코드는 ICP/PoseGraph 끝나고 TSDF 안에서 한 번에 통합하니까 여기선 안 옮김.

---

## 4. Normal — 점이 가진 방향

```python
pcd_down.estimate_normals(...)    # tsdf_builder.py:143-147
```

ICP point-to-plane을 쓰려면 점마다 *법선 벡터(normal)* 가 있어야 함. Normal이 뭐고 왜 필요한지 짧게.

### 4.1 Normal이란

점이 표면 위에 놓여 있다고 가정하자. 그 표면의 그 점에서의 "수직 방향"이 normal. 길이 1짜리 3-벡터.

```
   ↑ normal (위로 향함)
   │
═══●═══   ← 책상 표면, 점은 표면 위
```

벽의 점은 normal이 옆을 향함. 컵 옆면의 점은 normal이 컵 바깥쪽으로 향함.

### 4.2 왜 필요한가

§5에서 자세히 — 미리보기: ICP가 "이 점 두 개의 거리를 줄이자"가 아니라 "이 점이 *반대 점의 평면*에서 얼마나 떨어졌나"를 줄이는 게 훨씬 빠르고 정확함. 그 평면을 정의하려면 normal이 필요.

### 4.3 어떻게 추정하나 (직관)

각 점 주변 30개 이웃을 모음. 그 30개 점이 *대략 평면*을 이루면, 그 평면의 수직 방향이 normal. 수학적으로는 **PCA** (주성분 분석): 30개 점의 공분산 행렬의 *가장 작은 고유값에 해당하는 고유벡터*가 normal.

PCA를 모르면 그냥 "이웃 점들이 가장 얇게 펼쳐진 방향"이라고 외워. 사실 그게 PCA의 직관임.

### 4.4 코드에서 어디?

[tsdf_builder.py:142-148](backend/modules/pointcloud/tsdf_builder.py#L142-L148):

```python
pcd_down = pcd.voxel_down_sample(voxel_size)    # 2mm 격자로 다운샘플 (ICP 속도)
pcd_down.estimate_normals(
    o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel_size * 2.0,      # 4mm 안의 이웃을
        max_nn=30,                    # 최대 30개 모아 PCA
    )
)
```

`voxel_down_sample`: 2mm 격자 안에 점이 여러 개 있으면 평균 점 1개로 합침. 점이 수십만 개 → 수만 개로 줄어 ICP가 빨라짐. **TSDF integrate에는 원본 RGBD를 쓰지** 이 다운샘플 점군을 안 쓴다 (Open3D가 RGBDImage 자체를 받음). 다운샘플은 ICP 전용.

---

## 5. ICP — 두 점군 정렬

`build_mesh`의 가장 무거운 수학. 천천히 가자.

### 5.1 문제

두 점군 P와 Q가 있다. 거의 같은 물체를 다른 자세에서 찍은 거라 *모양은 비슷한데 위치/회전이 살짝 어긋남*. P를 어떻게 회전·이동(`T`)시키면 Q와 가장 잘 겹칠까?

수식으로:

```
T* = argmin  Σᵢ "i번째 점의 어긋남 정도"
      T∈SE(3)
```

여기서 "어긋남 정도"를 어떻게 정의하느냐가 ICP 변종 둘:
- **point-to-point**: 점-점 거리의 제곱합
- **point-to-plane**: 점이 반대 점의 평면에서 얼마나 떨어졌나의 제곱합

### 5.2 Point-to-point (이해를 위해)

```
T* = argmin  Σᵢ ‖pᵢ − T·qᵢ‖²
      T
```

- `qᵢ`: source 점군의 i번째 점
- `pᵢ`: 그것과 짝지어진 target 점
- `‖·‖²`: 길이의 제곱

직관: 각 source 점을 T로 옮긴 결과와 target 점 사이 거리의 제곱을 모두 더해 최소화.

**문제 둘**:
1. *짝짓기*: 누가 누구의 짝인지 어떻게 알아? 두 점군은 그냥 점 더미인데.
2. *비선형*: T가 회전을 포함하니까 sin/cos가 들어가 closed-form이 안 됨 (잠깐, 사실 점-점은 SVD로 닫힌 해가 있긴 함. 다음 절에서.)

ICP의 핵심 아이디어 — **이걸 번갈아 하자**:

```
반복:
  1. 현재 T로 source를 옮긴다.
  2. 옮긴 source의 각 점에 대해, target에서 가장 가까운 점을 짝으로 본다.
  3. 그 짝짓기를 고정하고 T를 새로 푼다.
  → T 갱신
수렴할 때까지.
```

"Iterative Closest Point" 이름이 여기서 옴. 매 iteration의 짝짓기는 KD-tree로 가장 가까운 점 검색.

### 5.3 Point-to-plane (실제 코드가 쓰는 것)

target 점에는 normal이 있다 (§4). 그러면 target 점이 그 자체로 평면을 정의:
- 평면 위의 점: `pᵢ`
- 평면 방향: `nᵢ` (normal)

source 점을 T로 옮긴 결과 `T·qᵢ`가 그 평면에서 얼마나 떨어져 있나? — **점에서 평면까지 거리 공식**: 점에서 평면 위의 점까지 벡터를, 평면 normal에 *투영*한 길이.

```
거리 = nᵢ · (pᵢ − T·qᵢ)
```

`·`는 내적. 그래서 잔차:

```
T* = argmin  Σᵢ ( nᵢ · (pᵢ − T·qᵢ) )²
      T
```

**왜 더 좋은가**:
- 표면 위에서 *옆으로* 미끄러지는 건 평면 거리에 영향이 없음 → 표면을 따라 매끄럽게 미끄러질 수 있음
- 점-점은 모든 방향 거리를 똑같이 처벌해서, 옆으로 어긋난 짝 때문에 잘못 끌릴 수 있음
- 결과: 적은 iteration으로 더 정확하게 수렴, 평면 많은 scene(책상, 벽)에서 특히 우수

### 5.4 SE(3) 위의 최소제곱 — 비선형 산 내려가기

위 잔차를 0으로 만드는 T를 찾는 게 ICP의 한 step. T는 SE(3) 위의 점 (=4×4 행렬인데 회전 제약 있음). 일반적인 최소제곱은 안 풀리고 **비선형 최소제곱**을 써야 함.

직관: 함수값을 줄이는 방향으로 한 스텝씩 가는 게 **gradient descent**. ICP는 그것의 사촌 **Gauss-Newton** 또는 **Levenberg-Marquardt (LM)** 를 씀. 산을 내려가되, 곡률 정보까지 써서 더 영리하게 큰 보폭으로.

핵심 아이디어 (한 번만 들어두고 외우진 않아도 됨):
- 잔차 함수 `r(T)`가 있다. 이걸 줄이고 싶다.
- 현재 `T`에서 잔차를 *T를 살짝 움직이는 변수* `ξ` (6차원)로 *선형근사*한다.
- 선형근사된 잔차의 최소를 찾는 건 일반 최소제곱 (= 한 번의 행렬 계산).
- 그 ξ만큼 T를 갱신한다.
- 다시 선형근사 … 반복.

**왜 ξ는 6차원인가**: 회전 3 + 이동 3 = 6. 회전을 직접 행렬로 다루면 9개 변수에 직교 제약이 붙어 골치 아픔. 작은 회전을 *축×각도* (axis-angle, 3-벡터)로 매개변수화하면 제약 없는 3개 변수. 이게 *Lie algebra* (so(3)/se(3))의 이름이 붙는 이유. 지금은 "회전+이동을 6개 숫자로 표현" 정도만 알면 됨.

### 5.5 짝짓기 + 인접 자세 + loop closure

코드는 *두 점군씩 짝지어* ICP를 돌린다 — `tsdf_builder.py:185-204`:

```python
# 인접 페어 (i, i+1) — 자세 순서대로
for i in range(n - 1):
    out = _run_icp(i, i + 1)
    if out: edges.append((i + 1, i, T_ij, info, False))

# 비인접 페어 (i, j>i+1) — 카메라 거리가 가까우면 loop closure 후보
for i in range(n):
    for j in range(i + 2, n):
        dist = float(np.linalg.norm(cam_centers[i] - cam_centers[j]))
        if dist > PAIR_UNCERTAIN_DIST: continue
        out = _run_icp(i, j)
        # ...
```

왜 두 종류:
- **인접 페어 (uncertain=False)**: 시간상 순서대로 (i, i+1). 거의 항상 잘 정합돼서 신뢰도 높음.
- **Loop closure (uncertain=True)**: 시간상 멀리 떨어진 두 자세인데 *공간상* 가까운(카메라 위치가 비슷한) 페어. 같은 부분을 다른 각도에서 본 거라 정합되면 좋고, 안 되면 버림. 누적 드리프트를 잡는 핵심.

### 5.6 코드에서 어디?

[tsdf_builder.py:152-204](backend/modules/pointcloud/tsdf_builder.py#L152-L204) 전체가 ICP 단계.

핵심 한 호출:

```python
result = o3d.pipelines.registration.registration_icp(
    source=pcds_down[j],                             # 옮길 점군
    target=pcds_down[i],                             # 기준 점군
    max_correspondence_distance=icp_max_dist,        # 10mm 안에서만 짝짓기
    init=T_init,                                     # T_cam_i ← cam_j 초기값
    estimation_method=
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
)
```

- `source` / `target`: 누구를 누구에게 맞출지. 결과 `result.transformation`은 `T_target←source` = `T_cam_i ← cam_j`.
- `max_correspondence_distance`: KD-tree 짝짓기 시 이 거리 이상은 짝으로 안 봄. 캘 σ_t ~8mm를 고려해 10mm로 잡음.
- `init`: 시작 T. §1에서 만든 `T_base_cam_init`으로부터 유도 (`tsdf_builder.py:160`):
  ```python
  T_init = np.linalg.inv(T_base_cam_init[i]) @ T_base_cam_init[j]
  ```
  = `T_cam_i ← base · T_base ← cam_j` = `T_cam_i ← cam_j`. §1.5의 합성 문법.

`information matrix`: §6에서 다룸 (PoseGraph가 신뢰도로 쓰는 6×6 행렬).

---

## 6. PoseGraph — 누적 오차 잡기

여러 ICP가 끝나면 *pair-wise* 변환들이 잔뜩 쌓임. 이걸 어떻게 *전역적*으로 일관되게 만들까? 답은 **graph optimization**.

### 6.1 왜 pair-wise만으론 부족한가

자세 0, 1, 2가 있다. ICP가:
- T_01: cam_0 ← cam_1
- T_12: cam_1 ← cam_2
- T_02: cam_0 ← cam_2 (loop closure)

라고 줬다. 이상적이라면:

```
T_01 · T_12 = T_02     (§1.5 합성)
```

근데 실제로는 ICP 오차 때문에 *불일치*. `T_01 · T_12 ≠ T_02`. 이 불일치를 어디에 떨어뜨릴지가 문제.

Pair-wise는 그 결정을 *못 내림* — 그냥 셋 다 들고 있을 뿐. **PoseGraph**가 이걸 해결.

### 6.2 그래프로 표현

- **노드(node)**: 자세 `i`의 `T_base ← cam_i` (= 카메라 i의 base 좌표계 pose)
- **엣지(edge)**: ICP로 측정한 두 자세 사이 상대 변환 `T_cam_i ← cam_j`

```
[cam_0] ─T_01─ [cam_1] ─T_12─ [cam_2]
   └────────T_02 (loop)──────────┘
```

총 `n`개 노드, 그보다 많은 엣지 (인접 n-1개 + 가까운 loop closure 몇 개).

### 6.3 잔차: 측정 vs 추정의 차이

각 엣지마다 잔차를 정의:

```
잔차_ij = log( T_ij_meas⁻¹ · (T_base←cam_i)⁻¹ · T_base←cam_j )
```

읽는 법:
- `T_base←cam_i` / `T_base←cam_j`: 현재 *추정*된 두 노드 pose
- 추정에서 유도된 *예측* 상대변환 = `(T_base←cam_i)⁻¹ · T_base←cam_j` = `T_cam_i ← cam_j`
- 측정값 `T_ij_meas`와 비교
- 두 SE(3)의 "차이"는 `T_meas⁻¹ · T_pred`로 하나의 SE(3)을 만들고, 그걸 `log`로 6-벡터로 풀어내림 (Lie algebra)

잔차 6-벡터가 0이면 측정과 예측이 일치, 0에서 멀수록 어긋남. 모든 엣지의 잔차 제곱합을 최소화하는 게 목표:

```
{T_base←cam_i}* = argmin  Σ_(i,j 엣지)  잔차_ij ᵀ · Ω_ij · 잔차_ij
                    {T}
```

`Ω_ij`는 **information matrix** — 다음 절.

### 6.4 Information matrix — 엣지의 신뢰도

엣지마다 6×6 행렬. "이 엣지가 얼마나 신뢰할 만한가"를 잔차 차원별로 표현. 잔차에 information으로 가중치를 줘서, 신뢰 안 가는 엣지는 덜 끌어당기도록.

직관: covariance의 역. covariance가 크면 (불확실하면) information 작음 → 덜 처벌. 반대도 마찬가지.

Open3D는 `get_information_matrix_from_point_clouds()`로 자동 계산 — ICP가 끝난 후 *어느 점들이 짝지어졌나*를 보고 6 자유도 각각의 제약 정도를 추산. 평면적 scene이면 평면 방향 information은 작음 (옆으로 미끄러져도 잘 모름).

### 6.5 Levenberg-Marquardt가 푸는 것

위 비용함수는 SE(3) 위의 비선형 최소제곱. ICP와 *같은 류*의 문제 — 그래서 같은 도구(LM)가 풂. 차이는 *변수의 개수*뿐:
- ICP: 1개 T (6 DOF)
- PoseGraph: `n`개 T (6n DOF) — 그래서 `n=10`이면 60 DOF

LM의 매 step:
1. 현재 추정에서 잔차를 *선형근사*.
2. 선형 시스템 풂 → 모든 노드 pose에 대한 갱신 δξ_i (6-벡터 × n).
3. 노드 갱신. 잔차가 줄었으면 step 받아들이고, 안 줄었으면 step 크기 줄여 재시도.
4. 수렴까지 반복.

(`reference_node=0` 옵션 — 첫 번째 노드 pose를 *고정*하지 않으면 전체가 자유롭게 떠다닐 수 있어서 해가 안 정해짐. anchor.)

### 6.6 코드에서 어디?

[tsdf_builder.py:207-235](backend/modules/pointcloud/tsdf_builder.py#L207-L235):

```python
pose_graph = o3d.pipelines.registration.PoseGraph()
for T in T_base_cam_init:
    pose_graph.nodes.append(
        o3d.pipelines.registration.PoseGraphNode(T.copy())   # 초기 노드 pose
    )
for src, tgt, T_ts, info, uncertain in edges:
    pose_graph.edges.append(
        o3d.pipelines.registration.PoseGraphEdge(
            source_node_id=src,
            target_node_id=tgt,
            transformation=T_ts,                              # 측정값
            information=info,                                 # 가중치
            uncertain=uncertain,                              # loop closure?
        )
    )

option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=icp_max_dist,
    edge_prune_threshold=0.25,                               # 잔차 큰 엣지는 버림
    reference_node=0,                                        # anchor
)
o3d.pipelines.registration.global_optimization(
    pose_graph,
    o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
    o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
    option,
)

T_base_cam_refined = [pose_graph.nodes[i].pose for i in range(n)]
```

`pose_graph.nodes`에 초기 추정을 박고, `pose_graph.edges`에 측정 + information을 박고, `global_optimization`이 LM을 돌려 노드 pose를 갱신. 끝나면 `pose_graph.nodes[i].pose`가 *정합된* `T_base←cam_i`.

여기까지가 ICP+PoseGraph로 캘리브레이션 floor를 *mm 단위로 끌어내리는* 메커니즘. 다음 §7부터는 점군이 아니라 *부피*를 다룸.

---

## 7. TSDF — 점들을 표면으로

```python
volume = o3d.pipelines.integration.ScalableTSDFVolume(...)    # :238
for i in range(n):
    extrinsic = np.linalg.inv(T_base_cam_refined[i])
    volume.integrate(rgbds[i], intrinsics[i], extrinsic)      # :245
```

이 7줄이 *여러 RGBD를 하나의 부피 표현으로 합치는* 작업. 마법처럼 보이는데 분해해 보면 별 거 아님.

### 7.1 SDF (Signed Distance Function)

3D 공간 위의 각 점 `x`에 대해 "가장 가까운 표면까지 거리"를 함수로 정의. **부호** 붙임:
- 표면 *바깥*: + (양수)
- 표면 *위*: 0
- 표면 *안쪽*: − (음수)

```
바깥 +
─────●───── 표면 (SDF=0)
안쪽 −
```

물체 하나에 대해 SDF가 정해지면 그 함수의 *zero level set* (= SDF가 0인 모든 점)이 곧 표면. 즉 **SDF가 정해지면 표면이 정해진다**.

### 7.2 Voxel grid

연속적인 함수를 컴퓨터로 다루려면 이산화. 3D 공간을 작은 cube(voxel)로 쪼개고, 각 voxel에 SDF 값 하나를 저장. OMX_F는 voxel 크기 2mm (= `DEFAULT_VOXEL_SIZE`).

책상 위 30cm × 30cm × 30cm 작업 공간이면 voxel 약 (150)³ ≈ 3.4M개. `ScalableTSDFVolume`은 *비어 있는 voxel은 메모리 안 잡고* hash로 관리해서 실제 메모리는 훨씬 적음.

### 7.3 Truncated의 의미

표면에서 *멀리* 떨어진 voxel은 SDF 값을 정확히 알 필요 없음 (어차피 거기 표면은 없으니까). 그래서 거리값을 `±sdf_trunc` (10mm)로 *잘라* 저장 — 그게 "Truncated" SDF.

이점:
- 메모리/계산 절감
- 멀리 떨어진 voxel이 표면 추정에 영향 못 주게 함

### 7.4 카메라 ray로 voxel 업데이트 (한 자세)

`volume.integrate(rgbd, intrinsic, extrinsic)`이 한 자세분 통합을 한다. 내부적으로:

```
for 각 voxel v in 카메라 시야:
    1. v를 카메라 좌표로 변환 (extrinsic = T_cam←world 사용)
    2. v를 이미지에 투영 (intrinsic 사용) → 픽셀 (u, v)
    3. depth 이미지에서 그 픽셀의 측정 거리 d_meas 읽음
    4. voxel의 카메라 Z 좌표 d_voxel 계산
    5. SDF 추정값:  sdf_obs = d_meas - d_voxel
       - d_voxel < d_meas  → voxel이 표면보다 카메라 쪽 → 바깥 → +
       - d_voxel > d_meas  → voxel이 표면 뒤 → 안쪽 → −
       - d_voxel ≈ d_meas  → 표면 위
    6. ±sdf_trunc로 자름
    7. voxel의 기존 SDF 값과 weighted average:
          sdf_new = (w_old · sdf_old + w · sdf_obs) / (w_old + w)
          w_new   = w_old + w
       색상도 같은 방식.
```

자세 i 통합이 끝나면 voxel grid에는 *그 자세의 시야에 들어온 voxel들*에 대해 SDF가 갱신돼 있음. 다음 자세 통합 시 weighted average로 더 많은 정보 합쳐짐.

**왜 weighted average가 답인가**:
- 한 자세 단독 측정은 노이즈가 많음
- N 자세의 측정을 평균하면 노이즈 √N배 감소
- TSDF가 N개의 RGBD를 *합치는* 메커니즘이 정확히 이 가중 평균
- ICP/PoseGraph가 자세 정합을 mm로 끌어왔으니, 평균 결과가 깔끔한 표면을 그림

### 7.5 코드에서 어디?

[tsdf_builder.py:237-245](backend/modules/pointcloud/tsdf_builder.py#L237-L245):

```python
volume = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=voxel_size,           # 2mm
    sdf_trunc=sdf_trunc,               # 10mm
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
)
for i in range(n):
    extrinsic = np.linalg.inv(T_base_cam_refined[i])    # T_cam←base
    volume.integrate(rgbds[i], intrinsics[i], extrinsic)
```

`extrinsic`이 `T_cam←base`인 이유 — Open3D 내부 ray-casting이 *world 좌표인 voxel을 카메라 좌표로 보내려면* `T_cam←world` 변환이 필요해서. §1.6의 inverse.

자세 `n`개 통합이 끝나면 `volume` 안에 voxel grid가 채워져 있음. 아직 mesh는 아님 — 다음 §8에서.

---

## 8. Marching Cubes — 표면 뽑기

```python
mesh = volume.extract_triangle_mesh()    # tsdf_builder.py:248
```

이 한 줄이 voxel grid에서 mesh를 뽑는다. 알고리즘 이름은 **marching cubes**.

### 8.1 Zero-level set 추출 직관

voxel grid의 각 voxel 모서리에 SDF 값이 있음. 어떤 voxel의 8개 모서리 중 *일부는 +, 일부는 −* 이면, 그 voxel 내부 어딘가에서 SDF가 0인 면이 지나간다.

```
+ ─── +              표면이 이 voxel을 안 지남
│     │
+ ─── +

+ ─── −              표면이 이 voxel을 가로지름
│     │              → 삼각형 1~4개로 근사
+ ─── +
```

Marching cubes는 voxel을 순회하며 각 voxel의 8개 모서리 부호 패턴을 보고 (총 2⁸ = 256가지) *미리 계산된 lookup table*에서 어떤 삼각형을 그릴지 결정. 그 삼각형의 정점 위치는 모서리 위에서 SDF가 0이 되는 곳을 *선형 보간*으로 찾음.

수학은 사실상 부호 패턴 매칭 + 선형 보간. 어렵지 않음.

### 8.2 Cluster cleanup

```python
cluster_ids, cluster_sizes, _ = mesh.cluster_connected_triangles()
small_clusters = np.where(cluster_sizes < MIN_TRIANGLE_CLUSTER_SIZE)[0]
# ... 작은 cluster의 삼각형 제거 ...
```

추출된 mesh에 *떠 있는 작은 조각*들이 섞여 있을 수 있음 (depth 노이즈 + outlier가 만든 잘못된 표면). **Connected components** — 인접한 삼각형끼리 그룹화 — 후 작은 그룹을 버림.

`MIN_TRIANGLE_CLUSTER_SIZE = 500`: 삼각형 500개 미만 그룹은 버림. 그래서 메인 mesh만 남음.

### 8.3 코드에서 어디?

[tsdf_builder.py:247-263](backend/modules/pointcloud/tsdf_builder.py#L247-L263):

```python
mesh = volume.extract_triangle_mesh()       # marching cubes
mesh.compute_vertex_normals()               # vertex별 normal 다시 계산 (rendering용)

cluster_ids, cluster_sizes, _ = mesh.cluster_connected_triangles()
cluster_ids = np.asarray(cluster_ids)
cluster_sizes = np.asarray(cluster_sizes)
if len(cluster_sizes) > 0:
    small_clusters = np.where(cluster_sizes < MIN_TRIANGLE_CLUSTER_SIZE)[0]
    if len(small_clusters) > 0:
        triangle_mask = np.isin(cluster_ids, small_clusters)
        mesh.remove_triangles_by_mask(triangle_mask)
        mesh.remove_unreferenced_vertices()

out_path.parent.mkdir(parents=True, exist_ok=True)
o3d.io.write_triangle_mesh(str(out_path), mesh, write_vertex_normals=True)
```

PLY로 저장 후 끝. 프론트엔드 `MeshLayer.tsx`가 이 PLY를 읽어 three.js로 띄움.

---

## 9. 정리 — 전체 흐름 다시 한번

이제 처음으로 돌아가서 빠르게 산책. 이쯤이면 한 줄씩 의미가 잡혀야 함.

```
build_mesh(scans, arm_cfgs, out_path):

  # §1: 자세별 초기 T_base_cam 계산
  for each scan s:
      arm_rad = motor_raw → URDF rad           (joint_offset 적용)
      R, t   = Kinematics.fk(arm_rad)      (sag + link offset 적용)
      T_base_ee = [R | t]                       (4×4)
      T_base_cam_init[i] = T_base_ee · T_ee_cam   (hand_eye 곱)

      # §2-3: depth filter + RGBD → 점군 + normal
      depth = bilateral_filter(depth)
      rgbd = RGBD(color, depth, intrinsic)
      pcd_down = downsample + estimate_normals(rgbd)

  # §5: pair-wise ICP
  for 인접 (i, i+1):
      ICP(pcd[i+1] → pcd[i], init = T_base_cam_init로부터) → T_ij, info
  for 가까운 비인접 (i, j):
      ICP 같은 식 → 신뢰 있으면 loop closure 엣지로

  # §6: PoseGraph optimization
  pose_graph.nodes = T_base_cam_init           (초기 추정)
  pose_graph.edges = ICP 결과들
  global_optimization(LM)                       (모든 노드 pose를 일관되게)
  T_base_cam_refined = pose_graph.nodes[i].pose

  # §7: TSDF integrate
  for each i:
      volume.integrate(rgbd[i], intrinsic[i], inv(T_base_cam_refined[i]))

  # §8: mesh 추출 + 정리
  mesh = marching_cubes(volume)
  remove_small_clusters(mesh)
  save_ply(mesh)
```

각 줄에 대응하는 챕터를 머리에 떠올릴 수 있으면, 이 문서가 제 역할을 한 거.

---

## 10. 더 공부하고 싶을 때

수학 약한 사람용 + 점진적 어려움:

1. **SE(3) / 회전행렬 직관** — Steven LaValle, *Planning Algorithms*, Ch. 4 (online free): http://lavalle.pl/planning/. 회전과 좌표계의 가장 친절한 입문.
2. **Lie algebra 살짝** — Joan Solà, "A micro Lie theory for state estimation in robotics" (PDF 한 편짜리). exp/log를 그림으로 설명.
3. **ICP 원조 논문** — Besl & McKay 1992, "A Method for Registration of 3-D Shapes" — point-to-point. Chen & Medioni 1991은 point-to-plane.
4. **Pose Graph SLAM** — Grisetti et al. "A Tutorial on Graph-Based SLAM" — 이 분야 정수.
5. **TSDF / KinectFusion** — Newcombe et al. 2011, "KinectFusion: Real-time dense surface mapping" — TSDF의 현대적 출발점.
6. **Marching cubes** — Lorensen & Cline 1987 (오리지널). 위키피디아 페이지도 충분히 친절.
7. **Open3D 튜토리얼** — http://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html — 우리 코드와 같은 흐름의 공식 예제.

문서를 읽다가 어떤 챕터에서 막히면 그 챕터에 대응하는 위 자료 하나만 읽어도 한 단계 깊이가 추가됨.

---

## 11. 자주 하는 헷갈림 (체크리스트)

이 정도가 머리에 자동 떠오르면 이 문서를 졸업.

- [ ] `T_A_B`의 첨자 순서: 목적지_출발지. `T_A_B · p_B = p_A`.
- [ ] `T_a_b · T_b_c = T_a_c`. 가운데 첨자 소거.
- [ ] SE(3) 역행렬은 회전 transpose + `−Rᵀt`.
- [ ] Open3D `TSDF.integrate(rgbd, intrinsic, extrinsic)` → `extrinsic = T_cam←world`. 즉 `inv(T_world←cam)`.
- [ ] Open3D `registration_icp(source, target, ...)` → `result.transformation = T_target←source`.
- [ ] Open3D `PoseGraphNode.pose = T_world←cam_i = T_base←cam_i`.
- [ ] 점군의 normal은 그 점에서의 표면 수직 방향, 길이 1.
- [ ] Point-to-plane ICP는 점이 *반대 점의 평면*에서 떨어진 거리 제곱합을 최소화.
- [ ] PoseGraph는 *N개* pose를 한꺼번에 *일관성 있게* 푸는 graph 최적화.
- [ ] TSDF voxel 값은 "그 voxel에서 가장 가까운 표면까지의 부호 있는 거리", `±sdf_trunc`로 잘림.
- [ ] Marching cubes는 voxel 모서리의 부호 패턴 → 삼각형 lookup.

---

*이 문서가 부족하면 표시해 두고 다음 round 때 보강.*
