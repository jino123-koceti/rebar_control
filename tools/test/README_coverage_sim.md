# 커버리지 시뮬 — 다른 PC에서 돌리기

장비(젯슨)에서는 352점짜리 테스트베드가 느리다. **ROS 도 CAN 도 필요 없으니**
워크스테이션에서 돌리는 쪽이 낫다.

## 필요한 것

```
python3 >= 3.8   numpy   pyyaml   matplotlib
```

그림의 한글 글꼴. 없으면 글자가 네모로 나온다:

```bash
sudo apt install fonts-noto-cjk          # Ubuntu
```

`coverage_plot.py` 가 `Noto Sans CJK JP → Noto Sans CJK KR → NanumGothic →
DejaVu Sans` 순으로 찾는다. (Noto Sans CJK JP 로도 한글은 정상 출력된다)

## 받아서 돌리기

```bash
git clone https://github.com/jino123-koceti/rebar_control.git
cd rebar_control
git checkout feature/year3-new-equipment

mkdir -p out
python3 tools/test/coverage_sim.py --only bed --plot out
```

`--only` 없이 돌리면 전체 시나리오(8장)를 그린다. 오래 걸리니 먼저
`--only bed` 로 한 장만 보는 편이 낫다.

| `--only` | 내용 |
|---|---|
| `bed` | **테스트베드 4200 × 3000mm · 배근 200mm** (22열 × 16행 = 352점) |
| `field` | 7×5 작은 판 (빠름, 동작 확인용) |
| `wall` | 15×11 + 기둥 300mm — 2차년도 비교 포함 |
| `gap` | 기둥 사이 틈 통과/불통과 |
| (없음) | 전부 |

`-v` 를 붙이면 정차마다 검출·결속·이동을 찍는다.

## 경로

저장소 뿌리를 **모듈 위치에서 자동으로 찾는다.** 다른 데 두고 쓰려면
`REBAR_ROOT=/경로/rebar_control` 로 덮어쓴다.

읽는 데이터 두 개 (둘 다 저장소 안에 있다):

```
data/calibration/stage_camera.yaml        자세별 캘리 모델 A·b
src/rebar_base_control/config/axes.yaml   자세별 가동범위(envelope)
```

## 무엇이 나오나

콘솔에 정차 수·결속 수·미결속 사유가, `--plot` 폴더에 PNG 가 나온다.
미결속은 세 가지로 구분해 보고한다 — 뭉뚱그리면 버그인지 한계인지 모른다.

| 표시 | 뜻 |
|---|---|
| `도달 불가` | 그 셀을 덮는 어떤 자리에도 **몸체가 안 들어간다** (기둥 옆) |
| `격자 위상` | 설 자리는 있는데 **횡이동 50mm 격자에 안 맞는다** |
| `★ 미처리` | 갈 수 있는데 계획이 멈췄다 = **진짜 결함** |
| `★★` | **설 수 없는 자리에 섰다** = 감지 거리나 계획이 틀렸다 |

`★` 가 붙은 것만 고칠 거리다.

## 느리면

352점에서 가장 비싼 것은 후보 평가다. 손댈 곳:

- `CoverageMap.max_rows` (기본 9) — 한 셀에 대해 볼 횡이동 격자 행 수
- `candidates()` 의 U 후보 3개
- `coverage_space.route()` 의 `max_nodes` (기본 40000) — A\* 는 ㄱ자가
  막힐 때만 돈다. 장애물 없는 판에서는 아예 안 돈다

## 주의

비용 상수(주행 70mm/s · 횡 7.2초/회전 · 결속 1점 0초)는 **실측 전 임시값**이다
(`coverage_map.Cost`). 정차 수·커버리지는 그대로지만 "이동 N초" 는 상대 비교용으로만 볼 것.
