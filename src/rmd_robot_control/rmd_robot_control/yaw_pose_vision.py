"""yaw 자세 별칭을 **작업영역 카메라로 가린다.**

## 무엇을 푸는가

yaw 가동범위는 건 약 33.7° 인데 단회전 엔코더는 건 **28.8° 마다 접힌다.** 그래서
`|건 각도| > 14.4°` 인 자세는 후보가 **둘**이고 단회전만으로는 못 고른다 —
1번(−16.77°)과 4번(+15.18°)이 그렇다. 전원 재투입 후 하필 거기 있으면 시스템이
자세를 못 가려 **호밍 자체가 막힌다.** 지금은 사람이 12시로 옮기거나
`/stage/yaw_declare` 로 알려줘야 한다.

## 왜 비전으로 풀 수 있는가

작업영역 카메라(Gemini 2L)는 **yaw 와 같이 돌지 않는다.** 근거는 캘리브레이션이
자세별이라는 것이다 — `stage_camera.yaml` 의 `b` 가 1번 `[213.9, 261.3, …]`,
4번 `[218.1, 139.3, …]` 으로 **Y 로 122mm** 차이난다. 건만 카메라 앞에서 돈다.

## 왜 안전한가 — 비전은 **한 비트만** 고른다

후보 둘은 **항상 28.8° 떨어져** 있고 가동범위가 ±17° 이므로, **반드시 하나는
음수각 하나는 양수각**이다:

    1번 −16.77°  ↔  별칭 +12.03°
    4번 +15.18°  ↔  별칭 −13.62°

즉 단회전이 못 주는 정보는 **부호 한 비트**뿐이다. 비전은 각도를 재는 것이 아니라
그 비트만 고른다. 정밀도가 필요 없고, **후보 밖의 값은 만들어 낼 수 없다.**
엔코더가 주(主)이고 비전은 보조다.

## 무엇을 보는가 — 건 끝단 덩어리의 **주축 방향**

2026-10-07 에 yaw 를 손으로 전 구간 돌리며 47장을 모아 재 봤다. 건 33.7° 회전에
영상 주축이 **119° 돌았다** (3.5배 증폭):

      건 각도   주축각   면적
      -16.77    +65.5   17249     ← 1번
      -13.00    +25.4   13693
       -6.56     +7.6    6281
       +7.93     -1.8    8044
      +13.35    -20.0   11438
      +15.06    -29.8   12913     ← 4번

**모호가 생기는 양 끝에서 신호가 가장 세다** — 면적이 13,000~17,000 으로 커지고
주축각이 극단으로 간다. 12시 근처는 신호가 약하지만 거기는 애초에 모호하지 않다.

같이 재 본 것 중 **못 쓰는 것**도 적어 둔다 (다시 시도하지 않도록):
  · **암부 면적**만으로는 안 된다 — 12시에서 최소인 **대칭** 곡선이라
    1번 47,993 / 4번 48,573 으로 **부호를 못 가린다.**
  · **무게중심 x** 는 단조가 아니다 (625→651→605→623→597 로 출렁인다).
    상단 밴드에 프레임 레일·철근이 섞여 건을 못 짚는다.
  · 빨간 라벨 화소수도 부호를 잘 가르지만(음수 2~14 / 양수 45~576), 스티커는
    가려지거나 떨어질 수 있어 **형상보다 약한 근거**다. 교차검증용으로만 쓴다.

⚠ **임계값은 조명과 크롭에 의존한다.** 아래 기본값은 2026-10-07 조명에서 얻은
**잠정값**이고, 라벨의 바탕(`noon_single`·에지값)이 그때는 눈대중·잠정이었다.
**검증된 호밍 뒤에 `tools/test/vision_map_collect.py` 로 다시 모아 갱신할 것.**
"""

import math

# 상단 크롭 — 건만 들어오고 프레임 레일은 피한다.
# ⚠ 넓게 잡으면 12시 근처에서 **레일이 더 큰 덩어리**가 되어 건 대신 잡힌다
#   (2026-10-07 에 x=380~1000 으로 잡았다가 몇 장에서 레일을 쟀다).
CROP = dict(y0=0, y1=210, x0=430, x1=940)
DARK = 75                 # 이보다 어두우면 건 후보. 데크는 밝다
MIN_AREA = 2500           # 이보다 작으면 건을 못 찾은 것으로 본다

# 판정 경계 — **검증된 호밍 뒤 1° 간격 32점 스윕으로 확정** (2026-10-07).
#   X=0 고정, `yaw_anchor='호밍 완료'` 라 각도 라벨이 추정이 아니다.
#   별칭이 실제로 생기는 |건|>14.4° 구간에서 **4/4 정답, 틀림 0**:
#       -16.62° → +49.7°    -15.46° → +37.4°    -14.54° → +29.7°
#       +14.47° → -27.9°
#   가장 빠듯한 -14.54° 에서도 경계까지 **9.7° 여유**다. 그리고 주축각이 각도에
#   따라 **단조**로 변해(-16.6→+49.7, -10.6→+10.5, +14.5→-27.9) 경계 근처에서
#   급변하지 않는다 — 조금 흔들려도 판정이 뒤집히지 않는다.
#   ⚠ |건|<14.4° 는 단회전만으로 유일하게 갈려 **비전이 불리지 않는다.** 스윕에서
#     그 구간 24점이 '거부' 로 나오지만 실사용에서는 호출되지 않는 영역이다.
ANG_NEG = 20.0            # 주축각이 이보다 크면 **음수 후보**
ANG_POS = -15.0           # 이보다 작으면 **양수 후보**
#   그 사이는 판정하지 않는다 — 추측하느니 사람에게 묻는다


def gun_blob(bgr):
    """상단에서 가장 큰 어두운 덩어리의 (면적, 주축각°, 무게중심x).

    못 찾으면 None. 찾지 못한 것을 0 으로 돌려주면 호출부가 "부호 양수" 로
    오해하므로 **반드시 None 이어야 한다.**
    """
    import cv2
    import numpy as np

    c = CROP
    crop = bgr[c['y0']:c['y1'], c['x0']:c['x1']]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    mask = (gray < DARK).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    ys, xs = np.nonzero(lab == i)
    if len(xs) < MIN_AREA:
        return None
    pts = np.stack([xs, ys], 1).astype(np.float64)
    ctr = pts.mean(0)
    cov = ((pts - ctr).T @ (pts - ctr)) / len(pts)
    ev, evec = np.linalg.eigh(cov)
    ang = math.degrees(math.atan2(evec[1, -1], evec[0, -1]))
    ang = (ang + 90) % 180 - 90          # -90~+90 으로 접는다
    return (len(xs), ang, float(ctr[0] + c['x0']))


# ── 레일 마스크 ───────────────────────────────────────────────────────────
# 카메라가 **고정**이라 프레임 레일은 영상에서 늘 같은 자리다. 45장에서 90% 이상
# 항상 어두운 화소를 모아 만들었다. 이걸 빼지 않으면 "가장 큰 어두운 덩어리" 가
# **건이 아니라 레일**이 된다 — 2026-10-07 에 그걸 모르고 레일을 재면서
# "x=0 에서는 어떤 특징도 안 갈린다" 는 틀린 결론을 냈다.
RAIL_MASK = ('/home/koceti/ros2_ws/src/rebar_control/'
             'data/vision/rail_mask.png')
_mask = None

# 구간 경계 — 레일 마스크 후 건 면적. 실측(y 중앙 줄):
#   x=0  19k   x=100  37k   x=200  45~65k   x=300  82~113k   x=400  105~155k
AREA_NEAR = 30000         # 이보다 작으면 "건이 멀다" — `near_x_mm` 이 없을 때만 쓴다
# X 를 알 때 쓰는 경계. 비전 판정이 검증된 것은 **X=0 근처**다
# (1번 +56~57° / 4번 -29~-40°, y 100~250 에서 8/8). 그 밖은 검증되지 않았다.
FAR_X_MM = 60.0


def _rail():
    global _mask
    if _mask is None:
        import cv2
        m = cv2.imread(RAIL_MASK, cv2.IMREAD_GRAYSCALE)
        _mask = None if m is None else (m > 127)
    return _mask


def gun_masked(bgr):
    """레일을 빼고 가장 큰 어두운 덩어리 → (면적, 박스좌단, 무게중심x).

    건이 카메라에 가까운 구간(x 큼)에서 쓴다. 거기서는 건이 화면을 크게 차지해
    레일만 빼면 확실히 잡힌다.
    """
    import cv2
    import numpy as np
    rail = _rail()
    if rail is None:
        return None
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    m = ((g < DARK) & (~rail)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    xs = np.nonzero(lab == i)[1]
    return (int(st[i, cv2.CC_STAT_AREA]), int(st[i, cv2.CC_STAT_LEFT]),
            float(xs.mean()))


# x 가 큰 구간에서 자세를 가르는 기준 — 건 박스 좌단.
# 실측: 1번이 4번보다 **항상 오른쪽**이다 (x≥100 에서 +70~+630px, 16/16).
LEFT_GAP = 40             # 이보다 작으면 애매 → 거부


def pick_sign(bgr, left_ref=None, near_x_mm=None):
    """영상으로 건 각도의 **부호**를 고른다 → -1 / +1, 모르면 None.

    ⚠ **구간마다 듣는 특징이 다르다.** 하나로 전 구간을 덮으려다 실패했다:
      · 주축각 하나로 밀었더니 x 가 커지면 **뒤집혔다** (1번인데 -86°, 4번인데
        +77.6°). 거부가 아니라 **틀린 답을 자신 있게 내는** 최악의 실패였다.
      · 그 뒤 레일 마스크+최대덩어리로 바꿨더니 이번엔 x=0 에서 건이 아닌 물체를
        집어 "구분 불가" 가 됐다.
    그래서 **건 면적으로 구간을 먼저 가리고** 구간별 특징을 쓴다.

    `left_ref` 는 x 가 큰 구간에서 비교 기준이 되는 박스 좌단이다. 없으면
    그 구간은 판정하지 않는다 — 절대 임계로는 못 가른다(X 에 따라 변한다).
    """
    gm = gun_masked(bgr)
    if gm is None:
        return None, '레일 마스크를 못 읽었거나 덩어리가 없다'
    area, x0, _cx = gm

    # ⚠ **구간은 면적으로 가르지 않는다.** 면적은 X 와 단조가 아니고 구간끼리
    #   겹친다 (x=0 이 18.8k~38.1k, x=100 이 31.9k~56.9k, x=400 이 28.9k 로
    #   내려오기도 한다). 호출부가 `near_x_mm` 로 **실제 X** 를 주면 그것을 쓴다 —
    #   X 호밍이 끝난 뒤에 부르므로 알 수 있는 값이다. 추정할 이유가 없다.
    far = (abs(near_x_mm) <= FAR_X_MM) if near_x_mm is not None \
        else (area < AREA_NEAR)

    if far:
        # 건이 멀다 → 상단 크롭 안의 **주축각**. 실측 차이 85.6~96.5°
        blob = gun_blob(bgr)
        if blob is None:
            return None, f'건이 멀고(면적 {area}) 크롭에서 끝단을 못 찾았다'
        _a, ang, _c = blob
        if ang >= ANG_NEG:
            return -1, f'먼 구간 — 주축각 {ang:+.1f}° ≥ {ANG_NEG:+.1f}° → 음수각'
        if ang <= ANG_POS:
            return +1, f'먼 구간 — 주축각 {ang:+.1f}° ≤ {ANG_POS:+.1f}° → 양수각'
        return None, f'먼 구간 — 주축각 {ang:+.1f}° 가 경계 사이다'

    # 건이 가깝다 → 박스 좌단. **절대값이 아니라 두 후보 비교**라야 한다
    if left_ref is None:
        return None, (f'가까운 구간(면적 {area}) 은 비교 기준(left_ref)이 있어야 '
                      f'한다 — 절대 임계로는 못 가른다')
    gap = x0 - left_ref
    if gap >= LEFT_GAP:
        return -1, f'가까운 구간 — 좌단 {x0} 가 기준 {left_ref} 보다 {gap:+d}px 오른쪽'
    if gap <= -LEFT_GAP:
        return +1, f'가까운 구간 — 좌단 {x0} 가 기준 {left_ref} 보다 {gap:+d}px 왼쪽'
    return None, f'가까운 구간 — 좌단 차이 {gap:+d}px 가 작다 ({LEFT_GAP}px 미만)'


def choose(candidates, bgr, near_x_mm=None):
    """후보 각도(도) 중 하나를 고른다. 못 고르면 (None, 사유).

    ⚠ 후보가 **부호가 갈리는 둘** 일 때만 쓴다. 같은 부호면 비전이 가릴 수 없고,
      그런 경우는 애초에 단회전이 가려 주므로 여기까지 오지 않는다.
    """
    cands = list(candidates)
    if len(cands) != 2:
        return None, f'후보가 2개가 아니다 ({len(cands)}개)'
    neg = [c for c in cands if c < 0]
    pos = [c for c in cands if c >= 0]
    if len(neg) != 1 or len(pos) != 1:
        return None, '후보 부호가 갈리지 않는다 — 비전으로 못 가린다'
    sign, why = pick_sign(bgr, near_x_mm=near_x_mm)
    if sign is None:
        return None, why
    return (neg[0] if sign < 0 else pos[0]), why
