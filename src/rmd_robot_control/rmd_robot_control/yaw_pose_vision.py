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

# 판정 경계 [잠정 2026-10-07]. 실측: 음수각 +35.7~+65.5, 양수각 -28.7~-53.3
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


def pick_sign(bgr):
    """영상으로 건 각도의 **부호**를 고른다 → -1 / +1, 모르면 None.

    돌려주는 둘째 값은 사람이 읽을 사유다 — 거부했을 때 왜인지 알아야 한다.
    """
    blob = gun_blob(bgr)
    if blob is None:
        return None, '건 끝단을 못 찾았다 (가려졌거나 조명이 다르다)'
    area, ang, _cx = blob
    if ang >= ANG_NEG:
        return -1, f'주축각 {ang:+.1f}° ≥ {ANG_NEG:+.1f}° → 음수각 (면적 {area})'
    if ang <= ANG_POS:
        return +1, f'주축각 {ang:+.1f}° ≤ {ANG_POS:+.1f}° → 양수각 (면적 {area})'
    return None, (f'주축각 {ang:+.1f}° 가 판정 경계 사이다 '
                  f'({ANG_POS:+.1f}~{ANG_NEG:+.1f}) — 추측하지 않는다')


def choose(candidates, bgr):
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
    sign, why = pick_sign(bgr)
    if sign is None:
        return None, why
    return (neg[0] if sign < 0 else pos[0]), why
