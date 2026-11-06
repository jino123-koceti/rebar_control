#!/usr/bin/env python3
"""리미트 차단의 mm 방향 환산 — **호밍이 리미트에서 빠져나올 수 있는가.**

    python3 tools/test/mm_dir_check.py

2026-11-06: `use_safety:=true` 로 켠 첫날 호밍이 실패했다. L1 이 Z 의 후퇴를
`z+`(= z_min 으로 더 들어가는 방향)로 읽고 0 으로 깎았다. 옛 식은 **원점이 늘
mm 최소단에 있다**고 가정했는데 Z 는 원점이 위(mm 0)이고 아래로 음수다.

여기서 보는 것은 하나다 — **탐색 방향은 막히고 후퇴 방향은 열리는가.**
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src', 'rmd_robot_control'))
from rmd_robot_control.axis_config import mm_dir_of, HOME_MM_DIR   # noqa: E402

# safety_node.LIMIT_BLOCKS — 그 리미트에 닿았을 때 막는 mm 방향
LIMIT_BLOCKS = {'x_min': 'x-', 'x_max': 'x+', 'y_min': 'y-', 'y_max': 'y+',
                'z_min': 'z+', 'z_max': 'z-'}
# axes.yaml 실측값 (= axis_config.HOMING_AXES 의 dir)
HOME_DIR = {'x': +1, 'y': +1, 'z': -1}

OK = FAIL = 0


def check(name, got, want):
    global OK, FAIL
    good = got == want
    print(f"  {'OK ' if good else '★NG'} {name:52s} {got!s:>5s}  (기대 {want})")
    if good:
        OK += 1
    else:
        FAIL += 1


print('■ 두 표가 같은 사실을 말하는가 (LIMIT_BLOCKS ↔ HOME_MM_DIR)')
for ax, into in HOME_MM_DIR.items():
    check(f'{ax}: 원점 리미트로 들어가는 방향', f'{ax}{into}',
          LIMIT_BLOCKS[f'{ax}_min'])

print('\n■ 호밍 탐색은 막히고 후퇴는 열린다 (원점 리미트에 닿은 상태)')
for ax in ('x', 'y', 'z'):
    d = HOME_DIR[ax]
    blocked = LIMIT_BLOCKS[f'{ax}_min']
    seek = mm_dir_of(ax, d * 15.0, d)            # homing: +dir × speed
    back = mm_dir_of(ax, -d * 15.0, d)           # homing: -dir × back_off
    check(f'{ax}: 탐색({d * 15:+.0f}dps)은 {blocked} 로 막힌다', seek, blocked)
    check(f'{ax}: 후퇴({-d * 15:+.0f}dps)는 열린다', back != blocked, True)

print('\n■ 반대쪽 리미트(far)에서도 빠져나온다')
for ax in ('x', 'y', 'z'):
    d = HOME_DIR[ax]
    far = LIMIT_BLOCKS[f'{ax}_max']
    check(f'{ax}: {far} 에 닿았을 때 원점 쪽은 열린다',
          mm_dir_of(ax, d * 15.0, d) != far, True)

print('\n■ x·y 는 예전 동작과 같아야 한다 (실기에서 검증된 축이다)')
for ax in ('x', 'y'):
    d = HOME_DIR[ax]
    for dps in (+30.0, -30.0):
        old = f"{ax}{'+' if (-1 if dps > 0 else 1) * d > 0 else '-'}"
        check(f'{ax} {dps:+.0f}dps 가 옛 식과 일치', mm_dir_of(ax, dps, d), old)

print('\n■ Z 는 옛 식과 **달라야** 한다 (그게 버그였다)')
d = HOME_DIR['z']
for dps in (+30.0, -30.0):
    old = f"z{'+' if (-1 if dps > 0 else 1) * d > 0 else '-'}"
    check(f'z {dps:+.0f}dps 가 뒤집혔다', mm_dir_of('z', dps, d) != old, True)

print('\n■ 교차검증 — 전혀 다른 두 경로가 같은 답을 내는가')
# ① stage_node 는 **mm 오차 부호**로만 판정한다 (`_blocked(name, err)`), 부호
#    환산이 없으니 정답을 들고 있는 셈이다. 그 규약을 모터 속도로 바꿔 쓰면:
#    "양수 dps → 엔코더 각 감소 → mm 감소"(mm_per_deg 가 셋 다 양수)
#    = `'-' if dps > 0 else '+'`. home_dir 도 축별 표도 필요 없는 식이다.
# ② 위의 `mm_dir_of` 는 home_dir 과 "원점이 mm 어느 끝인가" 로 푼다.
# 근거가 겹치지 않는데 답이 같다면, 둘 다 맞을 가능성이 아주 높다.
for ax in ('x', 'y', 'z'):
    d = HOME_DIR[ax]
    for dps in (+15.0, -15.0, +120.0, -120.0):
        simple = f"{ax}{'-' if dps > 0 else '+'}"
        check(f'{ax} {dps:+.0f}dps — stage_node 규약과 일치',
              mm_dir_of(ax, dps, d), simple)

print(f"\n{'통과' if not FAIL else '★ 실패 있음'} — {OK} OK / {FAIL} NG")
sys.exit(1 if FAIL else 0)
