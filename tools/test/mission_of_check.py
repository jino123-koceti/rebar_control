#!/usr/bin/env python3
"""`mission_of` 의 mm → 횡이동 회전수 환산. **장비가 필요 없다.**

    source install/setup.bash && python3 tools/test/mission_of_check.py

왜 테스트가 있는가 — 이 환산이 **조용히 틀렸다.** `step_v()`(배근 피치
≈200mm)로 나눠서 50mm 횡 구간이 `round(0.25)=0` 이 되어 사라졌고, 미션에
lateral 이 아예 안 들어갔다. 예외도 로그도 없었다. 드러난 경로는 CSV 의
`plan_turns=1 / moved_turns=0` 한 줄이었다 (2026-11-05).

같은 종류의 실수가 다시 들어오면 **여기서 걸린다** — 단위가 섞인 코드는
눈으로 읽어 안전을 확인할 수 없다.
"""
import types
from rmd_robot_control import drive_autonomy_node as D

cls = [v for v in vars(D).values()
       if isinstance(v, type) and hasattr(v, 'mission_of')][0]
stub = types.SimpleNamespace(
    map=types.SimpleNamespace(lat_mm=50.0, step_v=lambda: 200.0),
    max_leg_turns=6, max_leg_mm=600.0, tie_at_stop=True, moved=None)

cases = [
    ('경로 2점 · 횡 50mm (1회전)',   [(0, 0), (0, 50)],              0.0,  50.0),
    ('경로 2점 · 횡 150mm (3회전)',  [(0, 0), (0, 150)],             0.0, 150.0),
    ('경로 3점 · 주행 후 횡 100mm',  [(0, 0), (300, 0), (300, 100)], 300.0, 100.0),
    ('경로 없음 · 계획 총량 2회전',  [],                             250.0, 100.0),
    ('음(−) 방향 횡 −100mm',         [(0, 0), (0, -100)],            0.0, -100.0),
]
ok = True
for label, path, dmm, dv in cases:
    plan = {'path': path, 'turns': int(round(dv / 50.0)), 'drive_mm': dmm,
            'kind': 'tie'}
    out = cls.mission_of(stub, plan)
    want = int(round(dv / 50.0))
    got = stub.moved[1]
    good = got == want and abs(stub.moved[0] - dmm) < 1.5
    ok &= good
    print(f"{'OK ' if good else '★NG'} {label:26s} → {' | '.join(out):32s} "
          f"회전 {got:+d} (기대 {want:+d})")
print('\n전부 통과' if ok else '\n★ 실패 있음')
