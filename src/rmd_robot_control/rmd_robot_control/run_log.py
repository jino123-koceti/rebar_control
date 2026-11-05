#!/usr/bin/env python3
"""자율 주행 한 번을 **측정 가능한 기록**으로 남긴다. ROS 를 모르는 순수 모듈.

왜 필요한가 — 두 가지가 같은 데이터를 요구한다:

  **시스템을 믿기 위해**  정차마다 얼마나 덮었나, 얼마나 걸렸나, 왜 실패했나.
      로그를 눈으로 읽어서는 "지난번보다 나아졌나" 에 답할 수 없다.
  **선행연구와 겨루기 위해**  `docs/design/PRIOR_WORK_ANALYSIS.md` — 우리의
      차별점은 **커버리지 계획**인데, 그 주장을 뒷받침하려면 덮인 비율·정차
      수·시간·실패 분포가 있어야 한다. 2026-10-15~16 야외 시도는 계측을 안 해
      **쓸 수 없는 데이터**가 됐다. 그 실수를 반복하지 않는다.

## 두 파일로 남긴다

    <dir>/<run>_stops.csv    정차 1회 = 1행. 커버리지·시간·계획
    <dir>/<run>_points.csv   결속 1점 = 1행. **예측과 실제의 차이**

`points` 의 `resid_mm` 가 핵심이다 — 플래너가 "여기 있을 것" 이라 계산한
자리와 결속부가 "여기 있었다" 고 보고한 자리의 거리다. 이것이 쌓이면
**정차마다 재검출하는 구조가 드리프트를 실제로 흡수하는가**에 답할 수 있다
(지금은 가정일 뿐이다 — `TYING_POSITION_ACCURACY.md` 의 36mm 건 참고).

CSV 를 쓰는 이유는 **아무 도구로나 열리기 때문**이다. 분석을 누가 어디서
할지 모르는 데이터는 가장 단순한 형식으로 둔다.
"""

import csv
import os
import time

STOP_COLS = [
    'run', 'stop', 't_iso',
    # 어디에 섰나
    'pose_u', 'pose_v',
    # 무엇을 계획했나
    'plan_kind', 'plan_drive_mm', 'plan_turns', 'plan_cells', 'plan_cost_s',
    # 실제로 무엇을 시켰나 (계획과 다를 수 있다 — A* 경로에서 만들어지므로)
    'mission', 'moved_mm', 'moved_turns',
    # 무엇을 보았나
    'det_total', 'det_used',
    # 무엇을 묶었나
    'tie_reported', 'tie_matched', 'tie_ok',
    # 지도가 어떻게 됐나 — 커버리지 주장의 근거
    'cells_pending', 'cells_tied', 'cells_empty', 'cells_blocked',
    # 시간 — 비용 상수 실측의 재료이기도 하다
    't_observe_s', 't_plan_s', 't_mission_s',
    'note',
]

POINT_COLS = [
    'run', 'stop', 'pose',
    # 결속부가 보고한 자리 (스테이지 mm)
    'rep_x', 'rep_y',
    # 플래너가 예측한 자리 (같은 자세로 환산)
    'pred_x', 'pred_y',
    # 그 둘의 거리 — **이 열이 이 파일의 이유다**
    'resid_mm',
    'cell_i', 'cell_j', 'state',
]


class RunLog:
    """한 번의 자율 주행. 못 쓰면 **조용히 꺼진다** (측정이 주행을 막지 않는다)."""

    def __init__(self, directory, tag=''):
        self.ok = False
        self.run = time.strftime('%Y%m%d_%H%M%S') + (f'_{tag}' if tag else '')
        self._f = {}
        self._w = {}
        if not directory:
            return
        try:
            os.makedirs(directory, exist_ok=True)
            for name, cols in (('stops', STOP_COLS), ('points', POINT_COLS)):
                path = os.path.join(directory, f'{self.run}_{name}.csv')
                f = open(path, 'w', newline='', encoding='utf-8')
                w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
                w.writeheader()
                self._f[name], self._w[name] = f, w
            self.ok = True
        except OSError:
            # 기록이 안 된다고 주행을 멈추지 않는다. 호출부가 ok 를 보고 알린다.
            self.close()

    def _row(self, name, fields):
        if not self.ok:
            return
        try:
            fields.setdefault('run', self.run)
            fields.setdefault('t_iso', time.strftime('%Y-%m-%dT%H:%M:%S'))
            self._w[name].writerow(fields)
            self._f[name].flush()       # 프리즈해도 여기까지는 남는다
        except (OSError, ValueError):
            self.ok = False

    def stop(self, **fields):
        self._row('stops', fields)

    def point(self, **fields):
        self._row('points', fields)

    def close(self):
        for f in self._f.values():
            try:
                f.close()
            except OSError:
                pass
        self._f, self._w = {}, {}

    def paths(self, directory):
        return [os.path.join(directory, f'{self.run}_{n}.csv')
                for n in ('stops', 'points')]
