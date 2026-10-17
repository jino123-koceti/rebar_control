#!/usr/bin/env python3
"""L6 — 자유공간. **로봇이 있을 수 있는 곳**을 다룬다. ROS 무관 순수 모듈.

`coverage_map` 과 나눈 이유는 질문이 다르기 때문이다.

    coverage_map   "여기서 묶을 수 있나"   ← 결속 영역(건이 닿는 범위)
    coverage_space "여기 있을 수 있나"     ← 몸체 영역(로봇이 차지하는 범위)

처음에는 하나로 두고 결속 영역을 점유 영역으로 썼는데, 그러면 **장애물이
건의 작업 범위에 들어오기만 해도 그 자리에 못 서는 것**이 되어 지나치게
엄하다. 실제로 막는 것은 몸체이고, 결속 영역은 "거기서 일할 수 있나" 일 뿐이다.

## 지나갈 수 있나 를 어떻게 푸는가

장애물을 **크기 있는 사각(AABB)** 으로 들고, 로봇 몸체도 사각으로 본다.
그러면 "폭이 충분한가" 를 매번 재지 않아도 된다 — 두 사각이 겹치는지만
보면 되고, 이것이 곧 로봇 폭만큼 장애물을 부풀린 것(C-space)과 같다.
통로 통과는 경로를 따라 몸체를 밀어 보는 것으로 검사한다(`move_ok`).

⚠ **장애물을 점(격자 한 칸)으로 들면 이 질문에 답할 수 없다.** 기둥이 얼마나
  두꺼운지를 표현할 수가 없기 때문이다. 그래서 여기서는 사각으로 든다.
"""

# 로봇 몸체 (주행방향 길이 × 횡방향 폭, mm). **3차년도 장비 실측 2026-10-17.**
#   폭 940 · 길이 1160 — 폭보다 길다. 통로 통과는 폭이, 장애물 앞뒤 접근은
#   길이가 좌우한다. 2차년도(1037 × 720)와 가로세로가 뒤바뀌어 있으니 옛
#   수치를 그대로 쓰지 말 것.
BODY_LEN_U = 1160.0       # [실측 2026-10-17]
BODY_WID_V = 940.0        # [실측 2026-10-17]
# 몸체 둘레 여유. 주행 오차(±5mm)와 검출 오차를 덮을 만큼.
BODY_MARGIN = 60.0


class FreeSpace:
    """작업 좌표(정준 좌표계)에서의 장애물과 몸체 점유.

    장애물은 `(u0, u1, v0, v1)` 사각의 모음이다. 센서가 본 것만 들어온다 —
    안 본 곳은 '비어 있다' 가 아니라 '모른다' 이고, 여기서는 막지 않는다
    (막으면 로봇이 아무 데도 못 간다). 모르는 곳으로 가는 위험은 주행 중
    실시간 차단(deck_edge·범퍼)이 맡는다.
    """

    def __init__(self, body=None, margin=BODY_MARGIN, center=(0.0, 0.0)):
        """body : (길이_U, 폭_V) mm. center : 포즈 기준 몸체 중심.

        ⚠ **몸체 중심축 = 결속영역 중심축** 이다 (사용자 확인 2026-10-17).
          두 축이 정 가운데로 일치하므로 `center` 에는 결속영역 중심을 준다.
          추정이 아니라 확인된 값이니 다시 의심하지 말 것 — 치우쳐 있다고
          보면 기둥 앞뒤 접근 가능 범위가 수백 mm 틀어진다.
        """
        lu, wv = body or (BODY_LEN_U, BODY_WID_V)
        cu, cv = center
        self.body = (cu - lu / 2 - margin, cu + lu / 2 + margin,
                     cv - wv / 2 - margin, cv + wv / 2 + margin)
        self.rects = []          # [(u0, u1, v0, v1, why, t)]

    # ---- 등록 ----
    def add(self, u0, u1, v0, v1, why='장애물', t=0.0, merge=True):
        """장애물 사각을 더한다. 겹치면 합친다 (같은 것을 여러 번 본다)."""
        box = [float(min(u0, u1)), float(max(u0, u1)),
               float(min(v0, v1)), float(max(v0, v1))]
        if merge:
            for r in self.rects:
                if _overlap(box, r):
                    r[0], r[1] = min(r[0], box[0]), max(r[1], box[1])
                    r[2], r[3] = min(r[2], box[2]), max(r[3], box[3])
                    return r
        r = box + [why, t]
        self.rects.append(r)
        return r

    def clear(self):
        self.rects = []

    # ---- 질의 ----
    def body_at(self, ru, rv):
        b = self.body
        return (ru + b[0], ru + b[1], rv + b[2], rv + b[3])

    def occupiable(self, ru, rv):
        """그 자리에 **설 수 있나.** 몸체가 장애물과 겹치면 못 선다."""
        box = self.body_at(ru, rv)
        return not any(_overlap(box, r) for r in self.rects)

    def move_ok(self, a, b, step=50.0):
        """a → b 로 **갈 수 있나.** a, b 는 (ru, rv).

        로봇은 주행과 횡이동을 동시에 하지 않는다. ㄱ자로 — 먼저 주행, 그다음
        횡이동 — 두 구간을 따로 민다. 이 밀어 보기가 곧 통로 통과 검사다.

        ⚠ 두 가지를 조심해야 한다 (둘 다 시뮬에서 교착을 만들었다):
          ① **길이 0인 구간을 검사하면 안 된다.** 출발 자리를 검사하게 되어
             횡이동만 하는 후보가 전부 거부된다.
          ② **이미 설 수 없는 자리에 서 있을 수 있다.** 장애물은 가까이 가야
             보이므로 움직인 뒤에야 알게 된다. 그때 중간 자리까지 따지면
             나갈 길이 없어진다 → 목적지만 보고 보낸다(왔던 길로 후퇴).
        """
        if not self.occupiable(*a):
            return self.occupiable(*b)                     # ② 탈출
        for x0, x1, fixed, axis in ((a[0], b[0], a[1], 'u'),
                                    (a[1], b[1], b[0], 'v')):
            if abs(x1 - x0) < 1e-6:
                continue                                   # ① 길이 0
            n = max(1, int(abs(x1 - x0) / step))
            for i in range(1, n + 1):
                x = x0 + (x1 - x0) * i / n
                if not (self.occupiable(x, fixed) if axis == 'u'
                        else self.occupiable(fixed, x)):
                    return False
        return True

    def route(self, a, b, drive_mm_s=70.0, lat_turn_s=7.2, turn_s=2.0,
              u_step=100.0, v_step=50.0, pad=1600.0, max_nodes=40000):
        """a → b **경로**를 찾는다. 반환 (비용초, [포즈…]) 또는 None.

        왜 ㄱ자 한 번으로는 안 되는가: 기둥 벽의 틈으로 들어갔다 나오려면
        '옆으로 → 앞으로 → 옆으로' 처럼 **구간이 세 개 이상** 필요하다.
        ㄱ자만 보면 그런 경로가 없는 것으로 판정되어, 갈 수 있는 자리를
        포기한다 (시뮬에서 건너편을 묶고 돌아오지 못했다).

        포즈 격자 위의 A* 다. U 는 `u_step`, V 는 **횡이동 1회전**(`v_step`)
        단위 — 그 이하로는 설 수 없으므로 격자가 곧 실제 가능한 자리다.
        축을 바꿀 때 `turn_s` 를 물려 쓸데없는 지그재그를 막는다.
        """
        import heapq
        ua, va = a
        ub = ua + round((b[0] - ua) / u_step) * u_step
        vb = va + round((b[1] - va) / v_step) * v_step
        lo_u, hi_u = min(ua, ub) - pad, max(ua, ub) + pad
        lo_v, hi_v = min(va, vb) - pad, max(va, vb) + pad

        def h(u, v):
            return (abs(ub - u) / drive_mm_s
                    + abs(vb - v) / v_step * lat_turn_s)

        start = (0, 0)
        goal = (int(round((ub - ua) / u_step)), int(round((vb - va) / v_step)))
        pos = lambda n: (ua + n[0] * u_step, va + n[1] * v_step)
        seen = {start: (0.0, None, None)}
        pq = [(h(ua, va), 0.0, start, None)]
        n_pop = 0
        while pq:
            _f, g, cur, ax = heapq.heappop(pq)
            if cur == goal:
                out, k = [], cur
                while k is not None:
                    out.append(pos(k)); k = seen[k][1]
                return g, out[::-1]
            n_pop += 1
            if n_pop > max_nodes:
                return None
            if g > seen.get(cur, (1e18,))[0] + 1e-9:
                continue
            for du, dv, a2 in ((1, 0, 'u'), (-1, 0, 'u'), (0, 1, 'v'), (0, -1, 'v')):
                nxt = (cur[0] + du, cur[1] + dv)
                pu, pv = pos(nxt)
                if not (lo_u <= pu <= hi_u and lo_v <= pv <= hi_v):
                    continue
                if not self.occupiable(pu, pv):
                    continue
                ng = g + (u_step / drive_mm_s if a2 == 'u' else lat_turn_s)
                if ax is not None and a2 != ax:
                    ng += turn_s
                if ng < seen.get(nxt, (1e18,))[0] - 1e-9:
                    seen[nxt] = (ng, cur, a2)
                    heapq.heappush(pq, (ng + h(pu, pv), ng, nxt, a2))
        return None

    def gap_along(self, ru, rv, axis='v', span=4000.0, step=25.0):
        """지금 자리에서 그 축으로 **연속으로 설 수 있는 구간**들 (mm).

        "기둥 둘 사이로 지나갈 수 있나" 를 사람이 눈으로 확인할 때 쓴다.
        반환: [(시작, 끝), ...] — 길이가 0 이면 그 방향으로는 못 간다.
        """
        out, lo = [], None
        n = int(span / step)
        for i in range(-n, n + 1):
            x = (ru if axis == 'u' else rv) + i * step
            ok = (self.occupiable(x, rv) if axis == 'u'
                  else self.occupiable(ru, x))
            if ok and lo is None:
                lo = x
            elif not ok and lo is not None:
                out.append((lo, x - step)); lo = None
        if lo is not None:
            out.append((lo, (ru if axis == 'u' else rv) + n * step))
        return out


def _overlap(a, b):
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]
