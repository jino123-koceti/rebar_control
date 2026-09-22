#!/usr/bin/env python3
"""yaw_absolute.py — Yaw 단일턴 절대 위치 복원 (호스트측 랩 복원)

배경: RMD-X4 멀티턴(0x92)은 급전원차단에 랩카운트 소실(brownout). 단일턴(0x94)은 유지됨.
      Yaw는 스트로크 ~396°(1.1턴)라, 단일턴 겹침은 [0°, ~37°] 9%뿐이고 나머지 91%는
      0x94 하나로 절대위치가 유일 결정됨. 겹침 구간만 PC 파일의 lap비트로 home/max쪽 구분.

좌표: yaw_from_home (deg) — home 리미트=0, max=+stroke(~396). 단조증가.
참고 실측(2026-06-15, 0x147):
  home  : 0x94 = 0.00°  (0x92 = -359.43)
  max   : 0x94 = 36.98° (0x92 = +36.98)
  → overlap_max ≈ 37°, stroke ≈ 396°

순수 로직(ROS 비의존) + atomic 파일 영속. 단독 실행 시 self-test 수행.
"""
import json
import os
import tempfile


def _wrap360(deg):
    """0~360 정규화."""
    return deg % 360.0


def _ang_diff(a, b):
    """두 각도의 최소 차이(deg, -180~180). wrap 고려."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


class YawAbsolute:
    """Yaw 단일턴(0x94) → 절대 yaw_from_home 복원 + 영속.

    Parameters
    ----------
    home_st_deg : home 리미트에서의 0x94 (deg)
    max_st_deg  : max에서의 0x94 (deg) = 겹침 상한
    stroke_deg  : home→max 전체 가동범위 (yaw_from_home 최대값)
    tol_deg     : 단일턴 정합성 허용오차
    state_path  : 영속 파일 경로
    """

    def __init__(self, home_st_deg=0.0, max_st_deg=36.98, stroke_deg=396.0,
                 tol_deg=3.0, state_path='/var/lib/rebar/yaw_absolute.json'):
        self.home_st = float(home_st_deg)
        self.max_st = float(max_st_deg)
        self.stroke = float(stroke_deg)
        self.tol = float(tol_deg)
        self.state_path = state_path

    # ---------- 핵심 매핑 ----------
    def rel_from_st(self, st_deg):
        """0x94 단일턴 → home 기준 상대각 rel (0~360, home=0)."""
        return _wrap360(st_deg - self.home_st)

    def is_overlap(self, st_deg):
        """겹침 구간(0x94 ∈ [0, max_st])인지. 겹침이면 lap 정보 필요."""
        rel = self.rel_from_st(st_deg)
        # 허용오차만큼 여유 (경계 근처는 보수적으로 겹침 처리)
        return rel <= (self.max_st + self.tol)

    def yaw_from_st(self, st_deg, lap):
        """0x94 + lap비트 → yaw_from_home(deg).

        lap=0: home쪽 (yaw=rel),  lap=1: max쪽 (yaw=rel+360)
        """
        rel = self.rel_from_st(st_deg)
        yaw = rel + (360.0 if lap else 0.0)
        return yaw

    # ---------- 부팅 복원 ----------
    def resolve(self, st_deg, saved=None):
        """부팅 시 절대 yaw 복원.

        Returns
        -------
        (yaw_deg, trusted, reason)
          trusted=True  → yaw_deg 신뢰, 호밍 불필요
          trusted=False → yaw_deg=None, 호밍 폴백 필요
        """
        if not self.is_overlap(st_deg):
            # 유일 구간: 0x94만으로 결정, 파일 불필요
            return self.yaw_from_st(st_deg, lap=0), True, 'unique'

        # 겹침 구간: 파일 필요
        if saved is None:
            return None, False, 'overlap_no_file'

        # 저장된 단일턴과 현재 0x94 일치 검사 (전원오프 중 무이동 확인)
        if abs(_ang_diff(saved['single_turn'], st_deg)) > self.tol:
            return None, False, 'moved'  # 움직임 감지 → 호밍

        # 일치 → 저장된 lap 신뢰
        return self.yaw_from_st(st_deg, lap=saved['lap']), True, 'overlap_file'

    def lap_of_yaw(self, yaw_deg):
        """yaw_from_home → lap비트 (저장용)."""
        return 1 if yaw_deg >= 360.0 else 0

    # ---------- 영속 (atomic) ----------
    def save(self, yaw_deg, st_deg):
        """현재 절대 yaw + 단일턴을 atomic 저장."""
        data = {
            'yaw_deg': float(yaw_deg),
            'single_turn': float(st_deg),
            'lap': self.lap_of_yaw(yaw_deg),
        }
        d = os.path.dirname(self.state_path)
        if d:
            os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=d or '.', prefix='.yaw_abs_', suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(data, f)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.state_path)  # atomic
        except Exception:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def load(self):
        """저장 파일 로드. 없음/손상 → None (호밍 폴백)."""
        try:
            with open(self.state_path) as f:
                data = json.load(f)
            if all(k in data for k in ('yaw_deg', 'single_turn', 'lap')):
                return data
        except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError):
            pass
        return None


# ========================= self-test =========================
def _selftest():
    ya = YawAbsolute(home_st_deg=0.0, max_st_deg=36.98, stroke_deg=396.0,
                     tol_deg=3.0, state_path='/tmp/yaw_abs_test.json')
    ok = True

    def check(name, cond):
        nonlocal ok
        print(f"  [{'OK' if cond else 'FAIL'}] {name}")
        ok = ok and cond

    # 1) 유일 구간 (3시 56°, 12시 194°, 9시 328°)
    for st, exp in [(56.0, 56.0), (194.0, 194.0), (328.0, 328.0)]:
        yaw, trusted, reason = ya.resolve(st, saved=None)
        check(f"unique st={st} → yaw={yaw} trusted={trusted}({reason})",
              trusted and abs(yaw - exp) < 0.01 and reason == 'unique')

    # 2) 겹침 + 파일없음 → 호밍
    yaw, trusted, reason = ya.resolve(20.0, saved=None)
    check(f"overlap no-file st=20 → 호밍", (not trusted) and reason == 'overlap_no_file')

    # 3) 겹침 + 파일일치(home쪽 lap0) → 신뢰 yaw=20
    saved_home = {'single_turn': 20.0, 'lap': 0, 'yaw_deg': 20.0}
    yaw, trusted, reason = ya.resolve(20.5, saved=saved_home)
    check(f"overlap file home st=20.5 → yaw={yaw} trusted", trusted and abs(yaw - 20.5) < 0.01)

    # 4) 겹침 + 파일일치(max쪽 lap1) → 신뢰 yaw≈380
    saved_max = {'single_turn': 36.98, 'lap': 1, 'yaw_deg': 396.98}
    yaw, trusted, reason = ya.resolve(36.5, saved=saved_max)
    check(f"overlap file max st=36.5 → yaw={yaw:.2f} trusted", trusted and abs(yaw - 396.5) < 0.01)

    # 5) 겹침 + 파일불일치(움직임) → 호밍
    yaw, trusted, reason = ya.resolve(15.0, saved=saved_max)  # 저장36.98 vs 현재15 → 큼
    check(f"overlap moved st=15 vs saved36.98 → 호밍", (not trusted) and reason == 'moved')

    # 6) max 앵커: 0x94=36.98 lap1 → 396.98
    check("max anchor yaw_from_st(36.98,1)=396.98", abs(ya.yaw_from_st(36.98, 1) - 396.98) < 0.01)

    # 7) 영속 round-trip
    ya.save(396.98, 36.98)
    ld = ya.load()
    check("save/load round-trip", ld is not None and ld['lap'] == 1 and abs(ld['yaw_deg'] - 396.98) < 0.01)
    os.unlink('/tmp/yaw_abs_test.json')

    print(f"\n{'== ALL PASS ==' if ok else '== FAIL =='}")
    return ok


if __name__ == '__main__':
    import sys
    sys.exit(0 if _selftest() else 1)
