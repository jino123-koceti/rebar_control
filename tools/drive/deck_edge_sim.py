#!/usr/bin/env python3
"""데크끝 주행 상태머신 오프라인 리플레이 (무모션 검증).

로봇을 안 움직이는 dry-run은 시야가 안 바뀌어 STOP 전이가 안 옴 → 검증 불가.
대신 **로봇이 실제로 데크끝까지 주행한 녹화영상**을 상태머신에 통과시켜
FWD→SLOW→STOP→(settle)→REV→SLOW→STOP→DONE 전이를 재현·검증한다.

deck_edge_analyze가 만든 CSV(deck_frac 프레임별)에 drive-test와 동일한 VerdictFSM
(카메라별 임계)을 적용 → 전이 시점 리포트. seg 재실행 불필요(즉시).

  python3 tools/drive/deck_edge_sim.py \
     --front data/drive_rec/<ts>/front_edge.csv --back .../back_edge.csv
     [--front-stop 0.55 --front-slow 0.66 --back-stop 0.80 --back-slow 0.86 --settle 1.5]
"""
import csv
import argparse
import sys
import os

sys.path.insert(0, os.path.join('/home/koceti/ros2_ws', 'tools/drive'))
from deck_edge_analyze import VerdictFSM   # noqa: E402


def load(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    return [(int(r['frame']), float(r['t_s']), float(r['deck_frac'])) for r in rows]


def replay(seq, stop, slow, smooth, label):
    """CSV deck_frac 시퀀스 → VerdictFSM. 첫 SLOW/STOP 시점 반환 + 전환 로그."""
    fsm = VerdictFSM(stop, slow, smooth)
    prev = None
    first_slow = first_stop = None
    print(f"\n=== {label} (임계 stop<{stop}/slow<{slow}) ===")
    for f, t, d in seq:
        vd, sm, _ = fsm.step(d)
        if vd != prev:
            print(f"  f{f:>4} {t:>5.1f}s  deck_frac_sm={sm:.2f}  → {vd}")
            if vd == 'SLOW' and first_slow is None:
                first_slow = t
            if vd == 'STOP' and first_stop is None:
                first_stop = t
            prev = vd
    return first_slow, first_stop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--front', required=True, help='front_edge.csv')
    ap.add_argument('--back', required=True, help='back_edge.csv')
    ap.add_argument('--front-stop', type=float, default=0.55)
    ap.add_argument('--front-slow', type=float, default=0.66)
    ap.add_argument('--back-stop', type=float, default=0.80)
    ap.add_argument('--back-slow', type=float, default=0.86)
    ap.add_argument('--smooth', type=int, default=5)
    ap.add_argument('--settle', type=float, default=1.5)
    args = ap.parse_args()

    fs, fst = replay(load(args.front), args.front_stop, args.front_slow,
                     args.smooth, 'FWD(전진) — front.mp4')
    bs, bst = replay(load(args.back), args.back_stop, args.back_slow,
                     args.smooth, 'REV(후진) — back.mp4')

    print("\n" + "=" * 56)
    print("전체 주행테스트 시퀀스 시뮬레이션:")
    if fst is None:
        print("  ⚠ FWD: front 영상에서 STOP 미발생 (데크끝 접근 footage 없음/임계 확인)")
        return
    print(f"  FWD  전진 → SLOW {fs:.1f}s → STOP {fst:.1f}s (전면 데크끝) ✓")
    print(f"  ↓ settle {args.settle}s")
    if bst is None:
        print("  ⚠ REV: back 영상에서 STOP 미발생")
        return
    print(f"  REV  후진 → SLOW {bs:.1f}s → STOP {bst:.1f}s (후면 데크끝) ✓")
    print("  → DONE")
    print("\n✅ 상태전이 FWD→STOP→REV→STOP 정상 (실제 접근 footage 기준)")
    print("   실주행 노드도 이 판정으로 감속·정지함 (동일 VerdictFSM·임계).")


if __name__ == '__main__':
    main()
