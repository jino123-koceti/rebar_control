#!/usr/bin/env python3
"""주행 영상 기반 데크끝(deck-edge) 판정 테스트 (철근배근 기반).

⚠ 주행규칙: 로봇 무한궤도는 **철근 배근 위를 밟고** 주행한다.
  → **주행가능 = rebar_h/rebar_v 가 이어져 있는 곳**뿐이다.
  → 방수포(background)·floor·wall·obstacle·human 은 전부 **주행불가**.
  (구버전은 background를 주행가능에 포함했으나, 배근 없는 맨 방수포 위를 GO로
   오판했고 카메라별 baseline도 갈렸다. 2026-08-06 철근 기반으로 전환.)

7클래스 DeepLabV3+ seg로 프레임별:
  · 행별 철근비율 r(y) = 중앙컬럼 중 rebar_h|rebar_v 픽셀 비율 (수직 스무딩)
  · 발밑(하단)의 지지행부터 위로 스캔, GAP_FRAC 이상 끊기면 거기가 배근 끝 → edge_row
  · **rebar_frac = (H-edge_row)/H** = 배근이 이어진 세로범위
      (front: 정상 0.60 / 접근 0.42 / 데크끝 0.15,  back: 정상 0.63 / 데크끝 0.43)
  · 발밑에 지지행이 없으면(배근 이탈) rebar_frac=0 → 즉시 STOP
  · 부가 안전: 전방밴드 obstacle/human 침범(obs_nearmid) → STOP
→ 주석영상(seg오버레이+경계선+판정) + CSV. 임계값 시각 튜닝용.
  참고로 구버전 deck_frac(방수포 포함)도 CSV에 같이 기록해 비교 가능.

  python3 tools/drive/deck_edge_analyze.py --video data/drive_rec/<ts>/front.mp4
     [--stride 2] [--stop 0.45] [--slow 0.55] [--weights .../retrain_best_260804.pt]
저장: <video>_edge.mp4, <video>_edge.csv
"""
import os
import sys
import csv
import argparse
import numpy as np
import cv2

# 판정 로직·세그멘터는 패키지(rebar_vision)가 정본. 이 도구는 그걸 영상에 적용해
# 임계값을 눈으로 튜닝하는 용도다. (예전엔 반대로 여기에 로직이 있었으나, ROS 노드가
#  tools/를 import할 수 없어 rebar_vision/deck_edge.py + rebar_seg.py로 이관했다.)
ROOT = '/home/koceti/ros2_ws'
# 소스를 먼저 보게 고정한다. install/ 의 빌드본이 앞서면 튜닝 중인 소스 변경이
# 반영되지 않아 "고쳤는데 결과가 그대로"인 혼란이 생긴다.
sys.path.insert(0, os.path.join(ROOT, 'src/rebar_vision'))
from rebar_vision.rebar_seg import RebarSegmenter, colorize          # noqa: E402
from rebar_vision.deck_edge import (                                 # noqa: E402
    rebar_edge, deck_edge, band_features, VerdictFSM, COLOR,
    CENTER, STOP_THR, SLOW_THR, OBS_THR)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--weights',
                    default=os.path.join(ROOT, 'src/rebar_vision/model/retrain_best_260804.pt'))
    ap.add_argument('--stride', type=int, default=2, help='N프레임마다 처리(속도)')
    ap.add_argument('--stop', type=float, default=0.45,
                    help='rebar_frac<이 값 → STOP (데크끝 실측 front 0.15/back 0.43)')
    ap.add_argument('--slow', type=float, default=0.55,
                    help='rebar_frac<이 값 → SLOW (정상주행 실측 front 0.60/back 0.63)')
    ap.add_argument('--obs', type=float, default=0.06,
                    help='전방밴드 obstacle/human 비율>이 값 → 즉시 STOP')
    ap.add_argument('--smooth', type=int, default=5, help='rebar_frac 중앙값 스무딩 창')
    ap.add_argument('--imgsz', type=int, default=512)
    ap.add_argument('--device', default=None)
    args = ap.parse_args()

    import torch
    dev = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    print(f'모델 로드: {args.weights} (device={dev})')
    seg = RebarSegmenter(args.weights, dev, args.imgsz)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f'⚠ 영상 열기 실패: {args.video}'); return
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 15
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    base = os.path.splitext(args.video)[0]
    out_mp4, out_csv = base + '_edge.mp4', base + '_edge.csv'
    vw = cv2.VideoWriter(out_mp4, cv2.VideoWriter_fourcc(*'mp4v'),
                         fps / args.stride, (w, h))
    cf = open(out_csv, 'w', newline='')
    wr = csv.writer(cf); wr.writerow(
        ['frame', 't_s', 'edge_row', 'rebar_frac', 'rebar_frac_sm', 'on_rebar',
         'vreb_mid', 'vreb_far', 'hreb_mid', 'nondeck_mid', 'obs_nearmid',
         'deck_frac_legacy', 'verdict'])
    print(f'처리: {n}프레임 stride={args.stride} smooth={args.smooth} → {out_mp4}')

    c0, c1 = int(w * CENTER[0]), int(w * CENTER[1])
    fsm = VerdictFSM(args.stop, args.slow, args.smooth)
    i = 0
    counts = {'GO': 0, 'SLOW': 0, 'STOP': 0}
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        if i % args.stride != 0:
            i += 1; continue
        mask = seg.predict(fr)
        edge_row, rebar_frac, on_rebar = rebar_edge(mask)         # ★ 철근배근 기반 주판정
        bf = band_features(mask)
        hard = (not on_rebar) or bf['obs_nearmid'] > args.obs      # 배근이탈/장애물=즉시정지
        vd, sm, col = fsm.step(rebar_frac, hard)
        _, deck_frac_legacy, _ = deck_edge(mask)                   # (구버전 비교 기록용)
        counts[vd] += 1
        over = cv2.addWeighted(fr, 0.6, colorize(mask), 0.4, 0)
        cv2.line(over, (c0, min(int(edge_row), h - 1)),
                 (c1, min(int(edge_row), h - 1)), col, 2)
        why = '' if not hard else (' OFF-REBAR' if not on_rebar else ' OBSTACLE')
        cv2.putText(over, f'{vd}{why}  rebar_frac={sm:.2f} (raw {rebar_frac:.2f})',
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.putText(over, f'vreb_mid={bf["vreb_mid"]:.2f} obs={bf["obs_nearmid"]:.2f} '
                          f'(legacy deck_frac={deck_frac_legacy:.2f})',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(over, f'stop<{args.stop} slow<{args.slow} sm{args.smooth}  f{i}',
                    (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        vw.write(over)
        wr.writerow([i, round(i / fps, 2), int(edge_row), round(rebar_frac, 3),
                     round(sm, 3), int(on_rebar),
                     round(bf['vreb_mid'], 3), round(bf['vreb_far'], 3),
                     round(bf['hreb_mid'], 3), round(bf['nondeck_mid'], 3),
                     round(bf['obs_nearmid'], 3), round(deck_frac_legacy, 3), vd])
        i += 1
    cap.release(); vw.release(); cf.close()
    tot = sum(counts.values())
    print(f'완료: {tot}프레임  GO={counts["GO"]} SLOW={counts["SLOW"]} STOP={counts["STOP"]}')
    print(f'  주석영상: {out_mp4}')
    print(f'  CSV: {out_csv}')


if __name__ == '__main__':
    main()
