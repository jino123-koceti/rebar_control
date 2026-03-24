#!/usr/bin/env python3
"""
결속 작업 로그 분석 스크립트

journalctl에서 결속 작업 로그를 파싱하여 포인트별 분석 결과를 출력.

사용법:
  python3 analyze_tying_log.py                    # 최근 1시간
  python3 analyze_tying_log.py --since "15:00"    # 특정 시간 이후
  python3 analyze_tying_log.py --since "1h ago"   # 1시간 전부터
  python3 analyze_tying_log.py --since "15:00" --until "16:00"
"""

import argparse
import subprocess
import re
from collections import defaultdict
from datetime import datetime


def get_logs(since=None, until=None):
    """journalctl에서 robot-control 로그 가져오기"""
    cmd = ['journalctl', '-u', 'robot-control', '--no-pager', '-n', '5000']
    if since:
        cmd += ['--since', since]
    if until:
        cmd += ['--until', until]
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.stdout.splitlines()


def parse_tying_sessions(lines):
    """로그 라인에서 결속 세션 파싱"""
    sessions = []
    current = None

    # 패턴 정의
    p_start = re.compile(r'TYING_START 수신.*속도 (\d+)%.*XY:(\d+)dps.*방향: (\w+)')
    p_auto = re.compile(r'\[AUTO\] Tying 모드 진입.*속도 (\d+)%')
    p_accum = re.compile(r'누적: \(([-\d.]+), ([-\d.]+)\) \[([RL])\]')
    p_multi_done = re.compile(r'멀티검출 완료: (\d+)개 누적 \(R:(\d+), L:(\d+)\)')
    p_skip_neg = re.compile(r'음수 스킵: X=([-\d.]+)mm, Y=([-\d.]+)mm')
    p_skip_range = re.compile(r'범위 외 클러스터 제거: X=([-\d.]+), Y=([-\d.]+)')
    p_skip_over = re.compile(r'범위 외 제거: X=([-\d.]+), Y=([-\d.]+)')
    p_final = re.compile(r'검출 최종: (\d+)개 포인트.*\[(\w+) 패스\]')
    p_point = re.compile(r'\[P(\d+)\] X=([-\d.]+)mm Y=([-\d.]+)mm \((\w+)\)')
    p_move = re.compile(r'이동: P(\d+): X=([-\d.]+)mm, Y=([-\d.]+)mm')
    p_z_down = re.compile(r'Z하강: P(\d+): Z ([-\d.]+)mm 하강')
    p_trigger = re.compile(r'트리거: P(\d+): 트리거')
    p_z_up = re.compile(r'Z상승: P(\d+): Z ([-\d.]+)mm 상승')
    p_complete = re.compile(r'자동 결속 시퀀스 완료.*\((\d+)개 / (\d+)개')
    p_tying_complete = re.compile(r'TYING_COMPLETE 수신: (\d+)pt.*누적 (\d+)/(\d+)pt.*WP\[(\d+)\].*?([\d.]+)초')
    p_timestamp = re.compile(r'^(\w+ \d+ \d+:\d+:\d+)')

    for line in lines:
        ts_match = p_timestamp.search(line)
        ts = ts_match.group(1) if ts_match else ''

        # 세션 시작
        m = p_start.search(line)
        if m:
            if current:
                sessions.append(current)
            current = {
                'start_time': ts,
                'type': 'TYING_START',
                'speed': int(m.group(1)),
                'xy_dps': int(m.group(2)),
                'direction': m.group(3),
                'waypoints': [],
                '_cur_wp': None,
            }
            continue

        m = p_auto.search(line)
        if m:
            if current:
                sessions.append(current)
            current = {
                'start_time': ts,
                'type': 'AUTO',
                'speed': int(m.group(1)),
                'direction': 'auto',
                'waypoints': [],
                '_cur_wp': None,
            }
            continue

        if not current:
            continue

        # 멀티검출 완료 → 새 웨이포인트 시작
        m = p_multi_done.search(line)
        if m:
            wp = {
                'raw_total': int(m.group(1)),
                'raw_right': int(m.group(2)),
                'raw_left': int(m.group(3)),
                'accumulated': [],
                'filtered_neg': [],
                'filtered_range': [],
                'final_points': [],
                'executed': [],
                'pass_type': '',
                'time': ts,
            }
            current['_cur_wp'] = wp
            current['waypoints'].append(wp)
            continue

        # 누적 포인트 (멀티검출 완료 전의 raw 데이터)
        m = p_accum.search(line)
        if m and current.get('_cur_wp') is None:
            # 아직 웨이포인트 미생성, 임시 저장
            if '_pending_accum' not in current:
                current['_pending_accum'] = []
            current['_pending_accum'].append({
                'x': float(m.group(1)),
                'y': float(m.group(2)),
                'cam': m.group(3),
            })
            continue
        elif m and current.get('_cur_wp'):
            current['_cur_wp']['accumulated'].append({
                'x': float(m.group(1)),
                'y': float(m.group(2)),
                'cam': m.group(3),
            })
            continue

        wp = current.get('_cur_wp')
        if not wp:
            continue

        # 음수 스킵
        m = p_skip_neg.search(line)
        if m:
            wp['filtered_neg'].append({
                'x': float(m.group(1)), 'y': float(m.group(2)), 'reason': 'negative'})
            continue

        # 범위 외 제거
        m = p_skip_range.search(line)
        if m:
            wp['filtered_range'].append({
                'x': float(m.group(1)), 'y': float(m.group(2)), 'reason': 'range_cluster'})
            continue

        m = p_skip_over.search(line)
        if m:
            wp['filtered_range'].append({
                'x': float(m.group(1)), 'y': float(m.group(2)), 'reason': 'range_over'})
            continue

        # 검출 최종
        m = p_final.search(line)
        if m:
            wp['final_count'] = int(m.group(1))
            wp['pass_type'] = m.group(2)
            continue

        # 최종 포인트 좌표
        m = p_point.search(line)
        if m:
            wp['final_points'].append({
                'id': int(m.group(1)),
                'x': float(m.group(2)),
                'y': float(m.group(3)),
                'cam': m.group(4),
            })
            continue

        # 이동 실행
        m = p_move.search(line)
        if m:
            wp['executed'].append({
                'id': int(m.group(1)),
                'x': float(m.group(2)),
                'y': float(m.group(3)),
                'time': ts,
            })
            continue

        # 트리거
        m = p_trigger.search(line)
        if m:
            pid = int(m.group(1))
            for ex in wp['executed']:
                if ex['id'] == pid and 'triggered' not in ex:
                    ex['triggered'] = True
            continue

        # TYING_COMPLETE
        m = p_tying_complete.search(line)
        if m:
            wp['complete_pts'] = int(m.group(1))
            wp['cumulative_pts'] = int(m.group(2))
            wp['max_pts'] = int(m.group(3))
            wp['wp_index'] = int(m.group(4))
            wp['duration_sec'] = float(m.group(5))
            continue

        # 시퀀스 완료
        m = p_complete.search(line)
        if m:
            current['total_tied'] = int(m.group(1))
            current['total_detected'] = int(m.group(2))
            current['end_time'] = ts
            continue

    if current:
        sessions.append(current)

    # cleanup
    for s in sessions:
        s.pop('_cur_wp', None)
        s.pop('_pending_accum', None)

    return sessions


def print_report(sessions):
    """분석 리포트 출력"""
    if not sessions:
        print('결속 작업 로그를 찾을 수 없습니다.')
        return

    total_tied_all = 0
    total_detected_all = 0
    total_filtered_all = 0

    for si, session in enumerate(sessions):
        print()
        print('=' * 70)
        print(f'  결속 세션 #{si + 1}')
        print(f'  시작: {session["start_time"]}')
        print(f'  타입: {session["type"]}  속도: {session["speed"]}%  방향: {session.get("direction", "?")}')
        if 'end_time' in session:
            print(f'  종료: {session["end_time"]}')
        if 'total_tied' in session:
            print(f'  총 결속: {session["total_tied"]}/{session["total_detected"]}')
        print('=' * 70)

        session_tied = 0
        session_detected = 0
        session_filtered = 0

        for wi, wp in enumerate(session['waypoints']):
            n_final = wp.get('final_count', len(wp['final_points']))
            n_neg = len(wp['filtered_neg'])
            n_range = len(wp['filtered_range'])
            n_filtered = n_neg + n_range
            n_executed = len([e for e in wp['executed'] if e.get('triggered')])

            session_tied += n_executed
            session_detected += n_final + n_filtered
            session_filtered += n_filtered

            print(f'\n  --- 웨이포인트 #{wi + 1} ({wp["pass_type"]} 패스) ---')
            print(f'  시간: {wp["time"]}')
            print(f'  Raw 검출: {wp["raw_total"]}개 (R:{wp["raw_right"]}, L:{wp["raw_left"]})')

            # 클러스터링 후 유니크 포인트 수
            unique_count = n_final + n_filtered
            print(f'  클러스터링 후: {unique_count}개 → 최종 {n_final}개 + 필터링 {n_filtered}개')

            # 필터링된 포인트
            if wp['filtered_neg']:
                print(f'\n  [필터링 - 음수 영역] {n_neg}개')
                for p in wp['filtered_neg']:
                    print(f'    X={p["x"]:.1f}mm, Y={p["y"]:.1f}mm → 스킵 (X < 0)')

            if wp['filtered_range']:
                print(f'\n  [필터링 - 범위 초과] {n_range}개')
                for p in wp['filtered_range']:
                    reason = '리미트 초과' if 'over' in p['reason'] else '범위 외 클러스터'
                    print(f'    X={p["x"]:.1f}mm, Y={p["y"]:.1f}mm → 스킵 ({reason})')

            # 최종 포인트
            if wp['final_points']:
                print(f'\n  [최종 포인트] {n_final}개')
                for p in wp['final_points']:
                    print(f'    P{p["id"]}: X={p["x"]:.1f}mm, Y={p["y"]:.1f}mm ({p["cam"]})')

            # 실행 결과
            if wp['executed']:
                print(f'\n  [체결 실행] {n_executed}개')
                for e in wp['executed']:
                    status = 'O 체결' if e.get('triggered') else 'X 이동만'
                    print(f'    P{e["id"]}: X={e["x"]:.1f}mm, Y={e["y"]:.1f}mm → {status}')

            # 소요시간
            if 'duration_sec' in wp:
                print(f'\n  소요시간: {wp["duration_sec"]:.1f}초')
                if 'cumulative_pts' in wp:
                    print(f'  누적: {wp["cumulative_pts"]}/{wp["max_pts"]}pt (WP[{wp.get("wp_index", "?")}])')

        total_tied_all += session_tied
        total_detected_all += session_detected
        total_filtered_all += session_filtered

        print(f'\n  --- 세션 요약 ---')
        print(f'  웨이포인트: {len(session["waypoints"])}개')
        print(f'  체결: {session_tied}개')
        print(f'  필터링: {session_filtered}개')
        print(f'  검출률: {session_detected}개 (체결 {session_tied} + 필터링 {session_filtered})')

    # 전체 요약
    print()
    print('=' * 70)
    print('  전체 요약')
    print('=' * 70)
    print(f'  총 세션: {len(sessions)}개')
    print(f'  총 웨이포인트: {sum(len(s["waypoints"]) for s in sessions)}개')
    print(f'  총 체결: {total_tied_all}개')
    print(f'  총 필터링: {total_filtered_all}개')
    if total_detected_all > 0:
        pct = total_tied_all / total_detected_all * 100
        print(f'  체결률: {total_tied_all}/{total_detected_all} ({pct:.0f}%)')

    # 필터링 사유 통계
    all_neg = []
    all_range = []
    for s in sessions:
        for wp in s['waypoints']:
            all_neg.extend(wp['filtered_neg'])
            all_range.extend(wp['filtered_range'])

    if all_neg or all_range:
        print(f'\n  --- 필터링 사유 통계 ---')
        if all_neg:
            print(f'  음수 영역 (X < 0): {len(all_neg)}개')
            for p in all_neg:
                print(f'    X={p["x"]:.1f}, Y={p["y"]:.1f}')
        if all_range:
            print(f'  범위 초과 (X > 411 or Y 범위 밖): {len(all_range)}개')
            for p in all_range:
                print(f'    X={p["x"]:.1f}, Y={p["y"]:.1f} ({p["reason"]})')

    print()


def main():
    parser = argparse.ArgumentParser(description='결속 작업 로그 분석')
    parser.add_argument('--since', default='1h ago',
                        help='분석 시작 시간 (기본: 1h ago)')
    parser.add_argument('--until', default=None,
                        help='분석 종료 시간')
    args = parser.parse_args()

    print(f'로그 조회: --since "{args.since}"' +
          (f' --until "{args.until}"' if args.until else ''))

    lines = get_logs(since=args.since, until=args.until)
    print(f'로그 라인: {len(lines)}개')

    sessions = parse_tying_sessions(lines)
    print_report(sessions)


if __name__ == '__main__':
    main()
