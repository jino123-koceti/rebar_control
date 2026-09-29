#!/usr/bin/env python3
"""구조 규칙 검사 — docs/design/YEAR3_ARCHITECTURE.md §6 의 R1~R5.

왜 필요한가: 2차년도에도 계층 설계와 리팩토링 계획 문서가 있었지만 무너졌다.
문서는 위반을 막지 못한다. 규칙은 검사로만 유지된다.

검사 항목
  R1 모터 ID 리터럴 금지   0x14[1-8] 은 axes.yaml 과 프로토콜 파일에만
  R2 자원 소유자 1개       can2/can3/EZIO/시리얼을 여는 파일이 자원별로 하나
  R3 토픽 계약 일치        발행/구독 이름이 짝을 이루는지 (yaw_min vs yaw_home 사고 방지)
  R4 노드 크기 상한        아키텍처 문서의 규모를 넘는 파일
  R5 중복 구현             같은 기능 모듈이 둘 이상 (횡이동·CAN·odom)

기준선(baseline) 방식
  지금은 위반이 많다. 전부 고칠 때까지 검사를 빨간 상태로 두면 아무도 안 본다.
  그래서 현재 위반을 tools/check/baseline.json 에 기록하고, **늘어날 때만 실패**한다.
  줄어들면 "기준선을 갱신하라" 고 알려준다. 리팩토링이 진행되며 기준선이 0 에 수렴한다.

사용
    python3 tools/check/check_structure.py              # 검사
    python3 tools/check/check_structure.py --update     # 기준선 갱신
    python3 tools/check/check_structure.py --strict     # 기준선 무시하고 전부 실패
"""

import argparse
import json
import os
import re
import sys
from collections import defaultdict

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
BASELINE = os.path.join(os.path.dirname(__file__), 'baseline.json')

SKIP_DIRS = ('__pycache__', '/build/', '/install/', '/log/', '/.git/',
             'rebar_control_old', '/docs/', '/runs/')

# R1 — ID 리터럴이 허용되는 파일
R1_ALLOW = ('config/axes.yaml', 'rmd_x4_protocol.py', 'tools/check/')

# R2 — 자원별 개방 패턴. 하나의 물리 버스를 여러 파일이 여는 것을 잡는다.
R2_RESOURCES = {
    'CAN (python-can)': r'can\.Bus\s*\(|can\.interface\.Bus\s*\(',
    'CAN (SocketCAN raw)': r'AF_CAN',
    'EZIO (Plus-E)': r'FAS_Connect\w*\s*\(',
}
# 시리얼은 장치마다 포트가 달라(그리퍼·Pololu) 한 자원으로 묶으면 오판이다.
# 참고 정보로만 출력한다.
R2_INFO = {'시리얼 포트': r'serial\.Serial\s*\('}

# R4 — 아키텍처 문서 §4 의 규모 상한
R4_LIMITS = {
    'motor_bridge': 600, 'position_control_node': 600, 'remote_bridge': 200,
    'ezi_io_node': 300, 'gripper_node': 300, 'seengrip_node': 300,
    'trigger_node': 150, 'pololu_node': 150, 'safety_node': 400,
    'drive_node': 400, 'drive_controller': 400, 'stage_node': 500,
    'joint_controller': 500, 'lateral_node': 300, 'homing_node': 600,
    'homing_controller': 600, 'mode_arbiter': 300, 'navigator': 500,
    'path_follower': 400, 'rebar_controller': 400, 'rebar_drive_node': 800,
    'tying_orchestrator_node': 800, 'sequence_controller': 400,
}

# R5 — 같은 기능이 여러 파일에 구현된 것
R5_GROUPS = {
    '횡이동': (r'lateral', ('lateral_node.py', 'lateral_axes.py', 'lateral_motion.py',
                            'lateral_encoder_calibration.py')),
    'CAN 송수신': (r'can_', ('can_manager.py', 'can_sender.py', 'can_parser.py')),
    '자세·odom': (r'odom|pose', ('encoder_odom.py', 'pose_mux.py', 'odom_to_pose.py')),
}


def py_files():
    for base, dirs, files in os.walk(os.path.join(ROOT, 'src')):
        if any(sk in base + '/' for sk in SKIP_DIRS):
            continue
        for f in files:
            if f.endswith('.py'):
                yield os.path.join(base, f)


def rel(p):
    return os.path.relpath(p, ROOT)


def r1_id_literals():
    """파일 단위로 센다. 줄 번호를 키로 쓰면 한 줄만 고쳐도 '신규 위반' 이 되어
    기준선이 쓸모없어진다 (2026-09-29 실제로 그렇게 됐다)."""
    per_file = {}
    for p in py_files():
        r = rel(p)
        if any(a in r for a in R1_ALLOW):
            continue
        try:
            src = open(p, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for i, line in enumerate(src.splitlines(), 1):
            if re.search(r'0x14[1-8]\b', line):
                # 주석 안의 설명은 봐준다 (문서화 목적)
                code = line.split('#')[0]
                if re.search(r'0x14[1-8]\b', code):
                    per_file[r] = per_file.get(r, 0) + 1
    return [f"{f} ({n}건)" if False else f for f, n in sorted(per_file.items())]


def r2_resource_owners():
    hits = defaultdict(set)
    for p in py_files():
        try:
            src = open(p, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for name, pat in R2_RESOURCES.items():
            if re.search(pat, src):
                hits[name].add(rel(p))
    out = []
    for name, files in sorted(hits.items()):
        if len(files) > 1:
            # 쌍 단위로 낸다. 목록 문자열을 키로 쓰면 파일 하나가 바뀔 때
            # 전체가 '신규' 로 잡혀 무엇이 늘었는지 안 보인다.
            for f in sorted(files):
                out.append(f"{name} ← {f}")
    return out


def r2_info():
    """실패시키지 않는 참고 정보 (장치마다 포트가 다른 시리얼 등)."""
    hits = defaultdict(set)
    for p in py_files():
        try:
            src = open(p, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for name, pat in R2_INFO.items():
            if re.search(pat, src):
                hits[name].add(rel(p))
    return [f"{n}: " + ", ".join(sorted(f)) for n, f in sorted(hits.items())]


def r3_topic_contract():
    """발행/구독 이름 짝을 맞춘다.

    f-string 으로 만든 토픽(예: f'/limit_sensors/{name}')도 인식해야 한다.
    안 그러면 동적으로 발행하는 토픽을 '발행 없음' 으로 오판한다 — 실제로
    ezi_io_node 의 /limit_sensors/* 가 그렇게 잡혔다.
    """
    pub, sub, pub_pat = defaultdict(set), defaultdict(set), []
    for p in py_files():
        try:
            src = open(p, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for t in re.findall(r"create_publisher\(\s*\w+\s*,\s*f?['\"]([^'\"]+)", src):
            if '{' in t:                      # f-string 템플릿 → 정규식으로
                pub_pat.append(re.compile('^' + re.sub(r'\{[^}]*\}', '[^/]+',
                                                      re.escape(t).replace('\\{', '{')
                                                      .replace('\\}', '}')) + '$'))
            else:
                pub[t].add(rel(p))
        for t in re.findall(r"create_subscription\(\s*\w+\s*,\s*f?['\"]([^'\"]+)", src):
            if '{' not in t:
                sub[t].add(rel(p))

    def norm(t):
        return t if t.startswith('/') else '/' + t

    pub_n = {norm(t) for t in pub}
    sub_n = {norm(t) for t in sub}
    out = []
    for t in sorted(sub_n - pub_n):
        if t.startswith(('/tf', '/clock', '/parameter_events', '/zed', '/camera')):
            continue
        if any(pat.match(t) for pat in pub_pat):   # 동적 발행에 해당
            continue
        srcs = ", ".join(sorted(next(v for k, v in sub.items() if norm(k) == t)))
        out.append(f"구독만 있고 발행이 없음: {t}  ({srcs})")
    return out


def r4_node_size():
    out = []
    for p in py_files():
        stem = os.path.basename(p)[:-3]
        limit = R4_LIMITS.get(stem)
        if limit is None:
            continue
        n = sum(1 for _ in open(p, encoding='utf-8', errors='replace'))
        if n > limit:
            # 줄 수를 키에 넣으면 한 줄 고칠 때마다 신규 위반이 된다 → 파일만
            out.append(f"{rel(p)} (상한 {limit})")
    return out


def r5_duplicates():
    out = []
    for group, (_pat, names) in R5_GROUPS.items():
        found = []
        for p in py_files():
            if os.path.basename(p) in names:
                found.append(rel(p))
        if len(found) > 1:
            for f in sorted(found):
                out.append(f"{group} ← {f}")
    return out


def _r1_total():
    """R1 은 파일 단위로 세므로 총 리터럴 건수를 따로 보여준다."""
    total = 0
    for p in py_files():
        r = rel(p)
        if any(al in r for al in R1_ALLOW):
            continue
        try:
            src = open(p, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for line in src.splitlines():
            if re.search(r'0x14[1-8]\b', line.split('#')[0]):
                total += 1
    return total


CHECKS = [
    ('R1 모터 ID 리터럴', r1_id_literals),
    ('R2 자원 소유자', r2_resource_owners),
    ('R3 토픽 계약', r3_topic_contract),
    ('R4 노드 크기', r4_node_size),
    ('R5 중복 구현', r5_duplicates),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--update', action='store_true', help='현재 위반을 기준선으로 저장')
    ap.add_argument('--strict', action='store_true', help='기준선 무시')
    ap.add_argument('--verbose', '-v', action='store_true', help='기준선에 있는 위반도 출력')
    a = ap.parse_args()

    base = {}
    if os.path.exists(BASELINE) and not a.strict:
        base = json.load(open(BASELINE, encoding='utf-8'))

    results, failed, improved = {}, [], []
    for name, fn in CHECKS:
        cur = sorted(fn())
        results[name] = cur
        known = set(base.get(name, []))
        new = [v for v in cur if v not in known]
        gone = [v for v in known if v not in cur]

        extra = ""
        if name.startswith('R1'):
            extra = f" / 리터럴 총 {_r1_total()}건"
        mark = "OK " if not new else "NG "
        print(f"{mark}{name}: 위반 {len(cur)}건{extra}"
              + (f", 신규 {len(new)}건" if new else "")
              + (f", 해소 {len(gone)}건" if gone else ""))
        for v in new:
            print(f"     + {v}")
        if a.verbose:
            for v in cur:
                if v in known:
                    print(f"       {v}")
        if new:
            failed.append(name)
        if gone:
            improved.append((name, len(gone)))

    if a.update:
        json.dump(results, open(BASELINE, 'w', encoding='utf-8'),
                  ensure_ascii=False, indent=1, sort_keys=True)
        total = sum(len(v) for v in results.values())
        print(f"\n기준선 갱신: {BASELINE} (총 {total}건)")
        return 0

    for line in r2_info():
        print(f"   (참고) {line}")

    print()
    if improved:
        print("개선됨 — 기준선을 갱신하세요 (--update):")
        for name, n in improved:
            print(f"  {name}: {n}건 해소")
    if failed:
        print(f"실패: 신규 위반이 있는 검사 {len(failed)}개 — {', '.join(failed)}")
        return 1
    print("통과: 신규 위반 없음")
    return 0


if __name__ == '__main__':
    sys.exit(main())
