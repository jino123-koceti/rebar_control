#!/usr/bin/env python3
"""
Zenoh 수신 테스트 스크립트
로봇에서 실행하여 외부 PC로부터 명령이 도착하는지 확인
"""

import zenoh
import time

def test_receive():
    print("="*70)
    print("Zenoh 수신 테스트 시작")
    print("="*70)

    # Zenoh 세션 열기 (peer 모드)
    config = zenoh.Config()
    print("\n[1] Zenoh 세션 열기 (peer 모드)...")
    session = zenoh.open(config)
    print("    ✅ 세션 열림")

    # 수신 핸들러
    def listener(sample):
        timestamp = time.strftime('%H:%M:%S')
        print(f"\n[{timestamp}] 📥 메시지 수신!")
        print(f"    Key: {sample.key_expr}")
        print(f"    Payload: {sample.payload.decode('utf-8')[:200]}")
        print("-"*70)

    # rebar/command 구독
    print("\n[2] 'rebar/command' 구독 시작...")
    sub = session.declare_subscriber('rebar/command', listener)
    print("    ✅ 구독 시작됨")

    print("\n" + "="*70)
    print("대기 중... (외부 PC에서 명령을 전송하세요)")
    print("종료하려면 Ctrl+C를 누르세요")
    print("="*70 + "\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n종료 중...")

    # 정리
    sub.undeclare()
    session.close()
    print("✅ 정리 완료")

if __name__ == '__main__':
    test_receive()
