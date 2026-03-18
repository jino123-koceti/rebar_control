Jetson에서 경로 전송 방법
1. 전송 시점
경로 생성이 끝났을 때 rebar/status 메시지에 mission_status: "path_gen_complete"와 함께 waypoints를 포함해 보냅니다.

2. 전송 채널
토픽: rebar/status
형식: MessagePack 바이너리
별도 토픽 없이 기존 status 메시지에 포함해 보냅니다.
3. 메시지 구조 예시
status = {
    "timestamp": "2026-03-13 16:50:31",
    "control_mode": "manual",
    "mission_status": "path_gen_complete",   # 또는 state / sate
    "position": {"x": 2367.97, "y": 371.59, "theta": 0.138},
    "speed": 0.0,
    "current_waypoint": 0,
    "total_waypoints": 5,                    # 생성된 웨이포인트 개수
    "waypoint_count": 5,                     # 위와 동일 (둘 중 하나만 있어도 됨)
    "waypoints": [                           # ★ 필수: 맵에 표시할 경로
        {"x": 497.78, "y": 0.76},
        {"x": 1000.0, "y": 100.0},
        {"x": 1500.0, "y": 200.0},
        {"x": 2000.0, "y": 300.0},
        {"x": 2368.0, "y": 371.62}
    ],
    "work_area": { ... },                    # 기존 work_area 유지
    "errors": []
}
4. waypoints 형식
항목	설명
위치	status 최상위의 waypoints (또는 work_area.waypoints)
단위	mm (position, work_area와 동일)
형식	[{"x": float, "y": float}, ...]
대소문자	x/y 또는 X/Y 모두 지원
5. Python 발행 예시
import msgpack
# 경로 생성 완료 시
status = {
    "mission_status": "path_gen_complete",
    "position": {"x": 2367.97, "y": 371.59, "theta": 0.138},
    "current_waypoint": 0,
    "total_waypoints": len(waypoints_list),
    "waypoints": [
        {"x": float(x_mm), "y": float(y_mm)}
        for (x_mm, y_mm) in waypoints_list
    ],
    "work_area": work_area_dict,  # 기존 work_area
    # ... 기타 필드
}
payload = msgpack.packb(status)
# Zenoh로 rebar/status에 발행
6. 주의사항
최소 1회 포함: path_gen_complete가 처음 들어올 때 waypoints를 포함해야 UI 맵에 경로가 표시됩니다.
이후 생략 가능: 같은 status를 반복 보낼 때는 waypoints를 생략해도 됩니다. UI는 첫 수신 시에만 맵을 갱신합니다.
좌표 단위: 반드시 mm로 보내야 합니다. (position, work_area와 동일)
요약
필드	필수	설명
mission_status	✓	"path_gen_complete"
waypoints	✓	[{"x": mm, "y": mm}, ...]
waypoint_count 또는 total_waypoints	권장	웨이포인트 개수 표시용
이 형식으로 rebar/status에 포함해 보내면 UI 맵에 경로가 표시됩니다.