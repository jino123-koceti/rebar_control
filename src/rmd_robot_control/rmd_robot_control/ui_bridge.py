"""웹 UI 브릿지 — 브라우저에서 호밍·미션을 돌리고 상태를 본다.

왜 웹인가 (2026-10-07): 리허설에서 터미널에 `ros2 topic pub --once ...` 를 치는
것은 느리고 틀리기 쉽다. 특히 **멈춰야 할 때** 긴 명령을 치는 것이 위험하다.

왜 DDS 직결이 아닌가: ROS 2 기본 탐색은 **멀티캐스트**다. 공유기·AP 를 거치면
멀티캐스트를 막거나 흘리지 않는 경우가 흔해 "같은 네트워크인데 토픽이 안 보인다"
가 된다. 리허설 당일에 그걸 디버깅하게 된다. HTTP 는 유니캐스트라 라우터만
통과하면 되고, 노트북에 ROS 2 를 깔 필요도 없다.

왜 2차년도 체인을 안 쓰나: `zenoh_client` 는 `/mission/command`(String) 를 쓰는데
3차년도는 `/mission/start`·`/mission/abort`(Empty) 와 `/homing_cmd`(String) 다.
그대로 띄우면 **명령이 아무 데도 안 닿는다.**

⚠⚠ **이 UI 의 정지를 1차 수단으로 삼지 말 것.** 가장 빠르고 확실한 정지는
  **리모콘 비상정지**다. 여기는 브라우저→와이파이→HTTP→ROS 를 거치므로 그만큼
  느리고, 통신이 끊기면 눌러도 안 간다. 보조 수단이다.

⚠ 호밍은 **자동으로 돌지 않는다.** 사람이 눌러야 한다 — 되살아난 즉시 자동으로
  돌면 아무도 시키지 않았는데 장비가 움직이고, `respawn` 과 겹치면 크래시 루프가
  재호밍 루프가 된다 (런치 주석 참고).

규격은 `docs/guides/UI_INTERFACE.md` 에 있다.

  GET  /            UI 페이지 (같이 들어 있다)
  GET  /state       상태 JSON 한 덩어리
  POST /cmd/homing  전체 호밍
  POST /cmd/mission 미션 시작
  POST /cmd/stop    정지 (세 토픽 동시)

사용
  ros2 run rmd_robot_control ui_bridge
  브라우저에서  http://<젯슨IP>:8080
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE_FILE = os.path.join(HERE, 'ui_page.html')


class Bridge(Node):
    """상태를 모아 두고 명령을 쏜다. 웹 스레드가 `snapshot()` 을 읽는다."""

    def __init__(self):
        super().__init__('ui_bridge')
        self.declare_parameter('port', 8080)
        self.port = int(self.get_parameter('port').value)

        self._raw = {}
        self._seen = {}            # 토픽별 마지막 수신 시각
        self.estop = None
        self.safety = None         # estop 외에 막는 조건까지 — _on_safety 참고
        self.estop_t = 0.0

        self.homing_pub = self.create_publisher(String, '/homing_cmd', 10)
        self.mission_pub = self.create_publisher(Empty, '/mission/start', 10)
        self.abort_pub = self.create_publisher(Empty, '/mission/abort', 10)
        self.stage_stop = self.create_publisher(Empty, '/stage/stop', 10)

        for key, topic in (('homing', '/homing_status'),
                           ('mission', '/mission/status'),
                           ('stage', '/stage/status'),
                           ('drive', '/drive/status')):
            self.create_subscription(
                String, topic,
                (lambda k: (lambda m: self._set(k, m.data)))(key), 10)
        try:
            from rebar_base_interfaces.msg import SafetyState
            self.create_subscription(SafetyState, '/safety/state',
                                     self._on_safety, 1)
        except ImportError:
            self.get_logger().warning(
                'SafetyState 를 못 불러왔다 — 비상정지 표시가 빠진다')

    def _set(self, key, raw):
        try:
            self._raw[key] = json.loads(raw)
            self._seen[key] = time.time()
        except ValueError:
            pass                      # 깨진 JSON 은 버린다. 옛 값을 그대로 둔다

    def _on_safety(self, msg):
        # ⚠⚠ **estop 만 보면 안 된다.** 2026-10-07 에 STOP 스위치가 눌린 채로
        #   화면이 "비상정지 해제" 를 띄웠고, [전체 호밍] 을 눌러도 중재기가
        #   조용히 거절했다 — 눌렀는데 아무 일도 안 일어나고 이유가 화면에
        #   없었다. 막는 조건을 **전부** 넘긴다.
        self.estop = bool(msg.estop)
        self.safety = {
            'estop': bool(msg.estop),
            'stop_switch': bool(getattr(msg, 'stop_switch', False)),
            'inputs_stale': bool(getattr(msg, 'inputs_stale', False)),
            'blocked_axes': list(getattr(msg, 'blocked_axes', [])),
            'reason': str(getattr(msg, 'reason', '')),
        }
        self.estop_t = time.time()

    def snapshot(self):
        """⚠ **얼마나 오래됐는지도 같이 준다.** 값만 주면 UI 가 '멈춘 로봇' 과
        '끊긴 통신' 을 구분하지 못한다 — 화면상 똑같아 보인다."""
        now = time.time()
        age = {k: round(now - t, 1) for k, t in self._seen.items()}
        if self.estop is not None:
            age['safety'] = round(now - self.estop_t, 1)
        return {**{k: self._raw.get(k, {}) for k in
                   ('homing', 'mission', 'stage', 'drive')},
                'estop': self.estop, 'safety': self.safety,
                'age_sec': age, 'now': round(now, 1)}

    def command(self, what):
        if what == 'homing':
            self.homing_pub.publish(String(data='all'))
        elif what == 'mission':
            self.mission_pub.publish(Empty())
        elif what == 'stop':
            # ⚠ **세 경로로 한 번에.** 어느 한 노드가 죽어 있어도 나머지가
            #   멈춘다. 멈추는 쪽은 중복이 안전하다.
            self.homing_pub.publish(String(data='stop'))
            self.abort_pub.publish(Empty())
            self.stage_stop.publish(Empty())
        else:
            return False
        self.get_logger().info(f'UI 명령: {what}')
        return True


def make_handler(node):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass                      # 접속 로그로 터미널을 덮지 않는다

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            # UI 를 노트북에서 띄우든 장비에서 받아가든 둘 다 되게 연다
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(body)

        def do_OPTIONS(self):
            self._send(204, b'', 'text/plain')

        def do_GET(self):
            if self.path.startswith('/state'):
                body = json.dumps(node.snapshot(), ensure_ascii=False).encode()
                self._send(200, body, 'application/json; charset=utf-8')
                return
            try:
                with open(PAGE_FILE, 'rb') as f:
                    self._send(200, f.read(), 'text/html; charset=utf-8')
            except OSError:
                self._send(500, 'ui_page.html 을 못 읽었다'.encode(),
                           'text/plain; charset=utf-8')

        def do_POST(self):
            if self.path.startswith('/cmd/'):
                ok = node.command(self.path.rsplit('/', 1)[-1])
                self._send(200 if ok else 400,
                           json.dumps({'ok': ok}).encode(), 'application/json')
            else:
                self._send(404, b'{}', 'application/json')
    return H


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    srv = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f'웹 UI — http://<젯슨IP>:{node.port}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
