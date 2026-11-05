#!/usr/bin/env python3
"""데크끝 판정 실시간 창 — 네 방향을 한 화면에 띄운다.

    source install/setup.bash
    python3 tools/vision/deck_edge_monitor.py --port 8080   # 브라우저로 (권장)
      → http://localhost:8080  — VSCode 원격이면 포트가 자동 전달된다
    DISPLAY=:0 python3 tools/vision/deck_edge_monitor.py    # 젯슨 모니터에 창
    python3 tools/vision/deck_edge_monitor.py --save /tmp/de.png --no-show

왜 필요한가 — 2026-11-05 에 **후방 판정이 거짓 GO 를 냈다.** 데크를 벗어났는데
바닥의 금속 레일을 철근으로 잡아 `rebar_frac 0.727` 이 나왔다. 숫자만 봐서는
알 수 없었고, **무엇을 철근으로 보고 있는지 눈으로 봐야** 알 수 있었다.
판정을 믿기 전에 그 판정이 무엇을 근거로 하는지 보는 창이 있어야 한다.

화면 구성 (방향마다):
    왼쪽   원본 + 빨간 선(배근 끝) + 노란 상자(비율을 재는 중앙 밴드)
    오른쪽 세그 오버레이 — **초록이 '밟을 수 있는 철근'** 이다. 여기가 핵심이다
    제목   rebar_frac 과 판정, 임계

⚠ 추론이 `deck_edge` 노드와 GPU 를 나눠 쓴다. 한 틱에 **한 방향만** 돌려
  부담을 나눈다 (네 방향 한 바퀴 약 2초).
"""
import argparse
import glob
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'src', 'rebar_vision'))
from rebar_vision.rebar_seg import RebarSegmenter, colorize   # noqa: E402
from rebar_vision import deck_edge as de                      # noqa: E402

try:
    from PIL import Image as PILImage, ImageDraw, ImageFont
    _FONT = (glob.glob('/usr/share/fonts/**/NotoSansCJK*', recursive=True) or [None])[0]
except ImportError:
    _FONT = None

VIEWS = [('전방', '/zed_front/zed_node/rgb/color/rect/image'),
         ('후방', '/zed_back/zed_node/rgb/color/rect/image'),
         ('좌측', '/camera_left/color/image_raw'),
         ('우측', '/camera_right/color/image_raw')]
TILE = (520, 292)


def to_bgr(m):
    """⚠ ZED 는 bgra8, Orbbec 은 rgb8 이다. 무조건 변환하면 R·B 가 뒤바뀐다."""
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    e = m.encoding
    if e == 'bgra8':
        return a[:, :, :3].copy()
    if e == 'bgr8':
        return a.copy()
    if e == 'rgba8':
        return cv2.cvtColor(a, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)


def put(img, text, xy, size=26, color=(255, 255, 255)):
    if _FONT is None:
        cv2.putText(img, text, xy, cv2.FONT_HERSHEY_SIMPLEX, size / 36, color, 2)
        return img
    pil = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(pil)
    d.text(xy, text, font=ImageFont.truetype(_FONT, size), fill=tuple(color[::-1]),
           stroke_width=2, stroke_fill=(0, 0, 0))
    img[:] = cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)
    return img


class MJPEG(BaseHTTPRequestHandler):
    """최신 프레임만 내보내는 MJPEG 스트림. 원격(SSH/VSCode)에서 보기 위한 것."""

    frame = None          # 클래스 변수로 공유 (단일 생산자·다중 소비자)

    def log_message(self, *a):
        pass              # 요청마다 찍으면 터미널이 묻힌다

    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=f')
        self.end_headers()
        try:
            while True:
                buf = MJPEG.frame
                if buf is not None:
                    self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(buf)).encode()
                                     + b'\r\n\r\n' + buf + b'\r\n')
                threading.Event().wait(0.2)
        except (BrokenPipeError, ConnectionResetError):
            pass


class Monitor(Node):

    def __init__(self, weights, device, show, save, port=0):
        super().__init__('deck_edge_monitor')
        self.seg = RebarSegmenter(weights, device=device)
        de.set_class_map(self.seg.class_map)
        self.show, self.save = show, save
        if port:
            srv = HTTPServer(('0.0.0.0', port), MJPEG)
            threading.Thread(target=srv.serve_forever, daemon=True).start()
            self.get_logger().info(f'MJPEG 스트림 — http://localhost:{port}')
        self.rgb = {}
        self.panel = {k: None for k, _ in VIEWS}
        self.fsm = {k: de.VerdictFSM() for k, _ in VIEWS}
        for key, topic in VIEWS:
            self.create_subscription(
                Image, topic, lambda m, k=key: self.rgb.__setitem__(k, to_bgr(m)),
                qos_profile_sensor_data)
        self.turn = 0
        self.create_timer(0.5, self.tick)

    def tick(self):
        key, _ = VIEWS[self.turn % len(VIEWS)]
        self.turn += 1
        img = self.rgb.get(key)
        if img is None:
            self.panel[key] = put(np.full((*TILE[::-1], 3), 40, np.uint8),
                                  f'{key} — 영상 없음', (16, 16), 30, (0, 0, 255))
            return self.draw()
        m = self.seg.predict(img)
        h, w = m.shape
        er, frac, on = de.rebar_edge(m)
        j = de.judge(m, self.fsm[key])
        ov = cv2.addWeighted(img, 0.5, colorize(m, self.seg.class_map), 0.5, 0)
        c0, c1, y = int(w * de.CENTER[0]), int(w * de.CENTER[1]), int(er)
        for im_ in (img, ov):
            cv2.line(im_, (0, y), (w, y), (0, 0, 255), 4)
            cv2.rectangle(im_, (c0, y), (c1, h - 1), (0, 255, 255), 3)
        tile = np.hstack([cv2.resize(img, TILE), cv2.resize(ov, TILE)])
        col = {'GO': (0, 220, 0), 'SLOW': (0, 180, 255)}.get(j['verdict'], (0, 0, 255))
        cv2.rectangle(tile, (0, 0), (tile.shape[1], 40), (0, 0, 0), -1)
        put(tile, f"{key}  {frac:.3f}  {j['verdict']}"
                  + (f"  ({j['reason']})" if j['reason'] else ''), (10, 6), 26, col)
        self.panel[key] = tile
        self.draw()

    def draw(self):
        blank = np.full((TILE[1], TILE[0] * 2, 3), 30, np.uint8)
        g = [self.panel[k] if self.panel[k] is not None else blank for k, _ in VIEWS]
        out = np.vstack([np.hstack(g[:2]), np.hstack(g[2:])])
        put(out, 'STOP < 0.45   SLOW < 0.55      빨강=배근 끝 · 노랑=측정 밴드 · '
                 '초록=밟을 수 있는 철근', (12, out.shape[0] - 34), 24, (200, 200, 200))
        ok, buf = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if ok:
            MJPEG.frame = buf.tobytes()
        if self.save:
            cv2.imwrite(self.save, out)
        if self.show:
            cv2.imshow('deck_edge', out)
            cv2.waitKey(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default=os.path.expanduser(
        '~/ros2_ws/src/rebar_control/model/rebar_seg_260914.pt'))
    ap.add_argument('--device', default='cuda')
    ap.add_argument('--save', default='')
    ap.add_argument('--no-show', action='store_true')
    ap.add_argument('--port', type=int, default=0, help='MJPEG 스트림 포트')
    a = ap.parse_args()
    rclpy.init()
    n = Monitor(a.weights, a.device, not a.no_show, a.save, a.port)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
