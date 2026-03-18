# Rebar UI 개발 가이드

**작성일:** 2025-12-17
**대상:** 외부 노트북(Laptop) 기반 PyQt UI 개발
**통신 프로토콜:** Zenoh

---

## 목차

1. [시스템 개요](#시스템-개요)
2. [UI 요구사항](#ui-요구사항)
3. [통신 프로토콜 (Zenoh)](#통신-프로토콜-zenoh)
4. [UI 구현 가이드](#ui-구현-가이드)
5. [메시지 포맷](#메시지-포맷)
6. [UI 개발 체크리스트](#ui-개발-체크리스트)
7. [예제 코드](#예제-코드)

---

## 시스템 개요

### 아키텍처

```
┌─────────────────────────────────────────────────────────┐
│                  Laptop (UI)                             │
│  ┌───────────────────────────────────────────────────┐  │
│  │  PyQt Application                                 │  │
│  │  - 맵 생성 및 시각화                              │  │
│  │  - 경로 생성 (마우스 클릭)                        │  │
│  │  - 제어 버튼 (홈, 시작, 정지)                     │  │
│  │  - 상태 모니터링 (위치, 속도, 배터리)             │  │
│  │  - 로그 출력                                      │  │
│  └───────────────────────────────────────────────────┘  │
│                         ↕                                │
│  ┌───────────────────────────────────────────────────┐  │
│  │  Zenoh Client                                     │  │
│  │  - 명령 전송: "rebar/command"                     │  │
│  │  - 상태 수신: "rebar/status", "rebar/pose"        │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
                         ↕ WiFi Network
┌─────────────────────────────────────────────────────────┐
│                  Jetson AGX Orin (Robot)                 │
│  ┌───────────────────────────────────────────────────┐  │
│  │  zenoh_client.py (rebar_control)                  │  │
│  │  - Zenoh ↔ ROS2 브릿지                            │  │
│  └───────────────────────────────────────────────────┘  │
│                         ↕                                │
│  ┌───────────────────────────────────────────────────┐  │
│  │  ROS2 Nodes (rebar_control, rebar_base_control)  │  │
│  │  - navigator, rebar_controller                    │  │
│  │  - drive_controller, can_sender                   │  │
│  └───────────────────────────────────────────────────┘  │
│                         ↕                                │
│              Hardware (CAN, ZED X)                       │
└─────────────────────────────────────────────────────────┘
```

---

## UI 요구사항

### 1. 맵 생성 및 시각화

#### 작업 영역 정의
- 로봇을 원점 (0, 0)에 위치
- 리모콘(Manual 모드)으로 사각형 영역 주행
  - 예: (0, 0) → (1000, 0) → (1000, 1000) → (0, 1000) → (0, 0)
- UI에서 작업 영역 표시 (회색 사각형)

#### 맵 위젯 기능
- **2D 맵 렌더링** (QPainter 사용)
- **좌표계:** mm 단위 (0~1000 x 0~1000)
- **표시 요소:**
  - 작업 영역 경계 (회색 사각형)
  - 계획 경로 (파란 선)
  - 실제 주행 경로 (빨간 선, 실시간 업데이트)
  - 로봇 현재 위치 (녹색 원 + 방향 표시)
  - 웨이포인트 마커 (파란 점)

#### 마우스 인터랙션
- **좌클릭:** 웨이포인트 추가
- **우클릭:** 웨이포인트 삭제
- **휠:** 확대/축소 (Zoom)
- **드래그:** 맵 이동 (Pan)

---

### 2. 제어 패널

#### 주요 버튼
```
┌──────────────────────────┐
│  [홈 위치로 이동]         │
│  [경로 전송]              │
│  [자율 작업 시작]         │
│  [일시 정지]              │
│  [작업 재개]              │
│  [작업 중단]              │
│  [비상 정지] (빨간색)     │
└──────────────────────────┘
```

#### 모드 표시
```
현재 모드: [MANUAL / AUTO / NAVIGATING / IDLE / EMERGENCY_STOP]
```

---

### 3. 상태 모니터링

#### 표시 정보
```
┌─────────────────────────────────┐
│ 로봇 상태                        │
├─────────────────────────────────┤
│ 위치: X: 123.4 mm, Y: 456.7 mm  │
│ 방향: 45.5°                     │
│ 속도: 1.2 m/s                   │
│                    │
│                      │
│                                 │
│ 미션 진행                        │
├─────────────────────────────────┤
│ 현재 웨이포인트: 3 / 10          │
│ 진행률: ████████░░ 80%          │
│ 상태: Navigating                │
└─────────────────────────────────┘
```

---

### 4. 로그 출력

#### 로그 위젯
```
┌─────────────────────────────────────────────────────┐
│ [2025-12-17 14:23:45] Navigator: 목표 지점 3 도달   │
│ [2025-12-17 14:23:40] Drive: 속도 5.2 m/s          │
│ [2025-12-17 14:23:35] Mission: 자율 작업 시작       │
│ [2025-12-17 14:23:30] Zenoh: 연결 성공              │
└─────────────────────────────────────────────────────┘
```

#### 로그 레벨
- **INFO:** 일반 정보 (검은색)
- **WARNING:** 경고 (주황색)
- **ERROR:** 오류 (빨간색)
- **SUCCESS:** 성공 (초록색)

---

## 통신 프로토콜 (Zenoh)

### Zenoh 개요

**Zenoh (Zero Overhead Network Protocol):**
- 경량 pub-sub 프로토콜
- DDS보다 낮은 latency
- WiFi, 인터넷 환경에서 효율적
- Python, C++, Rust 등 다양한 언어 지원

### 설치

```bash
# Laptop (UI 개발 환경)
pip3 install eclipse-zenoh msgpack
```

### Zenoh 모드

**Peer 모드 (추천):**
- 같은 네트워크에서 자동 discovery
- Router 불필요
- 설정 간단

```python
import zenoh

# Peer 모드로 세션 열기
session = zenoh.open()
```

**Client 모드 (옵션):**
- 별도 Zenoh Router 필요
- 다른 네트워크 환경에서 사용

```python
# Client 모드 (Router 주소 지정)
config = zenoh.Config()
config.insert_json5("connect/endpoints", '["tcp/192.168.1.100:7447"]')
session = zenoh.open(config)
```

---

## 통신 채널 정의

### 1. UI → Robot (명령 전송)

#### Zenoh Key
```
"rebar/command"
```

#### Payload 형식
**타입:** String (UTF-8)

#### 지원 명령어

| 명령어 | 설명 | Payload 예시 |
|--------|------|--------------|
| `E-STOP` | 비상 정지 | `"E-STOP"` |
| `STOP` | 정지 | `"STOP"` |
| `MANUAL` | Manual 모드 전환 | `"MANUAL"` |
| `AUTO` | Auto 모드 전환 | `"AUTO"` |
| `GO_HOME` | 홈 위치로 이동 | `"GO_HOME"` |
| `PLAN_PATH` | 경로 계획 시작 | `"PLAN_PATH"` |
| `START_MISSION` | 자율 작업 시작 | `"START_MISSION"` |
| `PAUSE_MISSION` | 작업 일시정지 | `"PAUSE_MISSION"` |
| `RESUME_MISSION` | 작업 재개 | `"RESUME_MISSION"` |
| `ABORT_MISSION` | 작업 중단 | `"ABORT_MISSION"` |
| `WAYPOINTS:<json>` | 웨이포인트 전송 | `"WAYPOINTS:{...}"` |

#### 웨이포인트 전송 형식

```json
{
  "command": "WAYPOINTS",
  "waypoints": [
    {"x": 0.0, "y": 0.0},
    {"x": 100.0, "y": 0.0},
    {"x": 100.0, "y": 100.0},
    {"x": 200.0, "y": 100.0}
  ]
}
```

전송 시:
```python
import json

waypoints_data = {
    "command": "WAYPOINTS",
    "waypoints": [
        {"x": 0.0, "y": 0.0},
        {"x": 100.0, "y": 0.0},
        # ...
    ]
}

json_str = json.dumps(waypoints_data)
command = f"WAYPOINTS:{json_str}"
session.put("rebar/command", command)
```

---

### 2. Robot → UI (상태 수신)

#### 2-1. 로봇 상태 정보

**Zenoh Key:**
```
"rebar/status"
```

**Payload 형식:**
- **타입:** MessagePack (binary)
- **주기:** 10Hz (0.1초마다)

**데이터 구조 (Python dict):**

```python
{
    "timestamp": "2025-12-17 14:22:47",
    "control_mode": "auto",           # 제어 모드
    "mission_status": "navigating",   # 미션 상태
    "position": {
        "x": 123.4,                   # mm
        "y": 456.7,                   # mm
        "theta": 1.57                 # radian
    },
    "speed": 1.2,                     # m/s
    "heading": 45.5,                  # degrees
    "battery": 87.5,                  # %
    "temperature": 45,                # °C
    "current_waypoint": 3,            # 현재 웨이포인트 번호
    "total_waypoints": 10,            # 전체 웨이포인트 수
    "errors": []                      # 에러 메시지 리스트
}
```

**control_mode 값:**
- `"idle"` - 대기
- `"manual"` - Manual 모드 (리모콘)
- `"auto"` - Auto 모드
- `"navigating"` - 자율 주행 중
- `"emergency_stop"` - 비상 정지

**mission_status 값:**
- `"idle"` - 대기
- `"planning"` - 경로 계획 중
- `"navigating"` - 주행 중
- `"mission_done"` - 미션 완료
- `"error"` - 오류 발생

---

#### 2-2. 로봇 위치 정보 (고주파)

**Zenoh Key:**
```
"rebar/pose"
```

**Payload 형식:**
- **타입:** MessagePack (binary)
- **주기:** 20Hz (0.05초마다)

**데이터 구조:**

```python
{
    "timestamp": "2025-12-17 14:22:47.123",
    "x": 123.4,           # mm
    "y": 456.7,           # mm
    "theta": 1.57,        # radian
    "speed": 1.2          # m/s
}
```

**용도:**
- 실시간 로봇 위치 업데이트 (맵에 표시)
- 주행 경로 추적 (빨간 선)

---

## UI 구현 가이드

### 디렉토리 구조 (권장)

```
rebar_ui/
├─ main.py                    # 메인 진입점
├─ ui/
│  ├─ main_window.py          # 메인 윈도우 (QMainWindow)
│  ├─ map_widget.py           # 맵 위젯 (QWidget)
│  ├─ control_panel.py        # 제어 패널 (QWidget)
│  ├─ status_panel.py         # 상태 표시 (QWidget)
│  └─ log_widget.py           # 로그 출력 (QTextEdit)
├─ core/
│  ├─ zenoh_bridge.py         # Zenoh 통신 (QThread)
│  ├─ data_model.py           # 데이터 모델
│  └─ constants.py            # 상수 정의
├─ resources/
│  ├─ icons/                  # 아이콘 파일
│  └─ styles.qss              # Qt 스타일시트
└─ requirements.txt
```

### requirements.txt

```
PyQt5>=5.15.0
eclipse-zenoh>=0.11.0
msgpack>=1.0.0
numpy>=1.20.0
```

---

### 핵심 클래스 구조

#### 1. main_window.py

```python
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QHBoxLayout
from ui.map_widget import MapWidget
from ui.control_panel import ControlPanel
from ui.status_panel import StatusPanel
from ui.log_widget import LogWidget
from core.zenoh_bridge import ZenohBridge

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Rebar Control System")
        self.setGeometry(100, 100, 1400, 900)

        # Zenoh 통신 스레드
        self.zenoh_bridge = ZenohBridge()
        self.zenoh_bridge.status_received.connect(self.on_status_update)
        self.zenoh_bridge.pose_received.connect(self.on_pose_update)
        self.zenoh_bridge.start()

        # UI 구성
        self.setup_ui()

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 메인 레이아웃
        main_layout = QHBoxLayout(central_widget)

        # 왼쪽: 제어 패널 + 상태 패널
        left_panel = QVBoxLayout()
        self.control_panel = ControlPanel()
        self.status_panel = StatusPanel()
        left_panel.addWidget(self.control_panel)
        left_panel.addWidget(self.status_panel)
        left_panel.addStretch()

        # 오른쪽: 맵 위젯
        self.map_widget = MapWidget()

        # 하단: 로그 위젯
        self.log_widget = LogWidget()

        # 레이아웃 조합
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.map_widget, 3)
        right_layout.addWidget(self.log_widget, 1)

        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_layout, 3)

        # 시그널 연결
        self.control_panel.command_sent.connect(self.send_command)
        self.map_widget.waypoints_changed.connect(self.on_waypoints_changed)

    def send_command(self, command):
        """명령 전송"""
        self.zenoh_bridge.send_command(command)
        self.log_widget.add_log(f"명령 전송: {command}", "INFO")

    def on_status_update(self, status):
        """상태 업데이트"""
        self.status_panel.update_status(status)
        self.map_widget.update_robot_position(
            status['position']['x'],
            status['position']['y'],
            status['position']['theta']
        )

    def on_pose_update(self, pose):
        """위치 업데이트 (고주파)"""
        self.map_widget.add_trajectory_point(pose['x'], pose['y'])

    def on_waypoints_changed(self, waypoints):
        """웨이포인트 변경 시"""
        self.log_widget.add_log(f"웨이포인트 {len(waypoints)}개 생성", "INFO")

    def closeEvent(self, event):
        """종료 시 정리"""
        self.zenoh_bridge.stop()
        self.zenoh_bridge.wait()
        event.accept()
```

---

#### 2. zenoh_bridge.py (핵심!)

```python
from PyQt5.QtCore import QThread, pyqtSignal
import zenoh
import msgpack
import json
import time

class ZenohBridge(QThread):
    """Zenoh 통신 스레드"""

    status_received = pyqtSignal(dict)  # 상태 수신
    pose_received = pyqtSignal(dict)    # 위치 수신 (고주파)

    def __init__(self):
        super().__init__()
        self.running = False
        self.session = None

        # Zenoh Keys
        self.COMMAND_KEY = "rebar/command"
        self.STATUS_KEY = "rebar/status"
        self.POSE_KEY = "rebar/pose"

    def run(self):
        """스레드 실행"""
        self.running = True

        # Zenoh 세션 열기 (Peer 모드)
        self.session = zenoh.open()

        print("[Zenoh] 세션 열림 (Peer 모드)")

        # 구독 시작
        self.status_sub = self.session.declare_subscriber(
            self.STATUS_KEY,
            self.on_status_received
        )

        self.pose_sub = self.session.declare_subscriber(
            self.POSE_KEY,
            self.on_pose_received
        )

        print(f"[Zenoh] 구독 시작: {self.STATUS_KEY}, {self.POSE_KEY}")

        # 스레드 유지
        while self.running:
            time.sleep(0.1)

    def on_status_received(self, sample):
        """상태 메시지 수신 콜백"""
        try:
            # MessagePack 디코딩
            data = msgpack.unpackb(sample.payload, raw=False)

            # Qt 시그널 발행 (메인 스레드로 전달)
            self.status_received.emit(data)

        except Exception as e:
            print(f"[Zenoh] 상태 디코딩 오류: {e}")

    def on_pose_received(self, sample):
        """위치 메시지 수신 콜백"""
        try:
            # MessagePack 디코딩
            data = msgpack.unpackb(sample.payload, raw=False)

            # Qt 시그널 발행
            self.pose_received.emit(data)

        except Exception as e:
            print(f"[Zenoh] 위치 디코딩 오류: {e}")

    def send_command(self, command):
        """명령 전송"""
        if self.session:
            try:
                self.session.put(self.COMMAND_KEY, command)
                print(f"[Zenoh] 명령 전송: {command}")
            except Exception as e:
                print(f"[Zenoh] 명령 전송 오류: {e}")

    def send_waypoints(self, waypoints):
        """웨이포인트 전송"""
        waypoints_data = {
            "command": "WAYPOINTS",
            "waypoints": waypoints
        }

        json_str = json.dumps(waypoints_data)
        command = f"WAYPOINTS:{json_str}"

        self.send_command(command)

    def stop(self):
        """스레드 종료"""
        self.running = False

        if self.session:
            self.session.close()
            print("[Zenoh] 세션 닫힘")
```

---

#### 3. map_widget.py

```python
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QPointF, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QTransform
import math

class MapWidget(QWidget):
    """맵 시각화 위젯"""

    waypoints_changed = pyqtSignal(list)

    def __init__(self):
        super().__init__()

        # 작업 영역 (mm 단위)
        self.work_area = {
            'min_x': 0,
            'min_y': 0,
            'max_x': 1000,
            'max_y': 1000
        }

        # 웨이포인트 리스트 [(x, y), ...]
        self.waypoints = []

        # 로봇 위치
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # radian

        # 주행 경로 (실시간 추적)
        self.trajectory = []  # [(x, y), ...]

        # 줌/팬
        self.scale = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0

        self.setMinimumSize(800, 600)
        self.setMouseTracking(True)

    def paintEvent(self, event):
        """그리기"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 배경
        painter.fillRect(self.rect(), QColor(240, 240, 240))

        # Transform 설정 (좌표 변환)
        transform = QTransform()
        transform.translate(self.width() / 2, self.height() / 2)
        transform.scale(self.scale, -self.scale)  # Y축 반전
        transform.translate(self.offset_x, self.offset_y)
        painter.setTransform(transform)

        # 1. 작업 영역 그리기
        self.draw_work_area(painter)

        # 2. 웨이포인트 그리기
        self.draw_waypoints(painter)

        # 3. 계획 경로 그리기
        self.draw_planned_path(painter)

        # 4. 주행 경로 그리기 (빨간 선)
        self.draw_trajectory(painter)

        # 5. 로봇 위치 그리기
        self.draw_robot(painter)

    def draw_work_area(self, painter):
        """작업 영역 그리기"""
        pen = QPen(QColor(100, 100, 100), 2)
        painter.setPen(pen)

        brush = QBrush(QColor(255, 255, 255))
        painter.setBrush(brush)

        x = self.work_area['min_x']
        y = self.work_area['min_y']
        w = self.work_area['max_x'] - self.work_area['min_x']
        h = self.work_area['max_y'] - self.work_area['min_y']

        painter.drawRect(x, y, w, h)

    def draw_waypoints(self, painter):
        """웨이포인트 그리기 (파란 점)"""
        pen = QPen(QColor(0, 0, 255), 2)
        painter.setPen(pen)

        brush = QBrush(QColor(0, 0, 255))
        painter.setBrush(brush)

        for (x, y) in self.waypoints:
            painter.drawEllipse(QPointF(x, y), 5, 5)

    def draw_planned_path(self, painter):
        """계획 경로 그리기 (파란 선)"""
        if len(self.waypoints) < 2:
            return

        pen = QPen(QColor(0, 0, 255), 2, Qt.DashLine)
        painter.setPen(pen)

        for i in range(len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i + 1]
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

    def draw_trajectory(self, painter):
        """주행 경로 그리기 (빨간 선)"""
        if len(self.trajectory) < 2:
            return

        pen = QPen(QColor(255, 0, 0), 3)
        painter.setPen(pen)

        for i in range(len(self.trajectory) - 1):
            x1, y1 = self.trajectory[i]
            x2, y2 = self.trajectory[i + 1]
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

    def draw_robot(self, painter):
        """로봇 위치 그리기 (녹색 원 + 방향)"""
        pen = QPen(QColor(0, 200, 0), 3)
        painter.setPen(pen)

        brush = QBrush(QColor(0, 255, 0, 150))
        painter.setBrush(brush)

        # 로봇 원
        painter.drawEllipse(QPointF(self.robot_x, self.robot_y), 15, 15)

        # 방향 표시 (화살표)
        arrow_len = 25
        end_x = self.robot_x + arrow_len * math.cos(self.robot_theta)
        end_y = self.robot_y + arrow_len * math.sin(self.robot_theta)

        pen = QPen(QColor(0, 150, 0), 4)
        painter.setPen(pen)
        painter.drawLine(
            QPointF(self.robot_x, self.robot_y),
            QPointF(end_x, end_y)
        )

    def mousePressEvent(self, event):
        """마우스 클릭"""
        if event.button() == Qt.LeftButton:
            # 좌클릭: 웨이포인트 추가
            map_pos = self.screen_to_map(event.x(), event.y())

            # 작업 영역 내부인지 확인
            if self.is_inside_work_area(map_pos[0], map_pos[1]):
                self.waypoints.append(map_pos)
                self.waypoints_changed.emit(self.waypoints)
                self.update()

        elif event.button() == Qt.RightButton:
            # 우클릭: 가장 가까운 웨이포인트 삭제
            map_pos = self.screen_to_map(event.x(), event.y())
            self.remove_nearest_waypoint(map_pos[0], map_pos[1])

    def screen_to_map(self, screen_x, screen_y):
        """화면 좌표 → 맵 좌표 변환"""
        # Transform 역변환
        cx = self.width() / 2
        cy = self.height() / 2

        map_x = (screen_x - cx) / self.scale - self.offset_x
        map_y = -(screen_y - cy) / self.scale - self.offset_y  # Y축 반전

        return (map_x, map_y)

    def is_inside_work_area(self, x, y):
        """작업 영역 내부인지 확인"""
        return (self.work_area['min_x'] <= x <= self.work_area['max_x'] and
                self.work_area['min_y'] <= y <= self.work_area['max_y'])

    def remove_nearest_waypoint(self, x, y):
        """가장 가까운 웨이포인트 삭제"""
        if not self.waypoints:
            return

        min_dist = float('inf')
        min_idx = -1

        for i, (wx, wy) in enumerate(self.waypoints):
            dist = math.sqrt((x - wx)**2 + (y - wy)**2)
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        if min_dist < 50:  # 50mm 이내
            del self.waypoints[min_idx]
            self.waypoints_changed.emit(self.waypoints)
            self.update()

    def update_robot_position(self, x, y, theta):
        """로봇 위치 업데이트"""
        self.robot_x = x
        self.robot_y = y
        self.robot_theta = theta
        self.update()

    def add_trajectory_point(self, x, y):
        """주행 경로 추가"""
        self.trajectory.append((x, y))

        # 너무 많으면 오래된 것 삭제
        if len(self.trajectory) > 1000:
            self.trajectory.pop(0)

        self.update()

    def clear_waypoints(self):
        """웨이포인트 초기화"""
        self.waypoints.clear()
        self.waypoints_changed.emit(self.waypoints)
        self.update()

    def clear_trajectory(self):
        """주행 경로 초기화"""
        self.trajectory.clear()
        self.update()
```

---

#### 4. control_panel.py

```python
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QFont

class ControlPanel(QWidget):
    """제어 패널"""

    command_sent = pyqtSignal(str)

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

        # 타이틀
        title = QLabel("제어 패널")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(title)

        # 버튼들
        self.btn_home = QPushButton("홈 위치로 이동")
        self.btn_home.clicked.connect(lambda: self.command_sent.emit("GO_HOME"))
        layout.addWidget(self.btn_home)

        self.btn_send_waypoints = QPushButton("경로 전송")
        self.btn_send_waypoints.clicked.connect(self.on_send_waypoints)
        layout.addWidget(self.btn_send_waypoints)

        self.btn_start = QPushButton("자율 작업 시작")
        self.btn_start.clicked.connect(lambda: self.command_sent.emit("START_MISSION"))
        layout.addWidget(self.btn_start)

        self.btn_pause = QPushButton("일시 정지")
        self.btn_pause.clicked.connect(lambda: self.command_sent.emit("PAUSE_MISSION"))
        layout.addWidget(self.btn_pause)

        self.btn_resume = QPushButton("작업 재개")
        self.btn_resume.clicked.connect(lambda: self.command_sent.emit("RESUME_MISSION"))
        layout.addWidget(self.btn_resume)

        self.btn_abort = QPushButton("작업 중단")
        self.btn_abort.clicked.connect(lambda: self.command_sent.emit("ABORT_MISSION"))
        layout.addWidget(self.btn_abort)

        self.btn_estop = QPushButton("비상 정지")
        self.btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_estop.clicked.connect(lambda: self.command_sent.emit("E-STOP"))
        layout.addWidget(self.btn_estop)

        layout.addStretch()

        self.setLayout(layout)

    def on_send_waypoints(self):
        """경로 전송 버튼"""
        # 부모 윈도우에서 처리 (waypoints 필요)
        self.command_sent.emit("SEND_WAYPOINTS")
```

---

#### 5. status_panel.py

```python
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QGroupBox
from PyQt5.QtGui import QFont

class StatusPanel(QWidget):
    """상태 표시 패널"""

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

        # 로봇 상태 그룹
        robot_group = QGroupBox("로봇 상태")
        robot_layout = QVBoxLayout()

        self.lbl_mode = QLabel("모드: IDLE")
        self.lbl_position = QLabel("위치: X: 0.0, Y: 0.0")
        self.lbl_heading = QLabel("방향: 0.0°")
        self.lbl_speed = QLabel("속도: 0.0 m/s")
        self.lbl_battery = QLabel("배터리: 0%")
        self.lbl_temp = QLabel("온도: 0°C")

        robot_layout.addWidget(self.lbl_mode)
        robot_layout.addWidget(self.lbl_position)
        robot_layout.addWidget(self.lbl_heading)
        robot_layout.addWidget(self.lbl_speed)
        robot_layout.addWidget(self.lbl_battery)
        robot_layout.addWidget(self.lbl_temp)

        robot_group.setLayout(robot_layout)
        layout.addWidget(robot_group)

        # 미션 상태 그룹
        mission_group = QGroupBox("미션 진행")
        mission_layout = QVBoxLayout()

        self.lbl_mission_status = QLabel("상태: IDLE")
        self.lbl_waypoint = QLabel("웨이포인트: 0 / 0")
        self.lbl_progress = QLabel("진행률: 0%")

        mission_layout.addWidget(self.lbl_mission_status)
        mission_layout.addWidget(self.lbl_waypoint)
        mission_layout.addWidget(self.lbl_progress)

        mission_group.setLayout(mission_layout)
        layout.addWidget(mission_group)

        layout.addStretch()

        self.setLayout(layout)

    def update_status(self, status):
        """상태 업데이트"""
        # 제어 모드
        mode = status.get('control_mode', 'UNKNOWN').upper()
        self.lbl_mode.setText(f"모드: {mode}")

        # 위치
        pos = status.get('position', {})
        x = pos.get('x', 0.0)
        y = pos.get('y', 0.0)
        self.lbl_position.setText(f"위치: X: {x:.1f} mm, Y: {y:.1f} mm")

        # 방향
        heading = status.get('heading', 0.0)
        self.lbl_heading.setText(f"방향: {heading:.1f}°")

        # 속도
        speed = status.get('speed', 0.0)
        self.lbl_speed.setText(f"속도: {speed:.2f} m/s")

        # 배터리
        battery = status.get('battery', 0.0)
        self.lbl_battery.setText(f"배터리: {battery:.1f}%")

        # 온도
        temp = status.get('temperature', 0)
        self.lbl_temp.setText(f"온도: {temp}°C")

        # 미션 상태
        mission_status = status.get('mission_status', 'IDLE').upper()
        self.lbl_mission_status.setText(f"상태: {mission_status}")

        # 웨이포인트
        current = status.get('current_waypoint', 0)
        total = status.get('total_waypoints', 0)
        self.lbl_waypoint.setText(f"웨이포인트: {current} / {total}")

        # 진행률
        if total > 0:
            progress = int((current / total) * 100)
            self.lbl_progress.setText(f"진행률: {progress}%")
        else:
            self.lbl_progress.setText("진행률: 0%")
```

---

#### 6. log_widget.py

```python
from PyQt5.QtWidgets import QTextEdit
from PyQt5.QtGui import QTextCursor, QColor
from datetime import datetime

class LogWidget(QTextEdit):
    """로그 출력 위젯"""

    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setMaximumHeight(200)

    def add_log(self, message, level="INFO"):
        """로그 추가"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # 색상 지정
        if level == "ERROR":
            color = QColor(255, 0, 0)
        elif level == "WARNING":
            color = QColor(255, 165, 0)
        elif level == "SUCCESS":
            color = QColor(0, 200, 0)
        else:  # INFO
            color = QColor(0, 0, 0)

        # 로그 추가
        self.setTextColor(color)
        self.append(f"[{timestamp}] {message}")

        # 스크롤 최하단으로
        self.moveCursor(QTextCursor.End)
```

---

#### 7. main.py

```python
import sys
from PyQt5.QtWidgets import QApplication
from ui.main_window import MainWindow

def main():
    app = QApplication(sys.argv)

    # 메인 윈도우
    window = MainWindow()
    window.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
```

---

## 메시지 포맷

### 좌표 단위

| 항목 | UI | Zenoh | ROS2 |
|------|-----|-------|------|
| 위치 (x, y) | mm | mm | m |
| 각도 | degree | radian | radian |
| 속도 | m/s | m/s | m/s |

**변환:**
```python
# mm → m
ros_x = ui_x / 1000.0

# m → mm
ui_x = ros_x * 1000.0

# radian → degree
degree = radian * 180.0 / math.pi

# degree → radian
radian = degree * math.pi / 180.0
```

---

### 웨이포인트 형식

**UI → Zenoh (JSON):**
```json
{
  "command": "WAYPOINTS",
  "waypoints": [
    {"x": 0.0, "y": 0.0},
    {"x": 100.0, "y": 0.0},
    {"x": 100.0, "y": 100.0}
  ]
}
```

**단위:** mm

**전송:**
```python
import json

waypoints = [
    {"x": 0.0, "y": 0.0},
    {"x": 100.0, "y": 0.0},
    {"x": 100.0, "y": 100.0}
]

waypoints_data = {
    "command": "WAYPOINTS",
    "waypoints": waypoints
}

json_str = json.dumps(waypoints_data)
command = f"WAYPOINTS:{json_str}"

zenoh_session.put("rebar/command", command)
```

---

## UI 개발 체크리스트

### Phase 1: 기본 UI 골격
- [ ] PyQt5 설치 및 환경 설정
- [ ] 프로젝트 디렉토리 구조 생성
- [ ] main.py, main_window.py 생성
- [ ] 빈 윈도우 실행 테스트

### Phase 2: Zenoh 통신
- [ ] Zenoh 설치 (pip3 install eclipse-zenoh msgpack)
- [ ] zenoh_bridge.py 구현
  - [ ] Peer 모드 세션 초기화
  - [ ] "rebar/command" 발행 테스트
  - [ ] "rebar/status" 구독 테스트
  - [ ] "rebar/pose" 구독 테스트
- [ ] Jetson과 통신 확인 (echo 테스트)

### Phase 3: 제어 패널
- [ ] control_panel.py 구현
- [ ] 버튼 클릭 → 명령 전송 연결
- [ ] 각 버튼 동작 테스트

### Phase 4: 상태 패널
- [ ] status_panel.py 구현
- [ ] Zenoh 상태 수신 → UI 업데이트
- [ ] 실시간 업데이트 확인 (10Hz)

### Phase 5: 맵 위젯
- [ ] map_widget.py 구현
  - [ ] 기본 그리기 (작업 영역)
  - [ ] 좌표 변환 (screen ↔ map)
  - [ ] 마우스 클릭 (웨이포인트 추가)
  - [ ] 우클릭 (웨이포인트 삭제)
  - [ ] 계획 경로 그리기 (파란 선)
  - [ ] 로봇 위치 표시 (녹색 원)
  - [ ] 주행 경로 그리기 (빨간 선)
- [ ] 로봇 위치 업데이트 테스트

### Phase 6: 웨이포인트 전송
- [ ] 웨이포인트 → JSON 변환
- [ ] "WAYPOINTS:<json>" 명령 전송
- [ ] Jetson에서 수신 확인

### Phase 7: 로그 위젯
- [ ] log_widget.py 구현
- [ ] 로그 레벨별 색상 지정
- [ ] 명령/상태 로그 출력

### Phase 8: 통합 테스트
- [ ] 전체 시스템 실행 (Jetson + Laptop)
- [ ] 명령 전송 → 로봇 동작 확인
- [ ] 상태 수신 → UI 업데이트 확인
- [ ] 웨이포인트 전송 → 자율 주행 확인

### Phase 9: UI 개선
- [ ] 스타일시트 적용 (Qt CSS)
- [ ] 아이콘 추가
- [ ] 줌/팬 기능 (마우스 휠, 드래그)
- [ ] 그리드 표시 (맵 위젯)
- [ ] 미션 진행 프로그레스 바

### Phase 10: 에러 처리
- [ ] Zenoh 연결 실패 처리
- [ ] 타임아웃 처리 (상태 수신 없을 때)
- [ ] 비정상 데이터 처리

---

## 테스트 시나리오

### 1. 통신 테스트

**목표:** Laptop ↔ Jetson 통신 확인

**절차:**
1. Jetson에서 `rebar_control` 실행
   ```bash
   ros2 launch rebar_control control_system.launch.py
   ```

2. Laptop에서 UI 실행
   ```bash
   python3 main.py
   ```

3. Zenoh 연결 확인
   - UI 로그에 "Zenoh 세션 열림" 표시
   - Jetson 로그에 "zenoh_client: 연결됨" 표시

4. 명령 전송 테스트
   - UI에서 "홈 위치로 이동" 버튼 클릭
   - Jetson 로그에 "GO_HOME 명령 수신" 표시

5. 상태 수신 테스트
   - UI 상태 패널에 로봇 상태 표시
   - 10Hz 업데이트 확인

---

### 2. 맵 생성 테스트

**목표:** 작업 영역 생성

**절차:**
1. 로봇을 원점 (0, 0)에 배치

2. 리모콘으로 Manual 모드 전환 (S10 스위치)

3. 리모콘으로 사각형 주행
   - (0, 0) → (1000, 0)
   - (1000, 0) → (1000, 1000)
   - (1000, 1000) → (0, 1000)
   - (0, 1000) → (0, 0)

4. UI 맵에 작업 영역 표시 확인

---

### 3. 경로 생성 및 전송 테스트

**목표:** UI에서 경로 생성 → Jetson 전송

**절차:**
1. UI 맵에서 마우스 클릭으로 웨이포인트 생성
   - 예: (100, 100), (100, 900), (900, 900), (900, 100)

2. "경로 전송" 버튼 클릭

3. Jetson 로그 확인
   - "WAYPOINTS 명령 수신: 4개"

4. UI 로그 확인
   - "웨이포인트 4개 전송"

---

### 4. 자율 주행 테스트

**목표:** 경로 추종 확인

**절차:**
1. 리모콘으로 Auto 모드 전환 (S20 스위치)

2. UI에서 "자율 작업 시작" 버튼 클릭

3. 로봇 주행 확인
   - UI 맵에 빨간 선(주행 경로) 표시
   - 로봇 위치 실시간 업데이트

4. 웨이포인트 도달 확인
   - UI 상태 패널: "웨이포인트: 1 / 4"
   - 진행률 업데이트

5. 미션 완료 확인
   - UI 상태: "MISSION_DONE"

---

### 5. 비상 정지 테스트

**목표:** 비상 정지 동작 확인

**절차:**
1. 자율 주행 중

2. UI에서 "비상 정지" 버튼 클릭

3. 로봇 즉시 정지 확인

4. UI 상태: "EMERGENCY_STOP"

---

## 디버깅 가이드

### Zenoh 연결 안 됨

**증상:**
- UI에서 "Zenoh 세션 열림" 표시되지만 상태 수신 없음

**원인:**
- Jetson과 Laptop이 다른 네트워크
- 방화벽 차단

**해결:**
1. 같은 WiFi 연결 확인
   ```bash
   # Laptop
   ip addr show

   # Jetson
   ip addr show
   ```

2. Ping 테스트
   ```bash
   # Laptop → Jetson
   ping 192.168.x.x
   ```

3. Zenoh Router 사용 (옵션)
   ```python
   # Client 모드로 변경
   config = zenoh.Config()
   config.insert_json5("connect/endpoints", '["tcp/192.168.1.100:7447"]')
   session = zenoh.open(config)
   ```

---

### 상태 업데이트 느림

**증상:**
- UI 업데이트가 느림 (1초 이상 지연)

**원인:**
- MessagePack 디코딩 오버헤드
- 메인 스레드 블로킹

**해결:**
1. QThread에서 디코딩 수행 (이미 구현됨)
2. 불필요한 업데이트 최소화

---

### 좌표 변환 오류

**증상:**
- 로봇 위치가 맵 밖에 표시

**원인:**
- 좌표 단위 불일치 (mm vs m)
- Transform 설정 오류

**해결:**
1. 단위 확인
   ```python
   # Zenoh에서 mm 단위로 수신 확인
   print(f"Position: x={data['position']['x']} mm")
   ```

2. Transform 디버깅
   ```python
   # map_widget.py
   print(f"Robot: ({self.robot_x}, {self.robot_y})")
   ```

---

## 참고 자료

### Zenoh 공식 문서
- https://zenoh.io/docs/manual/abstractions/
- Python API: https://zenoh.io/docs/apis/python/

### PyQt5 튜토리얼
- https://doc.qt.io/qtforpython/
- QPainter: https://doc.qt.io/qt-5/qpainter.html

### MessagePack
- https://msgpack.org/
- Python: https://github.com/msgpack/msgpack-python

---

## 다음 단계

### Phase 4: UI 고급 기능
- [ ] 경로 편집 (웨이포인트 드래그)
- [ ] 작업 영역 수동 입력
- [ ] 미션 저장/불러오기 (JSON 파일)
- [ ] 다중 경로 관리
- [ ] 3D 뷰 (RViz2 임베딩, 옵션)

### Phase 5: 데이터 분석
- [ ] 주행 데이터 기록 (CSV)
- [ ] 위치 오차 분석
- [ ] 속도 프로파일 그래프
- [ ] 배터리 소모 추적

---

**문서 버전:** 1.0
**최종 수정:** 2025-12-17
