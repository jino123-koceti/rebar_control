// 워크샵 발표자료(14장) 생성:  node docs/presentations/build_deck.js
//   입력: docs/presentations/figures/*  (make_figures.py 로 생성)
//   출력: docs/presentations/rebar_autonomy_workshop.pptx  (발표자 노트 = 발표 스크립트)
// 폰트는 발표 PC(윈도우) 기준 '맑은 고딕'. 여기 LibreOffice 미리보기는 대체폰트라 폭이 약간 다르다.
const path = require('path');
const pptxgen = require('pptxgenjs');

const FIG = path.join(__dirname, 'figures');
const OUT = path.join(__dirname, 'rebar_autonomy_workshop.pptx');

// ── 팔레트: 스틸 네이비(주) + 안전 주황(강조) ──
const C = {
  dark: '1B2A34', steel: '5B7083', tint: 'EEF2F5', accent: 'E8590C',
  ink: '1F2937', muted: '6B7280', white: 'FFFFFF', line: 'D5DCE3',
  green: '16A34A', red: 'DC2626',
};
const FONT = 'Malgun Gothic';
const W = 13.333, H = 7.5, M = 0.5;

// 그림 가로세로비 (make_figures.py 출력 실측)
const AR = {
  s03_software_architecture: 1.77, s04_state_machine: 2.08, s05_detection: 2.38,
  s06_accuracy: 1.35, s06_cad_overlay: 1.60, s06_pipeline: 4.45, s07_reach_ranges: 1.27,
  s08_pitch_step: 2.76, s09_deck_edge: 1.44, s09_hysteresis: 2.01,
  s10_heading_response: 1.38, s10_levelrod_102501_front_final: 1.60,
  s10_levelrod_persistence: 1.97, s11_tie_memory: 1.83, s12_incident_timeline: 2.38,
  s12_safety_layers: 1.11, s13_100pt: 3.00,
  s09_deck_edge_front_stop: 2.88, s09_deck_edge_back_go: 2.87, s13_lanes_20260922: 1.67, s13_step_error: 1.58,
};

const pres = new pptxgen();
pres.layout = 'LAYOUT_WIDE';
pres.title = '철근 결속 로봇의 자율작업 알고리즘';

// ── 헬퍼 ────────────────────────────────────────────────
function txt(slide, text, o) {
  slide.addText(text, Object.assign({ fontFace: FONT, color: C.ink, isTextBox: true, margin: 0 }, o));
}

// 박스(x,y,w,h) 안에 비율 유지로 맞춰 넣는다. align: 'c' | 'l' | 't'
function fig(slide, name, x, y, w, h, align = 'c') {
  const ar = AR[name];
  if (!ar) throw new Error('AR 누락: ' + name);
  let fw = w, fh = w / ar;
  if (fh > h) { fh = h; fw = h * ar; }
  const fx = align === 'l' ? x : x + (w - fw) / 2;
  const fy = align === 't' ? y : y + (h - fh) / 2;
  const ext = name.includes('levelrod_1') ? '.jpg' : '.png';
  slide.addImage({ path: path.join(FIG, name + ext), x: fx, y: fy, w: fw, h: fh });
  return { x: fx, y: fy, w: fw, h: fh };
}

function header(slide, section, title) {
  txt(slide, section, { x: M, y: 0.32, w: 8, h: 0.3, fontSize: 12, bold: true, color: C.accent });
  txt(slide, title, { x: M, y: 0.6, w: W - 2 * M, h: 0.62, fontSize: 28, bold: true, color: C.dark });
}

// 반복 모티프: 하단 "핵심 한 줄" 카드
function takeaway(slide, text, y = 6.55) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
    x: M, y, w: W - 2 * M, h: 0.55, fill: { color: C.tint }, line: { color: C.tint }, rectRadius: 0.08 });
  slide.addShape(pres.shapes.OVAL, {
    x: M + 0.15, y: y + 0.125, w: 0.3, h: 0.3, fill: { color: C.accent }, line: { color: C.accent } });
  txt(slide, '!', { x: M + 0.15, y: y + 0.125, w: 0.3, h: 0.3, fontSize: 13, bold: true,
    color: C.white, align: 'center', valign: 'middle' });
  txt(slide, text, { x: M + 0.6, y, w: W - 2 * M - 0.8, h: 0.55, fontSize: 15, bold: true,
    color: C.dark, valign: 'middle' });
}

function pageNo(slide, n) {
  txt(slide, String(n), { x: W - M - 0.5, y: H - 0.38, w: 0.5, h: 0.25, fontSize: 10,
    color: C.muted, align: 'right' });
}

function placeholder(slide, label, x, y, w, h, dark = false) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
    x, y, w, h, rectRadius: 0.12, fill: { color: dark ? '2A3B47' : C.tint },
    line: { color: dark ? '8FA3B3' : C.steel, width: 1.5, dashType: 'dash' } });
  txt(slide, label, { x, y, w, h, fontSize: 14, color: dark ? 'B8C6D1' : C.steel,
    align: 'center', valign: 'middle' });
}

// 흰 카드 (그림자 없이 옅은 테두리)
function card(slide, x, y, w, h, fill = C.white) {
  slide.addShape(pres.shapes.ROUNDED_RECTANGLE, {
    x, y, w, h, rectRadius: 0.1, fill: { color: fill }, line: { color: C.line, width: 1 } });
}

// 큰 숫자 + 설명
function stat(slide, x, y, w, big, label, color = C.accent) {
  txt(slide, big, { x, y, w, h: 0.75, fontSize: 36, bold: true, color });
  txt(slide, label, { x, y: y + 0.75, w, h: 0.55, fontSize: 13, color: C.muted, valign: 'top' });
}

// ══ 1. 표지 ══════════════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.dark };
  txt(s, '전북대 × 연구팀 조인트 워크샵', { x: M + 0.2, y: 1.3, w: 6.5, h: 0.4, fontSize: 14,
    bold: true, color: C.accent });
  txt(s, '철근 결속 로봇의\n자율작업 알고리즘', { x: M + 0.2, y: 1.85, w: 6.8, h: 1.9,
    fontSize: 40, bold: true, color: C.white, valign: 'top', lineSpacingMultiple: 1.05 });
  txt(s, '배근을 보고, 배근 위를 걷고, 배근을 묶는다', { x: M + 0.2, y: 3.95, w: 6.8, h: 0.5,
    fontSize: 20, color: 'B8C6D1' });
  txt(s, '[발표자]  ·  [소속]\n2026. 9.', { x: M + 0.2, y: 5.6, w: 6, h: 0.8, fontSize: 14,
    color: '8FA3B3', valign: 'top' });
  placeholder(s, '로봇 실물 사진', 7.6, 1.2, 5.2, 5.1, true);
  s.addNotes(
`[0:30] 안녕하십니까. 오늘은 저희가 개발 중인 철근 결속 로봇에서 하드웨어보다는 자율작업 알고리즘을 말씀드리겠습니다.
한 줄로 정리하면, 이 로봇은 배근을 보고, 배근 위를 걷고, 배근을 묶습니다. 이 세 가지를 각각 어떻게 푸는지 순서대로 보여드리겠습니다.`);
}

// ══ 2. 문제 정의 ═════════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '01  문제 정의', '결속 자동화가 어려운 이유');
  placeholder(s, '현장 배근 · 결속 작업 사진', M, 1.5, 5.6, 4.8);
  const items = [
    ['간격이 불규칙하다', '도면대로 깔리지 않는다. 줄이 빠지고, 틀어지고, 레벨봉·스페이서가 섞인다 → 미리 만든 지도가 맞지 않는다'],
    ['2층 배근', '하부근 × 상부근이 겹치면 탑뷰에서 가짜 교차점이 보인다 (현장 층간 150 mm)'],
    ['주행면이 철근 자체', '무한궤도가 배근 위를 밟고 간다. 방수포 위로 나가면 떨어진다 → "바닥 찾기"가 곧 위험'],
  ];
  items.forEach(([t, d], i) => {
    const y = 1.5 + i * 1.65;
    card(s, 6.5, y, 6.33, 1.45);
    txt(s, String(i + 1).padStart(2, '0'), { x: 6.75, y: y + 0.2, w: 0.8, h: 0.6, fontSize: 28,
      bold: true, color: C.accent });
    txt(s, t, { x: 7.6, y: y + 0.2, w: 5.0, h: 0.4, fontSize: 18, bold: true, color: C.dark });
    txt(s, d, { x: 7.6, y: y + 0.65, w: 5.0, h: 0.7, fontSize: 13, color: C.ink, valign: 'top' });
  });
  takeaway(s, '세 번째 제약 — "배근 위만 주행 가능" — 이 뒤의 주행 판정 알고리즘을 통째로 규정한다');
  pageNo(s, 2);
  s.addNotes(
`[0:40] 결속은 배근 위를 걸어 다니며 교차점마다 허리를 굽혀 묶는 작업입니다. 자동화가 어려운 이유는 세 가지입니다.
첫째, 배근이 도면대로 깔려 있지 않습니다. 줄이 빠지고 틀어져 있어서 미리 만든 지도가 맞지 않습니다.
둘째, 2층 배근입니다. 위아래 철근이 겹치면 위에서 볼 때 가짜 교차점이 생깁니다.
셋째가 가장 중요합니다. 이 로봇은 바닥이 아니라 철근 배근 위를 직접 밟고 다닙니다. 방수포 위로 나가면 떨어집니다. 이 제약이 뒤에 나올 주행 판정을 통째로 규정합니다.`);
}

// ══ 3. 로봇 구성 ═════════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '01  시스템', '로봇 구성 — 7축 + 비전 3계통');
  const hw = [
    ['주행', '무한궤도 좌/우 차동구동'],
    ['레인 전환', '횡이동 · 평행4절 (승강=보폭 연동)'],
    ['결속부', 'X 345 × Y 288 mm · Z · 결속기'],
    ['자세', 'Yaw 399° — 우측 ↔ 좌측 자세'],
  ];
  hw.forEach(([k, v], i) => {
    const y = 1.5 + i * 1.0;
    card(s, M, y, 3.9, 0.85);
    txt(s, k, { x: M + 0.2, y: y + 0.1, w: 3.5, h: 0.32, fontSize: 15, bold: true, color: C.accent });
    txt(s, v, { x: M + 0.2, y: y + 0.43, w: 3.6, h: 0.32, fontSize: 12.5, color: C.ink });
  });
  txt(s, 'ROS2 Humble · Jetson · RMD 모터 CAN 1 Mbps', { x: M, y: 5.6, w: 3.9, h: 0.5,
    fontSize: 12, color: C.muted, valign: 'top' });
  fig(s, 's03_software_architecture', 4.7, 1.35, 8.13, 5.05);
  takeaway(s, '위를 보는 카메라는 "어디를 묶을지", 앞뒤를 보는 카메라는 "어디까지 갈 수 있는지" — 역할 분리');
  pageNo(s, 3);
  s.addNotes(
`[0:40] 로봇은 아래 주행부와 위 결속부로 나뉩니다. 주행은 무한궤도 차동구동, 레인 전환은 횡이동 기구, 위에는 X·Y·Z 스테이지에 요 회전이 붙어 있습니다. 요를 399도 돌려 우측·좌측 두 자세를 씁니다.
오른쪽이 소프트웨어 구성입니다. 카메라 역할이 분리돼 있습니다. 위에서 보는 Orbbec은 어디를 묶을지, 앞뒤 ZED는 어디까지 갈 수 있는지를 담당합니다.
빨간 점선을 봐주십시오. 주행 차단 신호는 상태머신을 거치지 않고 모터 직전 노드로 바로 들어갑니다. 위에서 무엇이 명령하든 막기 위해서입니다.`);
}

// ══ 4. 전체 흐름 ═════════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '02  자율작업 프레임', '지도 없이, 보이는 배근을 따라간다');
  fig(s, 's04_state_machine', M, 1.35, W - 2 * M, 5.0);
  takeaway(s, '웨이포인트 방식(A) → 반응형(B): 매 정차마다 다시 보고 다시 계산하므로 틀어진 격자도 따라간다');
  pageNo(s, 4);
  s.addNotes(
`[0:45] 사이클 하나가 이렇게 돕니다. 서서 보고, 닿는 데를 다 묶고, 다음 미결속 열까지 이동하고, 다시 섭니다. 배근 끝을 만나면 옆 레인으로 옮겨 반대 방향으로 반복합니다. 'ㄹ'자입니다.
처음에는 웨이포인트 방식을 먼저 만들었습니다. 영역을 등록하고 경로를 만들어 추종하는 방식이고, 100점 연속 결속 실적도 있습니다.
그런데 현장 배근은 지도와 어긋납니다. 그래서 지금은 지도를 안 만듭니다. 매번 눈앞의 배근을 다시 보고 다음 거리를 계산합니다. 대신 검출이 틀리면 이동이 바로 틀립니다. 이걸 어떻게 막았는지가 다음 장들입니다.`);
}

// ══ 5. 교차점 검출 ═══════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '03  핵심 알고리즘 · 보기', '교차점 검출 — YOLO + 다중 프레임 병합');
  fig(s, 's05_detection', M, 1.35, W - 2 * M, 5.05);
  takeaway(s, '검출은 전부 피치 계산에 쓰고, 결속은 도달범위 안 점만 — 두 용도의 범위를 분리한다');
  pageNo(s, 5);
  s.addNotes(
`[0:35] 상부 Orbbec 카메라로 교차점을 찾습니다. YOLO로 3프레임을 검출해 가까운 박스를 병합합니다.
처음엔 신뢰도가 0.34에 불과했는데, 테스트베드 도메인으로 재학습해 0.90, mAP50 0.99까지 올렸습니다.
색을 봐주십시오. 초록은 스테이지가 닿는 점, 노랑은 못 닿는 점입니다. 노란 점도 버리지 않습니다. 격자 간격을 재는 데 씁니다. 이 분리가 다음 장들의 핵심입니다.`);
}

// ══ 6. 위치식별 ══════════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '03  핵심 알고리즘 · 어디를 묶나', '위치식별 — 호모그래피·회귀 → CAD 강체변환');
  fig(s, 's06_pipeline', M, 1.3, 9.0, 2.05, 'l');
  card(s, 9.9, 1.35, 2.93, 1.95, C.tint);
  txt(s, '~2 mm', { x: 10.1, y: 1.5, w: 2.6, h: 0.85, fontSize: 40, bold: true, color: C.accent });
  txt(s, '4점 실측 XY 잔차\n스케일 ≈ 1.00', { x: 10.1, y: 2.35, w: 2.6, h: 0.8, fontSize: 13,
    color: C.ink, valign: 'top' });
  fig(s, 's06_cad_overlay', M, 3.55, 5.0, 2.85, 'l');
  fig(s, 's06_accuracy', 5.8, 3.55, 3.85, 2.85);
  txt(s, '왜 바꿨나', { x: 9.9, y: 3.6, w: 2.93, h: 0.35, fontSize: 15, bold: true, color: C.dark });
  txt(s, [
    { text: '회귀: 카메라마다 수십 쌍 실측, 옮기면 전부 재측정', options: { bullet: true, breakLine: true } },
    { text: 'CAD: 실측 자유도는 자세별 오프셋 2쌍뿐', options: { bullet: true, breakLine: true } },
    { text: '조건: depth가 컬러에 정합돼 있어야 함', options: { bullet: true } },
  ], { x: 9.9, y: 4.0, w: 2.93, h: 2.3, fontSize: 12.5, valign: 'top', paraSpaceAfter: 6 });
  takeaway(s, 'CAD를 믿고 실측으로 맞출 자유도를 줄였다 — 더 정확하고, 카메라를 옮겨도 CAD 값만 바꾸면 된다');
  pageNo(s, 6);
  s.addNotes(
`[0:45] 검출보다 어려운 건 화면 좌표를 결속건을 보낼 스테이지 좌표로 바꾸는 일입니다.
처음엔 픽셀과 depth를 넣어 로봇 좌표를 내는 회귀 모델을 실측으로 맞췄습니다. 오차 6mm 안팎이었고, 카메라를 옮기면 수십 쌍을 다시 찍어야 했습니다.
지금은 CAD 배치값에서 카메라-결속건 강체변환을 직접 유도합니다. depth로 역투영하고, CAD의 R, t로 옮기고, 자세별 오프셋만 실측으로 붙입니다. 잔차 2mm 수준입니다.
핵심은 정확도보다, 실측으로 맞출 자유도가 오프셋 두 쌍뿐이라는 점입니다.`);
}

// ══ 7. 결속 시퀀스 ═══════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '03  핵심 알고리즘 · 묶기', '결속 시퀀스 — 두 자세 지그재그');
  fig(s, 's07_reach_ranges', M, 1.35, 5.9, 5.0, 'l');
  const steps = [
    ['자세 분류', '우측 Y 0~142 / 좌측 Y 124~288. 겹치면 현재 자세 우선'],
    ['지그재그 순서', '첫 자세는 X 큰 쪽→0, 자세변경(X=0), 둘째는 0→큰 쪽'],
    ['한 점 결속', 'XY 이동 → Z 하강 → 트리거 → Z 상승'],
    ['Z 과부하 감지', '전류 2단계 감시 → 충돌 시 트리거 생략 후 순수 상승'],
    ['사이클 단축', 'Z가 올라오는 중 XY 선행 (여유 5 mm 유지)'],
  ];
  steps.forEach(([t, d], i) => {
    const y = 1.4 + i * 0.98;
    s.addShape(pres.shapes.OVAL, { x: 7.0, y: y + 0.05, w: 0.48, h: 0.48,
      fill: { color: C.dark }, line: { color: C.dark } });
    txt(s, String(i + 1), { x: 7.0, y: y + 0.05, w: 0.48, h: 0.48, fontSize: 14, bold: true,
      color: C.white, align: 'center', valign: 'middle' });
    txt(s, t, { x: 7.7, y, w: 5.1, h: 0.36, fontSize: 16, bold: true, color: C.dark });
    txt(s, d, { x: 7.7, y: y + 0.38, w: 5.1, h: 0.5, fontSize: 13, color: C.ink, valign: 'top' });
  });
  takeaway(s, 'Yaw 399°로 두 자세를 써서 한 번 정차에 Y 288 mm를 덮는다 — 자세변경은 정차당 최대 1회');
  pageNo(s, 7);
  s.addNotes(
`[0:40] 결속부는 한 자세로 Y 전체를 못 덮습니다. 그래서 요를 399도 돌려 우측·좌측 두 자세를 씁니다. 왼쪽 그림의 점들은 9월 22일 자율결속에서 실제로 묶은 자리입니다.
순서는 지그재그입니다. 첫 자세에서 X 먼 쪽부터 0까지 묶고, X 0에서 자세를 바꾼 뒤, 0부터 먼 쪽으로 묶습니다. 이러면 자세변경이 정차당 한 번입니다.
한 점은 XY 이동, Z 하강, 트리거, Z 상승입니다. Z 하강 중 전류를 감시해 충돌이면 트리거를 생략하고, Z가 올라오는 도중에 XY를 먼저 움직여 사이클을 줄였습니다.`);
}

// ══ 8. 피치 → 스텝 거리 ══════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '03  핵심 알고리즘 · 얼마나 가나', '격자 피치 → 다음 스텝 거리');
  fig(s, 's08_pitch_step', M, 1.3, W - 2 * M, 4.45);
  card(s, M, 5.85, W - 2 * M, 0.55, C.white);
  txt(s, [
    { text: 'd = max( 마지막 결속 열 − 도달범위 시작 + 30 mm ,  1 피치 )', options: { bold: true, color: C.dark } },
    { text: '     · 결측 열은 최소간격의 정수배로 정규화해 1피치 복원', options: { color: C.muted } },
  ], { x: M + 0.25, y: 5.85, w: W - 2 * M - 0.5, h: 0.55, fontSize: 14, valign: 'middle' });
  takeaway(s, '"열 수 × 피치"는 첫 열이 범위 시작에 있을 때만 맞다 — 마지막으로 묶은 열을 기준으로 잰다');
  pageNo(s, 8);
  s.addNotes(
`[0:50] 이 시스템에서 가장 많이 틀렸던 부분입니다. 원리는 단순합니다. 닿는 범위 안은 다 묶었다고 보고, 다음 미결속 열까지 갑니다.
처음엔 열 수 곱하기 피치만큼 갔습니다. 오른쪽이 실패 사례입니다. 열이 154.7과 311.6, 피치 156.9라 313.8을 지령했는데, 지나쳐야 할 311.6보다 2.2mm 여유뿐이었습니다. 도달오차 14mm가 겹쳐 300만 갔고, 그 열이 남아서 다시 묶였습니다.
지금은 마지막으로 묶은 열을 기준으로 30mm 마진을 두고, 최소 1피치는 보장합니다. 왼쪽이 9월 22일 실제 정차로, 225 대신 1피치 249mm가 적용된 예입니다.
그리고 피치는 넓게, 결속 대상은 좁게, 범위를 분리했습니다.`);
}

// ══ 9. 주행가능 판정 ═════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '03  핵심 알고리즘 · 어디까지 가나', '주행가능 판정 — 바닥을 찾지 않고 철근만 본다');
  fig(s, 's09_deck_edge_front_stop', M, 1.35, 6.3, 2.2, 'l');
  fig(s, 's09_deck_edge_back_go', M, 3.7, 6.3, 2.2, 'l');
  txt(s, '위: 데크 끝 앞 (STOP, 0.34) · 아래: 배근 위 (GO, 0.66) — 오른쪽은 행별 철근비율 r(y)',
    { x: M, y: 6.0, w: 6.3, h: 0.3, fontSize: 11, color: C.muted });
  fig(s, 's09_hysteresis', 7.05, 1.3, 5.78, 2.9, 't');
  txt(s, [
    { text: '주행가능 = rebar_h ∪ rebar_v, 나머지는 여집합으로 전부 주행불가', options: { bullet: true, breakLine: true } },
    { text: 'rebar_frac: 발밑부터 위로 스캔, 높이 9% 넘게 끊기면 배근 끝', options: { bullet: true, breakLine: true } },
    { text: '임계 STOP 0.45 / SLOW 0.55 — 철근 기준이라 전·후방 공통', options: { bullet: true, breakLine: true } },
    { text: '정지는 즉시, 해제는 0.55 초과 + STOP 3연속 확인', options: { bullet: true } },
  ], { x: 7.1, y: 4.35, w: 5.7, h: 2.05, fontSize: 13, valign: 'top', paraSpaceAfter: 6 });
  takeaway(s, '방수포는 평평하고 깨끗해서 모델이 "바닥"으로 아주 잘 찾는다 — 그리고 그 위로 나가면 떨어진다');
  pageNo(s, 9);
  s.addNotes(
`[0:50] 처음엔 흔한 방식으로 바닥을 찾아 주행가능으로 봤습니다. 그런데 현장 바닥은 방수포입니다. 모델이 아주 잘 찾고, 그 위로 나가면 로봇이 떨어집니다.
그래서 뒤집었습니다. 주행가능은 철근뿐이고, 나머지는 여집합으로 전부 주행불가입니다. 클래스가 늘어도 모르는 것은 자동으로 안전한 쪽입니다.
지표는 rebar_frac 하나입니다. 발밑부터 위로 올라가며 배근이 이어지는 높이를 잽니다. 정상주행 0.55~0.68, 데크끝 0.47 이하라 잘 갈립니다.
오른쪽 위를 보십시오. 판정은 임계 근처에서 떨립니다. 단순 임계는 10번 뒤집히고, 저희 판정기는 2번입니다. 채터링 한 번이 레인 하나를 건너뛰게 만들기 때문에, 정지는 즉시, 해제는 보수적으로 합니다.`);
}

// ══ 10. 조향 + 장애물 ════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '03  핵심 알고리즘 · 곧게, 안전하게', '배근 정렬 조향과 레벨봉 회피');
  txt(s, '조향 — 가로철근 기울기 = 요 오차', { x: M, y: 1.3, w: 6.0, h: 0.35, fontSize: 16,
    bold: true, color: C.dark });
  fig(s, 's10_heading_response', M, 1.7, 6.0, 4.35, 'l');
  txt(s, '세로철근 소실점은 교차점에서 조각나 기각 · 초기 2.95° → 1스텝에 ±0.6° 안으로',
    { x: M, y: 6.1, w: 6.0, h: 0.35, fontSize: 11.5, color: C.muted });
  txt(s, '장애물 — 레벨봉(노란 수직봉)', { x: 6.85, y: 1.3, w: 6.0, h: 0.35, fontSize: 16,
    bold: true, color: C.dark });
  fig(s, 's10_levelrod_102501_front_final', 6.85, 1.7, 2.9, 2.2, 'l');
  txt(s, [
    { text: '색(HSV) + seg 두 경로 OR', options: { bullet: true, breakLine: true } },
    { text: '판자 = 채도로 제거', options: { bullet: true, breakLine: true } },
    { text: '케이블·그림자 = 원근폭으로 제거', options: { bullet: true } },
  ], { x: 9.95, y: 1.75, w: 2.9, h: 2.1, fontSize: 12.5, valign: 'top', paraSpaceAfter: 6 });
  fig(s, 's10_levelrod_persistence', 6.85, 4.0, 5.98, 2.45, 't');
  takeaway(s, '모양으로 못 가르는 스페이서는 "지속성"(최근 4프레임 중 3)으로 가른다 — 대가는 최악 4프레임 지연');
  pageNo(s, 10);
  s.addNotes(
`[0:45] 두 가지를 빠르게 말씀드립니다.
조향은 가로철근의 기울기를 요 오차로 씁니다. 세로철근 소실점도 시도했는데 교차점에서 조각나 엉뚱한 값이 나와 버렸습니다. P제어로 초기 3도 오차를 한 스텝에 잡고 이후 ±0.6도를 유지합니다.
장애물은 레벨봉입니다. 세그 모델이 처음엔 못 봐서 색 기반을 붙였고, 판자·케이블·그림자를 채도와 원근폭으로 걸렀습니다.
끝까지 남은 게 철근 받침인 스페이서입니다. 실루엣이 레벨봉과 거의 같습니다. 구분되는 건 지속성뿐이었습니다. 오검출은 9장 중 1장, 진짜 봉은 연속으로 보입니다. 그래서 4프레임 중 3프레임일 때만 정지합니다.`);
}

// ══ 11. 중복 결속 방지 ═══════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '04  신뢰성', '중복 결속 방지 — 누락은 다시 묶으면 되지만, 이중 결속은 되돌릴 수 없다');
  fig(s, 's11_tie_memory', M, 1.35, 8.1, 5.0, 'l');
  const story = [
    ['시도', '3클래스 모델 (crossing / tie / untie) — 묶인 점을 분류로 거른다', C.steel],
    ['철회', '같은 교차점의 75%가 프레임마다 뒤집힘 → 단일 클래스로 복귀', C.red],
    ['대안', '결속 좌표를 전역좌표로 기억, 반경 70 mm 안이면 제외', C.green],
  ];
  story.forEach(([k, v, col], i) => {
    const y = 1.4 + i * 1.35;
    card(s, 8.85, y, 3.98, 1.2);
    txt(s, k, { x: 9.05, y: y + 0.12, w: 3.6, h: 0.35, fontSize: 15, bold: true, color: col });
    txt(s, v, { x: 9.05, y: y + 0.48, w: 3.65, h: 0.65, fontSize: 12.5, color: C.ink, valign: 'top' });
  });
  stat(s, 8.95, 5.45, 3.9, '42%', '8/19 주행 재생: 중복 요청 5 / 12점을 제외');
  takeaway(s, '중복은 전부 후진 구간 — 전진 때 묶은 자리를 되돌아오며 다시 지났다. 한 정차 안의 병합으로는 못 막는다');
  pageNo(s, 11);
  s.addNotes(
`[0:40] 같은 자리를 두 번 묶으면 와이어가 낭비되고 결속기가 간섭합니다.
처음엔 모델이 묶인 점과 안 묶인 점을 분류하게 했습니다. 그런데 같은 교차점의 75%가 프레임마다 분류가 뒤집혀서 철회했습니다.
대안은 기억입니다. 묶은 자리를 주행 엔코더와 합친 전역좌표로 기억하고, 반경 70mm 안이면 뺍니다.
8월 19일 주행 기록을 재생하면 결속 요청 12점 중 5점, 42%가 이미 묶은 자리였고, 전부 후진 구간이었습니다. 이 메모리가 고유 지점 7개와 정확히 일치하게 걸러냅니다.`);
}

// ══ 12. 안전 계층 ════════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '04  신뢰성', '안전 — 대부분 설계가 아니라 실사고에서 역산됐다');
  fig(s, 's12_safety_layers', M, 1.3, 5.3, 5.1, 'l');
  fig(s, 's12_incident_timeline', 6.2, 1.3, 6.63, 2.85, 't');
  const cases = [
    ['결속 중 횡이동', '데크끝 검사가 결속대기 검사보다 앞 → 순서 교체'],
    ['범퍼 전방향 차단', '빠져나올 방법이 없어 갇힘 → 부딪힌 방향만 차단'],
    ['카메라 먹통 중 GO 유지', '낡은 프레임 재판정 → 이미지 노후화 검사 추가'],
  ];
  cases.forEach(([t, d], i) => {
    const y = 4.3 + i * 0.72;
    card(s, 6.2, y, 6.63, 0.62);
    txt(s, t, { x: 6.4, y, w: 2.6, h: 0.62, fontSize: 13.5, bold: true, color: C.red, valign: 'middle' });
    txt(s, d, { x: 9.0, y, w: 3.75, h: 0.62, fontSize: 12.5, color: C.ink, valign: 'middle' });
  });
  takeaway(s, '모터에 가까운 층일수록 "무엇이 명령하든" 막는다 — 위층 로직의 버그를 아래층이 받아낸다');
  pageNo(s, 12);
  s.addNotes(
`[0:40] 안전은 계층으로 쌓았습니다. 특징은 이 층들 대부분이 실제 사고에서 나왔다는 점입니다.
오른쪽 위가 지난주 사례입니다. 상부가 결속 시퀀스를 도는 도중에 하부가 옆으로 420mm 이동했습니다. 원인은 검사 순서였습니다. 데크끝 검사가 결속 대기 검사보다 앞에 있어서, 결속 중에 배근 끝이 확정되자 바로 횡이동으로 넘어갔습니다.
범퍼도 처음엔 전 방향을 막았는데 빠져나올 방법이 없어 갇혔습니다. 지금은 부딪힌 방향만 막습니다. 카메라가 멈췄는데 낡은 프레임을 계속 판정해 GO를 유지한 사례도 있었습니다.`);
}

// ══ 13. 실증 결과 ════════════════════════════════════════
{
  const s = pres.addSlide();
  header(s, '05  결과', '실증 — 테스트베드 자율결속');
  const stats = [
    ["5레인 · 23점", "'ㄹ'자 자율 커버리지 (9/22)\n11.5분, 레인 상한 도달로 정상 종료", C.accent],
    ['19.1분 / 100점', '웨이포인트 방식 100점 연속 3회 (3월)\n점당 11.2초', C.dark],
    ['−14.2 mm', '스텝 119회 도달오차 (−12~−15)\n허용오차 15 mm와 일치 → 체계 오차', C.steel],
  ];
  stats.forEach(([b, l, col], i) => {
    const x = M + i * 4.16;
    card(s, x, 1.3, 3.96, 1.2);
    txt(s, b, { x: x + 0.2, y: 1.36, w: 3.6, h: 0.55, fontSize: 24, bold: true, color: col });
    txt(s, l, { x: x + 0.2, y: 1.9, w: 3.66, h: 0.55, fontSize: 11.5, color: C.muted, valign: 'top' });
  });
  fig(s, 's13_lanes_20260922', M, 2.6, 6.6, 3.85, 'l');
  fig(s, 's13_100pt', 7.15, 2.7, 5.68, 2.1, 't');
  txt(s, [
    { text: '시간의 73%가 결속 동작 — 검출(2%)을 빠르게 해도 전체는 안 빨라진다', options: { bullet: true, breakLine: true } },
    { text: '→ 스테이지 사이클 단축이 다음 과제', options: { bullet: true } },
  ], { x: 7.2, y: 5.1, w: 5.6, h: 1.3, fontSize: 13, valign: 'top', paraSpaceAfter: 4 });
  takeaway(s, "검출 → 결속 → 이동 → 레인 전환까지 사람 개입 없이 한 판을 끝냈다 (테스트베드 기준)");
  pageNo(s, 13);
  s.addNotes(
`[0:50] 결과입니다. 가장 최근인 9월 22일, 'ㄹ'자로 5개 레인을 돌며 23점을 11.5분에 결속했고, 레인 상한에 도달해 정상 종료했습니다. 검출부터 레인 전환까지 사람 개입이 없었습니다.
3월에는 웨이포인트 방식으로 100점 연속 결속을 3회 해서 평균 19.1분, 점당 11.2초였습니다. 여기서 시간의 73%가 결속 동작이었습니다. 검출을 아무리 빠르게 해도 전체는 안 빨라진다는 뜻입니다.
세 번째 숫자는 약점입니다. 스텝 119회가 전부 목표보다 12~15mm 짧게 끝났고 평균 −14.2mm입니다. 도달 판정 허용오차 15mm와 일치해서, 판정이 허용오차만큼 일찍 끝나는 체계 오차로 보고 있습니다.`);
}

// ══ 14. 한계 & 협업 ══════════════════════════════════════
{
  const s = pres.addSlide();
  s.background = { color: C.dark };
  txt(s, '06  남은 과제', { x: M, y: 0.32, w: 8, h: 0.3, fontSize: 12, bold: true, color: C.accent });
  txt(s, '한계와 함께 풀고 싶은 문제', { x: M, y: 0.6, w: 12, h: 0.62, fontSize: 28, bold: true,
    color: C.white });
  txt(s, '현재 한계', { x: M, y: 1.5, w: 4.3, h: 0.4, fontSize: 17, bold: true, color: 'B8C6D1' });
  txt(s, [
    { text: '실증은 테스트베드 — 현장 연속 운용 전', options: { bullet: true, breakLine: true } },
    { text: '2층 배근 층 분리: 구현, 현장 미검증 (목업 층간 40 mm로는 분리 불가)', options: { bullet: true, breakLine: true } },
    { text: '결속 여부만 판단 — "잘 묶였는지"는 미판정', options: { bullet: true, breakLine: true } },
    { text: '결속 이력이 노드 재시작 시 초기화', options: { bullet: true, breakLine: true } },
    { text: '스텝 도달 −14 mm 체계 오차 보정 중', options: { bullet: true } },
  ], { x: M, y: 2.0, w: 4.3, h: 4.3, fontSize: 13.5, color: C.white, valign: 'top', paraSpaceAfter: 10 });
  const topics = [
    ['격자 기하 강건 추정', '결측·비정렬 격자에서 피치와 방향을 동시에. 지금은 1D 클러스터링 + 중앙값'],
    ['제한 데이터 학습', '현장 라벨링 비용이 지배적. 합성 데이터 · self-supervised 여지'],
    ['결속 품질 시각 검증', '묶인 자리가 제대로 묶였는지 판정'],
    ['평행4절 횡이동 구동', '승강=보폭 연동이라 설계 자유도가 없다. 부하 최적화'],
  ];
  topics.forEach(([t, d], i) => {
    const x = 5.3 + (i % 2) * 3.8, y = 1.5 + Math.floor(i / 2) * 2.4;
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, { x, y, w: 3.6, h: 2.2, rectRadius: 0.1,
      fill: { color: '26394A' }, line: { color: '3A4F61', width: 1 } });
    txt(s, String(i + 1).padStart(2, '0'), { x: x + 0.25, y: y + 0.2, w: 1, h: 0.5, fontSize: 22,
      bold: true, color: C.accent });
    txt(s, t, { x: x + 0.25, y: y + 0.75, w: 3.15, h: 0.4, fontSize: 16, bold: true, color: C.white });
    txt(s, d, { x: x + 0.25, y: y + 1.2, w: 3.15, h: 0.9, fontSize: 12.5, color: 'B8C6D1', valign: 'top' });
  });
  txt(s, '감사합니다', { x: 5.3, y: 6.5, w: 7.5, h: 0.5, fontSize: 18, bold: true, color: C.white,
    align: 'right' });
  s.addNotes(
`[0:45] 남은 과제입니다. 실증은 아직 테스트베드입니다. 2층 배근 층 분리는 구현했지만 목업 층간이 40mm라 검증을 못 했고, 현장은 150mm라 될 것으로 봅니다. 가장 큰 공백은 결속 품질입니다. 묶였는지만 보고, 잘 묶였는지는 못 봅니다.
그래서 오늘 같이 이야기해 보고 싶은 게 네 가지입니다. 결측과 비틀림이 있는 격자에서 피치와 방향을 같이 추정하는 문제, 현장 라벨링 비용을 줄이는 학습 문제, 결속 품질의 시각 검증, 그리고 기구 쪽으로 평행4절 횡이동의 구동 최적화입니다.
감사합니다.`);
}

pres.writeFile({ fileName: OUT }).then(f => console.log('saved', f));
