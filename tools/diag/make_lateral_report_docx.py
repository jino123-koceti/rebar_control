#!/usr/bin/env python3
"""횡이동 구동부 용량 검토 보고서를 Word(.docx)로 생성한다.

## 왜
심사·보고용으로 편집 가능한 문서가 필요하다. 아티팩트 페이지는 공유·열람용이고,
JPG는 삽입용이지만, 본문을 손볼 수 있는 형식이 따로 있어야 한다.

⚠ 이 장비에는 `libreoffice-writer`가 없어 HTML→docx 변환이 불가능하다
   ("no export filter"). 그래서 python-docx로 직접 작성한다.

사용:
    python3 tools/diag/make_lateral_report_docx.py
결과: reports/횡이동_구동부_용량검토.docx
"""
import os

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Cm, Pt, RGBColor

BASE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FIG = os.path.join(BASE, 'docs', 'reports', 'figures')
OUT = os.path.join(BASE, 'reports', '횡이동_구동부_용량검토.docx')

INK = RGBColor(0x33, 0x33, 0x33)
MUTED = RGBColor(0x66, 0x66, 0x66)


def setup(doc):
    st = doc.styles['Normal']
    st.font.name = '맑은 고딕'
    st.font.size = Pt(10.5)
    st.paragraph_format.space_after = Pt(6)
    st.paragraph_format.line_spacing = 1.4
    for s in doc.sections:
        s.left_margin = s.right_margin = Cm(2.4)
        s.top_margin = s.bottom_margin = Cm(2.2)


def para(doc, text, size=10.5, bold=False, color=INK, after=6, align=None):
    p = doc.add_paragraph()
    r = p.add_run(text)
    r.font.size = Pt(size)
    r.bold = bold
    r.font.color.rgb = color
    p.paragraph_format.space_after = Pt(after)
    if align:
        p.alignment = align
    return p


def rich(doc, parts, after=6):
    """parts: [(텍스트, 굵게), ...] — 문장 안에서 강조를 섞는다."""
    p = doc.add_paragraph()
    for t, b in parts:
        r = p.add_run(t)
        r.bold = b
        r.font.size = Pt(10.5)
        r.font.color.rgb = INK
    p.paragraph_format.space_after = Pt(after)
    return p


def box(doc, parts):
    """요약·경고용 음영 상자 (1x1 표로 구현)."""
    t = doc.add_table(rows=1, cols=1)
    t.style = 'Table Grid'
    c = t.cell(0, 0)
    c.paragraphs[0].clear() if hasattr(c.paragraphs[0], 'clear') else None
    p = c.paragraphs[0]
    for txt, b in parts:
        r = p.add_run(txt)
        r.bold = b
        r.font.size = Pt(10)
        r.font.color.rgb = INK
    _shade(c, 'F2F2F2')
    doc.add_paragraph().paragraph_format.space_after = Pt(4)
    return t


def _shade(cell, hexcolor):
    from docx.oxml.ns import qn
    from docx.oxml import OxmlElement
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:val'), 'clear')
    shd.set(qn('w:fill'), hexcolor)
    tcPr.append(shd)


def table(doc, header, rows, widths=None, right_cols=()):
    t = doc.add_table(rows=1, cols=len(header))
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for i, h in enumerate(header):
        c = t.rows[0].cells[i]
        p = c.paragraphs[0]
        r = p.add_run(h)
        r.bold = True
        r.font.size = Pt(9.5)
        _shade(c, 'E8E8E8')
    for row in rows:
        cells = t.add_row().cells
        for i, v in enumerate(row):
            txt = str(v)
            emph = txt.startswith('*')       # '*' 접두 = 강조 행
            if emph:
                txt = txt[1:]
            p = cells[i].paragraphs[0]
            r = p.add_run(txt)
            r.font.size = Pt(9.5)
            r.bold = emph
            r.font.color.rgb = INK
            if i in right_cols:
                p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    if widths:
        for i, w in enumerate(widths):
            for row in t.rows:
                row.cells[i].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


def figure(doc, filename, caption, width=15.6):
    path = os.path.join(FIG, filename)
    if not os.path.exists(path):
        para(doc, '[그림 누락: %s]' % filename, color=MUTED)
        return
    doc.add_picture(path, width=Cm(width))
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    para(doc, caption, size=9, color=MUTED, after=12,
         align=WD_ALIGN_PARAGRAPH.CENTER)


def main():
    doc = Document()
    setup(doc)

    # ── 표지 ─────────────────────────────────────────────
    h = doc.add_heading('횡이동 구동부 용량 검토', level=0)
    h.alignment = WD_ALIGN_PARAGRAPH.LEFT
    para(doc, '현행 1축 구성의 실측 분석과 3차년도 장비 모터 수 산정',
         size=11, color=MUTED, after=2)
    para(doc, '대상: RMD X4-36 (CAN ID 0x143)  ·  측정일: 2026-08-18  ·  '
              '철근 결속 로봇 개발', size=9.5, color=MUTED, after=14)

    box(doc, [
        ('요약.  ', True),
        ('현행 횡이동 모터는 최대 정격의 ', False), ('100%', True),
        ('를 사용 중이며 설계 여유가 없다. 동일 설정에서 성공과 실패가 반복되는 원인이 '
         '여기에 있다. 3차년도 장비는 중량(50→79 kg)과 승강 높이(70→100 mm)가 모두 '
         '증가하여 소요 토크가 ', False),
        ('2.26배', True), ('가 되므로, ', False), ('모터 4개 분담', True),
        ('이 필요하다.', False)])

    # ── 1 ────────────────────────────────────────────────
    doc.add_heading('1. 측정 방법', level=1)
    rich(doc, [
        ('횡이동은 위치 명령(0xA4)으로 제어되는데, 이 명령은 ', False),
        ('응답이 1회뿐', True),
        ('이라 이동 중 전류가 피드백에 남지 않는다. 따라서 0x9C(Read Motor Status 2)를 '
         '고속 폴링하는 전용 프로브(tools/diag/lateral_probe.py)를 제작하여 '
         '초당 17~19 샘플로 파형을 확보했다.', False)])
    rich(doc, [
        ('측정값은 진폭(iq) 단위이며 실효값은 √2로 나눈 값이다. 이 해석은 교차검증으로 '
         '확인되었다 — 토크 제한을 해제했을 때 실측 피크(30.5 A)를 √2로 나누면 '
         '21.6 A rms로, 데이터시트의 최대 상전류 ', False),
        ('21.5 A rms와 정확히 일치', True), ('한다.', False)])

    # ── 2 ────────────────────────────────────────────────
    doc.add_heading('2. 전류 파형 실측', level=1)
    figure(doc, 'load_lateral.jpg',
           '[그림 1] 횡이동 1회전당 모터 전류. 2026-08-18 12:51:30부터 20초간, 횡이동 2회. '
           '1회전(70 mm 이동)마다 0.5 A에서 30.5 A로 스파이크가 발생한다.')
    table(doc,
          ['구분', '실측(진폭)', '실효(rms)', '스펙', '대비'],
          [['정지 중', '0.5 A', '0.4 A', '—', '—'],
           ['이동 중 평균', '7.7 A', '5.5 A', '연속 6.1 A', '90%'],
           ['구간 실효값', '8.2 A', '5.8 A', '연속 6.1 A', '95%'],
           ['*피크', '*30.5 A', '*21.6 A', '*최대 21.5 A', '*100%']],
          widths=[3.6, 2.9, 2.9, 3.2, 2.2], right_cols=(1, 2, 3, 4))
    box(doc, [
        ('피크가 드라이버 한계에 정확히 붙어 있다. ', True),
        ('모터가 낼 수 있는 힘을 전부 쓰고 있다는 뜻으로, 부하가 조금만 증가해도 '
         '즉시 실패한다.', False)])

    # ── 3 ────────────────────────────────────────────────
    doc.add_heading('3. 성공·실패가 갈리는 원인', level=1)
    para(doc, '동일한 설정(속도·가감속·토크)에서 어떤 때는 철근을 넘고 어떤 때는 스톨이 '
              '발생하는 현상이 반복되었다. 원인은 설계 여유가 0%이기 때문이다.')
    rich(doc, [
        ('접촉 지점의 수 mm 차이, 철근 지름 편차, 로봇 자세, 정지마찰 등은 정상적인 여유가 '
         '있으면 흡수되는 잡음이다. 실제로 토크 상한을 100%·150%·200%·255%로 조정하고 '
         '속도(80~200 dps)와 가감속(1000~10000 dps/s)을 함께 바꾸어도 일관된 개선이 '
         '없었다. ', False),
        ('부족한 것은 설정이 아니라 힘 자체', True), ('이기 때문이다.', False)])

    # ── 4 ────────────────────────────────────────────────
    doc.add_heading('4. 열적 · 전기적 한계', level=1)
    para(doc, '모터가 과전류로 손상되는 경로는 영구자석 감자(demagnetization)와 드라이버 '
              '소자 파괴이며, 데이터시트의 최대 상전류 21.5 A rms가 이를 고려한 제조사 '
              '한계선이다. 현행 운용은 이 선을 정확히 밟고 있다.')
    para(doc, '발열은 3·I²·R(상저항 0.35 Ω)로 결정된다. 절연등급 F(권선 한계 155 °C) '
              '기준으로 계산하면 다음과 같다.')
    table(doc,
          ['조건', '전류', '동손', '권선 온도상승률', '한계 도달'],
          [['정격 연속', '6.1 A rms', '39 W', '1.7 K/s', '—'],
           ['구간 실효값', '8.2 A rms', '71 W', '3.1 K/s', '—'],
           ['*피크 유지', '*21.6 A rms', '*489 W', '*21.1 K/s', '*약 5초']],
          widths=[3.4, 2.9, 2.4, 3.4, 2.7], right_cols=(1, 2, 3, 4))
    box(doc, [
        ('미도달 상태로 버티면 약 5초 만에 절연 한계에 도달한다. ', True),
        ('실제로 목표 미달 상태가 수십 초 지속되어 모터에서 발연이 발생했고, 드라이버에 '
         '스톨 알람(0x0002)이 래치되었다. 드라이버가 보고하는 온도는 기판 센서 값이므로 '
         '권선 온도를 반영하지 못한다 — 센서가 정상이어도 안전하다고 볼 수 없다.', False)])

    # ── 5 ────────────────────────────────────────────────
    doc.add_page_break()
    doc.add_heading('5. 3차년도 장비 하중 조건', level=1)
    table(doc,
          ['항목', '현행', '3차년도', '배수'],
          [['작업공간', '700 × 440', '750 × 520', '면적 ×1.27'],
           ['장비 중량', '50 kg', '79 kg', '×1.58'],
           ['횡이동 보폭', '70 mm', '100 mm', '×1.43'],
           ['승강 높이', '70 mm', '100 mm', '×1.43'],
           ['*1회전당 일 (m·g·h)', '*34.3 J', '*77.5 J', '*×2.26']],
          widths=[4.6, 3.4, 3.4, 3.4], right_cols=(1, 2, 3))
    rich(doc, [
        ('기구는 ', False), ('평행 4절링크', True),
        ('로, 좌우 크랭크가 동기 회전하며 플랫폼을 원 궤적으로 평행이동시킨다. '
         '이 구조에서는 수평 이동량과 승강 높이가 크랭크 원에 의해 함께 결정되어 '
         '독립적으로 설계할 수 없다. 철근이 100 mm 배수 간격으로 시공되므로 보폭 100 mm는 '
         '고정 조건이며, 따라서 승강 100 mm도 불가피하다.', False)])
    rich(doc, [
        ('참고로 수평 이동은 중력에 대해 일을 하지 않으므로, 소요 토크를 결정하는 것은 ', False),
        ('중량 × 승강 높이', True), ('이다.', False)])

    # ── 6 ────────────────────────────────────────────────
    doc.add_heading('6. 모터 수 산정', level=1)
    figure(doc, 'lateral_motor_count.jpg',
           '[그림 2] 3차년도 조건에서 모터 수에 따른 피크 전류 사용률.')
    table(doc,
          ['구성', '피크(최대 대비)', '실효(연속 대비)', '모터당 동손', '판정'],
          [['현행 (1개, 현 하중)', '100%', '135%', '489 W', '여유 없음'],
           ['3차년도 · 2개', '113%', '128%', '569 W', '스펙 초과 — 불가'],
           ['3차년도 · 3개', '75%', '85%', '253 W', '빠듯 — 권장 안 함'],
           ['*3차년도 · 4개', '*57%', '*64%', '*142 W', '*권장']],
          widths=[3.8, 2.9, 2.9, 2.5, 3.4], right_cols=(1, 2, 3))
    box(doc, [
        ('3개가 아니라 4개를 권장하는 이유.  ', True),
        ('3개(75%)도 스펙 이내지만 여유가 25%에 불과하다. 현행 장비가 100%에서 성공·실패를 '
         '반복한 경험에 비추어, 철근 현장의 부하 편차를 흡수하기에 부족하다. 4개는 부하가 '
         '1.7배가 되어도 감당하며, 스톨 시 절연 한계 도달 시간도 약 5초에서 약 19초로 '
         '늘어난다.', False)])

    # ── 7 ────────────────────────────────────────────────
    doc.add_heading('7. 설계 시 반영 사항', level=1)

    doc.add_heading('가. 다축 동기화', level=2)
    para(doc, '4축이 미세하게 어긋나면 서로 맞버텨 합력은 그대로인데 전류만 증가한다. '
              '동일 명령을 동시 전송하고, 축 간 위치 편차가 임계를 넘으면 전체를 '
              '정지시키는 감시 로직이 필요하다.')

    doc.add_heading('나. 미도달 보호', level=2)
    rich(doc, [
        ('4축 구성에서도 목표 미달은 발생한다. ', False),
        ('스톨 알람 감지 → 0x76 System Reset → 출발 위치로 복귀', True),
        (' 순서의 보호가 필요하다. 알람 해제는 0x76만 유효하며 0x80/0x88/0x9B 조합으로는 '
         '해제되지 않음을 실측 확인했다. ', False),
        ('복귀 시 토크를 차단해서는 안 된다', True),
        (' — 들어 올려진 상태에서 출력을 끊으면 장비가 낙하한다.', False)])

    doc.add_heading('다. 전원 계통', level=2)
    para(doc, '4축이 동시에 피크를 인출하면 순간 전류가 현행의 2배를 넘는다. 별건으로 '
              '확인된 모터 전류 과도의 제어기 결합 문제를 고려하여, 제어기 전원을 모터 '
              '전력계에서 분리하는 설계를 초기 단계에서 반영해야 한다.')

    doc.add_heading('라. 계측', level=2)
    para(doc, '0x9C 폴링 기반 전류 감시를 4축으로 확장하면 축 간 편차와 발열을 동시에 '
              '감시할 수 있다.')

    # ── 부록 ─────────────────────────────────────────────
    doc.add_heading('부록. 참조 사양 (RMD X4-36)', level=1)
    table(doc,
          ['항목', '값', '항목', '값'],
          [['감속비', '36', '입력 전압', '24 V'],
           ['정격 토크', '10.5 N·m', '최대 토크', '34 N·m'],
           ['정격 상전류', '6.1 A(rms)', '최대 상전류', '21.5 A(rms)'],
           ['정격 속도', '83 RPM', '토크 상수', '1.9 N·m/A'],
           ['상저항', '0.35 Ω', '절연등급', 'F (155 °C)']],
          widths=[3.6, 3.6, 3.6, 3.6], right_cols=(1, 3))

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    doc.save(OUT)
    print('저장: %s' % OUT)


if __name__ == '__main__':
    main()
