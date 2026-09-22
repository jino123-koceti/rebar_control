# 멀티턴 절대 엔코더 전환 + 동적 자세 플래너 로드맵

**작성일**: 2026-06-15
**대상 패키지**: `rebar_base_control` (현재 활성), 일부 `rebar_vision`(orchestrator)
**브랜치**: `refactoring/phase2-base-control`

---

## 0. 배경 / 핵심 전환

기존 코드 가정:
- 싱글턴(0x90/0x94) = 절대치 (단, 1회전 360° 내에서만, 넘으면 wrap)
- 멀티턴(0x92) = 비절대 (전원오프 시 0 리셋 가정 → 매부팅 호밍 필요)

테스트로 확인된 새 사실 (2026-05-29):
- 멀티턴 전원오프 저장(`0x20 idx=0x04`) enable → **0x92가 다회전 + 전원오프 모두 유지되는 진짜 절대값**
- 영속성 검증: 전원 케이블 탈거 15s 후 0x92 유지 확인 (flash 비휘발 저장 동작) → **0단계(영속성 재검증) 생략 확정**

핵심 전환: **위치 SSOT를 0x92 멀티턴 절대값으로 통일**. 결과로
(a) 0x144~147의 호밍 의존 제거, (b) 0x143/0x147의 0x90 싱글턴 의존 로직 제거,
(c) 고정 시퀀스 자세변경 → 현재 자세 기반 동적 플래너로 전환.

---

## 1. 축별 영향 요약

| 축 | 현재 | 전환 후 | 비고 |
|---|---|---|---|
| 0x141/0x142 (구동휠) | 속도제어, odom은 0x92 차분(상대) | **변경 없음** | 무한회전, 절대 anchoring 불필요 |
| 0x143 (횡이동) | 0x90 싱글턴 + home cal + wrap/누적 상대제어 | 0x92 멀티턴 절대 직접 사용 | wrap/누적·home cal 제거 |
| 0x144 X / 0x145 Y / 0x146 Z | 0x92지만 비절대 가정 → 매부팅 호밍 기준점 | 고정 상수(config) 기준점 → 호밍 경량화 | deg_per_mm·좌표식 동일 |
| 0x147 Yaw | 0x90 L/R 판정 + 0x92 위치(호밍) + 5단계 우회 | 0x92 절대(399° 풀스트로크) | 0x90 hack 제거, 동적 자세변경 |

좌표 변환계수/식은 불변: X 4.497, Y 4.462, Z 13.45 deg/mm,
`x_mm=(1818.69-0x92x)/4.497`, `y_mm=(0x92y+989.36)/4.462`.

---

## 2. 단계별 로드맵

### Phase 1 — 결속부 영속 기준점 config화 (진행 중)
- `homing_controller`에 `use_persistent_multiturn_ref` 플래그 + `persistent_ref_x/y/z/yaw` 파라미터 추가.
- 기본 false → 거동 변화 없음(additive). 코드 플러밍 + 로더 + `_apply_persistent_refs()` 헬퍼.
- ⚠️ yaml 네임스페이스: homing 파라미터는 `homing_controller:` 블록에서 로드되어야 함
  (기존 homing 파라미터들이 `joint_controller:` 블록에 있으나 실제로는 코드 declare 기본값으로 동작 중인 quirk 존재).
- **상수값은 측정 후 기입**: 멀티턴 저장 enable 상태 풀 호밍 1회 → COMPLETE 로그 `References:` 값.

### Phase 2 — 호밍 기준점 override + 호밍 경량화
- `use_persistent_multiturn_ref=true` 시 `homing_references`를 config 상수로 주입.
- 부팅 시 0x92 1회 읽어 **정합성 검사**(상수 대비 허용오차) → 회전중 급전원차단 등 안전망.
- 풀 호밍 시퀀스 → "Z 안전상승 + 0x92 검증"으로 축소 평가.

### Phase 3 — 0x143 횡이동 0x90→0x92 절대 전환
- joint_controller의 0x143 핸들러를 싱글턴+wrap에서 0x92 멀티턴 절대 기준으로 교체.
- `home_encoder_90`/wrap 로직 제거. 별도 task로 분리 가능.

### Phase 4 — Yaw 0x90 L/R hack 제거
- `YAW_CHECK`의 0x90 Left/Right 판정 제거 → 0x92 절대값으로 자세 직접 판단.
- yaw 기준점 config 상수화(Phase 1·2에 포함).

### Phase 5 — 충돌 모델 `safe_Y(yaw)` 확정
- 자세별 안전 Y 범위 함수화. 측정 데이터: `multiturn_absolute_test_data_id.md`(3·4·6·7·8·10번).
- 미측정 구간 보강 + 안전마진. 미측정/불확실 구간은 보수적 차단.

### Phase 6 — 동적 자세 플래너 모듈화
- 현재 자세 (yaw, X, Y)는 0x92로 항상 정확 → (yaw × Y) 구성공간 충돌회피 경로계획.
- `homing_controller._l2r_loop`의 고정 4단계 시퀀스를 planner 호출로 교체.
- orchestrator의 자세변경(결속점 간 이동)도 동일 planner 사용 → 통일.
- 프로토타입: `scripts/sim/workpart_topview_sim.py`. 목표 모듈: `WorkpartKinematics(FK/IK + is_safe)`.
- 얻는 것: 임의 시작자세 동작, 최단경로, 재계획 가능, 호밍 자세복귀와 운용 자세변경 통일.

---

## 3. 공통 주의

- ⚠️ `0x64` ROM 리셋 시 모터 영점 변경 → config 상수 재측정 필수.
- Phase 6 안전성은 `safe_Y(yaw)` 모델 신뢰도에 직결 → 시뮬레이터 셀프테스트 + 실기 회귀 필수.
- 롤백: `use_persistent_multiturn_ref=false`로 전 단계 동작 복귀 (dual-mode).

## 4. 관련 문서
- `rmd_dual_encoder_migration_guide.md` — 0x90 출력축 절대(대안 경로)
- `workpart_workspace_analysis.md` — 작업영역/충돌 분석, 미해결 TODO
- `multiturn_absolute_test_data_id.md` — 자세별 충돌 측정점 정의
- 메모리: `multiturn_absolute_workpart.md`, `tying_sequence.md`, `homing_progress.md`
