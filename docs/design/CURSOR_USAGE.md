# Cursor 사용 지침 — Claude 토큰 절감 (VDSim)

작성 2026-06-05. 목적: VDSim 개발에서 Claude(Anthropic) 한도를 아끼기 위해 Cursor 병행.
전 에이전트 공통 원칙은 글로벌 `~/.claude/CLAUDE.md` "Cursor 위임" 섹션. 이 문서는 VDSim 프로젝트별 상세.
저장 위치(권장): `docs/design/CURSOR_USAGE.md`.

## 0. 과금 현실 (2026-06 웹확인, 변동 가능 — 가입 시점 재확인)
- Cursor 플랜: Hobby(Free) / Pro $20 / Pro+ $60 / Ultra $200. 유료플랜은 가격만큼 크레딧 풀 포함($20/$60/$200).
- **Auto mode = 유료플랜 쿼터 내 무제한** (Cursor 가 모델 선택). frontier 모델(Sonnet/Opus) 수동선택 시 크레딧 소모, 초과분 API 종량제.
- 버킷 분리: Cursor 작업은 Anthropic 한도가 아니라 Cursor 쿼터로 빠짐.
- 절감 핵심: 단순작업을 Cursor Auto/Composer 로 빼면 Anthropic 한도 절감 + Cursor 추가비용 0. Opus 를 Cursor 에서 쓰면 버킷만 다르고 비용은 비슷 → 의미 적음.
- (확인 필요) 정확한 플랜·단가·크레딧 정책: cursor.com/docs/models-and-pricing.

## 1. Task 라우팅 rubric (VDSim 구체)
| 작업 유형 | 도구·모델 | 근거 |
|---|---|---|
| 인터페이스/추상클래스 stub, SubsystemContext·DelayLine 보일러플레이트 (#159) | Cursor Composer/Auto | 패턴 정해짐, ctest 검증 |
| default 모듈 구현 (#160) | Cursor Composer/Auto | 기존 거동 복제, 정답 명확 |
| 테스트 케이스 작성, 포맷팅, 이름변경, mechanical 일괄수정 | Cursor Composer/Auto | 반복·기계적 |
| 코어→모듈 경유 리팩터 (#161) 의 기계적 적용 부분 | Cursor (단 default==현재거동 판정은 Claude) | 패턴 적용 |
| 물리 디버깅(타이어/적분기 발산), 수치 정합, 아키텍처 결정, FMI/cosim(VDS1) 프로토콜 설계 | **Claude Opus** | 불확실·추론 핵심 |
| 모호한 spec 해석, `V0.2_SUBSYSTEMS.md` 설계 변경, novelty 판단 | **Claude Opus** | 판단 |

판단 한 줄: 정답이 거의 정해졌고 `ctest` 로 검증되면 Cursor, 아니면 Claude.

## 2. 모델 선택 (Cursor 내)
| 작업 | 권장 모델 | 비용 |
|---|---|---|
| 인라인 편집, 빠른 리팩터, 보일러플레이트 | Composer (Cursor 자체, 약 4x 빠름) | Auto 무제한/저렴 |
| 긴 agent 런 + 중간 판단 | Sonnet (Opus 대비 저비용, 품질 근접) | 크레딧 소모 |
| 모델 고민 회피 | Auto (Cursor 가 선택) | 0 (포함, 무제한) |
| 진짜 어려운 아키텍처 | Opus — 단 이건 Claude Code 에서 하는 게 정석 | 크레딧 큼 |

**위임 기본 = `--model composer-2.5` 고정 (웬만하면 이걸로).** auto 는 Cursor 가 모델선택해 더 비싼 모델 고를 수 있어 회피. frontier(Sonnet/Opus)는 꼭 필요할 때만 수동선택. 크레딧 소진 대시보드 주시.
(확인 필요) 모델명·버전·벤치는 시점 따라 변동 — cursor.com/docs/models-and-pricing.

## 3. 핸드오프 / 일관성
- 진실원천: `docs/HANDOFF.md`(툴 독립적 진행상황) + `docs/design/V0.2_SUBSYSTEMS.md`(설계 spec). Cursor·Claude 둘 다 세션 시작 시 필독.
- 규약 미러링: `CLAUDE.md` 핵심 규약을 `.cursor/rules/`(project rules)로 미러 → Cursor 자동 적용. (확인 필요: .cursor/rules 포맷 — cursor.com/docs)
- MCP: Cursor CLI 가 desktop 과 `mcp.json`·rules·auth 공유. 기존 MCP 재사용 가능.

## 4. 가드레일 (CLAUDE.md 와 동일 강제)
- 검증 우선: 외부 사실/API/수치 추측 금지, 확인 후 기술.
- 규약: wheel FL=0/FR=1/RL=2/RR=3, ISO 8855 RH, YAML 파라미터.
  - **estimation noise = Simon `Q=process/R=measurement`.** (※ 원 지시의 "Thrun R=process" 는 글로벌 정정으로 폐기 — VDSim estimator(`estimator_in_loop` 등)는 Simon. SLAM 교재 맥락에 한해 Thrun.)
- 보안: 현대 산학 실측 tire 데이터 대외비 — 값/세부 커밋 금지, generic 지칭만.
- git: 명시 요청 없이 push/force/tag 이동 금지. commit 메시지 규약 준수.
- 합격기준: 변경 후 **ctest 187 green 유지.** 특히 #161 코어 리팩터는 default == 현재거동(재baseline 금지).

## 5. 워크플로 체크리스트
1. 세션 시작: `docs/HANDOFF.md` + 관련 `docs/design/*` 읽기.
2. 작업 분류: 위 rubric 으로 Cursor vs Claude 결정.
3. Cursor 위임: `~/bin/cursor_delegate.sh "<지시 + 가드레일>" <작업유형> [<workspace절대경로>]` (내부 composer-2.5 고정 + usage 자동로깅 → `~/cursor_eval/deleg.log`; 작업유형 예 boilerplate/test/refactor) 또는 IDE Composer. 직접 `cursor-agent -p --force --trust --model composer-2.5 "..."` 도 가능하나 deleg.log 미기록.
4. 빌드·검증: `cmake --build build -j && (cd build && ctest --output-on-failure)` → 187 green 확인.
5. commit (규약 준수, push 안 함).
6. `docs/HANDOFF.md` 갱신 → 다음 세션·툴이 이어받게.

## 6. Claude ↔ Cursor 충돌 방지
- 동시 편집 금지: 같은 파일·이슈를 두 툴이 동시에 만지지 말 것. 이슈/파일 단위로 분리.
- 브랜치: 위험·대형 작업은 브랜치 분리. 기계적 위임은 작은 단위 commit.
- 재빌드 경쟁: 한 툴이 빌드 중이면 다른 툴 빌드 회피(`build/` 충돌).
- 동기화: 작업 끝나면 HANDOFF.md 갱신 → 다음 툴이 최신 상태 인지.
- 책임: Cursor 산출물도 최종 검증(ctest/리뷰)은 Claude. 위임 ≠ 방치.

---
CLI vs SDK (둘 다 실재 — 2026-06 확인):
- **Cursor CLI (`cursor-agent`)**: 셸 헤드리스(`-p`), 구독쿼터 내 **Auto mode 무제한**, 언어 무관. → **토큰 절감 위임은 이걸로.**
- **Cursor SDK (`@cursor/sdk`)**: TypeScript, 2026-04 public beta, **토큰 종량제**, Python 공식 미지원(커뮤니티/REST). subagents·hooks·cloud VM 등 프로그래밍적 에이전트 인프라용. → 종량제라 절감 목적엔 부적합, 정교한 자동화 필요할 때만.
주의(확인 필요): 제품명/플래그/과금은 실행 시점 `cursor.com/docs/cli`·`models-and-pricing`·`changelog/sdk-release` 재확인.
