#!/usr/bin/env bash
# 19-x16: top 시연 인계 구간 다듬기 (tools/top_trim.py, 기본 B) → 원본 재생 확인 → 증강 (REACH_MIN 2,1) → 데이터셋 ×5 → 학습 (새 이름) → 시뮬 (평가 6 쌍 × 3 + 학습 4 쌍 × 3).
#   tools/top19x_trim.sh            (사용자가 "실기 끝, 학습 시작" 이라고 보낸 뒤에만. 로봇 프로그램이 돌면 학습은 learn_run.sh 가 멈춘다)
# 입력: ~/cc_analysis/top19x/run/files_edit.txt (19-x15 통과 24 판, 누름 편집 뒤). 설정은 top19x.sh 와 같다 (COPIES 9 · SHIFT 3 · 시도 4 · STEPS 30000 · ACT 20 · 배치 32 · seed 1).
# 이름: 출력 ~/cc_analysis/top19x16/run, 정책 runs/anypos_top19x16_x5 (이미 있으면 멈춤 — 19-j2 · top 1 · 2 차 정책은 건드리지 않는다).
# 환경 변수: MODE (A · B, 기본 B) STEPS COPIES SHIFT_CM N REACH_MIN (기본 2,1) FROM (trim · aug · dataset · train · eval · report)
set -u
T=$(cd "$(dirname "$0")" && pwd); TELEOP=$(dirname "$T"); PY=~/venvs/learn/bin/python
MODE=${MODE:-B}; STEPS=${STEPS:-30000}; COPIES=${COPIES:-9}; SHIFT_CM=${SHIFT_CM:-3}; N=${N:-3}; REACH_MIN=${REACH_MIN:-2,1}; FROM=${FROM:-trim}
NAME=${NAME:-top19x16}; F=5; RUN="anypos_${NAME}_x$F"
SRC=${SRC:-$HOME/cc_analysis/top19x/run/files_edit.txt}; O=${O:-$HOME/cc_analysis/$NAME/run}; LR=~/cc_analysis/learn/lerobot; RUNS=~/cc_analysis/learn/runs
# 19-x20 (기본 = 지금 그대로): ROLL_STOP=1 이면 다듬기 원본 재생 · 증강 · 평가 시뮬 모터 명령의 어깨 roll 을 실물 멈춤으로 묶는다 (--roll-stop). TRIM_TAG = 다듬은 기록 이름 앞 (기본 trim<MODE>)
ROLL_STOP=${ROLL_STOP:-0}; RS=""; [ "$ROLL_STOP" = 1 ] && RS="--roll-stop"; TRIM_TAG=${TRIM_TAG:-}; TT=""; [ -n "$TRIM_TAG" ] && TT="--tag $TRIM_TAG"
MAP=$TELEOP/config/reach_top_19x.json; EVAL_PAIRS=$TELEOP/config/pairs_top_eval_19x.txt; TRAIN_PAIRS=$TELEOP/config/pairs_top_train4_19x16.txt
mkdir -p "$O"; LOG=$O/run.log
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
fail() { local lg=${2:-$LOG}; say "🔴 $1"; echo "--- 마지막 오류 ($lg):"; grep -av '^\[[0-9:]*\] 🔴' "$lg" | tail -n 12 | cut -c1-300; exit 1; }
ORDER="trim aug dataset train eval report"
run_step() { local s; for s in $ORDER; do [ "$s" = "$FROM" ] && return 0; [ "$s" = "$1" ] && return 1; done; return 0; }
XID0=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
gpu_ok() {
  [ "$(cat /sys/class/power_supply/A*/online 2>/dev/null | head -1)" = "1" ] || { say "🔴 AC 전원이 아니다 — $1 전에 멈춤 (FROM=$2 로 다시)"; exit 9; }
  local x; x=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
  [ "$x" = "$XID0" ] || { say "🔴 NVRM Xid 가 늘었다 ($XID0 → $x) — 멈춤. 재부팅 뒤 FROM=$2 로 다시"; exit 9; }
}
say "시작: $NAME  다듬기 $MODE  (STEPS $STEPS, COPIES $COPIES, SHIFT_CM $SHIFT_CM, N $N, REACH_MIN $REACH_MIN, FROM $FROM${RS:+, ROLL_STOP 1}${TRIM_TAG:+, TRIM_TAG $TRIM_TAG})"
T0=$(date +%s)

if run_step trim; then
  say "1. 다듬기 $MODE + 원본 재생 확인 (tools/top_trim.py make, 위쪽 잡기 · 실물 기준)"
  rm -rf "$O/trim"
  nice -n 19 $PY -B "$T/top_trim.py" make --files "$SRC" --mode "$MODE" --out "$O/trim" $RS $TT >> "$LOG" 2>&1 || fail "다듬기 실패"
  tail -1 "$LOG" | cut -c1-300
  [ -s "$O/trim/files_trim.txt" ] || fail "원본 재생을 통과한 판이 없다"
fi

if run_step aug; then
  say "2. 증강 (판마다 $COPIES, 원 반지름 $SHIFT_CM cm, 위쪽 잡기 지도 칸 하한 L,R $REACH_MIN · 실물 기준)"
  rm -rf "$O/aug"
  nice -n 19 $PY -B "$T/aug_demo.py" --files "$O/trim/files_trim.txt" --copies "$COPIES" --factors 5,10 --shift-cm "$SHIFT_CM" --reach-map "$MAP" --reach-min "$REACH_MIN" $RS \
    --grasp-mode top --grasp-real --out "$O/aug" --tag "$NAME" >> "$LOG" 2>&1 || fail "증강 실패"
  tail -1 "$LOG" | cut -c1-400
fi

if run_step dataset; then
  say "3. 데이터셋 ×$F ($RUN)"
  nice -n 19 $PY "$T/to_lerobot.py" --files "$O/aug/files_x$F.txt" --cond xy --meas-sim --copies 3 --name "$RUN" >> "$LOG" 2>&1 || fail "데이터셋 실패"
  grep -a "에피소드 .* 프레임" "$LOG" | tail -1
fi

if run_step train; then
  [ -e "$RUNS/$RUN" ] && fail "$RUNS/$RUN 이 이미 있다 — 지우지 않는다. 이름을 바꾸거나 직접 옮긴 뒤 FROM=train 으로"
  gpu_ok "학습" train
  say "4. 학습 $RUN ($STEPS 걸음)"
  "$T/learn_run.sh" "$O/train_x$F.log" ~/venvs/learn/bin/lerobot-train --dataset.repo_id="local/$RUN" --dataset.root="$LR/$RUN" \
    --policy.type=act --policy.device=cuda --policy.push_to_hub=false --policy.chunk_size=20 --policy.n_action_steps=20 \
    --output_dir="$RUNS/$RUN" --steps="$STEPS" --batch_size=32 --save_freq="$STEPS" --log_freq=50 --wandb.enable=false --num_workers=2 --seed=1
  rc=$?
  [ $rc = 3 ] && { say "🔴 워치독 · 재생기가 돌고 있어 학습을 멈췄다 — 끝난 뒤 FROM=train 으로 다시 (그 전에 $RUNS/$RUN 을 확인)"; exit 3; }
  [ $rc = 0 ] || fail "학습 실패 (상태 $rc)" "$O/train_x$F.log"
fi

if run_step eval; then
  for P in eval train4; do
    gpu_ok "평가" eval
    PF=$EVAL_PAIRS; [ $P = train4 ] && PF=$TRAIN_PAIRS
    say "5. 시뮬 $P shift · ×$F ($(basename "$PF") × $N 번, 위쪽 잡기 · 실물 기준)"
    nice -n 19 $PY -B "$T/eval_pairs.py" --pairs "$PF" --demos "$O/aug/files_x1.txt" --method shift=shift \
      --method "x$F=act:$RUNS/$RUN/checkpoints/last/pretrained_model" --n "$N" --meas-sim --grasp-mode top --grasp-real --first-from "$O/trim/files_trim.txt" $RS --out "$O/eval_$P" >> "$LOG" 2>&1 || fail "평가 실패 ($P)"
  done
fi

if run_step report; then
  { echo "# 19-x16 결과 — top 시연 다듬기 $MODE ($NAME)"; echo;
    echo "- 다듬기 · 원본 재생 통과: $(grep -c . "$O/trim/files_trim.txt" 2>/dev/null) / $(grep -c . "$SRC") 판";
    echo "- 증강: $(python3 -c "import json;a=json.load(open('$O/aug/summary.json'));print({k:a[k] for k in ('demos','replay_ok','tries','accepted','pass_rate','lists')})" 2>/dev/null)";
    echo "- 학습 ×$F: $(grep -ao 'step:[^ ]* .*loss:[^ ]*' "$O/train_x$F.log" 2>/dev/null | tail -1 | grep -o 'step:[^ ]*\|loss:[^ ]*' | tr '\n' ' ')"; echo;
    for P in eval train4; do [ -f "$O/eval_$P/summary.md" ] && { echo "## $P"; sed 1d "$O/eval_$P/summary.md"; echo; }; done; } > "$O/REPORT.md"
fi
say "끝 ($(( ($(date +%s) - T0) / 60 )) 분) — $O/REPORT.md"
