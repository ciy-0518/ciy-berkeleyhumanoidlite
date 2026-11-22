#!/usr/bin/env bash
# 19-j2: 인계를 통일 · 깊이 맞춘 시연 (handover_swap.py make2 결과) → 증강 (인계 검사 --handover-check-j) → 데이터셋 ×5 → 학습 ×5 → 평가 (shift · ×5).
#   tools/handover_j.sh <이름> <시연 목록 파일>      예: tools/handover_j.sh demo19j ~/cc_analysis/learn/data/vcan_demo19j/files_ho.txt
# 설정은 19-c2 (anypos_c2.sh) 와 같다: STEPS 30000 · COPIES 9 · SHIFT_CM 3 · 씨앗 0 · 시도 4, 데이터셋 --cond xy --meas-sim --copies 3,
#   학습 ACT 묶음 20 · 배치 32 · seed 1, 평가 pairs_eval_19b.txt 24 쌍 × N 3 (--meas-sim). 다른 점: ×5 만 학습, 증강에 --handover-check-j.
# 환경 변수: STEPS COPIES SHIFT_CM N FROM (aug · dataset · train · eval · report)
# 지킴: CPU 작업 nice 19, GPU (학습 · 평가) 는 하나씩 · AC 전원일 때만 · 부팅 뒤 NVRM Xid 가 늘면 멈춤 (끝 상태 9). 학습은 tools/learn_run.sh (끝 상태 3).
set -u
[ $# -ge 2 ] || { sed -n 2,9p "$0"; exit 2; }
NAME=$1; FILES=$2
T=$(cd "$(dirname "$0")" && pwd); TELEOP=$(dirname "$T"); PY=~/venvs/learn/bin/python
O=~/cc_analysis/handover19j/$NAME; LR=~/cc_analysis/learn/lerobot; RUNS=~/cc_analysis/learn/runs
STEPS=${STEPS:-30000}; COPIES=${COPIES:-9}; SHIFT_CM=${SHIFT_CM:-3}; N=${N:-3}; FROM=${FROM:-aug}
EVAL_PAIRS=$TELEOP/config/pairs_eval_19b.txt; F=5
mkdir -p "$O"; LOG=$O/run.log
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
fail() { local lg=${2:-$LOG}; say "🔴 $1"; echo "--- 마지막 오류 ($lg):"; grep -av '^\[[0-9:]*\] 🔴' "$lg" | tail -n 12 | cut -c1-300; exit 1; }
ORDER="aug dataset train eval report"
run_step() { local s; for s in $ORDER; do [ "$s" = "$FROM" ] && return 0; [ "$s" = "$1" ] && return 1; done; return 0; }
XID0=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
gpu_ok() {
  [ "$(cat /sys/class/power_supply/A*/online 2>/dev/null | head -1)" = "1" ] || { say "🔴 AC 전원이 아니다 — $1 전에 멈춤 (FROM=$2 로 다시)"; exit 9; }
  local x; x=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
  [ "$x" = "$XID0" ] || { say "🔴 NVRM Xid 가 늘었다 ($XID0 → $x) — 멈춤. 재부팅 뒤 FROM=$2 로 다시"; exit 9; }
}
say "시작: $NAME  시연 $FILES ($(grep -c . "$FILES") 판)  (STEPS $STEPS, COPIES $COPIES, SHIFT_CM $SHIFT_CM, N $N, FROM $FROM)"
T0=$(date +%s)

if run_step aug; then
  say "1. 증강 (판마다 $COPIES, 원 반지름 $SHIFT_CM cm, 인계 검사 19-j2)"
  rm -rf "$O/aug"
  nice -n 19 $PY -B "$T/aug_demo.py" --files "$FILES" --copies "$COPIES" --factors 5,10 --shift-cm "$SHIFT_CM" --handover-check-j \
    --out "$O/aug" --tag "$NAME" >> "$LOG" 2>&1 || fail "증강 실패"
  tail -1 "$LOG" | cut -c1-400
fi

if run_step dataset; then
  say "2. 데이터셋 ×$F"
  nice -n 19 $PY "$T/to_lerobot.py" --files "$O/aug/files_x$F.txt" --cond xy --meas-sim --copies 3 --name "anypos_${NAME}_x$F" >> "$LOG" 2>&1 || fail "데이터셋 실패"
  grep -a "에피소드 .* 프레임" "$LOG" | tail -1
fi

if run_step train; then
  gpu_ok "학습" train
  say "3. 학습 ×$F ($STEPS 걸음)"
  rm -rf "$RUNS/anypos_${NAME}_x$F"
  "$T/learn_run.sh" "$O/train_x$F.log" ~/venvs/learn/bin/lerobot-train --dataset.repo_id="local/anypos_${NAME}_x$F" --dataset.root="$LR/anypos_${NAME}_x$F" \
    --policy.type=act --policy.device=cuda --policy.push_to_hub=false --policy.chunk_size=20 --policy.n_action_steps=20 \
    --output_dir="$RUNS/anypos_${NAME}_x$F" --steps="$STEPS" --batch_size=32 --save_freq="$STEPS" --log_freq=50 --wandb.enable=false --num_workers=2 --seed=1
  rc=$?
  [ $rc = 3 ] && { say "🔴 워치독 · 재생기가 돌고 있어 학습을 멈췄다 — 끝난 뒤 FROM=train 으로 다시"; exit 3; }
  [ $rc = 0 ] || fail "학습 실패 (상태 $rc)" "$O/train_x$F.log"
fi

if run_step eval; then
  gpu_ok "평가" eval
  say "4. 평가 shift · ×$F ($(basename "$EVAL_PAIRS") × $N 번)"
  nice -n 19 $PY -B "$T/eval_pairs.py" --pairs "$EVAL_PAIRS" --demos "$O/aug/files_x1.txt" --method shift=shift \
    --method "x$F=act:$RUNS/anypos_${NAME}_x$F/checkpoints/last/pretrained_model" --n "$N" --meas-sim --out "$O/eval" >> "$LOG" 2>&1 || fail "평가 실패"
  python3 "$T/eval_plot.py" "$O/eval/results.json" "$O/eval/map_1p5cm.png" ok15 >> "$LOG" 2>&1
  python3 "$T/eval_plot.py" "$O/eval/results.json" "$O/eval/map_5cm.png" ok5 >> "$LOG" 2>&1
fi

if run_step report; then
  { echo "# 19-j2 결과 — $NAME"; echo; echo "- 증강: $(python3 -c "import json;a=json.load(open('$O/aug/summary.json'));print({k:a[k] for k in ('demos','replay_ok','tries','accepted','pass_rate','why','lists')})" 2>/dev/null)";
    echo "- 학습 ×$F: $(grep -ao 'step:[^ ]* .*loss:[^ ]*' "$O/train_x$F.log" 2>/dev/null | tail -1 | grep -o 'step:[^ ]*\|loss:[^ ]*' | tr '\n' ' ')"; echo;
    [ -f "$O/eval/summary.md" ] && sed 1d "$O/eval/summary.md"; } > "$O/REPORT.md"
fi
say "끝 ($(( ($(date +%s) - T0) / 60 )) 분) — $O/REPORT.md"
