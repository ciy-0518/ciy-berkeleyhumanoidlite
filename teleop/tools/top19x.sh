#!/usr/bin/env bash
# 19-x: 위쪽 잡기 시연 → 점검 (top_check.py) → 내보내기 (demo_export) → 증강 (19-c2 설정 + 위쪽 잡기 지도 + 실물 기준) → 데이터셋 ×5
#        → 학습 (anypos_top19x_x5) → 시뮬 평가 (위쪽 잡기 · 실물 기준) → 실기 계획 real19x → 보고.
#   tools/top19x.sh <dry-run 기록 이름 …>        예: tools/top19x.sh dry_vcan_2130 dry_vcan_2215
#   (기록은 ~/run.sh dry --pairs config/pairs_top_19x.txt --grasp-mode top --grasp-real --start-gate 로 모은 것)
# 설정은 19-c2 · 19-j2 와 같다: STEPS 30000 · COPIES 9 · SHIFT_CM 3 · 시도 4, 데이터셋 --cond xy --meas-sim --copies 3, 학습 ACT 묶음 20 · 배치 32 · seed 1,
#   평가 config/pairs_top_eval_19x.txt 6 쌍 × N 3 (--meas-sim). 다른 점: 증강 · 평가 모두 --grasp-mode top --grasp-real, 범위 지도 config/reach_top_19x.json.
# 환경 변수: STEPS COPIES SHIFT_CM N FROM (check · export · aug · dataset · train · eval · plan · report)
# 지킴: CPU 작업 nice 19, GPU (학습 · 평가) 는 하나씩 · AC 전원일 때만 · 부팅 뒤 NVRM Xid 가 늘면 멈춤 (끝 상태 9). 학습은 tools/learn_run.sh (끝 상태 3).
#   지금 정책 (anypos_demo19j_x5) 과 real19j 결과물은 건드리지 않는다 (이름이 모두 top19x · real19x).
set -u
[ $# -ge 1 ] || [ "${FROM:-check}" != "check" ] || { sed -n 2,11p "$0"; exit 2; }
T=$(cd "$(dirname "$0")" && pwd); TELEOP=$(dirname "$T"); PY=~/venvs/learn/bin/python
O=~/cc_analysis/top19x/run; DATA=~/cc_analysis/learn/data/vcan_top19x; LR=~/cc_analysis/learn/lerobot; RUNS=~/cc_analysis/learn/runs
STEPS=${STEPS:-30000}; COPIES=${COPIES:-9}; SHIFT_CM=${SHIFT_CM:-3}; N=${N:-3}; FROM=${FROM:-check}
NAME=top19x; F=5; RUN="anypos_${NAME}_x$F"
EVAL_PAIRS=$TELEOP/config/pairs_top_eval_19x.txt; MAP=$TELEOP/config/reach_top_19x.json
mkdir -p "$O"; LOG=$O/run.log
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
fail() { local lg=${2:-$LOG}; say "🔴 $1"; echo "--- 마지막 오류 ($lg):"; grep -av '^\[[0-9:]*\] 🔴' "$lg" | tail -n 12 | cut -c1-300; exit 1; }
ORDER="check export aug dataset train eval plan report"
run_step() { local s; for s in $ORDER; do [ "$s" = "$FROM" ] && return 0; [ "$s" = "$1" ] && return 1; done; return 0; }
XID0=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
gpu_ok() {
  [ "$(cat /sys/class/power_supply/A*/online 2>/dev/null | head -1)" = "1" ] || { say "🔴 AC 전원이 아니다 — $1 전에 멈춤 (FROM=$2 로 다시)"; exit 9; }
  local x; x=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
  [ "$x" = "$XID0" ] || { say "🔴 NVRM Xid 가 늘었다 ($XID0 → $x) — 멈춤. 재부팅 뒤 FROM=$2 로 다시"; exit 9; }
}
[ $# -ge 1 ] && printf '%s\n' "$@" > "$O/records.txt"
say "시작: $NAME  기록 $(tr '\n' ' ' < "$O/records.txt" 2>/dev/null)  (STEPS $STEPS, COPIES $COPIES, SHIFT_CM $SHIFT_CM, N $N, FROM $FROM)"
T0=$(date +%s)

if run_step check; then
  say "1. 점검 (성공 · L>R · 인계 · 실물 기준 · 엄격 · 시작 자세)"
  python3 "$T/top_check.py" $(cat "$O/records.txt") --list-out "$O/pass.txt" | tee -a "$LOG" | tail -3
  [ -s "$O/pass.txt" ] || fail "통과한 시연이 없다"
fi

if run_step export; then
  say "2. 내보내기 → $DATA"
  nice -n 19 $PY -B "$T/demo_export.py" --out "$DATA" $(cat "$O/records.txt") >> "$LOG" 2>&1 || fail "내보내기 실패"
  : > "$O/files_demo.txt"
  while read -r n; do [ -f "$DATA/$n.npz" ] && echo "$DATA/$n.npz" >> "$O/files_demo.txt" || say "🟡 내보낸 파일 없음: $n"; done < "$O/pass.txt"
  say "   학습 시연 $(grep -c . "$O/files_demo.txt") 판 ($O/files_demo.txt — can_go --mode-auto 의 시연 거리도 이 목록)"
fi

if run_step aug; then
  say "3. 증강 (판마다 $COPIES, 원 반지름 $SHIFT_CM cm, 위쪽 잡기 지도 · 실물 기준)"
  rm -rf "$O/aug"
  nice -n 19 $PY -B "$T/aug_demo.py" --files "$O/files_demo.txt" --copies "$COPIES" --factors 5,10 --shift-cm "$SHIFT_CM" --reach-map "$MAP" \
    --grasp-mode top --grasp-real --out "$O/aug" --tag "$NAME" >> "$LOG" 2>&1 || fail "증강 실패"
  tail -1 "$LOG" | cut -c1-400
fi

if run_step dataset; then
  say "4. 데이터셋 ×$F"
  nice -n 19 $PY "$T/to_lerobot.py" --files "$O/aug/files_x$F.txt" --cond xy --meas-sim --copies 3 --name "$RUN" >> "$LOG" 2>&1 || fail "데이터셋 실패"
  grep -a "에피소드 .* 프레임" "$LOG" | tail -1
fi

if run_step train; then
  gpu_ok "학습" train
  say "5. 학습 $RUN ($STEPS 걸음)"
  rm -rf "$RUNS/$RUN"
  "$T/learn_run.sh" "$O/train_x$F.log" ~/venvs/learn/bin/lerobot-train --dataset.repo_id="local/$RUN" --dataset.root="$LR/$RUN" \
    --policy.type=act --policy.device=cuda --policy.push_to_hub=false --policy.chunk_size=20 --policy.n_action_steps=20 \
    --output_dir="$RUNS/$RUN" --steps="$STEPS" --batch_size=32 --save_freq="$STEPS" --log_freq=50 --wandb.enable=false --num_workers=2 --seed=1
  rc=$?
  [ $rc = 3 ] && { say "🔴 워치독 · 재생기가 돌고 있어 학습을 멈췄다 — 끝난 뒤 FROM=train 으로 다시"; exit 3; }
  [ $rc = 0 ] || fail "학습 실패 (상태 $rc)" "$O/train_x$F.log"
fi

if run_step eval; then
  gpu_ok "평가" eval
  say "6. 시뮬 평가 shift · ×$F ($(basename "$EVAL_PAIRS") × $N 번, 위쪽 잡기 · 실물 기준)"
  nice -n 19 $PY -B "$T/eval_pairs.py" --pairs "$EVAL_PAIRS" --demos "$O/aug/files_x1.txt" --method shift=shift \
    --method "x$F=act:$RUNS/$RUN/checkpoints/last/pretrained_model" --n "$N" --meas-sim --grasp-mode top --grasp-real --out "$O/eval" >> "$LOG" 2>&1 || fail "평가 실패"
  python3 "$T/eval_plot.py" "$O/eval/results.json" "$O/eval/map_1p5cm.png" ok15 >> "$LOG" 2>&1
  python3 "$T/eval_plot.py" "$O/eval/results.json" "$O/eval/map_5cm.png" ok5 >> "$LOG" 2>&1
fi

if run_step plan; then
  say "7. 실기 계획 real19x (평가 6 쌍 × 위쪽 잡기 정책 1 회, 지금 방식은 도달 불가 표시)"
  nice -n 19 $PY -B "$T/top19x_plan.py" --pairs "$EVAL_PAIRS" --demos-top "$O/files_demo.txt" --out ~/cc_analysis/real19x >> "$LOG" 2>&1 || fail "실기 계획 실패"
  tail -1 "$LOG"
fi

if run_step report; then
  { echo "# 19-x 결과 — 위쪽 잡기 ($NAME)"; echo; echo "- 점검 통과: $(grep -c . "$O/pass.txt" 2>/dev/null) 판";
    echo "- 증강: $(python3 -c "import json;a=json.load(open('$O/aug/summary.json'));print({k:a[k] for k in ('demos','replay_ok','tries','accepted','pass_rate','why','lists')})" 2>/dev/null)";
    echo "- 학습 ×$F: $(grep -ao 'step:[^ ]* .*loss:[^ ]*' "$O/train_x$F.log" 2>/dev/null | tail -1 | grep -o 'step:[^ ]*\|loss:[^ ]*' | tr '\n' ' ')"; echo;
    [ -f "$O/eval/summary.md" ] && sed 1d "$O/eval/summary.md"; echo; echo "- 실기 계획: ~/cc_analysis/real19x/runbook.md"; } > "$O/REPORT.md"
fi
say "끝 ($(( ($(date +%s) - T0) / 60 )) 분) — $O/REPORT.md"
