#!/usr/bin/env bash
# 19-c2: 임의 좌표 시연 → 내보내기 → 전수 엄격 검사 → 증강 → 학습 셋 → 평가 → 보고 (작업 지시 19-c1 에서 만듦).
#   tools/anypos_c2.sh <이름> <기록 이름…>        예: tools/anypos_c2.sh demo19b dry_vcan_2240 dry_vcan_2310
# 환경 변수 (괄호 = 기본):
#   STEPS (30000)       ACT 학습 걸음 (셋 모두 같게, 배치 32 · 묶음 20 = 17-a 와 같음)
#   COPIES (9)          판마다 증강 수 (×10 = 원본 + 9).  FACTORS ("5 10") 학습할 배율 (×1 은 늘 한다)
#   SHIFT_CM (3)        증강 옮김 원 반지름
#   N (3)               평가 반복.  EVAL_PAIRS (config/pairs_eval_19b.txt).  LIMIT_PAIRS (없음 = 다)
#   EXPORT (1)          0 이면 내보내기를 건너뛴다 (이미 ~/cc_analysis/learn/data 에 있는 기록)
#   FROM (export)       그 단계부터: export · census · aug · dataset · train · eval · report
# 출력: ~/cc_analysis/anypos_c2/<이름>/ (REPORT.md · census.json · aug/ · eval/ · train_x*.log)
#   데이터셋 ~/cc_analysis/learn/lerobot/anypos_<이름>_x<F>, 체크포인트 ~/cc_analysis/learn/runs/anypos_<이름>_x<F>
# 지킴: CPU 작업은 nice 19, GPU 작업 (학습 · 평가) 은 하나씩 차례로 · AC 전원일 때만 · 부팅 뒤 NVRM Xid 가 늘면 멈춤 (끝 상태 9).
#       학습은 tools/learn_run.sh 로 (워치독 · 재생기가 보이면 시작하지 않거나 멈춤 — 끝 상태 3).
set -u
[ $# -ge 2 ] || { sed -n 2,14p "$0"; exit 2; }
NAME=$1; shift; RECS="$*"
T=$(cd "$(dirname "$0")" && pwd); TELEOP=$(dirname "$T"); PY=~/venvs/learn/bin/python
O=~/cc_analysis/anypos_c2/$NAME; LR=~/cc_analysis/learn/lerobot; RUNS=~/cc_analysis/learn/runs; DATA=~/cc_analysis/learn/data/vcan_$NAME
STEPS=${STEPS:-30000}; COPIES=${COPIES:-9}; FACTORS=${FACTORS:-"5 10"}; SHIFT_CM=${SHIFT_CM:-3}; N=${N:-3}
EVAL_PAIRS=${EVAL_PAIRS:-$TELEOP/config/pairs_eval_19b.txt}; LIMIT_PAIRS=${LIMIT_PAIRS:-}; EXPORT=${EXPORT:-1}; FROM=${FROM:-export}
mkdir -p "$O"; LOG=$O/c2.log
say() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
# 19-c2a: 실패하면 그 로그의 마지막 줄들 (Traceback 끝) 을 화면에도 찍고 끝낸다. fail <글> [로그]
fail() { local lg=${2:-$LOG}; say "🔴 $1"; echo "--- 마지막 오류 ($lg):"; grep -av '^\[[0-9:]*\] 🔴' "$lg" | tail -n 12 | cut -c1-300; exit 1; }
ORDER="export census aug dataset train eval report"
run_step() { local s; for s in $ORDER; do [ "$s" = "$FROM" ] && return 0; [ "$s" = "$1" ] && return 1; done; return 0; }
XID0=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
gpu_ok() {
  [ "$(cat /sys/class/power_supply/A*/online 2>/dev/null | head -1)" = "1" ] || { say "🔴 AC 전원이 아니다 — $1 전에 멈춤 (전원을 꽂고 FROM=$2 로 다시)"; exit 9; }
  local x; x=$(journalctl -k -b 2>/dev/null | grep -c "NVRM: Xid")
  [ "$x" = "$XID0" ] || { say "🔴 NVRM Xid 가 늘었다 ($XID0 → $x) — 멈춤. 재부팅 뒤 FROM=$2 로 다시"; exit 9; }
}
say "시작: $NAME  기록 $RECS  (STEPS $STEPS, COPIES $COPIES, FACTORS 1 $FACTORS, N $N, FROM $FROM)"
T0=$(date +%s)

# 1. 내보내기
if run_step export && [ "$EXPORT" = "1" ]; then
  say "1. 내보내기 → $DATA"
  rm -rf "$DATA"
  nice -n 19 $PY "$T/demo_export.py" $RECS --out "$DATA" >> "$LOG" 2>&1 || fail "내보내기 실패"
fi

# 2. 전수 엄격 검사 (VR 기록의 판정: 성공 + 엄격 통과)
if run_step census; then
  say "2. 전수 엄격 검사"
  $PY - "$O" $RECS <<'PYEOF' >> "$LOG" 2>&1 || fail "전수 검사 실패"
import glob, json, sys
from pathlib import Path
sys.path.insert(0, str(Path.home() / "teleop-docs/_teleop/tools"))
import learn_data as L, demo_export as DE
out, recs = Path(sys.argv[1]), sys.argv[2:]
allm = [DE.load_episode(f)["meta"] | {"_path": f} for f in sorted(glob.glob(str(L.DATA / "**" / "*.npz"), recursive=True))]
allm = [m for m in allm if m.get("record") in recs and not m.get("aug")]
base = {e["path"] for e in L.episode_files("base", records=recs)}
rows = []
for m in allm:
    st = (m.get("quality") or {}).get("strict") or {}
    ex = L.load_exclude().get((m.get("record"), int(m.get("episode") or 0)))
    why = ("결과 " + str(m.get("result"))) if not m.get("success") else (("제외 목록: " + ex[:40]) if ex is not None else "기본 묶음 아님 (release · 정렬 · 실제 시간)") if m["_path"] not in base else (
          "엄격 판정 없음" if not st else ("엄격 실패 " + " ".join(st.get("flags", [])) if st.get("fail") else ""))
    rows.append({"name": f"{m['record']}_ep{int(m['episode']):02d}", "path": m["_path"], "start": (m.get("can_start_xy_cm")), "goal": (m.get("can_goal_xy_cm")),
                 "pair": m.get("combo_index"), "ok": why == "", "why": why})
ok = [r for r in rows if r["ok"]]
(out / "files_census.txt").write_text("\n".join(r["path"] for r in ok) + "\n", encoding="utf-8")
why = {}
for r in rows:
    if not r["ok"]:
        why[r["why"]] = why.get(r["why"], 0) + 1
pairs = sorted({tuple(r["start"] or []) + tuple(r["goal"] or []) for r in ok})
(out / "census.json").write_text(json.dumps({"records": recs, "episodes": len(rows), "pass": len(ok), "why": why, "pairs_covered": len(pairs), "rows": rows}, ensure_ascii=False, indent=1), encoding="utf-8")
print(f"전수 검사: 판 {len(rows)}, 통과 {len(ok)}, 까닭 {why}, 통과한 좌표 쌍 {len(pairs)} 가지")
PYEOF
  tail -1 "$LOG"
  [ -s "$O/files_census.txt" ] || { say "🔴 통과한 판이 없다"; exit 1; }
fi

# 3. 증강
if run_step aug; then
  say "3. 증강 (판마다 $COPIES, 원 반지름 $SHIFT_CM cm)"
  rm -rf "$O/aug"
  nice -n 19 $PY -B "$T/aug_demo.py" --files "$O/files_census.txt" --copies "$COPIES" --factors "$(echo $FACTORS | tr ' ' ',')" --shift-cm "$SHIFT_CM" \
    --out "$O/aug" --tag "$NAME" >> "$LOG" 2>&1 || fail "증강 실패"
  tail -1 "$LOG" | cut -c1-400
fi

# 4. 데이터셋 (조건 xy, 관측에 처짐 모델 + 잡음 3 벌 — 17-a act_multi_xy 와 같게)
if run_step dataset; then
  for F in 1 $FACTORS; do
    say "4. 데이터셋 ×$F"
    nice -n 19 $PY "$T/to_lerobot.py" --files "$O/aug/files_x$F.txt" --cond xy --meas-sim --copies 3 --name "anypos_${NAME}_x$F" >> "$LOG" 2>&1 || fail "데이터셋 ×$F 실패"
    grep -a "에피소드 .* 프레임" "$LOG" | tail -1
  done
fi

# 5. 학습 (하나씩)
if run_step train; then
  for F in 1 $FACTORS; do
    gpu_ok "학습 ×$F" train
    say "5. 학습 ×$F ($STEPS 걸음)"
    rm -rf "$RUNS/anypos_${NAME}_x$F"
    "$T/learn_run.sh" "$O/train_x$F.log" ~/venvs/learn/bin/lerobot-train --dataset.repo_id="local/anypos_${NAME}_x$F" --dataset.root="$LR/anypos_${NAME}_x$F" \
      --policy.type=act --policy.device=cuda --policy.push_to_hub=false --policy.chunk_size=20 --policy.n_action_steps=20 \
      --output_dir="$RUNS/anypos_${NAME}_x$F" --steps="$STEPS" --batch_size=32 --save_freq="$STEPS" --log_freq=50 --wandb.enable=false --num_workers=2 --seed=1
    rc=$?
    [ $rc = 3 ] && { say "🔴 워치독 · 재생기가 돌고 있어 학습을 멈췄다 — 끝난 뒤 FROM=train 으로 다시"; exit 3; }
    [ $rc = 0 ] || fail "학습 ×$F 실패 (상태 $rc) — $O/train_x$F.log" "$O/train_x$F.log"
  done
fi

# 6. 평가 (GPU 하나 — 방법 다섯을 한 프로세스에서 차례로)
if run_step eval; then
  gpu_ok "평가" eval
  say "6. 평가 ($(basename "$EVAL_PAIRS") × $N 번)"
  M=(--method nearest=nearest --method shift=shift)
  for F in 1 $FACTORS; do M+=(--method "x$F=act:$RUNS/anypos_${NAME}_x$F/checkpoints/last/pretrained_model"); done
  nice -n 19 $PY -B "$T/eval_pairs.py" --pairs "$EVAL_PAIRS" --demos "$O/aug/files_x1.txt" "${M[@]}" --n "$N" --meas-sim --out "$O/eval" \
    ${LIMIT_PAIRS:+--limit-pairs $LIMIT_PAIRS} >> "$LOG" 2>&1 || fail "평가 실패"
  python3 "$T/eval_plot.py" "$O/eval/results.json" "$O/eval/map_1p5cm.png" ok15 >> "$LOG" 2>&1
  python3 "$T/eval_plot.py" "$O/eval/results.json" "$O/eval/map_5cm.png" ok5 >> "$LOG" 2>&1
fi

# 7. 보고
if run_step report; then
  $PY - "$O" "$NAME" "$STEPS" "$FACTORS" "$T0" <<'PYEOF'
import json, sys, time, re
from pathlib import Path
o, name, steps, factors, t0 = Path(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4].split(), int(sys.argv[5])
L = [f"# 19-c2 결과 — {name}", ""]
c = json.loads((o / "census.json").read_text()) if (o / "census.json").exists() else {}
a = json.loads((o / "aug/summary.json").read_text()) if (o / "aug/summary.json").exists() else {}
L += ["## 시연 · 증강", "", f"- 전수 검사: 판 {c.get('episodes')}, 통과 {c.get('pass')}, 통과한 좌표 쌍 {c.get('pairs_covered')} 가지, 빠진 까닭 {c.get('why')}",
      f"- 증강: 그대로 재생 통과 {a.get('replay_ok')} / {a.get('demos')}, 시도 {a.get('tries')} 중 통과 {a.get('accepted')} ({a.get('pass_rate')}), 목록 {a.get('lists')}",
      f"- 증강 까닭: {a.get('why')}", f"- 모자란 판 (증강 수가 모자람): {a.get('short')}", ""]
L += ["## 학습", ""]
for F in ["1"] + factors:
    lg = o / f"train_x{F}.log"
    last = ""
    if lg.exists():
        m = re.findall(r"step:(\S+).*?loss:(\S+)", lg.read_text(errors="ignore"))
        last = f"마지막 걸음 {m[-1][0]}, loss {m[-1][1]}" if m else "(loss 줄 없음)"
    L.append(f"- ×{F}: {steps} 걸음, {last}")
L += [""]
s = o / "eval/summary.md"
if s.exists():
    L += ["## 평가", "", *s.read_text().splitlines()[1:], "", f"그림: {o}/eval/map_1p5cm.png · map_5cm.png"]
L += ["", f"전체 걸린 시간 {round((time.time() - t0) / 60)} 분"]
(o / "REPORT.md").write_text("\n".join(L) + "\n", encoding="utf-8")
print("\n".join(L))
PYEOF
fi
say "끝 ($(( ($(date +%s) - T0) / 60 )) 분) — $O/REPORT.md"
