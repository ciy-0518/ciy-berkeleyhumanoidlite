#!/usr/bin/env bash
# 학습·평가를 낮은 우선순위로 돌리고, 로봇 프로그램 (워치독·재생기) 이 보이면 멈추고 기록한다 (작업 지시 15a §0).
#   tools/learn_run.sh <로그 파일> <명령…>
# 시작 전에 보이면 시작하지 않는다 (끝 상태 3). 도중에 보이면 학습 프로세스 묶음을 멈추고 (TERM → 5 초 뒤 KILL) 끝 상태 3.
# 패턴의 [ ] 는 이 스크립트·감시 명령 자신이 잡히지 않게 하려는 것.
set -u
LOG="$1"; shift
PAT='vr_control_watchdog_bi[a]rm|replay_ru[n]'
robot_busy() { pgrep -af "$PAT" | grep -v "learn_run.sh" ; }
note() { echo "[learn_run $(date +%H:%M:%S)] $*" | tee -a "$LOG.guard"; }
if b=$(robot_busy); then note "🔴 시작 전에 로봇 프로그램이 돈다 — 시작하지 않음: $b"; exit 3; fi
note "시작 (nice 19): $*"
setsid nice -n 19 "$@" > "$LOG" 2>&1 &
PID=$!
while kill -0 "$PID" 2>/dev/null; do
  if b=$(robot_busy); then
    note "🔴 도중에 로봇 프로그램이 보임 — 학습 멈춤: $b"
    kill -TERM -- -"$PID" 2>/dev/null; sleep 5; kill -KILL -- -"$PID" 2>/dev/null
    exit 3
  fi
  sleep 5
done
wait "$PID"; rc=$?
note "끝 (상태 $rc)"
exit $rc
