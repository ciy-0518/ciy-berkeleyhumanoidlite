#!/usr/bin/env bash
# 실행 이름 묶기 (작업 지시 14-e). 긴 명령을 짧은 이름으로. 시작할 때 실제 명령을 한 줄로 찍는다.
#   ~/run.sh            목록
#   ~/run.sh P          로봇 기준값 (전원 켤 때마다)
#   ~/run.sh teleop     VR 실기동 (판 주차·몸 방향·VR 끊김 정지·자동 종료·걸림 감시·그리퍼·소리·VR 표시·로그)
#   ~/run.sh dry [시작 목표]   dry-run 가상 캔 시연 (기본 L1 → R1, 에피소드 기록, 놓고 손을 빼야 성공 — 14-h, 잡기 정렬 조건 — 14-l)
#   ~/run.sh dry --pairs <파일>   좌표 쌍 순환 시연 (19-b: 한 줄에 시작 x, y, 도착 x, y. 에피소드마다 다음 쌍, 엄격 판정 · 잡은 뒤 실제 크기 그림 켬)
#   ~/run.sh replay <에피소드> [옵션]   녹화 재생 (tools/replay_run.py, VR 없이)
#   ~/run.sh policy <체크포인트> [옵션]  학습한 정책 실행 (15b, replay_run.py --policy, 캔 L1 → R1. --can-start·--can-goal·--policy-ensemble·--dry-run)
#   ~/run.sh list       재생할 수 있는 에피소드 표
#   ~/run.sh export     새 dry-run 기록만 학습 데이터 (npz) 로
#   ~/run.sh can        CAN 두 개 (can0 왼팔, can1 오른팔) 1 Mbps 로 올리고 모터 ping (14-n). CAN_DRY=1 이면 명령만 찍는다
# 그리퍼 닫힘 각도는 config/gripper.json 한곳 (14-n, 지금 60°): teleop·dry 는 --gripper-close-deg 로 넘기고, replay 는 replay_run 이 읽는다.
# 이름 뒤에 붙인 것은 그대로 끝에 붙는다 (예: ~/run.sh teleop --dry-run). --dry-run 이 있으면 그리퍼 직렬 대신 가상 그리퍼.
# teleop·dry 는 그리퍼 열림 판정 --grip-open-rule finger (14-p: 컨트롤러 4.5~7 은 1.5초, 반대 손이 연 뒤 3초는 ≥ 7 만). 끄려면 끝에 --grip-open-rule pinch.
set -euo pipefail

TELEOP="$HOME/teleop-docs/_teleop"
TOOLS="$TELEOP/tools"
LOGS="$TELEOP/logs"
LOWLEVEL="$HOME/code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel"
TABLE="$TELEOP/config/table.json"
SERVER="100.85.65.19"
DATA="$HOME/cc_analysis/learn/data"
GRIP_JSON="$TELEOP/config/gripper.json"

# 원격 조종 공통 (13-d·14-c 실기동과 같은 값)
HAND_COMMON=(--retarget hand --auto-start --arm-gap-cm -2 --hand-gap-goal-cm -1.5 --yaw-in-limit-deg 100 100 --hand-pair on
             --mirror-limits on --hand-pair-center robot-hand --table "$TABLE" --table-body real --start-pose table
             --heading-deadband-deg 20 --heading-tau-s 2 --cue-sound "$SERVER:5021" --robot-viz "$SERVER:5022")

usage() {
  cat <<EOF
사용: ~/run.sh <이름> [더 붙일 것]
  P                  로봇 기준값 잡기 (로봇 전원을 켤 때마다 먼저)
  teleop             VR 실기동. 끝내기: 로봇 손을 처음 자리로 내리고 3 초 / 헤드셋 벗기·VR 끄기 = 멈춤
  dry [시작] [목표]  dry-run 가상 캔 시연 (기본 L1 R1). 예: ~/run.sh dry L2 R1
  dry --pairs <파일> 좌표 쌍 순환 시연 (19-b). 예: ~/run.sh dry --pairs ~/teleop-docs/_teleop/config/pairs_demo_19.txt
  replay <에피소드>  녹화 재생 (VR 없이, 아무 키나 = 멈춤). 예: ~/run.sh replay 2159_ep02
  policy <체크포인트> 학습한 정책 실행 (15b). 예: ~/run.sh policy ~/cc_analysis/learn/runs/act_base/checkpoints/last/pretrained_model --dry-run
                     옵션: --speed 0.5 --repeat 1 --wait-s 15 --dry-run --vr-guard --sag-comp (처짐 보정, 14-l)
  list               재생할 수 있는 에피소드 표
  export             새 dry-run 기록만 학습 데이터로 ($DATA/vcan_<오늘>)
  can                CAN 올리기 (can0 왼팔·can1 오른팔, 1 Mbps) + 모터 ping. sudo 암호를 한 번 묻는다
EOF
}

show_and_run() {          # 실제 명령을 한 줄로 찍고 실행 (종료 코드를 그대로 돌려준다)
  printf '[run.sh] '; printf '%q ' "$@"; printf '\n'
  exec "$@"
}

grip_deg_args() {         # 14-n: config/gripper.json → --gripper-open-deg A B --gripper-close-deg A B (A = L, B = R)
  python3 - "$GRIP_JSON" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding="utf-8"))
except (OSError, ValueError):
    d = {}
o, c = d.get("open_deg", {}), d.get("close_deg", {})
print(f"--gripper-open-deg {int(o.get('L', 0))} {int(o.get('R', 0))} --gripper-close-deg {int(c.get('L', 60))} {int(c.get('R', 60))}")
PY
}

can_step() {              # 14-n: 한 단계. 실패하면 무엇이 안 됐는지 한 줄 찍고 멈춘다. CAN_DRY=1 이면 찍기만
  local what="$1"; shift
  if [ -n "${CAN_DRY:-}" ]; then printf '[run.sh can] (찍기만) %s: ' "$what"; printf '%q ' "$@"; printf '\n'; return 0; fi
  if ! "$@"; then echo "[run.sh can] 🔴 $what 실패 — 여기서 멈춤 (명령: $*)"; exit 1; fi
}

grip_args() {             # --dry-run 이면 가상 그리퍼 (직렬을 열지 않는다)
  for a in "$@"; do [ "$a" = "--dry-run" ] && { echo "--grip-virtual"; return; }; done
  echo "--gripper-port /dev/ttyUSB0"
}

name="${1:-}"; [ $# -gt 0 ] && shift
case "$name" in
  P|p)
    show_and_run python3 "$TOOLS/arm_pose_logger.py" --part P "$@" ;;
  teleop)
    t=$(date +%H%M)
    cd "$LOWLEVEL"
    # shellcheck disable=SC2046
    show_and_run python3 vr_control_watchdog_biarm.py "${HAND_COMMON[@]}" $(grip_args "$@") $(grip_deg_args) --vr-loss-park --auto-end --park-table-relax --grip-open-rule finger --stall-stop \
      --log-csv "$LOGS/hand_table_$t.csv" --log-rx "$LOGS/rx_live_table_$t.jsonl" "$@" ;;
  dry)
    if [ "${1:-}" = "--pairs" ]; then          # 19-b: 좌표 쌍 파일 (한 줄에 시작 x, y, 도착 x, y). 엄격 판정 · 잡은 뒤 실제 크기 그림을 같이 켠다
      [ $# -ge 2 ] || { echo "사용: ~/run.sh dry --pairs <좌표 쌍 파일> [더 붙일 것]"; exit 2; }
      pairs="$2"; shift 2
      t=$(date +%H%M)
      cd "$LOWLEVEL"
      # shellcheck disable=SC2046
      show_and_run python3 vr_control_watchdog_biarm.py "${HAND_COMMON[@]}" --dry-run --grip-virtual --grip-virtual-timing real $(grip_deg_args) \
        --can-pairs "$pairs" --episodes --episode-end release --grasp-align --grasp-strict --viz-pair-after-grasp --grip-open-rule finger \
        --log-csv "$LOGS/dry_vcan_$t.csv" --log-rx "$LOGS/rx_dry_vcan_$t.jsonl" "$@"
    fi
    start="L1"; goal="R1"
    if [ $# -gt 0 ] && [[ "$1" != -* ]]; then start="$1"; shift; fi
    if [ $# -gt 0 ] && [[ "$1" != -* ]]; then goal="$1"; shift; fi
    t=$(date +%H%M)
    cd "$LOWLEVEL"
    # shellcheck disable=SC2046
    show_and_run python3 vr_control_watchdog_biarm.py "${HAND_COMMON[@]}" --dry-run --grip-virtual --grip-virtual-timing real $(grip_deg_args) \
      --virtual-can "$start" --virtual-can-goal "$goal" --episodes --episode-end release --grasp-align --grip-open-rule finger \
      --log-csv "$LOGS/dry_vcan_$t.csv" --log-rx "$LOGS/rx_dry_vcan_$t.jsonl" "$@" ;;
  replay)
    [ $# -ge 1 ] || { echo "에피소드 이름을 준다. 목록: ~/run.sh list"; exit 2; }
    show_and_run python3 "$TOOLS/replay_run.py" "$@" ;;
  policy)
    # 15b: 학습한 정책을 자동 순서 (재생과 같은 멈춤·그리퍼 동기·주차) 로. 정책 서버 (~/venvs/learn) 는 재생기가 띄우고 끈다
    [ $# -ge 1 ] || { echo "체크포인트를 준다. 예: ~/run.sh policy ~/cc_analysis/learn/runs/act_base/checkpoints/last/pretrained_model"; exit 2; }
    ck="$1"; shift
    show_and_run python3 "$TOOLS/replay_run.py" --policy "$ck" "$@" ;;
  list)
    show_and_run python3 "$TOOLS/replay_run.py" --list ;;
  export)
    mapfile -t new < <(python3 - "$LOGS" "$DATA" <<'PY'
import glob, os, sys
logs, data = sys.argv[1], sys.argv[2]
done = {os.path.basename(f).rsplit("_ep", 1)[0] for f in glob.glob(os.path.join(data, "**", "*.npz"), recursive=True)}
for f in sorted(glob.glob(os.path.join(logs, "dry_vcan_*_episodes.csv"))):
    s = os.path.basename(f)[:-len("_episodes.csv")]
    with open(f, encoding="utf-8") as fh:
        has_rows = sum(1 for line in fh if line.strip()) > 1      # 머리줄만 있으면 (에피소드 0 개) 건너뛴다
    if s not in done and has_rows:
        print(s)
PY
)
    if [ ${#new[@]} -eq 0 ]; then echo "[run.sh] 새 기록 없음 (이미 다 변환됨: $DATA)"; exit 0; fi
    show_and_run python3 "$TOOLS/demo_export.py" --out "$DATA/vcan_$(date +%Y%m%d)" "${new[@]}" "$@" ;;
  can)
    # 14-n: 사용자 ~/can_up.sh 와 같은 차례 (그 파일은 건드리지 않는다). sudo 는 처음에 한 번 묻는다
    echo "[run.sh can] can0 (왼팔)·can1 (오른팔) 를 1 Mbps 로 올리고 모터 ping"
    can_step "sudo 암호 확인" sudo -v
    for i in 0 1; do
      c="can$i"; arm=$([ "$i" = 0 ] && echo "왼팔" || echo "오른팔")
      echo "===== $c ($arm)"
      if [ -z "${CAN_DRY:-}" ]; then sudo ip link set "$c" down 2>/dev/null || true      # 이미 내려가 있으면 그냥 간다
      else can_step "$c 내리기" sudo ip link set "$c" down; fi
      can_step "$c 속도 1000000 설정 (장치가 없거나 이름이 다르면 여기서 실패)" sudo ip link set "$c" type can bitrate 1000000
      can_step "$c 올리기" sudo ip link set "$c" up
      can_step "$c 상태 보기" ip -details -statistics link show "$c"
    done
    echo "===== 모터 연결 확인 (ping_biarm)"
    can_step "Lowlevel 폴더로 이동" cd "$LOWLEVEL"
    can_step "모터 ping (ping_biarm.py)" uv run python ./scripts/motor/ping_biarm.py "$@"
    echo "[run.sh can] 끝 — 위 ping 결과에서 왼팔·오른팔 모터가 모두 답했는지 본다" ;;
  ""|-h|--help|help)
    usage ;;
  *)
    echo "모르는 이름: $name"; usage; exit 2 ;;
esac
