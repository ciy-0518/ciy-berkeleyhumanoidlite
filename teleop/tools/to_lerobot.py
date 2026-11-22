#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""demo_export npz → LeRobot 데이터셋 (작업 지시 15a A3). 이미지 없이 상태만. ~/venvs/learn 에서 돌린다.

    ~/venvs/learn/bin/python to_lerobot.py --set base                       # 기본 (release·정렬 성공, 지금 4)
    ~/venvs/learn/bin/python to_lerobot.py --set wide --name vcan_wide       # + 11-03 옛 시연 (파이프라인 시험만)
    ~/venvs/learn/bin/python to_lerobot.py --set base --meas-sim --copies 3  # 관측 관절에 처짐 예측 + 잡음 (실로봇 meas 흉내), 3 벌

특징 (learn_data 참고): observation.state (12), observation.environment_state (8), action (12), fps 20, task 한 줄.
출력: ~/cc_analysis/learn/lerobot/<name> (repo_id local/<name>, 허브에 올리지 않는다). 같은 이름이 있으면 지운다.
작업 지시 17-a (기본 끔 — 안 주면 지금과 같다):
  --records 기록… : 그 기록만 (옛 L1 시연을 빼고 학습 입력을 명시). 제외 목록 (logs/exclude_episodes.json) 은 learn_data 가 늘 거른다.
  --cond xy|zero  : observation.state 뒤에 조건 4 값 (시작·목표 자리 판 좌표 m, learn_data.COND_NAMES) → state 16. zero 는 0 (비교용).
      한 값뿐인 칸 (지금 자리들은 x 가 모두 29 cm) 은 정규화 std 가 0 이다. 15b 처럼 다른 값이 들어오면 터지지 않게
      meta/stats.json 의 그 칸 std 를 COND_STD_FLOOR (1 cm) 로 올린다 (평균은 그대로라 학습 값은 정규화 뒤 0 그대로).
  build_info.json: 기록·조건·빠진 판 (exclude) · 에피소드 수.
작업 지시 19-c1 (기본 끔): --files 목록.txt : learn_data 묶음 대신 이 npz 들만 (한 줄에 경로 하나 — 증강 도구 aug_demo.py 의 files_x*.txt).
  build_info.records 는 그 판들의 원래 기록 이름 (증강 판은 meta.aug.src_record) — 정책 첫 자세 (policy_server · policy_sim) 가 원래 시연에서 나오게."""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import learn_data as L  # noqa: E402

OUT = Path.home() / "cc_analysis/learn/lerobot"
TASK = "pick the can at L1 with the left gripper, hand over to the right, place at R1 and release"


def task_of(meta: dict) -> str:
    """작업 지시 16-a: --can-combos 기록 (meta can_start_slot) 이면 그 자리 이름을 넣은 글, 아니면 지금 TASK 그대로."""
    if not meta.get("can_start_slot"):
        return TASK
    return f"pick the can at {meta['can_start_slot']} with the left gripper, hand over to the right, place at {meta['can_goal_slot']} and release"


COND_STD_FLOOR = 0.01      # m (17-a)


def features(cond=None):
    if cond:
        return {"observation.state": {"dtype": "float32", "shape": (16,), "names": L.STATE_NAMES + L.COND_NAMES},
                "observation.environment_state": {"dtype": "float32", "shape": (8,), "names": L.ENV_NAMES},
                "action": {"dtype": "float32", "shape": (12,), "names": L.ACTION_NAMES}}
    return {"observation.state": {"dtype": "float32", "shape": (12,), "names": L.STATE_NAMES},
            "observation.environment_state": {"dtype": "float32", "shape": (8,), "names": L.ENV_NAMES},
            "action": {"dtype": "float32", "shape": (12,), "names": L.ACTION_NAMES}}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", choices=("base", "wide"), default="base")
    ap.add_argument("--name", default=None)
    ap.add_argument("--meas-sim", action="store_true", help="관측 관절 = 처짐 모델 예측 실제 + 잡음 (행동은 명령 그대로)")
    ap.add_argument("--copies", type=int, default=1, help="--meas-sim 잡음을 바꿔 몇 벌 (기본 1)")
    ap.add_argument("--records", nargs="+", default=None, help="17-a: 이 기록만 (예 dry_vcan_2132 dry_vcan_2152)")
    ap.add_argument("--cond", choices=L.COND_MODES, default=None, help="17-a: state 에 조건 4 값 (xy = 자리 판 좌표, zero = 0)")
    ap.add_argument("--files", default=None, help="19-c1: 이 목록의 npz 만 (한 줄에 경로 하나). 주면 --set · --records 대신")
    a = ap.parse_args()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    name = a.name or f"vcan_{a.set}{'_meas' if a.meas_sim else ''}"
    root = OUT / name
    if root.exists():
        shutil.rmtree(root)
    ds = LeRobotDataset.create(repo_id=f"local/{name}", fps=L.FPS, features=features(a.cond), root=root, robot_type="bhl_upper_biarm",
                               use_videos=False)
    if a.files:                                                # 19-c1 (켤 때만)
        import demo_export as DE
        paths = [x.strip() for x in Path(a.files).expanduser().read_text(encoding="utf-8").splitlines() if x.strip() and not x.startswith("#")]
        eps = [{"path": x, "meta": DE.load_episode(x)["meta"], "set": "base"} for x in paths]
        a.records = sorted({(e["meta"].get("aug") or {}).get("src_record") or e["meta"]["record"] for e in eps})
    else:
        eps = L.episode_files(a.set, records=a.records)
    index = []
    sims = {}
    for copy in range(a.copies if a.meas_sim else 1):
        for e in eps:
            ms = None
            if a.meas_sim:
                key = (e["meta"]["robot_ref"], copy)
                ms = sims.setdefault(key, L.MeasSim(e["meta"]["robot_ref"], seed=copy))
            arr = L.arrays(e["path"], ms, cond=a.cond)
            for n in range(len(arr["state"])):
                ds.add_frame({"observation.state": arr["state"][n], "observation.environment_state": arr["env"][n],
                              "action": arr["action"][n], "task": task_of(arr["meta"])})
            ds.save_episode()
            index.append({"episode_index": len(index), "file": Path(e["path"]).name, "set": e["set"], "copy": copy,
                          "frames": int(len(arr["state"])), "meas_sim": bool(a.meas_sim)})
    ds.finalize()
    (root / "source_index.json").write_text(json.dumps(index, ensure_ascii=False, indent=1), encoding="utf-8")
    if a.cond or a.records:                                    # 17-a (켤 때만)
        floored = []
        if a.cond:
            sp = root / "meta" / "stats.json"
            st = json.loads(sp.read_text(encoding="utf-8"))
            sd = st["observation.state"]["std"]
            for k in range(12, 16):
                if float(sd[k]) < COND_STD_FLOOR:
                    floored.append([L.COND_NAMES[k - 12], float(sd[k])])
                    sd[k] = COND_STD_FLOOR
            sp.write_text(json.dumps(st, indent=4), encoding="utf-8")
        info = {"records": a.records, **({"files": a.files, "aug_episodes": sum(1 for e in eps if e["meta"].get("aug"))} if a.files else {}), "cond": a.cond, "cond_names": L.COND_NAMES if a.cond else None, "std_floor": COND_STD_FLOOR if a.cond else None,
                "std_floored": floored, "excluded": L.excluded_list(a.records), "episodes": len(index), "meas_sim": bool(a.meas_sim), "copies": a.copies,
                "combos": sorted({f"{e['meta'].get('can_start_slot')}:{e['meta'].get('can_goal_slot')}" for e in eps})}
        (root / "build_info.json").write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding="utf-8")
        print("build_info:", json.dumps({k: v for k, v in info.items() if k != "excluded"}, ensure_ascii=False))
    print(f"{name}: 에피소드 {len(index)}, 프레임 {sum(i['frames'] for i in index)} → {root}")
    # 되읽기 확인
    back = LeRobotDataset(f"local/{name}", root=root)
    it = back[0]
    print("되읽기:", back.num_episodes, "에피소드", back.num_frames, "프레임,", {k: tuple(v.shape) for k, v in it.items() if hasattr(v, "shape")})
    return 0


if __name__ == "__main__":
    sys.exit(main())
