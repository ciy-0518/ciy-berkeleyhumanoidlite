#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""정책 서버 (작업 지시 15b): 체크포인트 (ACT·MLP) 를 읽고 127.0.0.1 UDP 로 관측을 받아 행동 묶음을 돌려준다. ~/venvs/learn 에서.
replay_run.py --policy 가 띄우고 끝낼 때 끈다 (사용자가 따로 띄우지 않는다). 시스템 파이썬 (워치독·재생기) 에는 torch 가 없어서 떼어 놓았다.

    ~/venvs/learn/bin/python policy_server.py --policy act:<…/pretrained_model> --port 0     # 0 = 빈 포트, 첫 줄에 "PORT <n>"
    ~/venvs/learn/bin/python policy_server.py --policy mlp:<…/model.pt> --write-first-pose    # 첫 자세만 써 두고 끝

메시지 (UTF-8 JSON, 한 데이터그램):
  {"op":"hello"}                                  → {"op":"hello","kind","chunk","n_action_steps","first_pose":[[5],[5]] (URDF rad),"ckpt","source_obs"}
      source_obs: 학습 데이터의 캔 출처가 한 값뿐이면 그 값 (지금 데이터는 모두 0 = 가상). 받는 쪽은 실제 출처 대신 이 값을 정책에 넣는다 —
      한 값뿐인 칸은 정규화 std 가 0 이라 다른 값 (실행 때 2 = 수동) 을 넣으면 값이 터진다 (15b 시험에서 확인). 출처가 섞인 데이터로 학습하면 None
  {"op":"act","id":n,"state":[12],"env":[8]}      → {"op":"act","id":n,"chunk":[[12]…k],"infer_ms"}
      17-a 조건 입력 정책 (learn_data.checkpoint_cond): hello 에 "cond" (xy / zero) 가 더 있고, state 는 16 (뒤 4 = 시작·목표 자리 판 좌표 m 또는 0).
  {"op":"bye"}                                    → 끝
  관측·행동 꼴은 tools/learn_data.py 와 같다 (관절 URDF rad + 그리퍼 0~1 / 캔 m·xyzw·출처, 행동 = 다음 관절 10 + 그리퍼 2).
  묶음은 정규화를 푼 값 (ACT 는 lerobot 전·후처리기, MLP 는 learn_mlp 통계). 묶음 섞기 (temporal ensembling) 는 받는 쪽 (policy_track) 이 한다.
첫 자세 (approach 목표): 체크포인트 폴더의 first_pose.json. 없으면 learn_data base 묶음 첫 프레임 평균으로 만들어 그 폴더에 쓴다 (policy_sim.first_pose 와 같은 값).
시험용: --delay-ms (답을 늦춤), --die-after-s (그 뒤 답하지 않고 끝냄), --test-offset (관절 하나에 각을 더함 — 안전장치가 깎는 상황). 부모 프로세스가 없어지거나 10 분 동안 요청이 없으면 스스로 끝난다."""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
IDLE_EXIT_S = 600.0


def ckpt_dir(spec: str) -> Path:
    kind, _, path = spec.partition(":")
    p = Path(path).expanduser()
    return p if kind == "act" else p.parent


def first_pose_for(spec: str, first_from=None) -> list:
    if first_from:                                      # 19-x21 (켤 때만): 다듬은 기록 (trimB*) 처럼 학습 데이터 폴더에 없는 기록으로 학습한 정책 — 목록의 첫 행동 평균, 파일은 쓰지 않는다
        import learn_data as L
        fl = [x.strip() for x in Path(first_from).expanduser().read_text(encoding="utf-8").splitlines() if x.strip()]
        return np.mean([np.asarray(L.arrays(x)["action"], float)[0, :10] for x in fl], 0).reshape(2, 5).tolist()
    f = ckpt_dir(spec) / "first_pose.json"
    if f.exists():
        return json.loads(f.read_text(encoding="utf-8"))["first_pose_urdf"]
    import learn_data as L
    recs = L.checkpoint_cond(spec)["records"]           # 17-a: 학습한 기록이 적혀 있으면 그 기록만 (없으면 None = 지금과 같다)
    eps_m = L.episode_files("base", records=recs)
    eps = [L.arrays(e["path"]) for e in eps_m]
    fp = np.mean([a["action"][0, :10] for a in eps], 0).reshape(2, 5).tolist()
    f.write_text(json.dumps({"first_pose_urdf": fp, "from": [e["meta"]["record"] + f"_ep{e['meta']['episode']:02d}" for e in eps_m],
                             "note": "learn_data base 묶음 첫 프레임 평균 (policy_sim.first_pose 와 같음), URDF rad, 왼·오른 × JOINTS"},
                            ensure_ascii=False, indent=1), encoding="utf-8")
    return fp


def source_obs(spec: str):
    """학습 데이터의 캔 출처 (environment_state 마지막 칸) 가 한 값뿐이면 그 값 (정규화 std 0 — 다른 값을 넣으면 정규화가 터진다), 아니면 None."""
    kind, _, path = spec.partition(":")
    try:
        if kind == "act":
            import glob
            from safetensors.torch import load_file
            f = glob.glob(str(Path(path).expanduser() / "policy_preprocessor_step_*normalizer*.safetensors"))[0]
            d = load_file(f)
            lo, hi = float(d["observation.environment_state.min"][-1]), float(d["observation.environment_state.max"][-1])
        else:
            import torch
            st = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)["stats"]["env"]
            lo = hi = float(st["mean"][-1]) if abs(float(st["std"][-1]) - 1.0) < 1e-9 else None
            if lo is None:
                return None
        return lo if abs(hi - lo) < 1e-9 else None
    except Exception:
        return None


class ActChunk:
    def __init__(self, path, device):
        import torch
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.factory import make_pre_post_processors
        p = Path(path).expanduser()
        self.pol = ACTPolicy.from_pretrained(str(p)).to(device).eval()
        self.pre, self.post = make_pre_post_processors(self.pol.config, pretrained_path=str(p))
        self.k, self.n = self.pol.config.chunk_size, self.pol.config.n_action_steps
        self.torch, self.device = torch, device

    def chunk(self, state, env):
        t = self.torch
        b = {"observation.state": t.tensor(np.asarray(state, np.float32))[None], "observation.environment_state": t.tensor(np.asarray(env, np.float32))[None]}
        with t.no_grad():
            c = self.pol.predict_action_chunk(self.pre(b))[0]          # (k, 12) 정규화 값
            c = self.post(c)                                            # 행마다 정규화 풀기 (select_action 뒤 post 와 같은 처리)
        return np.asarray(c.cpu().numpy(), float)


class MlpChunk:
    def __init__(self, path, device):
        import learn_mlp as LM
        self.p = LM.MlpPolicy(Path(path).expanduser(), device=device)
        self.k = self.n = self.p.cfg["chunk"]

    def chunk(self, state, env):
        return self.p.predict_chunk(np.asarray(state, np.float32), np.asarray(env, np.float32))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", required=True, help="act:<pretrained_model 폴더> / mlp:<model.pt>")
    ap.add_argument("--port", type=int, default=0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--write-first-pose", action="store_true")
    ap.add_argument("--first-from", default=None, metavar="LIST",
                    help="19-x21 (기본 끔): 시작 자세 = 이 npz 목록의 첫 행동 평균 (eval_pairs --first-from 과 같은 식). 체크포인트 폴더에 first_pose.json 을 쓰지 않는다")
    ap.add_argument("--delay-ms", type=float, default=0.0, help="시험: 답을 이만큼 늦춘다")
    ap.add_argument("--die-after-s", type=float, default=None, help="시험: 첫 act 요청 뒤 이 초가 지나면 답하지 않고 끝낸다")
    ap.add_argument("--test-offset", default=None, metavar="칸:도:초",
                    help="시험: 첫 act 요청 뒤 '초' 가 지나면 행동 '칸' (0~9 관절, URDF) 에 '도' 를 더한다 (안전장치가 깎는 상황 — 예 0:30:3 = 왼 어깨 앞뒤 +30°, 판 면)")
    a = ap.parse_args()
    fp = first_pose_for(a.policy, a.first_from)
    if a.write_first_pose:
        print("first_pose.json:", ckpt_dir(a.policy) / "first_pose.json")
        return 0
    kind, _, path = a.policy.partition(":")
    import torch
    dev = a.device if (a.device != "cuda" or torch.cuda.is_available()) else "cpu"
    pol = ActChunk(path, dev) if kind == "act" else MlpChunk(path, dev)
    import learn_data as L
    cond = L.checkpoint_cond(a.policy)["cond"]                            # 17-a: 조건 입력 정책이면 state 16
    pol.chunk(np.zeros(16 if cond else 12), np.zeros(8))                  # 첫 추론 (GPU 준비) 을 미리
    src_obs = source_obs(a.policy)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", a.port))
    sock.settimeout(1.0)
    print(f"PORT {sock.getsockname()[1]}", flush=True)
    ppid = os.getppid()
    last_req, first_act = time.monotonic(), None
    while True:
        if os.getppid() != ppid or time.monotonic() - last_req > IDLE_EXIT_S:
            break
        try:
            data, addr = sock.recvfrom(65536)
        except socket.timeout:
            continue
        last_req = time.monotonic()
        try:
            msg = json.loads(data.decode("utf-8"))
        except ValueError:
            continue
        if msg.get("op") == "act":                     # 밀린 요청이 있으면 가장 새 관측만 (늦게 답하면 앞 것은 이미 쓸모없다 — 쌓이지 않게)
            sock.setblocking(False)
            try:
                while True:
                    d2, a2 = sock.recvfrom(65536)
                    try:
                        m2 = json.loads(d2.decode("utf-8"))
                    except ValueError:
                        continue
                    if m2.get("op") == "act":
                        msg, addr = m2, a2
                    elif m2.get("op") == "bye":
                        msg = m2
                        break
            except (BlockingIOError, OSError):
                pass
            sock.settimeout(1.0)
        op = msg.get("op")
        if op == "bye":
            break
        if op == "hello":
            rep = {"op": "hello", "kind": kind, "chunk": pol.k, "n_action_steps": pol.n, "first_pose": fp, "ckpt": path, "device": dev,
                   "source_obs": src_obs}
            if cond:                                   # 17-a (조건 입력 정책만): 받는 쪽이 state 뒤에 자리 조건 4 값을 붙인다
                rep["cond"] = cond
        elif op == "act":
            first_act = first_act or time.monotonic()
            if a.die_after_s is not None and time.monotonic() - first_act >= a.die_after_s:
                print("시험: --die-after-s — 끝냄", flush=True)
                break
            t0 = time.perf_counter()
            c = pol.chunk(msg["state"], msg["env"])
            if a.test_offset:
                ti, td, ts = a.test_offset.split(":")
                if time.monotonic() - first_act >= float(ts):
                    c = c.copy()
                    c[:, int(ti)] += np.radians(float(td))
            if a.delay_ms > 0:
                time.sleep(a.delay_ms / 1000.0)
            rep = {"op": "act", "id": msg.get("id"), "chunk": np.round(c, 6).tolist(), "infer_ms": round((time.perf_counter() - t0) * 1000, 2)}
        else:
            continue
        sock.sendto(json.dumps(rep).encode("utf-8"), addr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
