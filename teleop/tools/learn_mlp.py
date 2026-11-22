#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""기준선 MLP (작업 지시 15a B2): 관측 (state 12 + env 8) → 행동 묶음 (chunk × 12). ~/venvs/learn 에서.

    ~/venvs/learn/bin/python learn_mlp.py --set base --seconds 120 --out ~/cc_analysis/learn/runs/mlp_base
관절 10 은 정규화 값의 MSE, 그리퍼 2 는 로짓 BCE. 묶음 끝을 넘는 칸은 손실에서 뺀다 (ACT 의 action_is_pad 와 같은 뜻).
저장: <out>/model.pt (가중치·정규화 통계·설정), <out>/loss.csv (걸음, 손실, 관절 손실, 그리퍼 손실, 경과 초).
작업 지시 17-a (기본 끔): --records 기록… (그 기록만), --cond xy|zero (state 16 = 조건 4 값, learn_data.arrays). 설정에 cond · d_in · 빠진 판을 남긴다."""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import learn_data as L  # noqa: E402


def build(chunk: int, hidden: int = 512, depth: int = 3, d_in: int = 20):
    import torch.nn as nn
    layers, d = [], d_in
    for _ in range(depth):
        layers += [nn.Linear(d, hidden), nn.GELU()]
        d = hidden
    layers.append(nn.Linear(d, chunk * 12))
    return nn.Sequential(*layers)


class MlpPolicy:
    """policy_sim 이 쓰는 꼴: reset(), act(state (12,), env (8,)) → 행동 (12,). 묶음을 n_action_steps 칸씩 쓰거나 (기본, ACT 와 같음)
    temporal ensembling (ensemble=True, 겹친 예측을 지수 가중 평균)."""

    def __init__(self, path, device="cuda", n_action_steps=None, ensemble=False, ens_k=0.01):
        import torch
        ck = torch.load(path, map_location=device, weights_only=False)
        self.cfg, self.stats = ck["cfg"], ck["stats"]
        self.net = build(self.cfg["chunk"], self.cfg["hidden"], self.cfg["depth"], self.cfg.get("d_in", 20)).to(device)
        self.net.load_state_dict(ck["state_dict"]); self.net.eval()
        self.device, self.torch = device, torch
        self.n_steps = n_action_steps or self.cfg["chunk"]
        self.ensemble, self.ens_k = ensemble, ens_k
        self.reset()

    def reset(self):
        self.queue, self.hist, self.t = [], [], 0

    def predict_chunk(self, state, env):
        s, e = self.stats["state"], self.stats["env"]
        x = np.concatenate([(state - s["mean"]) / s["std"], (env - e["mean"]) / e["std"]]).astype(np.float32)
        with self.torch.no_grad():
            y = self.net(self.torch.from_numpy(x)[None].to(self.device)).cpu().numpy()[0].reshape(self.cfg["chunk"], 12)
        a = self.stats["action"]
        out = y.copy()
        out[:, :10] = y[:, :10] * np.asarray(a["std"][:10]) + np.asarray(a["mean"][:10])
        out[:, 10:] = 1.0 / (1.0 + np.exp(-y[:, 10:]))
        return out

    def act(self, state, env):
        if self.ensemble:
            self.hist.append((self.t, self.predict_chunk(state, env)))
            self.hist = [(t0, c) for t0, c in self.hist if self.t - t0 < len(c)]
            preds = np.array([c[self.t - t0] for t0, c in self.hist])
            w = np.exp(-self.ens_k * np.arange(len(preds)))          # 오래된 예측일수록 크게 (ACT 논문 꼴: w_i = exp(-k i), i=0 이 가장 오래됨)
            self.t += 1
            return (w[:, None] * preds).sum(0) / w.sum()
        if not self.queue:
            self.queue = list(self.predict_chunk(state, env)[: self.n_steps])
        self.t += 1
        return self.queue.pop(0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", choices=("base", "wide"), default="base")
    ap.add_argument("--meas-sim", action="store_true")
    ap.add_argument("--chunk", type=int, default=20)
    ap.add_argument("--hidden", type=int, default=512)
    ap.add_argument("--depth", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--batch", type=int, default=256)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--out", required=True)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--records", nargs="+", default=None, help="17-a: 이 기록만")
    ap.add_argument("--cond", choices=L.COND_MODES, default=None, help="17-a: state 에 조건 4 값")
    ap.add_argument("--copies", type=int, default=1, help="17-a: --meas-sim 잡음을 바꿔 몇 벌 (to_lerobot --copies 와 같은 뜻, 기본 1)")
    a = ap.parse_args()
    import torch
    torch.manual_seed(a.seed); np.random.seed(a.seed)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    eps = L.episode_files(a.set, records=a.records)
    sims = {}
    arrs = []
    for copy in range(a.copies if a.meas_sim else 1):
        for e in eps:
            ms = sims.setdefault((e["meta"]["robot_ref"], copy), L.MeasSim(e["meta"]["robot_ref"], seed=copy)) if a.meas_sim else None
            arrs.append(L.arrays(e["path"], ms, cond=a.cond))
    st = L.stats(arrs)
    X, Y, P = [], [], []
    for r in arrs:
        x = np.concatenate([(r["state"] - st["state"]["mean"]) / st["state"]["std"], (r["env"] - st["env"]["mean"]) / st["env"]["std"]], 1)
        ch, pad = L.chunk_targets(r["action"], a.chunk)
        y = ch.copy()
        y[..., :10] = (ch[..., :10] - np.asarray(st["action"]["mean"][:10])) / np.asarray(st["action"]["std"][:10])
        X.append(x); Y.append(y); P.append(pad)
    X = torch.tensor(np.concatenate(X), dtype=torch.float32, device=dev)
    Y = torch.tensor(np.concatenate(Y), dtype=torch.float32, device=dev)
    M = torch.tensor(~np.concatenate(P), dtype=torch.float32, device=dev)[..., None]
    d_in = int(arrs[0]["state"].shape[1] + arrs[0]["env"].shape[1])
    net = build(a.chunk, a.hidden, a.depth, d_in).to(dev)
    opt = torch.optim.AdamW(net.parameters(), lr=a.lr, weight_decay=1e-4)
    out = Path(a.out).expanduser(); out.mkdir(parents=True, exist_ok=True)
    if dev == "cuda":
        torch.cuda.reset_peak_memory_stats()
    t0, step, rows = time.time(), 0, []
    bce = torch.nn.BCEWithLogitsLoss(reduction="none")
    while time.time() - t0 < a.seconds:
        idx = torch.randint(0, len(X), (a.batch,), device=dev)
        pred = net(X[idx]).view(-1, a.chunk, 12)
        m = M[idx]
        lj = (((pred[..., :10] - Y[idx][..., :10]) ** 2) * m).sum() / (m.sum() * 10)
        lg = (bce(pred[..., 10:], Y[idx][..., 10:]) * m).sum() / (m.sum() * 2)
        loss = lj + lg
        opt.zero_grad(); loss.backward(); opt.step()
        step += 1
        if step % 50 == 0:
            rows.append([step, float(loss), float(lj), float(lg), round(time.time() - t0, 2)])
    mem = torch.cuda.max_memory_allocated() / 2 ** 20 if dev == "cuda" else 0.0
    cfg = {"chunk": a.chunk, "hidden": a.hidden, "depth": a.depth, "set": a.set, "meas_sim": a.meas_sim,
           "episodes": [Path(e["path"]).stem for e in eps], "steps": step, "seconds": round(time.time() - t0, 1), "gpu_mem_mib": round(mem, 1),
           "params": sum(p.numel() for p in net.parameters())}
    if a.cond or a.records or a.copies != 1:                    # 17-a (켤 때만)
        cfg.update({"cond": a.cond, "d_in": d_in, "records": a.records, "copies": a.copies, "excluded": L.excluded_list(a.records)})
    torch.save({"state_dict": net.state_dict(), "stats": st, "cfg": cfg}, out / "model.pt")
    with open(out / "loss.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["step", "loss", "loss_joint", "loss_grip", "sec"]); w.writerows(rows)
    (out / "train_info.json").write_text(json.dumps(cfg, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps({k: v for k, v in cfg.items() if k != "episodes"}, ensure_ascii=False), "마지막 손실", rows[-1] if rows else None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
