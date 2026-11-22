#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""좌표 쌍 평가 그림 (작업 지시 19-c1). 시스템 python3 (matplotlib) 로 돌린다: eval_plot.py <results.json> <출력.png> [기준 ok15|ok5]
방법마다 판 위에 쌍을 선으로 (시작 ● → 도착 ■), 선 색 = 그 쌍의 성공 비율. 점 색 = 그 시작 · 도착이 든 쌍들의 성공 비율."""
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager as fm  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONTB = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FP = fm.FontProperties(fname=FONT) if Path(FONT).exists() else None
FB = fm.FontProperties(fname=FONTB) if Path(FONTB).exists() else FP
INK, MUTE, GRID = "#1f2933", "#52606d", "#d9dee4"
COLS = ["#c2410c", "#e8a33d", "#1f5fa8"]          # 0 / 일부 / 모두 성공


def col(rate: float) -> str:
    return COLS[0] if rate <= 0.0 else COLS[2] if rate >= 1.0 else COLS[1]


def main() -> int:
    d = json.load(open(sys.argv[1]))
    key = sys.argv[3] if len(sys.argv) > 3 else "ok15"
    rows, pairs = d["rows"], d["pairs"]
    labels = list(dict.fromkeys(r["method"] for r in rows))
    n = len(labels)
    cols = min(n, 3)
    nr = (n + cols - 1) // cols
    fig, axes = plt.subplots(nr, cols, figsize=(5.6 * cols, 2.6 * nr + 1.0), dpi=140, squeeze=False)
    fig.patch.set_facecolor("white")
    for k, lb in enumerate(labels):
        ax = axes[k // cols][k % cols]
        rr = [r for r in rows if r["method"] == lb]
        rate = {}
        for pi in range(1, len(pairs) + 1):
            q = [r[key] for r in rr if r["pair"] == pi]
            rate[pi] = (sum(q) / len(q)) if q else None
        ax.add_patch(Rectangle((-29.75, 7), 60, 40, fill=False, edgecolor=INK, linewidth=1.0))
        for pi, (sx, sy, gx, gy) in enumerate(pairs, 1):
            if rate[pi] is None:
                continue
            ax.plot([sy, gy], [sx, gx], color=col(rate[pi]), linewidth=1.4, alpha=0.75, zorder=2)
        pts = {}
        for pi, (sx, sy, gx, gy) in enumerate(pairs, 1):
            if rate[pi] is None:
                continue
            pts.setdefault(("s", sx, sy), []).append(rate[pi])
            pts.setdefault(("g", gx, gy), []).append(rate[pi])
        for (kind, x, y), v in pts.items():
            ax.scatter([y], [x], s=70, marker="o" if kind == "s" else "s", facecolor=col(float(np.mean(v))), edgecolor=INK, linewidth=0.8, zorder=3)
        tot = len(rr)
        ok = sum(r[key] for r in rr)
        ax.set_title(f"{lb} — 성공 {ok} / {tot} ({'1.5 cm' if key == 'ok15' else '5 cm'})", fontproperties=FB, fontsize=10.5, color=INK, loc="left")
        ax.set_xlim(31, -31)
        ax.set_ylim(18, 40)
        ax.set_aspect("equal")
        ax.grid(color=GRID, linewidth=0.5, zorder=0)
        ax.tick_params(colors=MUTE, labelsize=7)
        for sp in ax.spines.values():
            sp.set_visible(False)
        ax.set_xlabel("좌우 y (cm, + = 로봇 왼쪽)", fontproperties=FP, fontsize=8, color=MUTE)
        ax.set_ylabel("앞뒤 x (cm)", fontproperties=FP, fontsize=8, color=MUTE)
    for k in range(n, nr * cols):
        axes[k // cols][k % cols].axis("off")
    h = [Line2D([], [], color=COLS[2], linewidth=2, label="모두 성공"), Line2D([], [], color=COLS[1], linewidth=2, label="일부 성공"),
         Line2D([], [], color=COLS[0], linewidth=2, label="모두 실패"),
         Line2D([], [], marker="o", linestyle="none", markerfacecolor="white", markeredgecolor=INK, label="시작 (L)"),
         Line2D([], [], marker="s", linestyle="none", markerfacecolor="white", markeredgecolor=INK, label="도착 (R)")]
    fig.legend(handles=h, loc="lower center", ncol=5, frameon=False, prop=FP)
    fig.suptitle("좌표 쌍 평가 — 판 위 성공 · 실패 (위에서 본 판, 로봇은 아래쪽)", fontproperties=FB, fontsize=12, color=INK, x=0.01, ha="left")
    fig.tight_layout(rect=(0, 0.08, 1, 0.93))
    fig.savefig(sys.argv[2], facecolor="white")
    print("saved", sys.argv[2])
    return 0


if __name__ == "__main__":
    sys.exit(main())
