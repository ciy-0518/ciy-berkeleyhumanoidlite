#!/usr/bin/env python3
"""19-x7 누름 편집: 놓기 구간에서 잡힌 캔이 판에 닿은 뒤 R 이 더 내려간 프레임을 판 면 높이 (닿은 순간 R 끝 높이) 에서 멈춘다.

  press_edit.py --files <npz 목록> --out <폴더> [--list-out <새 목록>]

규칙 (모든 시연에 같게):
  1. 구간 = 인계 뒤 R 혼자 든 동안 (can_state) 첫 "캔 밑면이 판 면에 닿음" 프레임 kc 부터, R 이 연 뒤 R 끝이 닿은 높이로 다시 올라올 때까지.
     닿음 = 캔 밑면 높이 (can_pos z, 가상 캔이 닿으면 판 면에 세운다) ≤ 판 면 + CONTACT_TOL_CM.
  2. 그 구간에서 R 끝 (tip_cmd) 이 닿은 높이 zc 보다 낮은 프레임은 R 팔 값 (q_cmd · motor_cmd · tip_cmd) 을 바로 앞 "낮지 않은" 프레임 값으로 바꾼다.
     L · 그리퍼 · 캔 기록은 그대로 (가상 캔은 판 위에 서 있었다).
  3. 편집한 프레임 수 · 비율 · 최대 누름 (zc − R 끝 최저) 을 meta["press_edit"] 와 --out/press_edit.json 에 남긴다.
학습 데이터만 바꾼다 (원래 dry-run 기록 · 에피소드 파일은 그대로). 실물 기준의 누름은 수집 때 경고로만 남는다 (virtual_can 19-x7)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

T = Path(__file__).resolve().parent
sys.path.insert(0, str(T))
import table_limit as TL  # noqa: E402

CONTACT_TOL_CM = 0.05
HELD_BOTH, HELD_R = 3, 2
TABLE = T.parent / "config/table.json"


def edit_one(z: dict, tab) -> tuple[dict, dict]:
    st = np.asarray(z["can_state"], int)                     # virtual_can: 0 판 위, 1 L, 2 R, 3 둘 다, 4 넘어짐
    N = len(st)
    info = {"frames": N, "edited": 0, "ratio": 0.0, "press_max_cm": 0.0, "kc": None, "why": ""}
    both = np.where(st == HELD_BOTH)[0]
    if len(both) == 0:
        info["why"] = "인계 없음"
        return z, info
    rhold = np.where((np.arange(N) > both[0]) & (st == HELD_R))[0]   # 인계 뒤 R 혼자 든 프레임
    if len(rhold) == 0:
        info["why"] = "인계 뒤 R 혼자 든 구간 없음"
        return z, info
    iRo = int(rhold[-1]) + 1                                  # R 이 놓은 첫 프레임
    can = np.asarray(z["can_pos"], float)
    top = np.array([tab.top(float(x) / 100.0) * 100.0 for x in can[:, 0]])
    touch = [int(k) for k in rhold if can[k, 2] <= top[k] + CONTACT_TOL_CM]
    if not touch:
        info["why"] = "놓기 닿음 없음"
        return z, info
    kc = touch[0]
    tip = np.asarray(z["tip_cmd"], float)
    zc = float(tip[kc, 1, 2])
    out = {k: (np.array(v, copy=True) if k in ("q_cmd", "motor_cmd", "tip_cmd") else v) for k, v in z.items()}
    good = kc
    edited = 0
    low = zc
    for k in range(kc, N):
        if k >= iRo and tip[k, 1, 2] >= zc:
            break
        if tip[k, 1, 2] < zc:
            low = min(low, float(tip[k, 1, 2]))
            for key in ("q_cmd", "motor_cmd", "tip_cmd"):
                out[key][k, 1] = out[key][good, 1]
            edited += 1
        else:
            good = k
    info.update(edited=edited, ratio=round(edited / N, 4), press_max_cm=round(zc - low, 2), kc=kc, iRo=iRo)
    return out, info


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog="\n".join(__doc__.split("\n")[1:]))
    ap.add_argument("--files", required=True, help="npz 경로 목록 (한 줄에 하나)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--list-out", default=None, help="편집한 npz 목록 (기본 <out>/files_edit.txt)")
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    tab = TL.TableLimit(TABLE, body="real")
    paths = [p.strip() for p in Path(a.files).expanduser().read_text(encoding="utf-8").splitlines() if p.strip() and not p.startswith("#")]
    rows, new = [], []
    tot = ed = 0
    for p in paths:
        zf = np.load(p, allow_pickle=True)
        z = {k: zf[k] for k in zf.files}
        meta = json.loads(str(z["meta"]))
        z2, info = edit_one(z, tab)
        meta["press_edit"] = {"rule": "19-x7 놓기 닿음 뒤 R 끝 ≤ 닿은 높이 → 앞 프레임에 멈춤", **info}
        z2["meta"] = np.array(json.dumps(meta, ensure_ascii=False, default=float))
        dst = out / Path(p).name
        np.savez_compressed(dst, **z2)
        new.append(str(dst))
        tot += info["frames"]
        ed += info["edited"]
        rows.append({"file": p, **info})
        print(f"{Path(p).stem}: 편집 {info['edited']} / {info['frames']} 프레임 ({100 * info['ratio']:.1f} %), 최대 누름 {info['press_max_cm']:.2f} cm {info['why']}", flush=True)
    lst = Path(a.list_out).expanduser() if a.list_out else out / "files_edit.txt"
    lst.write_text("\n".join(new) + "\n", encoding="utf-8")
    summ = {"files": len(paths), "frames": tot, "edited": ed, "ratio": round(ed / tot, 4) if tot else 0.0, "rows": rows}
    (out / "press_edit.json").write_text(json.dumps(summ, ensure_ascii=False, indent=1, default=float), encoding="utf-8")
    print(f"모두 {len(paths)} 판: 편집 {ed} / {tot} 프레임 ({100 * summ['ratio']:.2f} %) → {lst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
