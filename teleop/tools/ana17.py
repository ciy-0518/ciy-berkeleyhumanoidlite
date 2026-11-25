#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""19-x17 실기 shift 8 판 로그 분석 (읽기만, CPU). 로그 logs/real19x16_bNN_shift.csv 를 판마다 같은 판의 VR 시연 (shift npz = 원본 dry 기록을 그 자리로 옮긴 명령) 과 비교.
자세 넷 (모두 모터 rad → virtual_can.hand_of FK, 지금 robot_ref · table.json):
  V 시연 (논리) = cmd + sag 열 (보정 전 목표 = npz 명령과 같음, 처음에 확인)
  I 의도       = V + tipdq (집게 끝 · 놓기 · 들기 오프셋까지 넣은 목표, 처짐 보정 전)
  C 보낸 명령  = cmd 열 (처짐 보정까지 넣어 모터에 보낸 값)
  E 엔코더     = meas 열 (실물)
사건 = 그리퍼 명령 바뀜 (재생은 grip_wait 동안 멈춘다). '닫힘 끝' = 그 닫기 뒤 grip_wait 마지막 행 (그리퍼가 다 닫힌 뒤, 팔은 그 자리에 멈춰 있음).
캔: 판 시작 자리 (plan16 좌표) 에 선 캔. L 잡기 = 첫 L 닫기 (시작 관문 닫기 다음). 붙이기 = 19-h 가정 B (닫히면 캔이 L 집게 축 가운데로).
R 깊이 s = virtual_can 둘째 손 판정 s (B 자세). 19-h 기준은 엔코더 A 기준 실패 ≤ 1.65, 성공 ≥ 2.35 (B 기준 실패 ≤ 1.48, 성공 ≥ 3.24). A 도 같이 낸다.
놓기: L 이 여는 순간 캔을 R 에 지금 자세로 다시 붙이고, R 이 여는 순간 캔 자세 (밑면 높이 · 기울기) 와 R 끝 속도 (0.2 초).
출발 경로: 단계 settle · approach (쉬는 자세 → 시연 첫 자세) 동안 집게 상자 (virtual_can 상자, 그 때 그리퍼 명령의 열림 · 닫힘) · 아래팔 선분 ↔ 선 캔 최소 거리.
출력: ana17.json, 화면 요약."""
import csv
import datetime as dt
import json
import math
import sys
from pathlib import Path

import numpy as np

H = Path.home()
TOOLS = Path(__file__).resolve().parent                      # 공개 저장소: teleop/tools
for p in (str(TOOLS), str(TOOLS.parents[1])):                  # 저장소 맨 위 (vr_control_watchdog_biarm.py)
    sys.path.insert(0, p)
import vr_control_watchdog_biarm as W  # noqa: E402  (워치독 먼저)
import virtual_can as VC  # noqa: E402
import policy_sim as PS  # noqa: E402
import demo_export as DE  # noqa: E402

D = H / "cc_analysis/real19x16"
LOGS = H / "teleop-docs/_teleop/logs"
SIDES = ("left", "right")
J = list(W.JOINT_ORDER)
POSJ = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch"]
UP = np.array([0.0, 0.0, 1.0])
PS.VCAN_EXTRA.update({"grasp_mode": "top", "real": True})


def fnum(v):
    try:
        x = float(v)
        return x if math.isfinite(x) else float("nan")
    except (TypeError, ValueError):
        return float("nan")


def ang(a, b, absdot=False):
    c = float(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.degrees(math.acos(min(1.0, abs(c)) if absdot else max(-1.0, min(1.0, c))))


def r(v, n=2):
    if isinstance(v, (str, bool)):
        return v
    return None if v is None or (isinstance(v, float) and not math.isfinite(v)) else round(float(v), n)


class Geo:
    def __init__(self, start, goal):
        self.env = PS.SimEnv(start=f"{start[0]:g},{start[1]:g}", goal=f"{goal[0]:g},{goal[1]:g}", strict=True)
        self.vc = self.env.vcan
        self.can0 = self.vc._slot_base(self.vc.start_name).copy()
        self.goal = self.vc._slot_base(self.vc.goal_name).copy()
        self.table = self.env.table

    def hand(self, side, q):
        return self.vc.hand_of(side, q)

    def elbow_wrist(self, side, q):
        qu = self.vc.q_of(side, q)
        M = np.eye(4)
        el = None
        for n, O in zip(VC.JOINTS, self.vc.models[side]._O):
            c_, s_ = math.cos(qu[n]), math.sin(qu[n])
            M = M @ O @ np.array([[c_, -s_, 0, 0], [s_, c_, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]])
            if n == "elbow_pitch":
                el = M[:3, 3].copy()
        return el * 100.0, M[:3, 3] * 100.0

    def top(self, x_cm):
        return self.table.top(x_cm / 100.0) * 100.0


def seg_seg(p1, q1, p2, q2):
    import handover_depth as HD
    return HD.seg_dist(p1, q1, p2, q2)


def poses(R):
    """행마다 자세 넷 (없으면 None)."""
    out = {k: [] for k in "VICE"}
    for r_ in R:
        V, I, C, E = {}, {}, {}, {}
        for s in SIDES:
            V[s], I[s], C[s], E[s] = {}, {}, {}, {}
            for j in J:
                c = fnum(r_.get(f"{s}_{j}_cmd"))
                g = fnum(r_.get(f"{s}_{j}_sag"))
                g = 0.0 if g != g else g
                V[s][j] = c + g
                td = fnum(r_.get(f"{s}_{j}_tipdq_deg")) if j in POSJ else 0.0
                I[s][j] = V[s][j] + math.radians(0.0 if td != td else td)
                C[s][j] = c
                E[s][j] = fnum(r_.get(f"{s}_{j}_meas"))
        for k, q in (("V", V), ("I", I), ("C", C), ("E", E)):
            ok = all(v == v for s in SIDES for v in q[s].values())
            out[k].append(q if ok else None)
    return out


def events(R):
    ev = []
    for i in range(1, len(R)):
        for h in ("L", "R"):
            a, b = R[i - 1].get(f"grip_{h}"), R[i].get(f"grip_{h}")
            if a in ("0", "1") and b in ("0", "1") and a != b:
                ev.append({"i": i, "h": h, "k": "c" if b == "1" else "o", "frame": int(R[i]["replay_frame"] or 0), "t": fnum(R[i]["t"])})
    return ev


def settle_end(R, i):
    """닫기 · 열기 명령 행 i 뒤 grip_wait 마지막 행 (없으면 i)."""
    k = i
    while k + 1 < len(R) and R[k + 1]["auto_stage"] != "grip_wait" and k - i < 3:
        k += 1
    if R[k]["auto_stage"] != "grip_wait":
        return i
    while k + 1 < len(R) and R[k + 1]["auto_stage"] == "grip_wait":
        k += 1
    return k


def handover_geom(g, hL, hR, can_rel_L):
    """L 이 쥔 캔 (L 틀 상대 자세) 을 지금 L 손에 놓고 R 둘째 손 판정."""
    vc = g.vc
    tL, RL = hL
    tR, RR = hR
    rr, rp = can_rel_L
    vc.rot, vc.pos, vc.holders, vc.align = RL @ rr, tL + RL @ rp, ["left"], None
    c = vc.grip_check(hR, VC.HANDOVER_HEIGHT_EXTRA_CM, True, side="right")
    _, side_cm = vc.strict_center(hR)
    z = vc.rot[:, 2]
    out = {"s": c["s"], "j": c["j"], "t": c["t"], "ok": bool(c["ok"]), "why": c["reason"], "strict_side": side_cm,
           "tilt_Rn_can": ang(RR[:, 2], z, True),                       # virtual_can 인계 각도 (R 셋째 축 ↔ 캔 축)
           "Rn_Ln": ang(RR[:, 2], RL[:, 2], True),                      # R ↔ L 셋째 축 상대각
           "axes": ang(RL[:, 0], RR[:, 0]),                              # 두 손 축 사이
           "L_n_vert": ang(RL[:, 2], UP, True), "R_n_vert": ang(RR[:, 2], UP, True), "can_vert": ang(z, UP, True),
           "L_tip": tL, "R_tip": tR, "can_pos": vc.pos.copy(), "can_rot": vc.rot.copy()}
    return out


def L_grasp(g, hL):
    vc = g.vc
    vc.pos, vc.rot, vc.holders, vc.align = g.can0.copy(), np.eye(3), [], None
    c = vc.grip_check(hL, 0.0, False, side="left")
    short, side_cm = vc.strict_center(hL)
    tL, RL = hL
    return {"t": c["t"], "s": c["s"], "j": c["j"], "ok": bool(c["ok"]), "why": c["reason"], "strict_short": short, "strict_side": side_cm,
            "L_n_vert": ang(RL[:, 2], UP, True), "L_h_down": math.degrees(math.asin(max(-1, min(1, -RL[2, 0])))),
            "tip_minus_can": (tL - g.can0)}


def clearance(g, side, q, closed):
    """선 캔 (시작 자리) ↔ 집게 · 링크 · 몸체 상자 최소 거리 (cm, 음수 = 겹침), 아래팔 선분 ↔ 캔 축 − 반지름."""
    vc = g.vc
    vc.pos, vc.rot, vc.holders = g.can0.copy(), np.eye(3), []
    tip, R_ = vc.hand_of(side, q)
    z = vc.rot[:, 2]
    rel = (vc.pos - tip)[None, :] + vc._bump_t[:, None] * z[None, :]
    s, j, n = -(rel @ R_[:, 0]), rel @ R_[:, 1], rel @ R_[:, 2]
    half = VC.JAW_HALF_CLOSED_CM if closed else VC.JAW_HALF_OPEN_CM
    names = ["날+", "날−", "링크", "몸체"]
    ds = [float(vc._box_dist(s, j, n, b).min()) - vc.radius for b in vc._hand_boxes(side, half)]
    k = int(np.argmin(ds))
    el, wr = g.elbow_wrist(side, q)
    fa = seg_seg(el, wr, g.can0, g.can0 + vc.height * z) - vc.radius
    return {"grip": ds[k], "part": names[k], "forearm": fa, "tip": tip}


def speed(R, P, i, side, g, win=0.2):
    """행 i 직전 win 초 R 끝 평균 속도 cm/s (실시간)."""
    t1 = fnum(R[i]["t"])
    k = i
    while k > 0 and fnum(R[k]["t"]) > t1 - win:
        k -= 1
    if P[k] is None or P[i] is None or t1 - fnum(R[k]["t"]) < 1e-3:
        return float("nan")
    return float(np.linalg.norm(g.hand(side, P[i][side])[0] - g.hand(side, P[k][side])[0]) / (t1 - fnum(R[k]["t"])))


def npz_speed(arr, jm, f, g, win=0.2):
    """VR 시연 (npz 명령) 의 R 끝 속도 cm/s, 시연 시간 20 Hz."""
    n = max(1, int(round(win * 20)))
    q = lambda k: {j: float(arr["motor_cmd"][k, 1, i]) for i, j in enumerate(J)}   # noqa: E731
    f0 = max(0, f - n)
    return float(np.linalg.norm(g.hand("right", q(f))[0] - g.hand("right", q(f0))[0]) / ((f - f0) / 20.0)) if f > f0 else float("nan")


def place_geom(g, hR_lo, can_lo, hR_ro):
    """L 이 연 순간 캔을 R 에 다시 붙여 R 이 연 순간 캔 자세."""
    tR0, RR0 = hR_lo
    pos, rot = can_lo
    rel = (RR0.T @ rot, RR0.T @ (pos - tR0))
    tR, RR = hR_ro
    rot2, pos2 = RR @ rel[0], tR + RR @ rel[1]
    z = rot2[:, 2]
    drop = g.vc.radius * math.sqrt(max(1.0 - float(z[2]) ** 2, 0.0))
    low = min(float(pos2[2]), float(pos2[2] + g.vc.height * z[2])) - drop
    top = g.top(float(pos2[0]))
    return {"can_low_h": low - top, "can_base_h": float(pos2[2]) - top, "can_tilt": ang(z, UP, True), "R_tip_h": float(tR[2]) - top,
            "R_n_vert": ang(RR[:, 2], UP, True), "goal_dx": float(pos2[0] - g.goal[0]), "goal_dy": float(pos2[1] - g.goal[1])}


def board(b):
    log = LOGS / f"{b['log']}.csv"
    R = list(csv.DictReader(log.open(encoding="utf-8", errors="ignore")))
    arr = dict(np.load(b["file"], allow_pickle=True))
    meta = json.loads(str(arr["meta"]))
    g = Geo(b["start"], b["goal"])
    P = poses(R)
    ev = events(R)
    out = {"board": b["board"], "log": log.name, "demo": b["demo"], "start": b["start"], "goal": b["goal"], "trained": b["trained"],
           "ref": g.env.ref_path.name, "log_ref_should": DE.ref_path_for(dt.datetime.fromtimestamp(log.stat().st_mtime - 120).isoformat()).name,
           "sim_real": meta["policy_summary"].get("real"), "sim_result": meta["policy_summary"].get("result")}
    # ---- V ↔ npz 명령 확인 (재생 행의 frame 으로)
    dif = []
    for i, r_ in enumerate(R):
        if r_["replay_phase"] == "play" and P["V"][i] is not None:
            f = int(r_["replay_frame"] or 0)
            if f < len(arr["motor_cmd"]):
                dif.append(max(abs(P["V"][i][s][j] - float(arr["motor_cmd"][f, si, ji])) for si, s in enumerate(SIDES) for ji, j in enumerate(J)))
    out["V_minus_npz_max_deg"] = r(math.degrees(max(dif)) if dif else float("nan"), 3)
    out["events"] = [f"{e['h']}{e['k']}@{e['frame']}" for e in ev]
    # ---- 출발 경로 (settle · approach)
    app = [i for i, r_ in enumerate(R) if r_["auto_stage"] in ("settle", "approach")]
    cl = {}
    for src in ("E", "C"):
        best = None
        for i in app:
            q = P[src][i]
            if q is None:
                continue
            for s in SIDES:
                closed = R[i].get("grip_L" if s == "left" else "grip_R") == "1"
                c = clearance(g, s, q[s], closed)
                for part, d in (("grip", c["grip"]), ("forearm", c["forearm"])):
                    if best is None or d < best["d"]:
                        best = {"d": d, "side": s, "part": c["part"] if part == "grip" else "아래팔", "frac": (i - app[0]) / max(1, len(app) - 1),
                                "tip_h": float(c["tip"][2]) - g.top(float(c["tip"][0])), "tip_minus_can_xy": [r(c["tip"][0] - g.can0[0]), r(c["tip"][1] - g.can0[1])],
                                "closed": closed}
        cl[src] = {k: (r(v) if isinstance(v, float) else v) for k, v in (best or {}).items()}
    # 시연 첫 자세 (재생 첫 행, 엔코더)
    i0 = next((i for i, r_ in enumerate(R) if r_["auto_stage"] == "play"), None)
    if i0 is not None and P["E"][i0] is not None:
        fp = {}
        for s in SIDES:
            tip = g.hand(s, P["E"][i0][s])[0]
            fp[s] = {"tip_h": r(float(tip[2]) - g.top(float(tip[0]))), "tip_minus_can_xy": [r(tip[0] - g.can0[0]), r(tip[1] - g.can0[1])]}
        cl["first_pose_E"] = fp
    # 쉬는 자세 (approach 첫 행)
    if app and P["E"][app[0]] is not None:
        rp = {}
        for s in SIDES:
            tip = g.hand(s, P["E"][app[0]][s])[0]
            rp[s] = {"tip_h": r(float(tip[2]) - g.top(float(tip[0]))), "tip_minus_can_xy": [r(tip[0] - g.can0[0]), r(tip[1] - g.can0[1])]}
        cl["rest_pose_E"] = rp
    out["approach"] = cl
    # ---- L 잡기
    lo1 = next((e for e in ev if e["h"] == "L" and e["k"] == "o"), None)
    lc = next((e for e in ev if e["h"] == "L" and e["k"] == "c" and lo1 and e["i"] > lo1["i"]), None)
    if lc is None:
        out["skip"] = "L 닫기 없음"
        return out
    kL = settle_end(R, lc["i"])
    LG, rel = {}, {}
    for src in "VIE":
        hL = g.hand("left", P[src][kL]["left"])
        lg = L_grasp(g, hL)
        LG[src] = {k: (r(v) if isinstance(v, float) else ([r(x) for x in v] if isinstance(v, np.ndarray) else v)) for k, v in lg.items()}
        tL, RL = hL
        canB = g.can0 - float(lg["j"]) * RL[:, 1]
        rel[src] = {"B": (RL.T @ np.eye(3), RL.T @ (canB - tL))}
        tL0, RL0 = g.hand("left", P[src][lc["i"]]["left"])        # A: 닫기 명령 순간 그대로
        rel[src]["A"] = (RL0.T @ np.eye(3), RL0.T @ (g.can0 - tL0))
    out["L_grasp"] = LG
    out["L_close_frame"] = lc["frame"]
    # ---- R 닫기마다 (L 이 쥔 동안)
    lo = next((e for e in ev if e["h"] == "L" and e["k"] == "o" and e["i"] > lc["i"]), None)
    rcs = [e for e in ev if e["h"] == "R" and e["k"] == "c" and e["i"] > lc["i"] and (lo is None or e["i"] < lo["i"])]
    ros = [e for e in ev if e["h"] == "R" and e["k"] == "o" and e["i"] > lc["i"] and (lo is None or e["i"] < lo["i"])]
    RC = []
    for e in rcs:
        k = settle_end(R, e["i"])
        row = {"frame": e["frame"]}
        for src in "VIE":
            hL, hR = g.hand("left", P[src][k]["left"]), g.hand("right", P[src][k]["right"])
            for lab in ("B", "A"):
                hg = handover_geom(g, hL, hR, rel[src][lab])
                if lab == "B":
                    row[src] = {kk: (r(v, 1) if kk in ("tilt_Rn_can", "Rn_Ln", "axes", "L_n_vert", "R_n_vert", "can_vert") else r(v)) for kk, v in hg.items()
                                if not isinstance(v, np.ndarray)}
                    row[src]["rel_R_minus_L_Rframe"] = [r(v) for v in hR[1].T @ (hL[0] - hR[0])]
                else:
                    row[src]["s_A"] = r(hg["s"])
        if "C" in P:
            hL, hR = g.hand("left", P["C"][k]["left"]), g.hand("right", P["C"][k]["right"])
            hg = handover_geom(g, hL, hR, rel["V"]["B"])
            row["C"] = {kk: r(hg[kk], 1) for kk in ("tilt_Rn_can", "Rn_Ln", "axes", "L_n_vert", "R_n_vert")}
        RC.append(row)
    out["R_closes"] = RC
    out["R_opens_while_L"] = [e["frame"] for e in ros]
    # ---- 놓기
    ro = next((e for e in reversed(ev) if e["h"] == "R" and e["k"] == "o"), None)
    if lo is not None and ro is not None and rcs:
        kR = settle_end(R, rcs[-1]["i"])
        PL = {}
        for src in "VIE":
            hL, hR = g.hand("left", P[src][kR]["left"]), g.hand("right", P[src][kR]["right"])
            hg = handover_geom(g, hL, hR, rel[src]["B"])
            # L 이 여는 순간: 캔은 L 을 따라온 자세
            hL2 = g.hand("left", P[src][lo["i"]]["left"])
            rr, rp = rel[src]["B"]
            can_lo = (hL2[0] + hL2[1] @ rp, hL2[1] @ rr)
            pg = place_geom(g, g.hand("right", P[src][lo["i"]]["right"]), can_lo, g.hand("right", P[src][ro["i"]]["right"]))
            pg["speed_R_cms"] = speed(R, P[src], ro["i"], "right", g) if src == "E" else (npz_speed(arr, None, ro["frame"], g) if src == "V" else speed(R, P[src], ro["i"], "right", g))
            PL[src] = {k: r(v, 1) for k, v in pg.items()}
        f = min(ro["frame"], len(arr["can_pos"]) - 1)
        cz = arr["can_quat"][f]
        x, y, z_, w = cz
        zc = np.array([2 * (x * z_ + w * y), 2 * (y * z_ - w * x), 1 - 2 * (x * x + y * y)])
        PL["sim_npz"] = {"can_base_h": r(float(arr["can_pos"][f, 2]) - g.top(float(arr["can_pos"][f, 0])), 1), "can_tilt": r(ang(zc, UP, True), 1)}
        out["place"] = PL
        out["L_open_frame"], out["R_open_frame"] = lo["frame"], ro["frame"]
    return out


def main():
    plan = json.loads((D / "plan16.json").read_text(encoding="utf-8"))
    res = []
    for b in plan["boards"]:
        o = board(b)
        res.append(o)
        print(f"\n=== 판 {o['board']} {o['demo']} 시작 {o['start']} → {o['goal']}  V−npz {o['V_minus_npz_max_deg']}°  사건 {' '.join(o['events'])}")
        a = o["approach"]
        print("  출발 경로 E:", a.get("E"), "\n            C:", a.get("C"), "\n  첫 자세:", a.get("first_pose_E"), " 쉬는 자세:", a.get("rest_pose_E"))
        if "L_grasp" in o:
            for s in "VIE":
                lg = o["L_grasp"][s]
                print(f"  L 잡기 {s}: t {lg['t']} s {lg['s']} j {lg['j']} ok {lg['ok']} {lg['why']} | 엄격 짧음 {lg['strict_short']} 비킴 {lg['strict_side']} | n수직 {lg['L_n_vert']} 숙임 {lg['L_h_down']}")
        for rc in o.get("R_closes", []):
            for s in "VIE":
                x = rc[s]
                print(f"  R 닫기 @{rc['frame']} {s}: s(B) {x['s']} s(A) {x['s_A']} j {x['j']} t {x['t']} ok {x['ok']} {x['why']} | 기울기 {x['tilt_Rn_can']} Rn↔Ln {x['Rn_Ln']} 축 {x['axes']} | L n수직 {x['L_n_vert']} R n수직 {x['R_n_vert']} 캔 {x['can_vert']}")
            print("              C:", rc.get("C"))
        if "place" in o:
            for s in ("V", "I", "E", "sim_npz"):
                print(f"  놓기 {s}:", o["place"][s])
    (D / "ana17.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=lambda v: v.tolist() if isinstance(v, np.ndarray) else str(v)), encoding="utf-8")


if __name__ == "__main__" and len(sys.argv) == 1:
    main()


# ---------------------------------------------------------------------------------------------- 둘째 단계 (19-x17 b)
def npz_pose(arr, f):
    f = min(max(int(f), 0), len(arr["motor_cmd"]) - 1)
    return {s: {j: float(arr["motor_cmd"][f, si, ji]) for ji, j in enumerate(J)} for si, s in enumerate(SIDES)}


def part2():
    """인계 깊이 팔별 나누기 · 관절 잔차 · 안전장치 몫 · 놓기 높이 시간 흐름 · 출발 경로 닫힌 집게."""
    plan = json.loads((D / "plan16.json").read_text(encoding="utf-8"))
    res = []
    for b in plan["boards"]:
        log = LOGS / f"{b['log']}.csv"
        R = list(csv.DictReader(log.open(encoding="utf-8", errors="ignore")))
        arr = dict(np.load(b["file"], allow_pickle=True))
        g = Geo(b["start"], b["goal"])
        P = poses(R)
        ev = events(R)
        o = {"board": b["board"]}
        # 출발 경로: 열린 · 닫힌 집게 최소 거리 (엔코더)
        app = [i for i, r_ in enumerate(R) if r_["auto_stage"] in ("settle", "approach")]
        for closed in (False, True):
            best = None
            for i in app:
                for s in SIDES:
                    c = clearance(g, s, P["E"][i][s], closed)
                    if best is None or c["grip"] < best[0]:
                        best = (c["grip"], s, c["part"], float(c["tip"][2]) - g.top(float(c["tip"][0])))
            o["app_" + ("closed" if closed else "open")] = [r(best[0]), best[1], best[2], r(best[3])]
        tip0 = g.hand("left", P["E"][app[0]]["left"])[0]
        o["L_rest_horiz_cm"] = r(math.hypot(tip0[0] - g.can0[0], tip0[1] - g.can0[1]))
        lo1 = next((e for e in ev if e["h"] == "L" and e["k"] == "o"), None)
        lc = next((e for e in ev if e["h"] == "L" and e["k"] == "c" and lo1 and e["i"] > lo1["i"]), None)
        lo = next((e for e in ev if e["h"] == "L" and e["k"] == "o" and e["i"] > lc["i"]), None)
        rcs = [e for e in ev if e["h"] == "R" and e["k"] == "c" and e["i"] > lc["i"] and (lo is None or e["i"] < lo["i"])]
        ro = next((e for e in ev if e["h"] == "R" and e["k"] == "o" and lo is not None and e["i"] > lo["i"]), None)
        kL = settle_end(R, lc["i"])
        # 캔 붙이기 (B) — 자세마다 자기 L 잡기로
        def relB(q):
            hL = g.hand("left", q["left"])
            lg = L_grasp(g, hL)
            canB = g.can0 - float(lg["j"]) * hL[1][:, 1]
            return (hL[1].T, hL[1].T @ (canB - hL[0]))
        srcs = {"N": lambda k, f: npz_pose(arr, f), "V": lambda k, f: P["V"][k], "I": lambda k, f: P["I"][k], "E": lambda k, f: P["E"][k]}
        rel = {s: relB(fn(kL, lc["frame"])) for s, fn in srcs.items()}
        # 안전장치 (두 팔 간격) 막음: L 닫기 ~ 마지막 R 닫기 사이 행 수
        gb = [int(r_["gap_blocks"] or 0) for r_ in R]
        if rcs:
            k_last = settle_end(R, rcs[-1]["i"])
            o["gap_block_rows_handover"] = sum(1 for i in range(kL, k_last + 1) if gb[i] > gb[i - 1])
        RC = []
        for e in rcs:
            k = settle_end(R, e["i"])
            f = e["frame"]
            row = {"frame": f}
            for s, fn in srcs.items():
                q = fn(k, f)
                hg = handover_geom(g, g.hand("left", q["left"]), g.hand("right", q["right"]), rel[s])
                row[s] = {"s": r(hg["s"]), "j": r(hg["j"]), "t": r(hg["t"]), "tilt": r(hg["tilt_Rn_can"], 1), "RnLn": r(hg["Rn_Ln"], 1), "axes": r(hg["axes"], 1),
                          "Ltip": [r(v, 1) for v in hg["L_tip"]], "Rtip": [r(v, 1) for v in hg["R_tip"]]}
            # 팔별: L 만 엔코더 · R 만 엔코더 (나머지는 의도)
            for lab, qL, qR in (("L_E_R_I", P["E"][k]["left"], P["I"][k]["right"]), ("L_I_R_E", P["I"][k]["left"], P["E"][k]["right"])):
                hg = handover_geom(g, g.hand("left", qL), g.hand("right", qR), rel["I"])
                row[lab] = r(hg["s"])
            # 관절 잔차 (도): 엔코더 − 보낸 명령, 엔코더 − 의도, 의도 − 시연 (안전장치 + 오프셋)
            row["E_minus_C_deg"] = {s[0].upper(): [r(math.degrees(P["E"][k][s][j] - P["C"][k][s][j]), 1) for j in J] for s in SIDES}
            row["E_minus_I_deg"] = {s[0].upper(): [r(math.degrees(P["E"][k][s][j] - P["I"][k][s][j]), 1) for j in J] for s in SIDES}
            row["V_minus_N_deg"] = {s[0].upper(): [r(math.degrees(P["V"][k][s][j] - npz_pose(arr, f)[s][j]), 1) for j in J] for s in SIDES}
            for s in SIDES:
                row[f"tip_E_minus_I_{s[0].upper()}"] = [r(v) for v in g.hand(s, P["E"][k][s])[0] - g.hand(s, P["I"][k][s])[0]]
                row[f"tip_V_minus_N_{s[0].upper()}"] = [r(v) for v in g.hand(s, P["V"][k][s])[0] - g.hand(s, npz_pose(arr, f)[s])[0]]
            RC.append(row)
        o["R_closes"] = RC
        # 놓기: R 열기 전 3 초 R 끝 높이 (판 위) 의도 · 엔코더 · 시연
        if ro is not None:
            t1 = fnum(R[ro["i"]]["t"])
            tr = []
            for i in range(max(0, ro["i"] - 80), ro["i"] + 1):
                tt = fnum(R[i]["t"]) - t1
                if tt < -3.0:
                    continue
                f = int(R[i]["replay_frame"] or 0)
                hE, hI, hN = (g.hand("right", P[s][i]["right"])[0] for s in ("E", "I")), None, None
                hE, hI = list(hE)
                hN = g.hand("right", npz_pose(arr, f)["right"])[0]
                tp = g.top(float(hE[0]))
                tr.append((round(tt, 2), round(float(hN[2]) - tp, 2), round(float(hI[2]) - tp, 2), round(float(hE[2]) - tp, 2), R[i]["auto_stage"],
                           R[i].get("right_place_tip_state"), R[i].get("right_carry_state"), R[i].get("right_carry_gain")))
            o["place_trace"] = tr[::4] + [tr[-1]]
            o["place_EminusI_at_open"] = r(tr[-1][3] - tr[-1][2])
            o["place_EminusI_min_max_last3s"] = [r(min(x[3] - x[2] for x in tr)), r(max(x[3] - x[2] for x in tr))]
            o["place_N_E_at_open"] = [tr[-1][1], tr[-1][3]]
        res.append(o)
        print(f"\n=== 판 {o['board']}  출발 열린 {o['app_open']} 닫힌 {o['app_closed']} L 쉬는 끝 ↔ 캔 수평 {o['L_rest_horiz_cm']}  인계 중 간격 막음 행 {o.get('gap_block_rows_handover')}")
        for rc in RC:
            print(f"  R 닫기 @{rc['frame']}: s N {rc['N']['s']} V {rc['V']['s']} I {rc['I']['s']} E {rc['E']['s']} | L만E {rc['L_E_R_I']} R만E {rc['L_I_R_E']} | 기울기 N {rc['N']['tilt']} E {rc['E']['tilt']} RnLn N {rc['N']['RnLn']} V {rc['V']['RnLn']} E {rc['E']['RnLn']} 축 N {rc['N']['axes']} E {rc['E']['axes']}")
            print(f"     E−C {rc['E_minus_C_deg']}  E−I {rc['E_minus_I_deg']}  V−N {rc['V_minus_N_deg']}")
            print(f"     끝 E−I L {rc['tip_E_minus_I_L']} R {rc['tip_E_minus_I_R']}  V−N L {rc['tip_V_minus_N_L']} R {rc['tip_V_minus_N_R']}")
        if "place_trace" in o:
            print("  놓기 R 끝 높이 (t, N, I, E, 단계, place_tip, carry, gain):", o["place_trace"][-6:], " E−I 열 때", o["place_EminusI_at_open"], "3 초 범위", o["place_EminusI_min_max_last3s"])
    (D / "ana17b.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=str), encoding="utf-8")


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "b":
    part2()
