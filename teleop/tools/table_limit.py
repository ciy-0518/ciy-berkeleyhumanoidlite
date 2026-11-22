#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""가대 면 제한 (작업 지시 11-b §4, 워치독 --table, 기본 끔). 판(_teleop/config/table.json) 과 로봇 팔 모델 사이 거리를 잰다.

모델은 ~/cc_analysis/table/can_map.py 와 같다 (cm, 손 축 위 위치 s = 집게 끝에서 손목 쪽으로):
  집게 끝 점 (s=0, 반지름 0) · 집게 (s=0~5.5, 1.0) · 링크 (s=5.5~6.8, 1.5) · 몸체 (s=6.8~13.3, 6.1 또는 0.9~3.0) · 아래팔 (팔꿈치→손목, 3.0) · 팔꿈치 (4.0)
  몸체 outer (기본): 평판 방향과 무관한 바깥 반지름 6.1 (REPORT_54:244). 손목 모터 부호를 VR 에서 확인하기 전이라 손목 각을 쓰지 않는다.
  몸체 real (작업 지시 12-c): 평판 치우침을 실제 방향으로. e90 엄지 위에서 서보·평판이 위, 집게가 좌우 수평 (사진 2026-11-02). 손목 각으로 돌린 네모 단면의 네 모서리를 잰다 (BODY_REAL).
  몸체 m2: 평판이 위로 가 있다고 볼 때 (0.9 / 3.0). 사용자 확인: e90 에서 집게 좌우 수평, 서보 몸통 위 (2026-11-02). 손목을 돌리면 맞지 않는다.
판: x_min~x_max, y_min~y_max, 윗면은 x 에 따라 선형. 판 아래는 바닥까지 박스.

거리 (m, 양수 = 떨어짐):
  판 위: 점 높이 − 반지름 × cosθ (θ = 손 축 기울기, 기운 원통의 아래쪽 끝)
  판 밖, 윗면보다 위: 판 가장자리까지 거리 − 반지름 (cosθ ↔ 1 을 방향으로 섞어 이어 붙임)
  판 밖, 윗면보다 아래: 수평 거리 − 반지름 (로봇 쪽 모서리를 지나 앞으로 들어가는 것을 막는다)
여유 (margin): 집게 끝 0.5, 집게 0, 몸체·아래팔 1.5 cm. 정규화 값 g = min(거리 − 여유). g < 0 이면 걸림.

손 목표 단계 (floor_g): 판을 뒤쪽(x 작은 쪽)으로 끝없이 늘려 윗면 높이만 본다 ("판 높이 아래 앞쪽 공간은 쓰지 않는다"). 옆(y)으로 판을 벗어나면
  FLOOR_FADE 에 걸쳐 풀어 끊김이 없다. 차렷처럼 손이 판 모서리 뒤로 내려가는 목표도 판 높이에서 멈춘다. 모터 쪽은 실제 판+박스 모양 (clear)."""
from __future__ import annotations

import json
import math
import struct
from pathlib import Path
from typing import Dict, Optional, Tuple

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "config" / "table.json"
BODY_MODELS = ("outer", "m2", "real")
GRIP_M = 0.133             # 손목 관절 → 집게 끝 (pos_retarget.ROBOT_GRIP_M 과 같은 값)
FLOOR_FADE = 0.05          # 옆으로 판을 벗어난 뒤 바닥 제한을 푸는 폭 (m)
# (s, 반지름) cm — 손 축 위 재는 점. 같은 구간은 양 끝과 가운데
_JAW = ((0.0, 1.0), (2.75, 1.0), (5.5, 1.0))
_LINK = ((5.5, 1.5), (6.8, 1.5))
_BODY = {"outer": ((6.8, 6.1), (9.15, 6.1), (11.5, 6.1), (13.3, 6.1)),
         "m2": ((6.8, 0.9), (9.15, 0.9), (11.5, 0.9), (11.5, 3.0), (13.3, 3.0))}
FOREARM_R, ELBOW_R = 0.03, 0.04
# real: (s, 서보 반대쪽 두께, 서보 쪽 두께, 좌우 반폭) cm. REPORT_54:244 — 평판 폭 5.7, 두께 7.0, 한쪽으로 2.4~2.8 치우침. 반대쪽 0.9·3.0 은 추정 (reach_C.py)
BODY_REAL = ((6.8, 0.9, 6.1, 2.85), (9.15, 0.9, 6.1, 2.85), (11.5, 0.9, 6.1, 2.85), (11.5, 3.0, 6.1, 3.0), (13.3, 3.0, 6.1, 3.0))

RVT_MAGIC = b"RVT1"
RVT_HDR = struct.Struct("<4sIHH" + "7f" + "3f" + "3f3f")   # 4+4+2+2 + 28 + 12 + 24 = 76
RVT_SLOT = struct.Struct("<4s" + "3f" + "3f" + "f" + "BB2x" + "3f")   # 4 + 12 + 12 + 4 + 4 + 12 = 48
RVT_KIND = {"arm": 0, "handover_table": 1, "handover_air": 2}
RVT_ARM = {"left": 0, "right": 1, "both": 2}


class TableLimit:
    def __init__(self, path=None, body: str = "outer", margin_body_cm: Optional[float] = None, margin_tip_cm: Optional[float] = None):
        self.path = Path(path) if path else DEFAULT_PATH
        self.cfg = json.loads(self.path.read_text(encoding="utf-8"))
        p = self.cfg["plate"]
        self.x0, self.x1 = p["x_min"] / 100.0, p["x_max"] / 100.0
        self.y0, self.y1 = p["y_min"] / 100.0, p["y_max"] / 100.0
        self.z0, self.z1 = p["z_top_at_x_min"] / 100.0, p["z_top_at_x_max"] / 100.0
        m = self.cfg.get("margin_cm", {})
        self.m_body = float(m.get("body_and_forearm", 1.5) if margin_body_cm is None else margin_body_cm) / 100.0
        self.m_tip = float(m.get("fingertip", 0.5) if margin_tip_cm is None else margin_tip_cm) / 100.0
        self.body = body if body in BODY_MODELS else "outer"
        self._hand = [(s / 100.0, r / 100.0, self.m_body) for s, r in _LINK + _BODY[self.body if self.body != "real" else "outer"]]
        self._link = [(s / 100.0, r / 100.0, self.m_body) for s, r in _LINK]
        # real 은 몸체가 판에 더 가까이 가게 해 주므로, 집게 표면이 끝 여유만큼 떠 있게 한다 (outer·m2 는 0 그대로)
        self.m_jaw = self.m_tip if self.body == "real" else 0.0
        self._jaw = [(s / 100.0, r / 100.0, self.m_jaw) for s, r in _JAW]

    def top(self, x: float) -> float:
        t = (min(max(x, self.x0), self.x1) - self.x0) / (self.x1 - self.x0)
        return self.z0 + t * (self.z1 - self.z0)

    def point(self, x: float, y: float, z: float, r: float, c: float) -> float:
        """점 (x, y, z) 를 반지름 r 인 몸으로 볼 때 판+박스까지 거리 (m). c = cosθ (손 축 기울기)."""
        hx = max(self.x0 - x, x - self.x1, 0.0)
        hy = max(self.y0 - y, y - self.y1, 0.0)
        dz = z - self.top(x)
        if hx == 0.0 and hy == 0.0:
            if dz >= 0.0:
                return dz - r * c
            # 판(박스) 속: 빠져나올 거리만큼 더 나쁘게
            return -min(-dz, x - self.x0, self.x1 - x, y - self.y0, self.y1 - y) - r
        h = math.hypot(hx, hy)
        if dz <= 0.0:
            return h - r
        d = math.hypot(h, dz)
        return d - r * (c * dz + h) / d

    @staticmethod
    def servo_frame(hx, jaw0, sign: float, roll: float):
        """body real: 손 축 hx, 손목 0 일 때 집게 벌어지는 축 jaw0, 부호 sign (e90 에서 서보가 위가 되게), 손목 각 roll (rad, 모터 − e90)
        → (서보 쪽 방향, 집게 벌어지는 방향). robot_viz.py 의 jaw 와 같은 식 (손 축 둘레 +roll)."""
        cx, cy, cz = hx[1] * jaw0[2] - hx[2] * jaw0[1], hx[2] * jaw0[0] - hx[0] * jaw0[2], hx[0] * jaw0[1] - hx[1] * jaw0[0]
        c, s = math.cos(roll), math.sin(roll)
        return ((sign * (c * cx - s * jaw0[0]), sign * (c * cy - s * jaw0[1]), sign * (c * cz - s * jaw0[2])),
                (c * jaw0[0] + s * cx, c * jaw0[1] + s * cy, c * jaw0[2] + s * cz))

    def _corners(self, tx, ty, tz, ax, ay, az, servo):
        """body real: 몸체 단면 모서리들 (x, y, z, m)."""
        (nx, ny, nz), (jx, jy, jz) = servo
        for s, lo, hi, w in BODY_REAL:
            s, lo, hi, w = s / 100.0, lo / 100.0, hi / 100.0, w / 100.0
            bx, by, bz = tx - s * ax, ty - s * ay, tz - s * az
            for a in (-lo, hi):
                for b in (-w, w):
                    yield bx + a * nx + b * jx, by + a * ny + b * jy, bz + a * nz + b * jz

    def clear(self, el, wr, hx, servo=None) -> Dict[str, float]:
        """팔 자세 (팔꿈치, 손목, 손 축 — 모두 로봇 base m) → 부분별 거리 (여유를 빼지 않은 값) 와 g (여유를 뺀 최소, m).
        servo = servo_frame() 값 (body real 만 쓴다. 없으면 바깥 반지름 6.1)."""
        ex, ey, ez = float(el[0]), float(el[1]), float(el[2])
        wx, wy, wz = float(wr[0]), float(wr[1]), float(wr[2])
        ax, ay, az = float(hx[0]), float(hx[1]), float(hx[2])
        c = math.sqrt(max(1.0 - az * az, 0.0))
        tx, ty, tz = wx + GRIP_M * ax, wy + GRIP_M * ay, wz + GRIP_M * az
        tip = self.point(tx, ty, tz, 0.0, c)
        jaw = min(self.point(tx - s * ax, ty - s * ay, tz - s * az, r, c) for s, r, _ in self._jaw)
        if self.body == "real" and servo is not None:
            body = min(min(self.point(tx - s * ax, ty - s * ay, tz - s * az, r, c) for s, r, _ in self._link),
                       min(self.point(x, y, z, 0.0, c) for x, y, z in self._corners(tx, ty, tz, ax, ay, az, servo)))
        else:
            body = min(self.point(tx - s * ax, ty - s * ay, tz - s * az, r, c) for s, r, _ in self._hand)
        fx, fy, fz = wx - ex, wy - ey, wz - ez
        n = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
        cf = math.sqrt(max(1.0 - (fz / n) ** 2, 0.0))
        arm = min(min(self.point(ex + t * fx, ey + t * fy, ez + t * fz, FOREARM_R, cf) for t in (0.0, 0.5, 1.0)),
                  self.point(ex, ey, ez, ELBOW_R, 1.0))
        g = min(tip - self.m_tip, jaw - self.m_jaw, body - self.m_body, arm - self.m_body)
        return {"tip": tip, "jaw": jaw, "body": body, "arm": arm, "g": g, "min": min(tip, jaw, body, arm)}

    def g(self, el, wr, hx, servo=None) -> float:
        return self.clear(el, wr, hx, servo)["g"]

    # ---- 손 목표 단계
    def _floor_w(self, y: float) -> float:
        """옆으로 판을 벗어나면 바닥 제한을 푼다 (0 ~ 1). 판 폭 + 몸 반지름 안은 1."""
        out = max(self.y0 - y, y - self.y1, 0.0) - 0.08
        if out <= 0.0:
            return 1.0
        return max(1.0 - out / FLOOR_FADE, 0.0)

    def floor_g(self, el, wr, hx, servo=None) -> float:
        """손 목표 단계 값 (m, 여유 뺌): 판을 뒤로 끝없이 늘린 면 위 높이만 본다. 옆으로 벗어나면 위쪽으로 풀림."""
        wx, wy, wz = float(wr[0]), float(wr[1]), float(wr[2])
        ax, ay, az = float(hx[0]), float(hx[1]), float(hx[2])
        c = math.sqrt(max(1.0 - az * az, 0.0))
        tx, ty, tz = wx + GRIP_M * ax, wy + GRIP_M * ay, wz + GRIP_M * az
        best = math.inf
        real = self.body == "real" and servo is not None
        for s, r, m in [(0.0, 0.0, self.m_tip)] + self._jaw + (self._link if real else self._hand):
            x, y, z = tx - s * ax, ty - s * ay, tz - s * az
            v = z - self.top(x) - r * c - m
            w = self._floor_w(y)
            best = min(best, v if w >= 1.0 else v + (1.0 - w) * 0.5)    # 풀린 몫은 큰 값으로 (반 미터면 닿지 않는다)
        if real:
            for x, y, z in self._corners(tx, ty, tz, ax, ay, az, servo):
                v = z - self.top(x) - self.m_body
                w = self._floor_w(y)
                best = min(best, v if w >= 1.0 else v + (1.0 - w) * 0.5)
        ex, ey, ez = float(el[0]), float(el[1]), float(el[2])
        fx, fy, fz = wx - ex, wy - ey, wz - ez
        n = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
        cf = math.sqrt(max(1.0 - (fz / n) ** 2, 0.0))
        for t in (0.0, 0.5, 1.0):
            x, y, z = ex + t * fx, ey + t * fy, ez + t * fz
            w = self._floor_w(y)
            v = z - self.top(x) - FOREARM_R * cf - self.m_body
            best = min(best, v if w >= 1.0 else v + (1.0 - w) * 0.5)
        return best

    @staticmethod
    def soft_lift(g: float, goal: float, band: float) -> float:
        """바닥 값 g (m) → 올릴 양 (m). g ≥ goal + band: 0, 그 아래 band·2 구간은 매끄럽게, 더 아래는 g 를 goal 로 맞춤 (기울기가 이어져 끊김이 없다)."""
        e = goal + band - g
        if e <= 0.0:
            return 0.0
        if e < 2.0 * band:
            return e * e / (4.0 * band)
        return e - band

    # ---- VR 표시 패킷 (RobotVizProbe.cpp FTableHdr / FTableSlot 과 같은 순서, 값 cm)
    def rvt_packet(self, seq: int, ps_left, ps_right) -> bytes:
        p, can = self.cfg["plate"], self.cfg["objects"]["can"]
        slots = self.cfg.get("can_slots", [])
        out = [RVT_HDR.pack(RVT_MAGIC, seq & 0xFFFFFFFF, len(slots), 0,
                            p["x_min"], p["x_max"], p["y_min"], p["y_max"], p["z_top_at_x_min"], p["z_top_at_x_max"], p.get("thickness_display", 1.0),
                            can["diameter"], can["height"], float(self.cfg.get("recommended_grip_h_cm", 8)),
                            *[float(v) * 100.0 for v in ps_left], *[float(v) * 100.0 for v in ps_right])]
        for s in slots:
            x, y = s["base_xy_cm"]
            ax = s.get("hand_axis", (0.0, 0.0, 0.0))
            gh = list(s.get("grip_h_cm", [])) + [0.0, 0.0]
            z = (self.z0 + (min(max(x / 100.0, self.x0), self.x1) - self.x0) / (self.x1 - self.x0) * (self.z1 - self.z0)) * 100.0
            out.append(RVT_SLOT.pack(s["name"].encode("ascii")[:4].ljust(4, b"\0"), float(x), float(y), z, *[float(v) for v in ax],
                                     float(s.get("tip_past_axis_cm", 0.0)), RVT_KIND[s["kind"]], RVT_ARM[s["arm"]],
                                     float(gh[0]), float(gh[1]), float(s.get("can_bottom_cm", 0.0))))
        return b"".join(out)


def unpack_rvt(payload: bytes) -> dict:
    """시험·기록용: RVT1 패킷 → dict."""
    v = RVT_HDR.unpack_from(payload, 0)
    if v[0] != RVT_MAGIC:
        raise ValueError("RVT1 아님")
    n = v[2]
    if len(payload) != RVT_HDR.size + n * RVT_SLOT.size:
        raise ValueError(f"크기 다름 ({len(payload)})")
    out = {"seq": v[1], "n": n, "plate": v[4:11], "can": v[11:14], "ps_left": v[14:17], "ps_right": v[17:20], "slots": []}
    for i in range(n):
        s = RVT_SLOT.unpack_from(payload, RVT_HDR.size + i * RVT_SLOT.size)
        out["slots"].append({"name": s[0].rstrip(b"\0").decode(), "pos": s[1:4], "axis": s[4:7], "tip_past": s[7], "kind": s[8], "arm": s[9], "grip_h": s[10:12], "bottom": s[12]})
    return out
