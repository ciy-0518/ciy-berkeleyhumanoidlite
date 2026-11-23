# VR Digital-Twin Teleoperation → Object-Centric Augmentation → ACT on Berkeley Humanoid Lite

Robot-free VR digital-twin teleoperation → object-centric augmentation → ACT for bimanual can handover on Berkeley Humanoid Lite (5-DoF arms).

This repository is a fork of the Berkeley Humanoid Lite low-level control code, extended with the teleoperation bridge, virtual-can demonstration recording, data augmentation, ACT training/inference scripts and real-robot evaluation tools used in our paper. The Unreal Engine source for the VR side (tracking sender, digital twin, virtual can) is in [`unreal/`](unreal/).

> Note: most code comments and command-line help strings are in Korean.

## System overview

```
 Server PC (Windows)                          Robot PC (Ubuntu)
 ┌───────────────────────────┐   UDP JSON    ┌──────────────────────────────────────┐
 │ Meta Quest 3 (hand track) │   ~14 Hz      │ vr_control_watchdog_biarm.py         │
 │ Unreal Engine 5.6         │ ────────────▶ │  · hand-position retarget + IK       │
 │  · robot-viz digital twin │               │  · safety limits (joint / table /    │
 │  · virtual can, success   │ ◀──────────── │    arm gap), 200 Hz watchdog loop    │
 │  · voice cues             │  robot state  │  · CAN ×2 (can0 left, can1 right)    │
 └───────────────────────────┘  (robot-viz)  │  · Arduino gripper (USB serial)      │
                                             └──────────────────────────────────────┘
```

- During demonstration collection the robot does not need to move (`--dry-run`): the operator sees the robot's digital twin and a virtual can in VR, and the robot PC simulates the can and judges success.
- Recorded demonstrations are augmented by shifting the can's start/goal positions (object-centric shift) and re-solving the arm IK, then used to train ACT (LeRobot).
- The trained policy runs on the real robot through `replay_run.py --policy`, with the same safety chain and calibration (sag, gripper-tip offset, place offset, carry lift, gripper sync).
- Two grasp modes are supported, each with its own policy: **bottom grasp** (left hand holds the lower part of the can, right hand takes the upper part) and **top grasp** (left hand holds the upper part, right hand takes the lower part). See [Top-grasp mode](#top-grasp-mode-learned-policy--geometric-corrections).

## Repository layout

```
vr_control_watchdog_biarm.py   teleoperation bridge (UDP receiver, retarget/IK, safety, 200 Hz watchdog, CAN, gripper)
berkeley_humanoid_lite_lowlevel/, csrc/, scripts/   upstream low-level control (CAN, motors)
teleop/
  run.sh                       short launcher for every step below (run.sh with no argument prints the list)
  config/                      table geometry, gripper angles, sag model, reach maps, start/goal coordinate pairs
  arduino/                     gripper firmware (Arduino)
  learn/requirements_learn.txt pinned Python environment for training (LeRobot 0.6.1, PyTorch)
  tools/
    teleop:        pos_retarget.py  table_limit.py  auto_target.py  gripper_link.py  robot_viz.py  arm_pose_logger.py
    voice cues:    cue_player.py (server PC)  session_cue.py  *_session.py  split_by_cue.py  voice_durations.json
    virtual can:   virtual_can.py  handover_depth.py  top_check.py
    recording:     demo_export.py  demo_replay.py  learn_data.py  to_lerobot.py  press_edit.py  top_trim.py
    augmentation:  aug_demo.py  handover_swap.py  policy_sim.py  learn_mlp.py
    calibration:   sag_comp.py  tip_offset.py  calib_tip.py
    corrections:   handover_assist.py (top grasp: left-grasp assist, right-handover assist, left-open safety)
    real robot:    replay_run.py  policy_track.py  policy_server.py  can_go.py  real_eval_prep.py  real_eval_fill.py
    run sheets:    top_shift_prep.py  top_policy_prep.py
    evaluation:    eval_pairs.py  eval_plot.py  top19x_plan.py
    pipelines:     anypos_c2.sh  handover_g.sh  handover_j.sh  top19x.sh  top19x_trim.sh  learn_run.sh
unreal/                        Unreal Engine 5.6 C++ source for the VR side (tracking sender, robot-viz); no assets
docs/UPSTREAM_README.md        original Berkeley Humanoid Lite low-level README (installation, CAN, motors)
```

## Setup

The scripts use fixed paths under `$HOME`. Clone and link as follows (no code edits needed):

```bash
mkdir -p ~/code_workspace/Berkeley-Humanoid-Lite-main
git clone -b learn https://github.com/ciy-0518/ciy-berkeleyhumanoidlite.git \
    ~/code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel
REPO=~/code_workspace/Berkeley-Humanoid-Lite-main/Berkeley-Humanoid-Lite-Lowlevel

mkdir -p ~/teleop-docs && ln -s "$REPO/teleop" ~/teleop-docs/_teleop   # tools look for ~/teleop-docs/_teleop
ln -s "$REPO/teleop/run.sh" ~/run.sh                                  # optional: the examples below use ~/run.sh
mkdir -p ~/teleop-docs/_teleop/logs ~/cc_analysis                     # recordings / training outputs
```

- Robot control: follow [docs/UPSTREAM_README.md](docs/UPSTREAM_README.md) (`pip install -r requirements.txt`, can-utils).
- Training: a separate virtual environment at `~/venvs/learn` built from [teleop/learn/requirements_learn.txt](teleop/learn/requirements_learn.txt).
- Server PC: runs the Unreal Engine 5.6 project (source in [`unreal/`](unreal/)) and `teleop/tools/cue_player.py` for voice cues.

**Network addresses — change these for your environment.** The code contains the IP addresses of our lab machines as defaults:

| Where | Value in this repo | Meaning |
|---|---|---|
| `teleop/run.sh` (`SERVER=`) | `SERVER_PC_IP` (placeholder) | server PC (voice cues :5021, robot-viz :5022) |
| `teleop/tools/replay_run.py` (`DEFAULTS`) | `SERVER_PC_IP` (placeholder) | same, for replay / policy runs |
| `unreal/` BodyTrackingSender `RemoteIp` | `ROBOT_PC_IP` (placeholder) | robot PC address the UE project sends hand data to |

Replace the placeholders with your own addresses.

The robot PC listens on `0.0.0.0:5000`, so only the sender side and `SERVER=` need to match your network. Command-line options (`--cue-sound`, `--robot-viz`) override the defaults.

## Running (in order)

```bash
~/run.sh can                 # 1. bring up can0 / can1 at 1 Mbps and ping the motors
~/run.sh P                   # 2. robot reference pose (each time the robot is powered on)

# 3. Demonstration collection in VR (robot stays still, virtual can)
~/run.sh dry --pairs ~/teleop-docs/_teleop/config/pairs_demo_19b.txt
#    → recordings in ~/teleop-docs/_teleop/logs/dry_vcan_<HHMM>_*  (~/run.sh list shows the episodes)

# 4.–6. Export → augmentation → LeRobot dataset → ACT training → simulated evaluation (one line)
bash ~/teleop-docs/_teleop/tools/anypos_c2.sh <name> dry_vcan_<HHMM> [dry_vcan_<HHMM> ...]
#    env vars: STEPS (30000), COPIES (9), FACTORS ("5 10"), SHIFT_CM (3), FROM (export|aug|dataset|train|eval)
#    outputs:  ~/cc_analysis/anypos_c2/<name>/, checkpoints in ~/cc_analysis/learn/runs/anypos_<name>_x<F>
#    internally: demo_export.py → npz
#                aug_demo.py (shift + IK, filters, simulation check)
#                to_lerobot.py --cond xy (state 16)
#                lerobot-train --policy.type=act --policy.chunk_size=20 --policy.n_action_steps=20
#                eval_pairs.py / eval_plot.py

# 7. Real-robot run
~/run.sh policy ~/cc_analysis/learn/runs/<run>/checkpoints/last/pretrained_model --dry-run   # check first
~/run.sh policy ~/cc_analysis/learn/runs/<run>/checkpoints/last/pretrained_model
python3 ~/teleop-docs/_teleop/tools/real_eval_prep.py --help    # evaluation sheet; real_eval_fill.py fills it from logs
```

Live teleoperation of the real robot: `~/run.sh teleop`.
New features are behind command-line flags that are off by default.

### Can coordinates (`can_go.py`)

`can_go.py` takes the can start and goal positions, checks them (table limits, reach maps, distance to the training demonstrations), shows the command, waits for Enter and then runs the policy. The coordinates are in the robot base frame, in cm (x forward, y to the robot's left). They come from one of two sources:

- **Measured on the table.** Give `--start x,y --goal x,y`, or use `--plate` to give distances measured from the table edges.
- **Sent over UDP by an external module.** Use `--start-from udp:<port>`. Each datagram is one JSON line, `{"start": [x, y], "conf": 0..1, "t": <unix time>}`, in robot-base cm, and only the latest value before the run is used.

The depth-camera module that produced UDP coordinates in our setup is **not** part of this repository.

```bash
python3 teleop/tools/can_go.py --start 27,13 --goal 33,-12 --print-only   # show the command and checks only
python3 teleop/tools/can_go.py --start 27,13 --goal 33,-12                # check → Enter → run
python3 teleop/tools/can_go.py --start-from udp:5030 --goal 33,-12        # can position from an external module
python3 teleop/tools/can_go.py --start 31,14 --goal 28,-24 --mode-auto    # choose the grasp mode by reach
```

`can_go.py --mode-auto` chooses the grasp mode from the reach maps, and it is off by default. When the bottom-grasp checks fail only on left-hand reach, it consults the top-grasp reach map (`config/reach_top_19x.json`) and switches to the top-grasp policy (`--policy-top`). The command it builds for top grasp does not include the correction chain described below. To reproduce the paper's top-grasp runs, use the `top_policy_prep.py` run sheet.

## Top-grasp mode (learned policy + geometric corrections)

Top grasp is meant to extend the workspace to can positions that the bottom-grasp policy cannot reach. In the paper’s real-robot runs, the reachable front edge was narrower than computed. It uses its own ACT policy, trained from VR demonstrations collected with `--grasp-mode top --grasp-real`. In those demonstrations, success is judged against the real gripper geometry, and the shoulder roll is limited to the angle where the upper arm physically meets the torso frame.

On its own, the top-grasp policy does not align the grippers precisely enough for the handover. The policy produces the overall motion: reach, grasp, handover, carry and place. Geometric corrections take over the final alignment:

| Correction | Flag (`replay_run.py --policy`) | What it does |
|---|---|---|
| Left-grasp assist | `--grasp-assist <json>` | Moves the left gripper to a can-relative target pose learned from passing demonstrations. It closes only within the target height and depth, and stops the run if the target is out of reach. |
| Right-handover assist and left-open safety | `--handover-assist <json>` | Moves the right gripper to the can-relative handover pose. The left hand may open only after the right hand has closed within the depth and offset criteria. After the retries run out, the right hand opens and backs off while the left hand keeps the can. |
| Staged gripper-tip offsets | `--tip-offset-cm`, `--grasp-tip-offset-cm` + `--grasp-tip-skip-start-grip` + `--grasp-tip-release-on-grasp`, `--place-tip-offset-cm` | Applies a different tip offset in each phase. An extra left offset is used until the grasp that actually takes the can. |
| Shoulder-roll hold | `--tip-roll-stop` | Produces the tip offset with the other joints while keeping the shoulder roll within its physical stop. |
| Place-time elbow sag | `--place-sag-elbow-scale` | Scales the right-elbow sag compensation while only the right hand holds the can. |
| Tip-offset joint cap | `--tip-cap-deg` | Caps the joint change that the tip offset may add. |
| End rules | `--policy-end-skip-start-grip`, `--policy-end-after-held`, `--grasp-assist-held-on-grasp` | Counts only grasps and releases that involve the can, so that a close in the air does not end the run or switch off the assist. |

All of these flags are off by default.

**Paper runs.** The top-grasp real-robot runs in the paper (19-x29) were made from a run sheet generated by `top_policy_prep.py`, with the following setup:

- policy: `anypos_top19x22_x5`
- correction configs: `config/grasp_assist_top_19x28.json` and `config/handover_assist_top_19x26.json`

```bash
# 1. Demonstration collection in VR, top grasp (robot stays still)
~/run.sh dry --pairs ~/teleop-docs/_teleop/config/pairs_top_19x.txt --grasp-mode top --grasp-real --start-gate

# 2. Check → export → press edit → augmentation → dataset → ACT training → simulated evaluation
bash ~/teleop-docs/_teleop/tools/top19x.sh dry_vcan_<HHMM> [dry_vcan_<HHMM> ...]

# 3. Trim the handover segment → augmentation → dataset → training → evaluation (the paper's policy anypos_top19x22_x5)
NAME=top19x22 SRC=<list of exported demo .npz> ROLL_STOP=1 TRIM_TAG=trimB22 MODE=B \
    bash ~/teleop-docs/_teleop/tools/top19x_trim.sh
#    trimmed-record list: ~/cc_analysis/top19x22/run/trim/files_trim.txt (policy start poses, used below)

# 4. Real-robot run sheet (one command per board, checks, results CSV)
CK=~/cc_analysis/learn/runs/anypos_top19x22_x5/checkpoints/last/pretrained_model
python3 ~/teleop-docs/_teleop/tools/top_policy_prep.py --boards <boards.json> --policy $CK \
    --first-from ~/cc_analysis/top19x22/run/trim/files_trim.txt \
    --offset-cm 1.5 --grasp-extra-left-cm 1.5 --grasp-release-on-grasp --grasp-held-on-grasp --tip-cap-deg 30 --end-after-held \
    --grasp-assist ~/teleop-docs/_teleop/config/grasp_assist_top_19x28.json \
    --handover-assist ~/teleop-docs/_teleop/config/handover_assist_top_19x26.json \
    --can-first --suffix 29 --tag real19x29 --out ~/cc_analysis/real19x29
#    <boards.json>: list of {"label", "start": [x, y], "goal": [x, y], "trained"} in robot-base cm
#    → runbook29.md, plan29.json, results29.csv
```

The command that the run sheet produces for one board looks like this:

```bash
~/run.sh policy ~/cc_analysis/learn/runs/anypos_top19x22_x5/checkpoints/last/pretrained_model \
  --can-start 31,14 --can-goal 28,-24 --sag-comp --no-grip-warm --grip-open-first --tip-cap-deg 30 \
  --tip-offset-cm L 1.5 0 0 R 1.5 0 0 --place-tip-offset-cm R -2 0 0 --carry-lift-cm R 2 --tip-roll-stop \
  --open-settle-cm 0.5 --open-settle-max-s 1.0 \
  --grasp-tip-offset-cm L 1.5 0 0 --grasp-tip-skip-start-grip --grasp-tip-release-on-grasp \
  --place-sag-elbow-scale 0 --grasp-assist-held-on-grasp \
  --policy-end-after-release 2 --policy-max-s 40 --policy-obs sag-model --policy-can-basis encoder-nocorr \
  --policy-end-skip-start-grip --policy-end-after-held \
  --grasp-assist ~/teleop-docs/_teleop/config/grasp_assist_top_19x28.json \
  --handover-assist ~/teleop-docs/_teleop/config/handover_assist_top_19x26.json \
  --policy-server-args "--first-from ~/cc_analysis/top19x22/run/trim/files_trim.txt" \
  --log-csv ~/teleop-docs/_teleop/logs/real19x29_b01_policy.csv
```

The correction configs are generated by `handover_assist.py --files <passing demos> --out <json>`, with `--grasp` for the left-grasp target. The 19x26 configs add the rule parameters, and 19x28 moves the left target 0.3 cm deeper along the left hand axis.

## Data format

Each episode is exported to `.npz` (`demo_export.py`) and converted to a LeRobot dataset (`to_lerobot.py`), **20 Hz**.

| Key | Dim | Content |
|---|---|---|
| `observation.state` | 16 | 10 arm joints (left/right × shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_roll) + 2 gripper states (L, R closed) + 4 condition values (can start x, y and goal x, y on the table, m) |
| `observation.environment_state` | 8 | virtual can position (x, y, z, m), orientation (quaternion), source flag |
| `action` | 12 | next joint targets (10) + gripper commands (2) |

The demonstration and augmented datasets are not stored in this repository (see Data availability).

The `files` list in the `config/*assist*.json` files records which demonstrations the targets were computed from. It is not read at run time.

## Data availability

TBD — full demonstration / augmented datasets will be provided as a GitHub Release attachment or an external link. A small sample episode is in `examples/` (TBD).

## Citation

```bibtex
@inproceedings{TBD,
  title     = {TBD},
  author    = {TBD},
  booktitle = {KRoC 2027},
  year      = {2027}
}
```

## License

MIT License — see [LICENSE](LICENSE). The low-level control code is from the [Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite) project (© 2025 The Berkeley Humanoid Lite Project Developers); our additions are released under the same license.
