# VR Digital-Twin Teleoperation → Object-Centric Augmentation → ACT on Berkeley Humanoid Lite

Robot-free VR digital-twin teleoperation → object-centric augmentation → ACT for bimanual can handover on Berkeley Humanoid Lite (5-DoF arms).

This repository is a fork of the Berkeley Humanoid Lite low-level control code, extended with the teleoperation bridge, virtual-can demonstration recording, data augmentation, ACT training/inference scripts and real-robot evaluation tools used in our paper. The Unreal Engine project (VR scene, digital twin, virtual can) is available on request.

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
    recording:     demo_export.py  demo_replay.py  learn_data.py  to_lerobot.py
    augmentation:  aug_demo.py  handover_swap.py  policy_sim.py  learn_mlp.py
    calibration:   sag_comp.py  tip_offset.py  calib_tip.py
    real robot:    replay_run.py  policy_track.py  can_go.py  real_eval_prep.py  real_eval_fill.py
    evaluation:    eval_pairs.py  eval_plot.py  top19x_plan.py
    pipelines:     anypos_c2.sh  handover_g.sh  handover_j.sh  top19x.sh  learn_run.sh
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
- Server PC: runs the Unreal Engine 5.6 project (available on request) and `teleop/tools/cue_player.py` for voice cues.

**Network addresses — change these for your environment.** The code contains the IP addresses of our lab machines as defaults:

| Where | Value in this repo | Meaning |
|---|---|---|
| `teleop/run.sh` (`SERVER=`) | `100.85.65.19` | server PC (voice cues :5021, robot-viz :5022) |
| `teleop/tools/replay_run.py` (`DEFAULTS`) | `100.85.65.19` | same, for replay / policy runs |
| Unreal project sender setting | `192.168.0.65:5000` | robot PC address the UE project sends hand data to |

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

## Data format

Each episode is exported to `.npz` (`demo_export.py`) and converted to a LeRobot dataset (`to_lerobot.py`), **20 Hz**.

| Key | Dim | Content |
|---|---|---|
| `observation.state` | 16 | 10 arm joints (left/right × shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_roll) + 2 gripper states (L, R closed) + 4 condition values (can start x, y and goal x, y on the table, m) |
| `observation.environment_state` | 8 | virtual can position (x, y, z, m), orientation (quaternion), source flag |
| `action` | 12 | next joint targets (10) + gripper commands (2) |

The demonstration and augmented datasets are not stored in this repository (see Data availability).

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
