# Unreal Engine project (VR side) — source only

This folder contains the C++ source and the project/config files of the Unreal Engine project that runs on the server PC in our paper. It covers:

- **Body and hand tracking sender** — sends the operator's upper-body joints and finger pinch to the robot PC as UDP JSON (~14 Hz).
- **Robot visualization (robot-viz)** — receives the robot's gripper poses and limit state from the robot PC (UDP 5022, 30 Hz) and draws the digital twin in VR. It also draws the limit-state colors, the table, the virtual can, and the start and goal circles.

Voice cues are **not** played by Unreal. They are received and played by the Python script [`teleop/tools/cue_player.py`](../teleop/tools/cue_player.py), which runs on the server PC and listens on UDP 5021.

> Note: most code comments and log messages are in Korean.

## Requirements

| Item | Version |
|---|---|
| Unreal Engine | 5.6 (we used 5.6.1, Epic Games Launcher build) |
| MetaXR plugin (`OculusXR`) | 1.113.0 |
| `OculusInteraction` plugin | 1.81.0 |
| `OpenXREyeTracker` | engine built-in |
| OS / compiler | Windows 10/11, Visual Studio with the C++ game-development workload |
| Headset | Meta Quest 3 over Quest Link (cable or Air Link), with body tracking enabled |

`Config/DefaultEngine.ini` selects the OpenXR backend of the MetaXR plugin (`XrApi=OVRPluginOpenXR`).

## Files

```
unreal/
  test1.uproject                       project file: engine version and plugins
  Config/
    DefaultEngine.ini                  XR settings, default map and game mode (Android File Server SecurityToken left empty)
    DefaultGame.ini                    project settings
    DefaultInput.ini                   input settings from the VR template
  Source/
    test1.Target.cs, test1Editor.Target.cs   build targets
    test1/
      test1.Build.cs                   module dependencies (Sockets, Networking, OculusXR*)
      test1.cpp, test1.h               module startup
      CsvMotionRecorder.h/.cpp         writes Saved/MotionLog.csv during play (debug log, not used by the paper; needed to build)
      BodyTrackingSenderComponent.h/.cpp   tracking sender (actor component)
      RobotVizProbe.cpp                robot-viz receiver and drawing (console commands, no component)
```

The module is still called `test1`, which was the original project name.

## What is not included, and why

The Blueprints, the map and the template content are **not** in this repository. This includes `VRPawn2`, `VRGameMode`, `VRTemplateMap` and the VR Template / MetaXR sample assets, for three reasons:

- **Size.** These are large binary `.uasset` / `.umap` files.
- **Template license.** Most of the content comes from the Unreal Engine VR Template and is covered by the Unreal Engine EULA. It is not ours to relicense under MIT.
- **Internal addresses.** Our Pawn Blueprint stores the IP address of our lab robot PC.

To run the project, create a new project from the **Virtual Reality** template in UE 5.6 and add this source (see Setup).

## Setup

1. Create a new C++ project named `test1` from the UE 5.6 **Virtual Reality** template. Then copy `Source/`, `Config/` and `test1.uproject` from this folder over it, or copy this folder and add the template content to it.
   These steps were not re-run from a clean template project; we verified that this source builds on its own (C++ only, no assets).
2. Install the MetaXR plugin 1.113.0 and OculusInteraction 1.81.0 for UE 5.6, then build the `test1Editor` target (Development, Win64).
3. Open the VR Pawn Blueprint (the template's `VRPawn`, or your copy of it). **Add a `BodyTrackingSender` component** (class `UBodyTrackingSenderComponent`) to it.
4. In that component's details, under **UDP**:
   - Set **RemoteIp** to the IP address of the robot PC.
   - Set **RemotePort** to **5000**.
   - The default `ROBOT_PC_IP` is a placeholder. If you leave it, nothing is sent: the log shows `BodyTrackingSender Invalid UDP IP: ROBOT_PC_IP` once, then a `UDP SendTo failed` warning at most once per second.
   - To find the robot PC address, run `hostname -I` or `ip addr` on the robot PC.
5. Network:
   - The server PC and the robot PC must be on the same network, either a LAN or a VPN.
   - Open UDP 5000 in the robot PC firewall. The robot PC listens on `0.0.0.0:5000`.
6. Robot-viz (UDP 5022) and voice cues (UDP 5021) are sent **from the robot PC to the server PC**. Set the server PC address on the robot PC side: the `SERVER=` variable in `teleop/run.sh`, or the `--robot-viz` / `--cue-sound` options. See *Network addresses* in the [top-level README](../README.md).
7. Start Quest Link, open the project, start **VR Preview**, then turn robot-viz on with the console commands below.

## Tracking sender (`BodyTrackingSenderComponent`)

- Starts Meta body tracking (`OculusXRMovement`) and retries every second until it is available.
- Sends one UDP datagram (JSON, UTF-8) per send interval to `RemoteIp:RemotePort`.
  - `SendInterval` defaults to 0.0667 s, and the code never goes below that. This gives about **14 Hz** in practice (at most 15 Hz).
- Message:
  - `{"active": true, "joints": [{"name", "pos", "rot"}, …], "grip": {"L": [src, pinch_cm], "R": [src, pinch_cm]}}`
  - `joints` lists 12 joints: Head, Chest, Hips, SpineMiddle, and L/R Shoulder, UpperArm, LowerArm, WristTwist.
  - `src` is `"hand"`, `"ctrl"` or `"none"`, from the current OpenXR interaction profile.
  - `pinch_cm` is the raw thumb-tip to index-tip distance in cm.
  - When tracking is not available, the message is `{"active": false, "joints": [], "grip": …}`.
- The robot PC side (`vr_control_watchdog_biarm.py`) receives this on port 5000.
- Other properties: debug joint spheres (`bShowInEngineDebug`), avatar-to-camera alignment, and gripper mapping parameters. The defaults are what we used, except `RemoteIp`.

## Robot visualization (`RobotVizProbe.cpp`)

Robot-viz is **not a component**. It is turned on and configured with console commands, so it needs no Blueprint change. It listens on **UDP 5022**, where the robot PC sends gripper and limit-state packets (RVZ1) at **30 Hz**. The robot PC also sends table and slot data (RVT1, 1 Hz) and virtual-can poses (RVC1, 30 Hz) there. It draws in the play world, aligned to the Pawn that has the `BodyTrackingSender` component. Packets come from `robot_viz.py`, `table_limit.py` and `virtual_can.py` in `teleop/tools/` on the robot PC.

Run these in the editor console during VR Preview (12 commands):

| Command | Effect |
|---|---|
| `teleop.RobotViz 1` / `0` | Start / stop receiving on UDP 5022 and drawing. Starting also hides the avatar body |
| `teleop.RobotVizStyle 1` / `0` | Solid shapes (default) / debug lines (fallback) |
| `teleop.RobotVizRealSize -1` / `0` / `1` | Gripper size: follow packet option / scaled / real size 13.3 cm (default) |
| `teleop.RobotVizSmooth 1` / `0` | Smooth motion between packets (default on) / raw positions |
| `teleop.RobotVizSmoothMs <ms>` | Smoothing time constant, default 40 ms (5–500) |
| `teleop.RobotVizMeas 1` / `0` | Small sphere at the measured (encoder) position (default off) |
| `teleop.RobotVizEcho 1` / `0` | White sphere at the wrist position the robot PC received (for coordinate checks, default off) |
| `teleop.RobotVizTable 1` / `0` | Table and can slots (needs RVT1 from the robot PC, default off) |
| `teleop.RobotVizCan <n>` | Virtual can at slot `n`: 1–4 left L1–L4, 5–8 right R1–R4, 9 handover on table, 10 handover in air, 0 off. 1 or more also turns the table on |
| `teleop.RobotVizHideProps 1` / `0` | Hide level props that cover the table while playing (default off) |
| `teleop.RobotVizHideBody 1` / `0` | Hide the Pawn's avatar body mesh. Tracking and sending continue |
| `teleop.RobotVizListNear` | Log meshes near the table and the Pawn's mesh components (changes nothing) |

Gripper color per arm, from the packet flags:

| Color | Meaning |
|---|---|
| Gray | No packet for 0.5 s |
| Red | Joint limit or collision guard |
| Orange | Table-surface limit |
| Blue | Fully extended |
| Yellow | Lagging, re-aligned, or handover exception |
| Green | Normal |

Other circles:

| Circle | Color |
|---|---|
| One-arm slot | Cyan |
| Handover slot | Magenta |
| Goal | Green (lighter while the can is inside the goal tolerance) |
| "Place now would stand" ring | Sky blue |

`teleop.RobotViz` turns off when VR Preview ends. The other settings stay until the editor is closed.

## Ports

| Port | Direction | Content | Rate | Handled by |
|---|---|---|---|---|
| UDP 5000 | server PC → robot PC | tracking JSON | ~14 Hz | `BodyTrackingSenderComponent` → `vr_control_watchdog_biarm.py` |
| UDP 5022 | robot PC → server PC | robot-viz (RVZ1 30 Hz, RVT1 1 Hz, RVC1 30 Hz) | 30 Hz | `robot_viz.py` etc. → `RobotVizProbe.cpp` |
| UDP 5021 | robot PC → server PC | voice cues | per event | watchdog `--cue-sound` → `teleop/tools/cue_player.py` (Python, not Unreal) |

## Source

Taken from our internal Unreal repository at commit `248d153`. The source is unchanged except for two lines:

- `BodyTrackingSenderComponent.h`: the default `RemoteIp` is now the placeholder `ROBOT_PC_IP`.
- `Config/DefaultEngine.ini`: `SecurityToken` (Android File Server) is empty.
