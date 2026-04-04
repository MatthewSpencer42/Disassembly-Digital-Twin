# Testing Launch Report

This document describes what to launch and in which order to test the EXOTica motion planner,
individual skills, and the full disassembly system. All commands run from the workspace root
unless otherwise noted.

```bash
cd /home/adip/workspace/disassembly_ws
source install/setup.bash
```

---

## 1. EXOTica Planner — Fake Hardware (No Robot Required)

The safest starting point. Tests IK solving and trajectory generation without touching hardware.

**Terminal 1 — Launch MoveIt + EXOTica (fake mode)**
```bash
ros2 launch dual_arm_moveit_config exotica.launch.py hardware_type:=fake use_rviz:=true
```
Wait for `MoveGroup ready` in the log (~10 s).

**Terminal 2 — Run EXOTica test script (dry-run, IK only)**
```bash
ros2 run disassembly_skill test_exotica_planner \
  --ros-args -p execute:=false -p tests:=T1,T4
```
Expected: T1 IK success for UF850, T4 IK success for xArm5 with orientation residual warning.

**Terminal 2 — Run EXOTica test script (execute trajectories)**
```bash
ros2 run disassembly_skill test_exotica_planner \
  --ros-args -p execute:=true -p hardware_type:=fake -p tests:=T1,T2,T3,T4,T5
```
Expected: smooth 50-point quintic trajectory execution for both arms. Check RViz for motion.

---

## 2. Tactile Descent — Fake Hardware

Tests the 50 Hz IK streaming loop. In fake mode the arm will move but no contact can be
detected, so descent will complete the full distance.

```bash
ros2 run disassembly_skill test_exotica_planner \
  --ros-args -p execute:=true -p hardware_type:=fake -p tests:=T3
```
Expected: smooth 20 mm Z descent at step_m=0.0005, rate_hz=50. No violent pecking.

---

## 3. Individual Skills — Fake Hardware

Each skill node waits for `/object_hold_state/is_held` before starting (except unscrew).
Use a separate terminal to manually publish the hold state trigger.

**Terminal 1 — MoveIt stack**
```bash
ros2 launch dual_arm_moveit_config exotica.launch.py hardware_type:=fake
```

**Terminal 2 — Skill node**
```bash
# Object Hold (approach + tactile descent + grasp)
ros2 run disassembly_skill object_hold_skill

# Unscrew (requires vision + FT data; fake mode will attempt without them)
ros2 run disassembly_skill unscrew_skill

# Object Flip (waits for is_held=true)
ros2 run disassembly_skill object_flip_skill

# Flip-Drop (waits for is_held=true)
ros2 run disassembly_skill object_flip_drop_skill

# Pickup (waits for is_held=true)
ros2 run disassembly_skill object_pickup_skill
```

**Terminal 3 — Trigger hold state (for skills that wait)**
```bash
ros2 topic pub --once /object_hold_state/is_held std_msgs/Bool "data: true"
```

---

## 4. Full System — Real Hardware

> **Safety checklist before running on real hardware:**
> - [ ] Workspace clear, no obstacles within arm reach
> - [ ] E-stop accessible
> - [ ] UF850 at 192.168.1.195, xArm5 at 192.168.1.239 (ping both)
> - [ ] RG6 gripper powered
> - [ ] FT300 on /dev/ttyACM0
> - [ ] RealSense connected
> - [ ] Vision model weights present at `dev_ws/src/disassembly_pipeline/vision_training/...`
>   (symlink from disassembly_ws if needed — see model path bug in architecture memory)

**Terminal 1 — Full bringup**
```bash
ros2 launch disassembly_skill disassembly_system.launch.py hardware_type:=real
```

Bringup stages and expected delays:
| Time | Stage | Log indicator |
|------|-------|---------------|
| 0 s  | EXOTica/MoveIt | `MoveGroup ready` |
| +5 s | Hand-eye TF publisher | `Publishing hand-eye transform` |
| +7 s | FT300 sensor | `FT300 connected` |
| +10 s| Vision system | `VisionAgent initialized` |
| +18 s| Debug feed republisher | `debug_feed_republisher started` |
| +20 s| rqt_image_view | GUI window appears |

**Terminal 2 — EXOTica planner test on real hardware (after bringup)**
```bash
ros2 run disassembly_skill test_exotica_planner \
  --ros-args -p execute:=true -p hardware_type:=real -p tests:=T1,T2,T4,T5
```
Run T1/T4 (IK only) first, then T2/T5 (execution) once IK looks correct.

**Terminal 2 — Start master agent (LLM orchestration)**
```bash
# OpenAI (gpt-5.1)
ros2 run disassembly_skill master_agent

# OR Groq (free-tier, llama-3.3-70b)
ros2 run disassembly_skill groq_master_agent
```

---

## 5. Monitoring During Execution

**Watch robot state**
```bash
ros2 topic echo /robot_state/manip_arm/update
```

**Watch hold state**
```bash
ros2 topic echo /object_hold_state/is_held
```

**Watch joint states (verify smooth trajectories)**
```bash
ros2 topic echo /joint_states
```

**Watch effort/torque (tactile descent)**
```bash
ros2 topic echo /uf850_arm/joint_efforts   # or equivalent topic
```

**View annotated vision feed**
```bash
ros2 run rqt_image_view rqt_image_view /vision/debug_feed_view
```

**Force-torque live feed**
```bash
ros2 topic echo /robotiq_ft_wrench
```

---

## 6. What to Check After Each Test

| Test | Pass Criteria |
|------|--------------|
| T1/T4 IK solve | Log: `IK solve OK`, joint values printed, no `IK failed` |
| T2/T5 pose execution | RViz shows smooth arc motion, no step jumps; log: `trajectory OK, N=50 points` |
| T3 tactile descent | 50 Hz loop visible in log, step=0.0005m, smooth Z approach; stops on contact or full distance |
| Full skill sequence | Each skill log shows `SUCCESS`, `/object_hold_state/is_held` toggles correctly |
| No servo mode used | No `start_servo` / `stop_servo` calls in skill logs — all motion through JTC trajectories |

---

## 7. Known Issues to Watch For

| Issue | What to Look For | Workaround |
|-------|-----------------|------------|
| xArm5 5-DOF orientation residual | Log: `xArm5 orientation residual` warning in T4/T5 | Expected; EXOTica finds best-effort |
| EXOTica fallback to MoveIt | Log: `WARN: EXOTica unavailable, falling back` | Check exotica node is running |
| Vision model path wrong | `FileNotFoundError` in vision log | Symlink dev_ws model weights |
| Hand-eye w=0.0 | Visual offset between detected object and arm target | Recalibrate with easy_handeye2 |
| tool_commander FAKE mode | `[FAKE MODE]` in tool_commander log at startup | Check /dev/ttyACM0 connection |
