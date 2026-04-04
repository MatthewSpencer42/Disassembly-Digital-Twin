# Dual-Arm Disassembly Execution — Deep-Scan Analysis Report

**Project:** Agentic HDD Disassembly System
**Scan Date:** 2026-04-01
**Scope:** Full execution logic — motion planning, arm coordination, success path, failure handling

---

## Table of Contents

1. [Core Motion Planning Algorithm](#1-core-motion-planning-algorithm)
2. [Intelligence Layer](#2-intelligence-layer)
3. [Order of Operations Between the Two Arms](#3-order-of-operations-between-the-two-arms)
4. [Success Logic — Perfect Execution Path](#4-success-logic--perfect-execution-path)
5. [Failure Logic — Detection and Recovery](#5-failure-logic--detection-and-recovery)
6. [Key Constraints Summary](#6-key-constraints-summary)

---

## 1. Core Motion Planning Algorithm

- **Primary algorithm**: Gradient-descent Inverse Kinematics (IK) operating in real time at 50 Hz
- The IK solver is **warm-started** from the arm's previous joint configuration at every cycle, giving it sub-millisecond convergence and smooth, continuous motion — the same pattern used in teleoperation and haptic feedback systems
- Each 50 Hz cycle follows this pipeline:
  1. Compute target Cartesian pose from sensor data or closed-loop function
  2. Solve IK from the warm-start (previous joint state)
  3. Blend solution with previous joints using exponential smoothing in joint-space
  4. Send blended joint command to the hardware controller
- **Step clamping** limits the maximum Cartesian displacement per cycle (~3 mm) as a hard safety ceiling, preventing large jumps
- **Point-to-point transits** (hover approach, bin navigation) use a separate offline trajectory planner that generates a full time-parameterised path before any motion begins — entirely distinct from the streaming loop
- The **hold arm** uses a different sub-algorithm: stepped tactile descent — discrete incremental joint-space moves with a torque check after each step, stopping on contact

---

## 2. Intelligence Layer

- A Large Language Model (GPT-5.1 via OpenAI Responses API) acts as the top-level orchestrator
- It reads the full parsed vision scene description on every iteration of its reasoning loop
- It operates on a **ReAct loop** (Reason → Plan → Action → Observe → repeat) with a hard recursion limit of 400 iterations
- **Mandatory rule embedded in the system prompt**: all screws visible in the scene must be removed before any structural component extraction begins
- After each macro-component removal (lid, module, frame), the LLM verifies the part is gone from the next vision frame; if still present, it retries the action automatically
- A **spatial exclusion memory** logs the 3-D coordinates of every completed screw to suppress ghost re-detections within a 10 mm radius sphere around each cleared location

---

## 3. Order of Operations Between the Two Arms

The two arms operate in **strictly serial** sequence — one acts while the other holds or waits.

```
UF850 (Manipulation Arm / Gripper)      xArm5 (Tool Arm / Screwdriver)
────────────────────────────────        ──────────────────────────────
[1] HOLD
    Open gripper
    Plan hover above chassis
    Tactile descent until contact        — idle —
    Hold state locked & broadcast

[2] — holding —                         UNSCREW  (repeated per screw)
                                           Pre-lift 30 mm
                                           Plan hover above screw target
                                           Real-time IK descent + visual align
                                           Contact detected → seating check
                                           Unscrew motor → grab screw
                                           Force-compliant extraction
                                           Navigate to disposal bin → drop

[3] FLIP / PICKUP                        — idle —
    (after all screws cleared)
    Flip chassis for underside access
    OR precision-grip PCB components

[4] FLIP-DROP                            — idle —
    Invert chassis to dump loose parts
    Re-grip chassis after dump
```

---

## 4. Success Logic — Perfect Execution Path

### Pre-flight Checks

- Vision system confirms the chassis and all screw targets are detected with valid 3-D coordinates
- Disposal bin location is confirmed and cached from ArUco marker detection
- Reach radius for every target is verified against the arm's kinematic limit before any motion begins

---

### Hold Phase — Manipulation Arm

1. Gripper opens to full width
2. Arm plans and executes a collision-free path to a hover pose 50 mm above the chassis centroid
3. Stepped descent begins — each step moves ~0.5 mm downward; joint torque is checked after the arm settles
4. When wrist torque exceeds the contact threshold, descent stops immediately
5. Hold state is broadcast on a latched channel and persisted to a shared state file accessible to all downstream skills

---

### Unscrew Phase — Tool Arm (repeated per screw)

| Step | What Happens |
|------|-------------|
| **1. Safe transit lift** | Arm lifts 30 mm from current position to clear any obstacles |
| **2. Hover approach** | Full offline trajectory plan executed to position screwdriver above target at hover height = surface + tool length + 5 mm |
| **3. Visual descent** | Real-time IK streaming at 50 Hz begins; camera pixel error converted to Cartesian XY correction per cycle; Z descends only when pixel error is below 30 px |
| **4. Contact detection** | Force-torque sensor detects surface contact; IK streaming stops |
| **5. Surface alignment** | XY nudges applied at 50 Hz for up to 3 s to precisely centre the bit on the screw head |
| **6. Seating check** | Screwdriver motor rotates briefly; torque spike above 3 N confirms the bit is properly seated in the slot |
| **7. Unscrew + grab** | Motor runs to fully unthread the screw; grab mechanism engages simultaneously |
| **8. Compliant extraction** | Upward force (measured live) × compliance gain = proportional Z-lift velocity; continues until force stabilises for 3 consecutive seconds (screw is free) |
| **9. Retract & dispose** | Tool and screw retract 15 mm; arm plans path to bin; screw is released; arm returns to neutral |

---

### Disposal Phase — Manipulation Arm

- After all screws are cleared, the chassis is flipped to expose the underside for the next pass
- PCB components are precision-gripped with the manipulation arm before any chassis inversion
- Flip-drop inverts the chassis to clear all loose parts; chassis is re-gripped afterwards to restore workspace

---

## 5. Failure Logic — Detection and Recovery

### Detection Signal Reference

| Signal | Source | What It Indicates |
|--------|--------|-------------------|
| Target not in vision data | Vision system | Part moved, occluded, or already removed |
| Reach radius exceeded | Kinematic check | Target is physically unreachable |
| Planning timeout / no solution | Trajectory planner | Configuration-space obstacle or near-singularity |
| Force-torque spike during descent | Force-torque sensor | Surface contact confirmed |
| Descent timer expired (15 s) | Timeout counter | Screw not found — spiral search exhausted |
| Torque spike during seating check < 3 N | Force-torque sensor | Bit not seated in slot — misalignment |
| Extraction force plateau (3 s no increase) | Force-torque sensor | Screw is free — extraction complete |
| Extraction timer expired (20 s) | Timeout counter | Screw is mechanically stuck |
| Part still visible after removal attempt | Vision verification (LLM) | Physical removal did not succeed |

---

### Recovery Flow A — Immediate Abort (no retry)

Triggered by: missing vision data, target not found, reach limit exceeded, or hover planning failure.

```
Condition detected (pre-motion)
  → No arm motion is initiated
  → Log error condition
  → Return failure to LLM orchestrator
  → LLM decides whether to retry or skip
```

Triggered by: hover approach planning fails (post-lift).

```
Planning failure detected (in motion)
  → Execute safety lift to transit height (30 mm)
  → Log error
  → Return failure to LLM orchestrator
```

---

### Recovery Flow B — Seating Retry (up to 3 attempts)

Triggered by: seating check torque spike is below the 3 N threshold (bit missed the slot).

```
Seating check fails
  ├── Retract screwdriver 5 mm upward (EXOTica Z-lift)
  ├── Reset spiral search state to centre origin
  ├── Reset force-torque baseline
  ├── Restart real-time IK descent with a fresh spiral pattern
  └── If retry count exceeds 3:
        → Execute safety lift to transit height
        → Navigate to disposal bin (workspace clearance)
        → Return failure to LLM orchestrator
```

---

### Recovery Flow C — Descent Timeout / Safe Halt

Triggered by: the spiral search timer (15 s) expires without detecting surface contact.

```
Timeout detected
  → Stop real-time IK streaming immediately
  → Execute safety lift to transit height (30 mm)
  → Plan and execute path to disposal bin zone (workspace clearance)
  → Return failure to LLM orchestrator
```

---

### Recovery Flow D — LLM-Level Retry

Triggered by: vision frame after a removal attempt still shows the target part.

```
LLM observes part still present in scene
  → Re-issues the identical tool call (unscrew / pickup / flip_drop)
  → Continues looping until:
      (a) Part disappears from the vision scene, or
      (b) Recursion limit (400 iterations) is reached → mission abort
```

---

### Recovery Flow E — Gripper Stall

Triggered by: gripper does not reach its commanded position within the timeout window.

```
Gripper stall detected on open command
  → Re-issue open command at maximum allowable force (100 N)
  → Wait an additional 2-second window for movement
  → Log warning if still stalled
```

---

## 6. Key Constraints Summary

- **Serial arm coordination**: the screwdriver arm never moves while the hold arm is descending, and vice versa — sequencing is strictly serial within each skill invocation
- **Startup inhibit**: no motion command is sent to either arm during the first 8 seconds after node startup (hardware safety delay to allow controller initialisation)
- **Jump guard**: any single joint command that would move more than 0.4 radians from the current position is silently dropped — prevents jitter or stale-command disasters
- **XY descent gating**: the Z-axis only descends when the visual pixel error is below 30 px; coarse alignment must be achieved before closing with the surface
- **Spatial exclusion memory**: a 10 mm spherical exclusion zone is logged around each successfully removed screw to prevent the vision system from reporting false re-detections of already-cleared locations
- **Hold-state prerequisite**: the manipulation arm broadcasts a persistent "is held" boolean on a latched channel; all downstream skills (unscrew, flip, pickup) are expected to confirm this state before proceeding
- **Screw-first mandate**: the LLM's system prompt enforces that all screws must be unthreaded before any lid, frame, or module extraction is attempted

---

*Report generated from static source analysis of `unscrew_skill.py`, `object_hold_skill.py`, `object_flip_skill.py`, `master_agent.py`, and `motion_backend.py`.*
