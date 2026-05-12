---
source_file: "/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/exotica_arm_teleop.py"
type: "rationale"
community: "Community 10"
location: "L618"
tags:
  - graphify/rationale
  - graphify/INFERRED
  - community/Community_10
---

# Disable teleop, move arms to home pose in background, clear calibration.

## Connections
- [[._handle_go_home()]] - `rationale_for` [EXTRACTED]
- [[ExoticaSingleArmPosePlanner]] - `uses` [INFERRED]
- [[MotionBackend]] - `uses` [INFERRED]
- [[RemoteExoticaIKClient]] - `uses` [INFERRED]

#graphify/rationale #graphify/INFERRED #community/Community_10