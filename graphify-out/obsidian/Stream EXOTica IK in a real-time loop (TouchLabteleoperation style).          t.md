---
source_file: "/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/motion_backend.py"
type: "rationale"
community: "Community 0"
location: "L1103"
tags:
  - graphify/rationale
  - graphify/INFERRED
  - community/Community_0
---

# Stream EXOTica IK in a real-time loop (TouchLab/teleoperation style).          t

## Connections
- [[.move_cartesian_realtime_exotica()]] - `rationale_for` [EXTRACTED]
- [[ExoticaDualArmPlanner]] - `uses` [INFERRED]
- [[ExoticaSingleArmPosePlanner]] - `uses` [INFERRED]
- [[RemoteExoticaIKClient]] - `uses` [INFERRED]

#graphify/rationale #graphify/INFERRED #community/Community_0