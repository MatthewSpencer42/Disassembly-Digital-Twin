---
source_file: "/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/motion_backend.py"
type: "rationale"
community: "Community 0"
location: "L1206"
tags:
  - graphify/rationale
  - graphify/INFERRED
  - community/Community_0
---

# Move EE straight up by distance_m at speed_mps using EXOTica IK streaming.

## Connections
- [[.retract_z_exotica()]] - `rationale_for` [EXTRACTED]
- [[ExoticaDualArmPlanner]] - `uses` [INFERRED]
- [[ExoticaSingleArmPosePlanner]] - `uses` [INFERRED]
- [[RemoteExoticaIKClient]] - `uses` [INFERRED]

#graphify/rationale #graphify/INFERRED #community/Community_0