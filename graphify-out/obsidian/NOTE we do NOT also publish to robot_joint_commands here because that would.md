---
source_file: "/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/motion_backend.py"
type: "rationale"
community: "Community 0"
location: "L380"
tags:
  - graphify/rationale
  - graphify/INFERRED
  - community/Community_0
---

# # NOTE: we do NOT also publish to /robot_joint_commands here because that would

## Connections
- [[ExoticaDualArmPlanner]] - `uses` [INFERRED]
- [[ExoticaSingleArmPosePlanner]] - `uses` [INFERRED]
- [[RemoteExoticaIKClient]] - `uses` [INFERRED]
- [[motion_backend.py_1]] - `rationale_for` [EXTRACTED]

#graphify/rationale #graphify/INFERRED #community/Community_0