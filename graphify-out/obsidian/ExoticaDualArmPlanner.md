---
source_file: "/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/exotica_planner.py"
type: "code"
community: "Community 0"
location: "L17"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Community_0
---

# ExoticaDualArmPlanner

## Connections
- [[NOTE we do NOT also publish to robot_joint_commands here because that would]] - `uses` [INFERRED]
- [[.__init__()_56]] - `method` [EXTRACTED]
- [[.__init__()_55]] - `calls` [INFERRED]
- [[._estimate_segment_time()]] - `method` [EXTRACTED]
- [[._generate_config()]] - `method` [EXTRACTED]
- [[._solution_to_robot_trajectory()]] - `method` [EXTRACTED]
- [[._state_vector_from_joint_map()]] - `method` [EXTRACTED]
- [[._target_vector()]] - `method` [EXTRACTED]
- [[.plan_joint_trajectory()]] - `method` [EXTRACTED]
- [[MotionBackend]] - `uses` [INFERRED]
- [[Move EE straight up by distance_m at speed_mps using EXOTica IK streaming.]] - `uses` [INFERRED]
- [[Stream EXOTica IK in a real-time loop (TouchLabteleoperation style).          t]] - `uses` [INFERRED]
- [[exotica_planner.py]] - `contains` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/Community_0