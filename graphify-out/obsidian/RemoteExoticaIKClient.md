---
source_file: "/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/exotica_planner.py"
type: "code"
community: "Community 0"
location: "L892"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Community_0
---

# RemoteExoticaIKClient

## Connections
- [[NOTE we do NOT also publish to robot_joint_commands here because that would]] - `uses` [INFERRED]
- [[.__init__()_59]] - `method` [EXTRACTED]
- [[.__init__()_55]] - `calls` [INFERRED]
- [[._check_joint_limits()_1]] - `method` [EXTRACTED]
- [[._estimate_segment_time()_2]] - `method` [EXTRACTED]
- [[._on_response()]] - `method` [EXTRACTED]
- [[._project_goal_state_near_reference()_2]] - `method` [EXTRACTED]
- [[._recover_planner_if_needed()]] - `calls` [INFERRED]
- [[._state_vector_from_joint_map()_3]] - `method` [EXTRACTED]
- [[._trajectory_to_robot_trajectory()_1]] - `method` [EXTRACTED]
- [[._wait_for_server()]] - `method` [EXTRACTED]
- [[._wrap_to_nearest()_2]] - `method` [EXTRACTED]
- [[.plan_pose_trajectory()_1]] - `method` [EXTRACTED]
- [[.solve_pose_goal_joint_positions()_1]] - `method` [EXTRACTED]
- [[Disable teleop, move arms to home pose in background, clear calibration.]] - `uses` [INFERRED]
- [[Drop-in replacement for ExoticaSingleArmPosePlanner that routes IK calls to the]] - `rationale_for` [EXTRACTED]
- [[ExoticaArmTeleop]] - `uses` [INFERRED]
- [[MotionBackend]] - `uses` [INFERRED]
- [[Move EE straight up by distance_m at speed_mps using EXOTica IK streaming.]] - `uses` [INFERRED]
- [[Stream EXOTica IK in a real-time loop (TouchLabteleoperation style).          t]] - `uses` [INFERRED]
- [[exotica_planner.py]] - `contains` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/Community_0