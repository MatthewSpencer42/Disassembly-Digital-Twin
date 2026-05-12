---
source_file: "/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/exotica_planner.py"
type: "code"
community: "Community 0"
location: "L457"
tags:
  - graphify/code
  - graphify/INFERRED
  - community/Community_0
---

# ExoticaSingleArmPosePlanner

## Connections
- [[NOTE we do NOT also publish to robot_joint_commands here because that would]] - `uses` [INFERRED]
- [[.__init__()_58]] - `method` [EXTRACTED]
- [[.__init__()_55]] - `calls` [INFERRED]
- [[._check_joint_limits()]] - `method` [EXTRACTED]
- [[._estimate_segment_time()_1]] - `method` [EXTRACTED]
- [[._generate_config()_2]] - `method` [EXTRACTED]
- [[._generate_urdf()_1]] - `method` [EXTRACTED]
- [[._init_planners()]] - `calls` [INFERRED]
- [[._project_goal_state_near_reference()_1]] - `method` [EXTRACTED]
- [[._recover_planner_if_needed()]] - `calls` [INFERRED]
- [[._state_vector_from_joint_map()_2]] - `method` [EXTRACTED]
- [[._trajectory_to_robot_trajectory()]] - `method` [EXTRACTED]
- [[._wrap_to_nearest()_1]] - `method` [EXTRACTED]
- [[.plan_pose_trajectory()]] - `method` [EXTRACTED]
- [[.solve_pose_goal_joint_positions()]] - `method` [EXTRACTED]
- [[Disable teleop, move arms to home pose in background, clear calibration.]] - `uses` [INFERRED]
- [[ExoticaArmTeleop]] - `uses` [INFERRED]
- [[ExoticaIKServerNode]] - `uses` [INFERRED]
- [[Initialize both EXOTica planners sequentially in a background thread.]] - `uses` [INFERRED]
- [[MotionBackend]] - `uses` [INFERRED]
- [[Move EE straight up by distance_m at speed_mps using EXOTica IK streaming.]] - `uses` [INFERRED]
- [[Pre-warms EXOTica IK planners for both arms and serves IK requests over topics.]] - `uses` [INFERRED]
- [[Receive a JSON IK request and dispatch it to the thread pool.]] - `uses` [INFERRED]
- [[Resolve an IK request and publish the response. Runs in the thread pool.]] - `uses` [INFERRED]
- [[Stream EXOTica IK in a real-time loop (TouchLabteleoperation style).          t]] - `uses` [INFERRED]
- [[exotica_planner.py]] - `contains` [EXTRACTED]

#graphify/code #graphify/INFERRED #community/Community_0