# Graph Report - .  (2026-04-30)

## Corpus Check
- Large corpus: 1449 files · ~21,352,380 words. Semantic extraction will be expensive (many Claude tokens). Consider running on a subfolder, or use --no-semantic to run AST-only.

## Summary
- 3055 nodes · 6322 edges · 51 communities detected
- Extraction: 62% EXTRACTED · 38% INFERRED · 0% AMBIGUOUS · INFERRED: 2404 edges (avg confidence: 0.78)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 7|Community 7]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]
- [[_COMMUNITY_Community 11|Community 11]]
- [[_COMMUNITY_Community 12|Community 12]]
- [[_COMMUNITY_Community 13|Community 13]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 17|Community 17]]
- [[_COMMUNITY_Community 18|Community 18]]
- [[_COMMUNITY_Community 19|Community 19]]
- [[_COMMUNITY_Community 20|Community 20]]
- [[_COMMUNITY_Community 21|Community 21]]
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 24|Community 24]]
- [[_COMMUNITY_Community 25|Community 25]]
- [[_COMMUNITY_Community 26|Community 26]]
- [[_COMMUNITY_Community 27|Community 27]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]
- [[_COMMUNITY_Community 31|Community 31]]
- [[_COMMUNITY_Community 33|Community 33]]
- [[_COMMUNITY_Community 34|Community 34]]
- [[_COMMUNITY_Community 35|Community 35]]
- [[_COMMUNITY_Community 36|Community 36]]
- [[_COMMUNITY_Community 37|Community 37]]
- [[_COMMUNITY_Community 39|Community 39]]
- [[_COMMUNITY_Community 40|Community 40]]
- [[_COMMUNITY_Community 42|Community 42]]
- [[_COMMUNITY_Community 44|Community 44]]
- [[_COMMUNITY_Community 45|Community 45]]
- [[_COMMUNITY_Community 46|Community 46]]
- [[_COMMUNITY_Community 47|Community 47]]
- [[_COMMUNITY_Community 48|Community 48]]
- [[_COMMUNITY_Community 49|Community 49]]
- [[_COMMUNITY_Community 50|Community 50]]
- [[_COMMUNITY_Community 51|Community 51]]
- [[_COMMUNITY_Community 55|Community 55]]
- [[_COMMUNITY_Community 57|Community 57]]
- [[_COMMUNITY_Community 179|Community 179]]

## God Nodes (most connected - your core abstractions)
1. `Get()` - 134 edges
2. `resize()` - 100 edges
3. `MotionBackend` - 72 edges
4. `SetZero()` - 67 edges
5. `Update()` - 60 edges
6. `Print()` - 59 edges
7. `type()` - 51 edges
8. `norm()` - 51 edges
9. `shutdown_on_exit()` - 49 edges
10. `ImagePrepApp` - 47 edges

## Surprising Connections (you probably didn't know these)
- `Params::load()` --calls--> `type()`  [INFERRED]
  /home/adip/workspace/disassembly_ws/src/agentic_disassembly/camera_calibaration/aruco_ros/aruco/src/aruco/markerdetector.cpp → /home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/disassembly_skill/device_config.py
- `IsStringType()` --calls--> `type()`  [INFERRED]
  /home/adip/workspace/disassembly_ws/src/agentic_disassembly/exotica/exotica_core/src/property.cpp → /home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/disassembly_skill/device_config.py
- `IsInitializerVectorType()` --calls--> `type()`  [INFERRED]
  /home/adip/workspace/disassembly_ws/src/agentic_disassembly/exotica/exotica_core/src/property.cpp → /home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/disassembly_skill/device_config.py
- `findInnerCorners()` --calls--> `copyTo()`  [INFERRED]
  /home/adip/workspace/disassembly_ws/src/agentic_disassembly/camera_calibaration/aruco_ros/aruco/src/aruco/fractallabelers/fractalmarker.cpp → /home/adip/workspace/disassembly_ws/src/agentic_disassembly/camera_calibaration/aruco_ros/aruco/src/aruco/marker.cpp
- `drawMarkers()` --calls--> `draw()`  [INFERRED]
  /home/adip/workspace/disassembly_ws/src/agentic_disassembly/camera_calibaration/aruco_ros/aruco/src/aruco/fractaldetector.cpp → /home/adip/workspace/disassembly_ws/src/agentic_disassembly/camera_calibaration/aruco_ros/aruco/src/aruco/marker.cpp

## Communities

### Community 0 - "Community 0"
Cohesion: 0.01
Nodes (104): main(), DeviceConfig, dict, ExoticaIKServerNode, Receive a JSON IK request and dispatch it to the thread pool., Resolve an IK request and publish the response. Runs in the thread pool., Pre-warms EXOTica IK planners for both arms and serves IK requests over topics., Initialize both EXOTica planners sequentially in a background thread. (+96 more)

### Community 1 - "Community 1"
Cohesion: 0.02
Nodes (231): ForwardPass(), GetFeedbackControl(), Solve(), SpecifyProblem(), GetDuration(), ReinitializeVariables(), SetInitialTrajectory(), SetJointVelocityLimits() (+223 more)

### Community 2 - "Community 2"
Cohesion: 0.02
Nodes (168): arucoMarker2Tf2(), detectMarkers(), rosCameraInfo2ArucoCamParams(), argConvGLcpara2(), arParamDecompMat(), CameraParameters(), dot(), getRTMatrix() (+160 more)

### Community 3 - "Community 3"
Cohesion: 0.02
Nodes (143): AssignScene(), Initialize(), Update(), AssignScene(), Initialize(), Update(), UpdateInternal(), CheckCollision() (+135 more)

### Community 4 - "Community 4"
Cohesion: 0.02
Nodes (83): generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description(), generate_launch_description() (+75 more)

### Community 5 - "Community 5"
Cohesion: 0.02
Nodes (86): Instantiate(), PublishObjectsAsMarkerArray(), Update(), UpdateAsCostWithJacobian(), UpdateAsInequalityConstraintWithJacobian(), AssignScene(), Initialize(), InitializeDebug() (+78 more)

### Community 6 - "Community 6"
Cohesion: 0.02
Nodes (41): _get_opencv_samples(), HandeyeCalibrationBackendOpenCV, _msg_to_opencv(), Computes the calibration through the OpenCV library and returns it.          :rt, getchar(), HandeyeCalibrationCommander, main(), filepath_for_calibration() (+33 more)

### Community 7 - "Community 7"
Cohesion: 0.03
Nodes (59): EvaluateTrajectory(), GetTaskCosts(), InitMessages(), InitTrajectory(), PerhapsUndoStep(), RememberOldState(), Step(), UpdateBwdMessage() (+51 more)

### Community 8 - "Community 8"
Cohesion: 0.02
Nodes (50): main(), ClassifierVisionNode, main(), Setup(), DashboardNode, main(), _scale_box(), _scale_point() (+42 more)

### Community 9 - "Community 9"
Cohesion: 0.03
Nodes (79): GetBounds(), GetGoal(), GetScalarTaskCost(), IsValid(), PreUpdate(), SetRho(), Update(), add() (+71 more)

### Community 10 - "Community 10"
Cohesion: 0.05
Nodes (27): ExoticaArmTeleop, Disable teleop, move arms to home pose in background, clear calibration., clamp(), mat_vec_mul(), matrix_to_quat(), quat_conjugate(), quat_multiply(), quat_normalize() (+19 more)

### Community 11 - "Community 11"
Cohesion: 0.04
Nodes (61): SpecifyProblem(), main(), what(), load(), main(), run(), GetRandomControlledState(), main() (+53 more)

### Community 12 - "Community 12"
Cohesion: 0.06
Nodes (30): copy(), _btn(), ImagePrepApp, main(), Navigation row + Save / Save Copy buttons., Operations row: resize | crop | image adjustments., Stretch to res×res square (may distort aspect ratio)., Scale so the longest side = res; shorter side proportional. (+22 more)

### Community 13 - "Community 13"
Cohesion: 0.04
Nodes (36): applyConfiguration(), checkJointMoved(), computeApproximateMutation1(), computeJacobian(), getJointFrame(), change(), concat(), frameToKDL() (+28 more)

### Community 14 - "Community 14"
Cohesion: 0.06
Nodes (45): get_ct(), GetCost(), GetCostJacobian(), GetEquality(), GetEqualityJacobian(), GetEqualityJacobianTriplets(), GetGoal(), GetGoalEQ() (+37 more)

### Community 15 - "Community 15"
Cohesion: 0.04
Nodes (17): InstantiateBase(), set_replace_cylinders_with_capsules(), SetACM(), SetAlwaysExternallyUpdatedCollisionScene(), SetReplacePrimitiveShapesWithMeshes(), SetRobotLinkPadding(), SetRobotLinkScale(), SetWorldLinkPadding() (+9 more)

### Community 16 - "Community 16"
Cohesion: 0.09
Nodes (38): on_deactivate(), read_background(), rq_com_compute_crc(), rq_com_do_zero_force_flag(), rq_com_get_received_data(), rq_com_get_str_firmware_version(), rq_com_get_str_production_year(), rq_com_get_str_serial_number() (+30 more)

### Community 17 - "Community 17"
Cohesion: 0.11
Nodes (5): main(), Teleop Control Panel — Tkinter GUI node for arm teleop control.  Buttons:   Righ, TeleopControlPanel, TeleopControlPanelNode, TestUnknownInitializerTypes

### Community 18 - "Community 18"
Cohesion: 0.12
Nodes (4): cross(), f(), QuadrotorDynamicsSolver(), WebcamHandTracker

### Community 19 - "Community 19"
Cohesion: 0.1
Nodes (7): AngleStabilizer, calculate_orientation_pca(), Point3DStabilizer, StaticAnchorTracker, GlobalVisionNode, Returns list of detected objects with bounding boxes and debug image., ScoutAgent

### Community 20 - "Community 20"
Cohesion: 0.09
Nodes (9): aruco(), getCandidates(), getImagePyramid(), getMarkerLabeler(), getParameters(), MarkerDetector(), Params::load(), setDetectionMode() (+1 more)

### Community 21 - "Community 21"
Cohesion: 0.11
Nodes (12): CalibrationMovements, _compute_poses_around_state(), _is_crazy_plan(), quaternion_from_euler(), quaternion_multiply(), # TODO: accept a list of delta values, # TODO: make repeatable, # TODO: use joint position http://docs.ros.org/melodic/api/moveit_tutorials/html (+4 more)

### Community 22 - "Community 22"
Cohesion: 0.08
Nodes (4): exotica(), Instantiate(), IsMultiQuery(), LazyPRMSolver()

### Community 23 - "Community 23"
Cohesion: 0.11
Nodes (10): launch_setup(), launch_setup(), ExoticaDualArmPosePlanner, _launch_setup(), _launch_setup(), _launch_setup(), joint_topics_for_hardware(), normalize_xacro_hardware_type() (+2 more)

### Community 24 - "Community 24"
Cohesion: 0.1
Nodes (8): GetEquality(), GetInequality(), IsValid(), PreUpdate(), SetRho(), SetRhoEQ(), SetRhoNEQ(), Update()

### Community 25 - "Community 25"
Cohesion: 0.08
Nodes (3): Eigen(), Eigen(), operator()

### Community 26 - "Community 26"
Cohesion: 0.14
Nodes (7): GetBounds(), IsStateValid(), IsValid(), PreUpdate(), SetRhoEQ(), SetRhoNEQ(), Update()

### Community 27 - "Community 27"
Cohesion: 0.24
Nodes (1): TeleopBridge

### Community 28 - "Community 28"
Cohesion: 0.24
Nodes (2): CameraCropRepublisher, MutuallyExclusiveCallbackGroup guarantees this never runs concurrently         w

### Community 29 - "Community 29"
Cohesion: 0.14
Nodes (7): GetTaskError(), GetHessian(), GetRho(), GetScalarTaskCost(), PreUpdate(), SetRho(), Update()

### Community 30 - "Community 30"
Cohesion: 0.19
Nodes (9): Random(), check_derivative_1d(), num_diff_1d(), test_huber(), test_smooth_l1(), random_quaternion(), random_state(), # TODO: Verify (+1 more)

### Community 31 - "Community 31"
Cohesion: 0.25
Nodes (2): ParseBool(), ParseBoolList()

### Community 33 - "Community 33"
Cohesion: 0.25
Nodes (2): exotica(), get_h()

### Community 34 - "Community 34"
Cohesion: 0.33
Nodes (4): KinematicsQueryOptions(), BioIKKinematicsQueryOptions(), isBioIKKinematicsQueryOptions(), toBioIKKinematicsQueryOptions()

### Community 35 - "Community 35"
Cohesion: 0.33
Nodes (2): _develop, DevelopCommand

### Community 36 - "Community 36"
Cohesion: 0.33
Nodes (1): AssignScene()

### Community 37 - "Community 37"
Cohesion: 0.4
Nodes (1): test_flake8()

### Community 39 - "Community 39"
Cohesion: 0.5
Nodes (2): add(), addspaces()

### Community 40 - "Community 40"
Cohesion: 0.5
Nodes (1): Python helpers for the dual-arm MoveIt configuration package.

### Community 42 - "Community 42"
Cohesion: 0.67
Nodes (2): TaskSpaceDim(), Update()

### Community 44 - "Community 44"
Cohesion: 1.0
Nodes (2): _find_repo_root(), setup_python_env()

### Community 45 - "Community 45"
Cohesion: 0.67
Nodes (2): generate_launch_description(), Launch the Orbbec Femto Bolt tuned for the crop+vision pipeline.      Key change

### Community 46 - "Community 46"
Cohesion: 0.67
Nodes (1): WrenchSimulator

### Community 47 - "Community 47"
Cohesion: 1.0
Nodes (2): Eigen(), operator()

### Community 48 - "Community 48"
Cohesion: 1.0
Nodes (2): Eigen(), operator()

### Community 49 - "Community 49"
Cohesion: 1.0
Nodes (2): Eigen(), operator()

### Community 50 - "Community 50"
Cohesion: 1.0
Nodes (2): Eigen(), operator()

### Community 51 - "Community 51"
Cohesion: 0.67
Nodes (2): Maps 0-180 degrees to Pico duty cycle., set_servo_angle()

### Community 55 - "Community 55"
Cohesion: 1.0
Nodes (1): RobotiqFTSensorHardware

### Community 57 - "Community 57"
Cohesion: 1.0
Nodes (1): Tkinter styling constants for the device config builder.

### Community 179 - "Community 179"
Cohesion: 1.0
Nodes (1): Returns the sample list as a rotation matrix and a translation vector.

## Knowledge Gaps
- **76 isolated node(s):** `MutuallyExclusiveCallbackGroup guarantees this never runs concurrently         w`, `Returns structured dict with screw_heads, tool_tips, and holes.`, `Returns detected objects with bounding boxes and simulated segments for agent_no`, `Returns status of the assembly (pass/fail/step_name).`, `Returns list of detected objects with bounding boxes and debug image.` (+71 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Community 27`** (17 nodes): `TeleopBridge`, `._after_switch()`, `.apply_deadzone()`, `._button_edge()`, `.continuous_twist_publisher()`, `._handle_list_controllers()`, `.__init__()`, `.initial_sync_timer_callback()`, `.joy_callback()`, `._log_mapping()`, `.manage_controller_mode()`, `.process_pending_home()`, `._publish_tool_cmd()`, `._publish_zero_twist()`, `._queue_servo_halt()`, `.send_traj_goal()`, `.start_active_servo()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 28`** (16 nodes): `CameraCropRepublisher`, `._accept_color_source()`, `._build_pointcloud()`, `.cb_color_camera_info()`, `.cb_color_compressed()`, `.cb_color_raw()`, `.cb_depth_camera_info()`, `.cb_depth_image()`, `._crop_camera_info()`, `._get_crop_roi()`, `.__init__()`, `._log_roi_once()`, `._publish_cropped_color()`, `._resize_for_display()`, `._resize_for_inference()`, `MutuallyExclusiveCallbackGroup guarantees this never runs concurrently         w`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 31`** (9 nodes): `Eigen()`, `exotica()`, `ParseBool()`, `ParseBoolList()`, `ParseDouble()`, `ParseInt()`, `ParseIntList()`, `Trim()`, `conversions.h`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 33`** (8 nodes): `joint_torque_minimization_proxy.h`, `joint_torque_minimization_proxy.cpp`, `exotica()`, `get_h()`, `Instantiate()`, `set_h()`, `TaskSpaceDim()`, `Update()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 35`** (6 nodes): `setup.py`, `_develop`, `setup.py`, `DevelopCommand`, `.initialize_options()`, `package_files()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 36`** (6 nodes): `pendulum_dynamics_solver.cpp`, `AssignScene()`, `f()`, `fu()`, `fx()`, `PendulumDynamicsSolver()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 37`** (5 nodes): `test_flake8.py`, `test_flake8()`, `test_flake8.py`, `test_flake8.py`, `test_flake8.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 39`** (5 nodes): `timers.h`, `add()`, `addspaces()`, `aruco()`, `__pf_aruco_methodName()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 40`** (4 nodes): `__init__.py`, `__init__.py`, `__init__.py`, `Python helpers for the dual-arm MoveIt configuration package.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 42`** (4 nodes): `manipulability.cpp`, `Instantiate()`, `TaskSpaceDim()`, `Update()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 44`** (3 nodes): `_find_repo_root()`, `setup_python_env()`, `runtime_env.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 45`** (3 nodes): `generate_launch_description()`, `Launch the Orbbec Femto Bolt tuned for the crop+vision pipeline.      Key change`, `orbbec_camera.launch.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 46`** (3 nodes): `wrench_simulator.hpp`, `WrenchSimulator`, `.WrenchSimulator()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 47`** (3 nodes): `Eigen()`, `operator()`, `autodiff_chain_hessian.h`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 48`** (3 nodes): `Eigen()`, `operator()`, `autodiff_chain_hessian_sparse.h`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 49`** (3 nodes): `Eigen()`, `operator()`, `autodiff_chain_jacobian_sparse.h`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 50`** (3 nodes): `Eigen()`, `operator()`, `autodiff_chain_jacobian.h`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 51`** (3 nodes): `Maps 0-180 degrees to Pico duty cycle.`, `set_servo_angle()`, `rpi_pico_main.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 55`** (2 nodes): `RobotiqFTSensorHardware`, `robotiq_ft_sensor_hardware.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 57`** (2 nodes): `styles.py`, `Tkinter styling constants for the device config builder.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 179`** (1 nodes): `Returns the sample list as a rotation matrix and a translation vector.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Get()` connect `Community 0` to `Community 1`, `Community 3`, `Community 6`, `Community 7`, `Community 8`, `Community 9`, `Community 10`, `Community 11`, `Community 12`, `Community 13`, `Community 17`, `Community 18`, `Community 19`, `Community 23`?**
  _High betweenness centrality (0.180) - this node is a cross-community bridge._
- **Why does `resize()` connect `Community 1` to `Community 2`, `Community 3`, `Community 4`, `Community 5`, `Community 8`, `Community 9`, `Community 12`, `Community 13`, `Community 14`, `Community 18`, `Community 25`, `Community 26`, `Community 28`?**
  _High betweenness centrality (0.149) - this node is a cross-community bridge._
- **Why does `PYBIND11_MODULE()` connect `Community 3` to `Community 0`, `Community 1`, `Community 5`, `Community 8`, `Community 11`?**
  _High betweenness centrality (0.057) - this node is a cross-community bridge._
- **Are the 131 inferred relationships involving `Get()` (e.g. with `.processing_loop()` and `.publish_agent_state()`) actually correct?**
  _`Get()` has 131 INFERRED edges - model-reasoned connections that need verification._
- **Are the 99 inferred relationships involving `resize()` (e.g. with `.cb_local()` and `._resize_for_inference()`) actually correct?**
  _`resize()` has 99 INFERRED edges - model-reasoned connections that need verification._
- **Are the 73 inferred relationships involving `Node` (e.g. with `generate_launch_description()` and `generate_launch_description()`) actually correct?**
  _`Node` has 73 INFERRED edges - model-reasoned connections that need verification._
- **Are the 30 inferred relationships involving `MotionBackend` (e.g. with `Compatibility wrapper for the updated dual-arm MoveIt backend.` and `MotionBackend`) actually correct?**
  _`MotionBackend` has 30 INFERRED edges - model-reasoned connections that need verification._