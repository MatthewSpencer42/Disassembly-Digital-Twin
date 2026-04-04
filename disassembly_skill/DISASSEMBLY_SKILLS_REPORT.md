# Disassembly Skills Package Analysis Report

## Current status

This report was originally generated from the older `dev_ws` package:

- `/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills`

The active package in this workspace is now:

- `/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill`

Important migration notes for the current workspace:

- package name is `disassembly_skill`, not `disassembly_skills`
- the motion backend now wraps the current `dual_arm_moveit_config` backend
- current robot names are `uf850_*`, `xarm5_*`, `rg6_right_drive_joint`, and `base_link`
- full bringup is now through `disassembly_skill/launch/disassembly_system.launch.py`
- current vision bringup uses the optimized `vision_agent` monolithic node plus a debug-feed republisher

Treat the rest of this document as historical package analysis and code archaeology, not as the authoritative runtime contract for the current workspace.

## Scope

This report analyzes the `disassembly_skills` ROS 2 Python package in:

- `/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills`

It covers:

- package structure
- runtime architecture
- each skill bot
- control flow
- success conditions
- failure conditions
- representative code excerpts
- orchestrators and launch/config assets

## Package Overview

The package is an `ament_python` ROS 2 package with multiple console entry points declared in [setup.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/setup.py). The main runtime nodes are:

- `motion_backend`
- `object_hold_skill`
- `object_pickup_skill`
- `object_flip_skill`
- `object_flip_drop_skill`
- `unscrew_skill`
- `master_agent`
- `groq_master_agent`
- `hdd_disassembly_script`

The package depends primarily on:

- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `moveit_msgs`
- `tf_transformations`

as declared in [package.xml](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/package.xml).

## High-Level Architecture

The package has three layers:

1. Motion layer
   - `MotionBackend` abstracts MoveIt, MoveIt Servo, TF, gripper force, and joint state tracking.
2. Skill layer
   - The skill bots call `MotionBackend` to execute concrete manipulation procedures.
3. Orchestration layer
   - `master_agent.py` and `groq_master_agent.py` use an LLM-driven ReAct loop.
   - `hdd_disassembly_script.py` is a deterministic state machine without an LLM.

## File Inventory

Core runtime modules:

- [motion_backend.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py)
- [object_hold_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_hold_skill.py)
- [object_pickup_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_pickup_skill.py)
- [object_flip_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_flip_skill.py)
- [object_flip_drop_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_flip_drop_skill.py)
- [unscrew_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py)
- [master_agent.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py)
- [groq_master_agent.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/groq_master_agent.py)
- [hdd_disassembly_script.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/hdd_disassembly_script.py)

Supporting assets:

- [README.md](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/README.md)
- [tags.yaml](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/config/tags.yaml)
- [apriltag.launch.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/launch/apriltag.launch.py)
- [vision_system.launch.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/launch/vision_system.launch.py)

## Shared Motion Layer: `MotionBackend`

Primary source: [motion_backend.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py)

### What it does

`MotionBackend` is the core hardware abstraction. It detects which robot group it controls and configures:

- UF850 arm behavior
- xArm5 behavior
- RG6 gripper behavior
- MoveIt action clients
- IK and Cartesian planning service clients
- MoveIt Servo start/stop services
- TF lookup and transforms
- joint state and effort monitoring

### Backend selection logic

Relevant code: [motion_backend.py#L19](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L19)

```python
if self.is_xarm5:
    self.backend_kind = "xarm5"
    self.prefix = "xarm"
    self.controller_name = "xarm_controller"
    self.servo_namespace = "/xarm_servo_node"
    self.default_ik_link = "xarm5_link5"
    self.joint_prefixes = ["xarm5", "slider"]
elif self.is_gripper:
    self.backend_kind = "rg6_gripper"
    self.prefix = "rg6"
    self.controller_name = "rg6_controller"
    self.servo_namespace = None
    self.default_ik_link = None
    self.joint_prefixes = ["rg6"]
else:
    self.backend_kind = "uf850"
    self.prefix = "uf"
    self.controller_name = "uf_controller"
    self.servo_namespace = "/uf_servo_node"
    self.default_ik_link = "u1_tool0"
    self.joint_prefixes = ["u1"]
```

Meaning:

- xArm5 gets servo and IK link `xarm5_link5`
- UF850 gets servo and IK link `u1_tool0`
- RG6 gripper has no servo, only joint/force control

### Shared TF model

Relevant code: [motion_backend.py#L92](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L92)

The node stores a shared TF buffer on the parent ROS node. This avoids creating multiple TF listeners for every backend instance, which is important because many skill nodes instantiate multiple backends in one process.

### Mode management

Relevant code:

- [motion_backend.py#L224](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L224)
- [motion_backend.py#L249](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L249)
- [motion_backend.py#L287](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L287)

Important design rule:

- planned motion and servo motion are treated as separate modes
- before planned motions, servo is flushed and stopped
- before servo motions, start service is called and warm-up zero twists are published

This is one of the main package stabilizers. Many skill methods explicitly call `stop_servo()` at entry to avoid inheriting servo state from a previous skill.

### Pose planning logic

Relevant code: [motion_backend.py#L320](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L320)

Representative excerpt:

```python
ik_res = self._call_ik_sync(req)
if ik_res.error_code.val == 1:
    if not self._execute_joint_goal(ik_res.solution.joint_state, velocity):
        return False
    return True

if self.is_xarm5:
    for yaw_deg in [15, -15, 30, -30, 45, -45, 90, -90, 180]:
        ...
        ik_res = self._call_ik_sync(req)
        if ik_res.error_code.val == 1:
            ...
            return True

return False
```

Meaning:

- try exact IK first
- if xArm5 fails, sweep yaw angles as fallback
- return `False` if all attempts fail

### Cartesian fallback logic

Relevant code: [motion_backend.py#L378](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L378)

This method computes a Cartesian path and rejects bad plans when:

- service is unavailable
- planning times out
- path fraction is under 90%
- returned trajectory is empty
- trajectory is trivial for a non-trivial move
- execution server rejects or times out

### Joint planning logic

Relevant code: [motion_backend.py#L487](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L487)

`move_to_joint_positions()`:

- waits for `/joint_states`
- optionally sets gripper force
- filters target joints using valid prefixes
- sends a MoveGroup goal
- waits for acceptance and result

Failure cases:

- no joint state within 2 seconds
- no matching joint name prefix
- MoveGroup unavailable
- goal rejection
- execution timeout
- non-success MoveIt error code

### Tactile descent logic

Relevant code: [motion_backend.py#L564](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L564)

Representative excerpt:

```python
baseline = sum(effort_samples) / len(effort_samples)
...
if (time.time() - start_t) > 0.2:
    if spike > (threshold_nm * 2.0):
        self._publish_zero_twist()
        return True
...
self.stop_immediately()
return False
```

Meaning:

- the method computes a torque baseline
- descends in servo mode
- declares contact when torque spike exceeds `2 x threshold_nm`
- returns `True` on contact
- returns `False` on timeout or missing state

### Closed-loop retract logic

Relevant code: [motion_backend.py#L661](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/motion_backend.py#L661)

This method uses TF, not just open-loop timing. It succeeds when:

- full target distance is reached within tolerance
- or at least 85% of the distance is achieved before a stall/timeout

This explains why several skills use it after contact rather than using ordinary planned retraction.

### Motion backend success and failure summary

Success conditions:

- correct controller mode entered
- transforms available
- MoveIt servers available
- joint states available
- IK or Cartesian planning succeeds
- tactile contact or closed-loop distance target detected

Failure conditions:

- TF unavailable
- servo start/stop service unavailable
- joint state not received
- prefix mismatch in target joints
- MoveIt action/service timeout
- IK failure
- Cartesian path too incomplete
- tactile descent times out
- gripper force control used on non-gripper backend

## Skill Bot 1: `ObjectHoldSkill`

Primary source: [object_hold_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_hold_skill.py)

### Purpose

This skill stabilizes a chassis or macro-part by approaching from above, touching down using tactile feedback, retracting slightly, and then closing the RG6 gripper to hold the object.

### ROS interfaces

Subscriptions:

- `/vision/agent_state`

Publishers:

- `/object_hold_state/is_held`
- `/robot_state/manip_arm/update`
- `/vision/reset_tracker`

### Internal flow

Relevant code: [object_hold_skill.py#L149](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_hold_skill.py#L149)

Representative excerpt:

```python
target_data = self._get_target_by_id(part_id)
if not target_data or 'xyz' not in target_data:
    target_data = self._get_target_by_label(target_label)
...
t_pose = self.uf850.get_transformed_pose(p, self.CAMERA_FRAME, self.PLANNING_FRAME)
...
if not self.gripper.move_to_joint_positions(...open...):
    return False
if not self.uf850.move_to_pose_robust(...):
    if not self.uf850.move_cartesian_to_pose(...):
        return False
if not self.uf850.move_linear_z_with_torque_stop(...):
    return False
if not self.uf850.retract_servo_z_closed_loop(0.005, ...):
    return False
self.uf850.stop_servo()
if not self.gripper.move_to_joint_positions(...close...):
    return False
return self.wait_for_gripper(self.CLOSE_DEG)
```

### Step-by-step flow

1. Read target from vision by ID.
2. If ID lookup fails, fall back to label match.
3. Transform target from camera frame to planning frame.
4. Compute a tilted hover pose based on object angle and tool length.
5. Open the gripper.
6. Move UF850 to hover pose.
7. If regular pose planning fails, try Cartesian planning fallback.
8. Wait for arm settling.
9. Perform tactile descent until torque spike.
10. Retract 5 mm in closed loop.
11. Stop servo.
12. Close gripper to hold object.
13. Publish hold status and state.

### Success path

The skill succeeds only if all of these are true:

- vision target exists with `xyz`
- TF transform succeeds
- gripper opens
- hover pose is reached by robust plan or Cartesian fallback
- arm settles
- tactile descent detects contact
- 5 mm retract succeeds
- gripper close command succeeds
- `wait_for_gripper()` confirms closure or acceptable close stall

On success, `execute_hold()` publishes:

- `/object_hold_state/is_held = True`
- robot state `HOLDING`
- `/tmp/disassembly_hold_state = true`

### Failure path

The skill fails if any stage returns `False`, including:

- target missing after ID and label lookup
- TF conversion failure
- hover planning failure from both planners
- arm never settles
- tactile descent fails to touch down
- retract fails
- gripper close command fails
- gripper never reaches or stalls at valid close position

The `finally` block in `execute_hold()` always stops servo and republishes final hold status, which is a good cleanup design.

### Important behavior

Relevant code: [object_hold_skill.py#L242](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_hold_skill.py#L242)

`execute_hold()` is one of the cleaner skill wrappers in the package because it:

- initializes hold status to false
- catches exceptions
- always stops servo
- always republishes consistent final state

## Skill Bot 2: `PickupSkill`

Primary source: [object_pickup_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_pickup_skill.py)

### Purpose

This skill extracts a component, especially a PCB, using UF850 plus RG6. It may first release a currently held chassis, clear space with xArm5, reacquire the target from vision, grasp it, move it to a drop pose, release it, and home the UF850.

### ROS interfaces

Subscriptions:

- `/object_hold_state/is_held`
- `/vision/agent_state`

Publishers:

- `/robot_state/manip_arm/update`
- `/vision/reset_tracker`
- `/object_hold_state/is_held`

### Internal flow

Relevant code: [object_pickup_skill.py#L121](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_pickup_skill.py#L121)

### Step-by-step flow

1. Force UF850 into trajectory mode with `stop_servo()`.
2. If chassis is currently held:
   - open gripper to release
   - clear hold state
   - retract 30 cm
   - if retract planning fails, fall back to manual joint lift
   - home UF850
3. If chassis is not held but arm is far from home:
   - perform the same safety retract and home sequence
4. Move xArm5 out of the workspace.
5. Reset vision tracker.
6. Re-detect target by ID, or by label if ID changes.
7. Transform target pose to world.
8. Open gripper and move to hover pose.
9. Partially close gripper to `0 rad` for search posture.
10. Start servo and descend until torque spike.
11. Retract 10 mm.
12. Close gripper fully to grasp component.
13. Mark hold state as `True`.
14. Retract 30 mm.
15. Move to fixed drop pose.
16. Open gripper and clear hold state.
17. Move UF850 to home.

### Representative code

Relevant code: [object_pickup_skill.py#L127](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_pickup_skill.py#L127)

```python
if self.is_holding_object:
    ... release ...
    if not self.uf850.retract_relative_z(0.30, velocity=0.1):
        ... fallback joint lift ...
    if not self.uf850.move_to_joint_positions(self.UF_HOME_JOINTS, velocity=0.2):
        return False
...
if not self.xarm5.move_to_joint_positions({...clear workspace...}):
    return False
...
self.vision_reset_pub.publish(String(data='reset'))
...
if not self.uf850.start_servo(): return False
if not self.uf850.move_linear_z_with_torque_stop(...): return False
if not self.uf850.retract_servo_z_closed_loop(0.01, ...): return False
...
if not self.uf850.move_to_pose_robust(self.DROP_POSE['x'], ...): return False
```

### Success path

The skill succeeds if:

- any precondition release/retract/home steps succeed
- xArm5 clears workspace
- target is reacquired after vision reset
- target TF transform succeeds
- hover pose is reachable
- contact is detected during tactile descent
- retraction and final grasp succeed
- drop pose is reached
- release succeeds
- UF850 returns home

On success:

- object is physically dropped
- hold state is reset to false
- `/tmp/disassembly_hold_state` becomes `false`
- function returns `True`

### Failure path

The skill can fail at many points:

- cannot release an already held chassis
- cannot retract 30 cm and fallback joint lift also fails
- cannot home UF850
- xArm5 cannot clear workspace
- target disappears after vision reset
- TF transform fails
- hover move fails
- servo start fails
- tactile descent fails
- retract after contact fails
- final grasp fails
- drop pose move fails
- final release fails

### Notable design choices

- The skill uses file-backed hold state recovery in `main()` via `/tmp/disassembly_hold_state`.
- It does not wrap the full sequence in a `try/finally`, so a mid-sequence failure can leave the robot in a partially progressed state.
- The comment for `GRIPPER_CLOSE_FORCE_N` says “Updated to 30N” but the code sets `80.0`, which is a maintenance inconsistency.

## Skill Bot 3: `ObjectFlipSkill`

Primary source: [object_flip_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_flip_skill.py)

### Purpose

This skill flips a held object by rotating UF850 joint 6 by 180 degrees, descending back onto the surface, releasing, and re-grasping.

### Internal flow

Relevant code: [object_flip_skill.py#L104](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_flip_skill.py#L104)

### Step-by-step flow

1. Stop servo to ensure clean state.
2. Wait up to 2 seconds for hold state to arrive.
3. Abort if no object is currently held.
4. Retract vertically by 10 cm using closed-loop servo retract.
5. Read current joint positions.
6. Rotate `u1_joint6` by `+pi` or `-pi` depending on its sign.
7. Start servo.
8. Descend until tactile contact.
9. Stop servo.
10. Open gripper to release.
11. Close gripper again to re-grasp.
12. Publish hold state `True`.

### Representative code

```python
if not self.is_holding_object:
    self.get_logger().error("❌ Error: No object held...")
    return False
...
if current_j6 > 0:
    joints["u1_joint6"] = current_j6 - math.pi
else:
    joints["u1_joint6"] = current_j6 + math.pi
...
if not self.uf850.start_servo():
    return False
if not self.uf850.move_linear_z_with_torque_stop(...):
    return False
...
if not self.gripper.move_to_joint_positions(...open...): return False
if not self.gripper.move_to_joint_positions(...close...): return False
```

### Success path

Success requires:

- valid current hold state
- retract succeeds
- 180 degree joint rotation succeeds
- tactile descent succeeds
- release succeeds
- re-grasp succeeds

On success it republishes hold state and keeps broadcasting it in `main()`.

### Failure path

Failure cases:

- no hold state received
- retract fails
- 180 degree flip move fails
- servo start fails
- tactile descent fails
- gripper release or re-grasp fails

### Observations

- This skill assumes the object remains controllable during a pure `joint6` rotation.
- There is no explicit recovery if release succeeds but re-grasp fails.

## Skill Bot 4: `FlipDropSkill`

Primary source: [object_flip_drop_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_flip_drop_skill.py)

### Purpose

This skill is a more aggressive “dump and re-grasp” routine. It moves to an intermediate flip zone, flips the object, flips back, returns, descends, opens the gripper to let loose parts fall out, then re-grasps the remaining chassis.

### Internal flow

Relevant code: [object_flip_drop_skill.py#L108](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/object_flip_drop_skill.py#L108)

### Step-by-step flow

1. Read current end-effector TF pose and orientation.
2. Retract 15 cm.
3. Travel to a fixed intermediate pose.
4. Rotate joint 6 by 180 degrees.
5. Rotate joint 6 back to original orientation.
6. Return to original XY above the pick location.
7. Start servo and descend until tactile contact.
8. Jog upward slightly by 5 mm.
9. Stop servo.
10. Open gripper to drop loose components.
11. Close gripper again to re-grasp the chassis.
12. Publish `HOLDING` and hold state `True`.

### Representative code

```python
start_tf = self.uf850.tf_buffer.lookup_transform(...)
...
if not self.uf850.move_to_pose_robust(self.INTERMEDIATE_POSE['x'], ...):
    return False
...
joints["u1_joint6"] = orig_j6 +/- math.pi
...
joints["u1_joint6"] = orig_j6
...
if not self.uf850.start_servo(): return False
if not self.uf850.move_linear_z_with_torque_stop(...): return False
self.uf850.jog_cartesian_servo(0.0, 0.0, 0.005, duration=0.5)
...
success = self.wait_for_gripper(self.CLOSE_DEG)
```

### Success path

Success requires:

- initial TF lookup succeeds
- retract succeeds
- transit pose succeeds
- 180 degree flip and flip-back succeed
- return pose succeeds
- tactile descent succeeds
- release succeeds
- re-grasp succeeds

On success:

- loose parts should have dropped
- chassis should be re-held
- hold state is published true

### Failure path

Failure cases:

- TF lookup fails
- any major move fails
- servo start or descent fails
- gripper fails to open or close
- final re-grasp does not settle

### Observations

- The skill does not explicitly verify that parts were actually dropped.
- It relies on geometry and gravity rather than vision verification.

## Skill Bot 5: `UnscrewSkill`

Primary source: [unscrew_skill.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py)

### Purpose

This is the most complex skill in the package. It uses xArm5 plus a screwdriver tool to:

- approach a screw
- visually align while descending
- use force detection for contact
- verify seating by force response during a short motor spin
- perform compliant extraction while spinning
- move to bin 1 and release the screw

### ROS interfaces

Subscriptions:

- `/vision/agent_state`
- `/vision/bin_coordinates`

Publishers:

- `/robot_state/tool_arm/update`
- `/tool_cmd`

### Configuration model

Relevant code: [unscrew_skill.py#L16](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py#L16)

The configuration is centralized in `self.CONFIG`, including:

- tool length
- hover distance
- reach limit
- pixel-to-meter calibration
- XY and Z alignment speeds
- force thresholds
- spiral search settings
- extraction timing and compliance gains

This is good for tuning, although there is no external YAML for these values.

### Stage 1: visual servo and staircase descent

Relevant code: [unscrew_skill.py#L120](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py#L120)

#### What it does

`perform_staircase_descent()` continuously reads:

- force/torque from `local_view`
- screw head detection from `local_view.screw_heads`
- crosshair location

It then:

- aligns XY sequentially
- slows or stops Z if XY error is large
- descends with smoothed XY velocity
- triggers contact logic when force rises above threshold

#### Contact logic

Relevant code: [unscrew_skill.py#L156](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py#L156)

After contact:

1. perform on-surface XY correction
2. run a short unscrew spin for 1 second
3. inspect force spike during and after the spin
4. if spike > 3 N, assume bit is seated
5. otherwise retract 5 mm and retry

#### Spiral fallback

Relevant code: [unscrew_skill.py#L231](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py#L231)

If vision loses the screw head:

- a square spiral search is executed in world XY
- search stops early if vision returns
- after timeout, the method returns `"TIMEOUT"`

#### Success conditions

Stage 1 succeeds when:

- contact force is detected
- surface XY correction converges enough
- seating check force spike exceeds threshold

#### Failure conditions

Stage 1 fails when:

- vision search times out
- alignment retries exceed max retries
- force logic never reaches a seated condition

### Stage 2: compliant extraction

Relevant code: [unscrew_skill.py#L318](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py#L318)

#### What it does

`perform_compliant_extraction()`:

- publishes tool command `-1` to unscrew
- publishes tool command `2` to grab
- monitors upward force
- drives upward servo motion proportionally to force
- stops when extraction force has stabilized long enough
- stops tool motor
- retracts 15 mm

#### Success conditions

Stage 2 succeeds when:

- extraction loop stabilizes before timeout
- post-grasp retract succeeds

#### Failure conditions

Stage 2 fails when:

- post-grasp retract fails
- or upstream servo behavior prevents stable extraction

One notable issue: the extraction loop itself does not explicitly return `False` on timeout before the motor stop section. It will drop out of the loop after timeout and still attempt to stop the motor and retract. That makes timeout behavior softer than the rest of the package.

### Full unscrew flow

Relevant code: [unscrew_skill.py#L373](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/unscrew_skill.py#L373)

### Step-by-step flow

1. Verify bin 1 coordinates are known.
2. Verify target screw exists in current target list.
3. Transform target to world frame and xArm base frame.
4. Abort if target is outside reach limit.
5. Lift safely by transit amount.
6. Move to hover above screw.
7. Start servo.
8. Run staircase descent.
9. If descent fails or times out:
   - retract to safety
   - navigate to bin
   - return `False`
10. Run compliant extraction.
11. If extraction succeeds:
   - navigate to bin 1
   - release screw
   - return `True`
12. Otherwise return `False`

### Representative code

```python
if not bin1_raw:
    print("❌ Bin 1 location missing. Aborting.")
    return False
...
if dist_base > self.CONFIG["REACH_LIMIT"]:
    print(f"❌ Reach {dist_base:.3f}m exceeds limit.")
    return False
...
staircase_res = self.perform_staircase_descent()
if staircase_res == "TIMEOUT" or staircase_res is False:
    self.moveit_backend.retract_servo_z_closed_loop(...)
    self._navigate_to_bin(bin1_raw)
    return False
...
if self.perform_compliant_extraction():
    self._navigate_to_bin(bin1_raw)
    return True
```

### Full success path

The unscrew skill succeeds when:

- bin coordinates exist
- target screw is visible and in reach
- hover approach succeeds
- servo mode starts
- staircase descent seats the bit
- compliant extraction succeeds
- bin navigation succeeds enough to release

### Full failure path

It fails when:

- bin 1 coordinates are missing
- screw target is missing
- TF transform fails
- screw is beyond reach limit
- hover approach fails
- servo fails to start
- alignment/search/seating logic fails
- extraction retract fails

### Assessment

This is the most sophisticated skill bot in the package. It combines vision servoing, force feedback, spiral search, and compliant motion. It is also the skill with the largest number of branch paths and the most tuning-sensitive parameters.

## Orchestrator Bot 6: `MasterAgentNode`

Primary source: [master_agent.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py)

### Purpose

This file is the OpenAI-backed autonomous planner. It embeds all skill bots in one process and asks an LLM to choose which tool to call next.

### Main responsibilities

Relevant code:

- [master_agent.py#L200](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py#L200)
- [master_agent.py#L248](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py#L248)
- [master_agent.py#L337](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py#L337)

It does all of the following:

- spawns a Tkinter dashboard in a separate process
- instantiates all concrete skills
- maintains filtered vision state
- keeps “cleared zones” memory to hide already removed parts from the LLM
- builds ReAct prompts for reasoning, plan, action
- calls OpenAI async Responses API
- parses tool calls like `Action: hold_object(part_id=..., label=...)`
- executes the skill function

### ReAct flow

Graph defined in [master_agent.py#L540](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py#L540) in the compiled graph section.

Operational cycle:

1. `think`
2. `plan`
3. `action`
4. `act`
5. loop until `Final Answer`

### Tool wrappers

Relevant code: [master_agent.py#L294](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py#L294)

The tool wrappers do not inspect boolean success deeply. For example:

```python
self.hold_skill.execute_hold(...)
return "Tool called successfully"
```

and similarly for `pickup`, `flip`, and `flip_drop`.

This is a major architectural limitation:

- the agent reports tool invocation success, not actual physical success
- unless the skill throws an exception, the observation text says success

### Vision memory

Relevant code: [master_agent.py#L337](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/master_agent.py#L337)

The master agent removes detections within `EXCLUSION_RADIUS` of previously unscrewed positions. This helps prevent the LLM from repeatedly targeting the same screw.

### Success path

The orchestrator “succeeds” when:

- vision arrives
- auto-start triggers
- the LLM keeps generating valid `Reasoning`, `Plan`, and `Action` lines
- actions map to existing tools
- eventually the model emits `Final Answer:`

### Failure path

Failure modes:

- missing OpenAI API key file
- invalid or unavailable OpenAI responses
- no valid `Action:` line from the model
- unknown tool names
- skill methods fail silently but wrapper still returns success text
- no robust perception-based completion check beyond what the model infers from vision

### Assessment

This bot is conceptually strong but operationally optimistic. Its biggest weakness is that tool wrappers do not convert the actual boolean result into the LLM observation.

## Orchestrator Bot 7: `GroqMasterAgentNode`

Primary source: [groq_master_agent.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/groq_master_agent.py)

### Purpose

This is the Groq-hosted version of the master agent. Architecturally it is almost the same as `master_agent.py`, but it uses:

- Groq API key file
- Groq chat completions endpoint
- model `llama-3.3-70b-versatile`

### Similarities to `MasterAgentNode`

- same skill set
- same dashboard concept
- same ReAct phases
- same cleared-zones memory
- same parsing of `Action: tool(args)`
- same tool-wrapper optimism problem

### Difference in mission prompt

Relevant code: [groq_master_agent.py#L195](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/groq_master_agent.py#L195)

The Groq version is more explicit about:

- using `hold_object` early
- retrying failed macro-component removals
- immediately re-holding after `pickup_object`

This is a better prompt than the OpenAI version in terms of operational state constraints.

### Success and failure

Success and failure are effectively the same as the OpenAI master agent, with these additional failure modes:

- missing Groq API key file
- Groq endpoint latency or timeout
- model-specific formatting errors

## Orchestrator Bot 8: `HDDDisassemblyScript`

Primary source: [hdd_disassembly_script.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/hdd_disassembly_script.py)

### Purpose

This is a non-LLM deterministic workflow specifically for HDD disassembly.

### Flow

The top-of-file docstring in [hdd_disassembly_script.py#L2](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/hdd_disassembly_script.py#L2) is accurate:

1. wait for vision
2. hold chassis
3. unscrew up to 2 screws
4. pickup PCB
5. re-hold chassis
6. flip

### Detailed behavior

Relevant code:

- [hdd_disassembly_script.py#L88](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/hdd_disassembly_script.py#L88)
- [hdd_disassembly_script.py#L121](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/hdd_disassembly_script.py#L121)
- [hdd_disassembly_script.py#L155](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/hdd_disassembly_script.py#L155)
- [hdd_disassembly_script.py#L196](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/disassembly_skills/hdd_disassembly_script.py#L196)

The script:

- retries holding the chassis up to 3 times
- unscrews up to 2 screws, continuing even if one fails
- retries PCB pickup up to 3 times with vision verification
- re-holds the chassis after pickup
- flips the part

### Success path

The script is successful if:

- vision is online
- hold works
- enough unscrewing happens to continue
- PCB is removed or absent
- re-hold succeeds
- flip succeeds

### Failure path

It aborts if:

- initial hold fails after 3 attempts
- re-hold after pickup fails after 3 attempts

It does not abort when:

- unscrewing fails on one or more screws
- PCB pickup fails after retries
- final flip fails

Instead, some of those steps are logged and the script continues or simply finishes.

### Assessment

This is the clearest end-to-end procedural bot in the package. For a known object class like HDDs, this is more predictable than the LLM agents.

## Launch and Config Assets

### `apriltag.launch.py`

Source: [apriltag.launch.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/launch/apriltag.launch.py)

What it does:

- launches `apriltag_ros/apriltag_node`
- remaps camera topics to RealSense color image and camera info
- loads [tags.yaml](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/config/tags.yaml)

Risk:

- the config path is hardcoded to `~/workspace/disassembly_ws/src/disassembly_skills/config/tags.yaml`, which is brittle if the workspace location changes

### `vision_system.launch.py`

Source: [vision_system.launch.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/launch/vision_system.launch.py)

What it intends to do:

- load camera calibration YAML
- publish static camera transform
- run a `vision_system/agent_node`
- run `disassembly_skills/vision_bridge`

Risk:

- it expects `config/camera_calibration.yaml`, but that file was not present in this package snapshot
- it refers to `vision_bridge`, but no matching entry point exists in [setup.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/setup.py)

This launch file appears incomplete or out of sync with the package.

## Testing

Tests present:

- [test_flake8.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/test/test_flake8.py)
- [test_pep257.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/test/test_pep257.py)
- [test_copyright.py](/home/adip/workspace/dev_ws/src/disassembly_pipeline/disassembly_skills/test/test_copyright.py)

There are no behavior-level tests for:

- skill success/failure logic
- vision parsing
- TF transforms
- hold-state persistence
- motion recovery paths

## Cross-Cutting Success and Failure Patterns

### Shared success pattern

Most skills succeed only when this full chain is intact:

1. vision payload parses correctly
2. target object exists
3. TF conversion succeeds
4. servo/planning services are available
5. motion reaches approach pose
6. tactile or force event occurs as expected
7. post-contact retract works
8. gripper/tool action completes

### Shared failure pattern

Most skills fail because of one of these:

- stale or missing vision target
- TF not available yet
- servo mode not starting cleanly
- MoveIt planning failure
- no force/effort feedback
- gripper stall or lack of state update
- inconsistent state handoff between skills

## Key Strengths

- `MotionBackend` is the real backbone of the package and contains useful recovery logic.
- The skills use multiple fallbacks in several places, especially hover approach and precondition retract.
- Hold-state persistence through `/tmp/disassembly_hold_state` is pragmatic.
- The unscrew skill contains substantial real-world compensation logic for vision loss and contact uncertainty.
- The HDD state machine is straightforward and easier to trust than the LLM orchestrators.

## Key Weaknesses

- LLM tool wrappers do not report actual boolean skill outcomes back to the agent.
- Several skills do not wrap the entire sequence in `try/finally`, so cleanup is inconsistent.
- Some launch/config assets appear stale or incomplete.
- There is almost no automated testing of behavior paths.
- State is split between ROS topics, local booleans, and a temp file.
- There are some code/comment mismatches and hardcoded constants that should probably be externalized.

## Most Important Risks

1. `master_agent.py` and `groq_master_agent.py` can tell the LLM that a tool succeeded even when the underlying skill returned `False`.
2. `vision_system.launch.py` appears broken or incomplete because referenced assets/executables are missing from the package snapshot.
3. Skills depend heavily on timing-based waits and may be sensitive to hardware latency.
4. Recovery after partial progress is inconsistent across skills.

## Recommended Improvements

1. Make every tool wrapper return the real boolean result and embed it in the observation string.
2. Add a shared skill base class for:
   - hold-state persistence
   - arm settle wait
   - gripper settle wait
   - cleanup/finalization
3. Move skill tuning values into YAML config.
4. Add unit tests for:
   - vision parsing
   - target fallback by label
   - hold-state file persistence
   - action parsing in master agents
5. Add integration tests or dry-run mocks for `MotionBackend`.
6. Fix or remove stale launch assets.

## Bottom Line

`disassembly_skills` is a robotics skill package built around a solid motion abstraction and several practical manipulation routines. The strongest concrete bot is `UnscrewSkill`, and the strongest deterministic workflow is `HDDDisassemblyScript`. The weakest area is the LLM orchestration layer, not because the ReAct structure is wrong, but because tool execution results are not faithfully propagated back into the reasoning loop.

If this package is being used on real hardware, the motion layer and skill logic are already meaningful. If it is being prepared for production autonomy, the next highest-value work is:

- result propagation
- recovery consistency
- launch/config cleanup
- tests for failure paths
