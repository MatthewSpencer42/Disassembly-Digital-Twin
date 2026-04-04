#!/usr/bin/env python3
from __future__ import annotations

import threading
from pathlib import Path
from tempfile import gettempdir

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dual_arm_moveit_config.runtime_config import joint_topics_for_hardware, normalize_xacro_hardware_type


class ExoticaDualArmPlanner:
    _setup_lock = threading.Lock()
    _setup_initialized = False

    def __init__(self, node, hardware_type: str = "fake"):
        self.node = node
        self.hardware_type = hardware_type
        self.available = False
        self.last_error = None
        self._solver = None
        self._problem = None
        self._scene = None
        self._np = None
        self.controlled_joint_names = []
        self._joint_index = {}
        self._joint_limits = {}

        try:
            import numpy as np
            import pyexotica as exo
            import exotica_ompl_solver_py  # noqa: F401
        except ModuleNotFoundError as exc:
            self.last_error = f"EXOTica is not installed: {exc}"
            return
        except Exception as exc:  # pragma: no cover - import edge case
            self.last_error = f"Failed to import EXOTica: {exc}"
            return

        self._np = np
        self._exo = exo
        self._share_dir = Path(get_package_share_directory("dual_arm_moveit_config"))
        self._joint_limits = yaml.safe_load(
            (self._share_dir / "config" / "joint_limits.yaml").read_text(encoding="utf-8")
        )["joint_limits"]

        try:
            self._ensure_setup_initialized()
            config_path = self._generate_config()
            self._solver = self._exo.Setup.load_solver(str(config_path))
            self._problem = self._solver.get_problem()
            self._scene = self._problem.get_scene()
            self.controlled_joint_names = list(self._scene.get_controlled_joint_names())
            self._joint_index = {
                joint_name: index for index, joint_name in enumerate(self.controlled_joint_names)
            }
            self.available = True
            self.node.get_logger().info(
                "EXOTica dual-arm planner initialized for joints: "
                + ", ".join(self.controlled_joint_names)
            )
        except Exception as exc:
            self.last_error = f"Failed to initialize EXOTica dual-arm planner: {exc}"

    @classmethod
    def _ensure_setup_initialized(cls):
        with cls._setup_lock:
            if cls._setup_initialized:
                return
            import pyexotica as exo

            exo.Setup.init_ros()
            cls._setup_initialized = True

    def _generate_config(self) -> Path:
        joint_commands_topic, joint_states_topic = joint_topics_for_hardware(self.hardware_type)
        xacro_hardware_type = normalize_xacro_hardware_type(self.hardware_type)

        urdf_xacro_path = self._share_dir / "config" / "dual_arm_world.urdf.xacro"
        urdf_path = Path(gettempdir()) / f"dual_arm_world_exotica_{self.hardware_type}.urdf"
        urdf_doc = xacro.process_file(
            str(urdf_xacro_path),
            mappings={
                "hardware_type": xacro_hardware_type,
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
            },
        )
        urdf_path.write_text(urdf_doc.toprettyxml(indent="  "), encoding="utf-8")

        template_path = self._share_dir / "config" / "exotica" / "dual_arm_rrt_connect.xml"
        config_path = Path(gettempdir()) / f"dual_arm_rrt_connect_{self.hardware_type}.xml"
        config_text = template_path.read_text(encoding="utf-8")
        zero_state = "0 0 0 0 0 0 0 0 0 0 0 0"
        config_text = config_text.replace("__URDF_PATH__", str(urdf_path))
        config_text = config_text.replace(
            "__SRDF_PATH__", str(self._share_dir / "config" / "dual_arm_world.srdf")
        )
        config_text = config_text.replace("__START_STATE__", zero_state)
        config_text = config_text.replace("__GOAL_STATE__", zero_state)
        config_path.write_text(config_text, encoding="utf-8")
        return config_path

    def _state_vector_from_joint_map(self, joint_positions: dict[str, float]):
        values = self._np.zeros(len(self.controlled_joint_names))
        for joint_name, index in self._joint_index.items():
            values[index] = float(joint_positions.get(joint_name, 0.0))
        return values

    def _target_vector(self, current_positions: dict[str, float], target_positions: dict[str, float]):
        goal = self._state_vector_from_joint_map(current_positions)
        for joint_name, position in target_positions.items():
            if joint_name in self._joint_index:
                goal[self._joint_index[joint_name]] = float(position)
        return goal

    def _estimate_segment_time(self, previous_row, current_row, velocity_scaling: float) -> float:
        segment_time = 0.05
        safe_scaling = max(float(velocity_scaling), 0.05)
        for joint_name, index in self._joint_index.items():
            max_velocity = float(self._joint_limits.get(joint_name, {}).get("max_velocity", 1.0))
            delta = abs(float(current_row[index]) - float(previous_row[index]))
            segment_time = max(segment_time, delta / max(max_velocity * safe_scaling, 1e-3))
        return segment_time

    def _solution_to_robot_trajectory(self, solution, velocity_scaling: float) -> RobotTrajectory:
        matrix = self._np.asarray(solution, dtype=float)
        if matrix.ndim == 1:
            matrix = matrix.reshape(1, -1)
        if matrix.shape[1] != len(self.controlled_joint_names) and matrix.shape[0] == len(
            self.controlled_joint_names
        ):
            matrix = matrix.T
        if matrix.shape[1] != len(self.controlled_joint_names):
            raise ValueError(
                f"EXOTica returned shape {matrix.shape}, expected columns={len(self.controlled_joint_names)}"
            )

        trajectory = RobotTrajectory()
        trajectory.joint_trajectory = JointTrajectory()
        trajectory.joint_trajectory.joint_names = list(self.controlled_joint_names)

        # Compute cumulative timestamps for each waypoint
        n = matrix.shape[0]
        timestamps = [0.0]
        for i in range(1, n):
            timestamps.append(timestamps[-1] + self._estimate_segment_time(matrix[i - 1], matrix[i], velocity_scaling))

        # Compute velocities using central differences (zero at boundaries)
        velocities = [self._np.zeros(matrix.shape[1]) for _ in range(n)]
        for i in range(1, n - 1):
            dt = timestamps[i + 1] - timestamps[i - 1]
            if dt > 1e-6:
                velocities[i] = (matrix[i + 1] - matrix[i - 1]) / dt

        # Compute accelerations using central differences (zero at boundaries)
        accelerations = [self._np.zeros(matrix.shape[1]) for _ in range(n)]
        for i in range(1, n - 1):
            dt = timestamps[i + 1] - timestamps[i - 1]
            if dt > 1e-6:
                accelerations[i] = (velocities[i + 1] - velocities[i - 1]) / dt

        for i, row in enumerate(matrix):
            elapsed = timestamps[i]
            point = JointTrajectoryPoint()
            point.positions = [float(value) for value in row]
            point.velocities = [float(value) for value in velocities[i]]
            point.accelerations = [float(value) for value in accelerations[i]]
            point.time_from_start = Duration(
                sec=int(elapsed),
                nanosec=int((elapsed - int(elapsed)) * 1_000_000_000),
            )
            trajectory.joint_trajectory.points.append(point)
        return trajectory

    def plan_joint_trajectory(
        self,
        current_positions: dict[str, float],
        target_positions: dict[str, float],
        velocity_scaling: float = 0.2,
    ) -> RobotTrajectory | None:
        if not self.available:
            return None

        start_state = self._state_vector_from_joint_map(current_positions)
        goal_state = self._target_vector(current_positions, target_positions)

        if self._np.allclose(start_state, goal_state, atol=1e-4):
            return self._solution_to_robot_trajectory(self._np.vstack([start_state, goal_state]), velocity_scaling)

        try:
            self._problem.start_state = start_state
            if hasattr(self._problem, "goal_state"):
                self._problem.goal_state = goal_state

            if hasattr(self._problem, "is_state_valid") and not self._problem.is_state_valid(start_state):
                self.last_error = "Current dual-arm state is invalid for EXOTica."
                return None
            if hasattr(self._problem, "is_state_valid") and not self._problem.is_state_valid(goal_state):
                self.last_error = "Requested dual-arm target is invalid for EXOTica."
                return None

            solution = self._solver.solve()
            if solution is None:
                self.last_error = "EXOTica returned no solution."
                return None
            trajectory = self._solution_to_robot_trajectory(solution, velocity_scaling)
            if not trajectory.joint_trajectory.points:
                self.last_error = "EXOTica returned an empty trajectory."
                return None
            self.last_error = None
            return trajectory
        except Exception as exc:
            self.last_error = f"EXOTica solve failed: {exc}"
            return None


class ExoticaDualArmPosePlanner:
    _setup_lock = threading.Lock()
    _setup_initialized = False

    _CONTROLLED_JOINT_NAMES = [
        "uf_slide_joint",
        "uf850_joint1",
        "uf850_joint2",
        "uf850_joint3",
        "uf850_joint4",
        "uf850_joint5",
        "uf850_joint6",
        "xarm5_joint1",
        "xarm5_joint2",
        "xarm5_joint3",
        "xarm5_joint4",
        "xarm5_joint5",
    ]

    _CONTINUOUS_JOINTS = {
        "uf850_joint1",
        "uf850_joint4",
        "uf850_joint6",
        "xarm5_joint1",
        "xarm5_joint5",
    }

    def __init__(self, node, hardware_type: str = "fake"):
        self.node = node
        self.hardware_type = hardware_type
        self.available = False
        self.last_error = None
        self._solver = None
        self._problem = None
        self._scene = None
        self._np = None
        self.controlled_joint_names = list(self._CONTROLLED_JOINT_NAMES)
        self._joint_index = {
            joint_name: index for index, joint_name in enumerate(self.controlled_joint_names)
        }
        self._joint_limits = {}

        try:
            import numpy as np
            import pyexotica as exo
        except ModuleNotFoundError as exc:
            self.last_error = f"EXOTica is not installed: {exc}"
            return
        except Exception as exc:
            self.last_error = f"Failed to import EXOTica: {exc}"
            return

        self._np = np
        self._exo = exo
        self._share_dir = Path(get_package_share_directory("dual_arm_moveit_config"))
        self._joint_limits = yaml.safe_load(
            (self._share_dir / "config" / "joint_limits.yaml").read_text(encoding="utf-8")
        )["joint_limits"]

        try:
            self._ensure_setup_initialized()
            config_path = self._generate_config()
            self._solver = self._exo.Setup.load_solver(str(config_path))
            self._problem = self._solver.get_problem()
            self._scene = self._problem.get_scene()
            scene_joint_names = list(self._scene.get_controlled_joint_names())
            if scene_joint_names:
                self.controlled_joint_names = scene_joint_names
                self._joint_index = {
                    joint_name: index for index, joint_name in enumerate(scene_joint_names)
                }
            self.available = True
            self.node.get_logger().info(
                "EXOTica dual-arm pose planner initialized for joints: "
                + ", ".join(self.controlled_joint_names)
            )
        except Exception as exc:
            self.last_error = f"Failed to initialize EXOTica dual-arm pose planner: {exc}"

    @classmethod
    def _ensure_setup_initialized(cls):
        with cls._setup_lock:
            if cls._setup_initialized:
                return
            import pyexotica as exo

            exo.Setup.init_ros()
            cls._setup_initialized = True

    def _generate_urdf(self) -> Path:
        joint_commands_topic, joint_states_topic = joint_topics_for_hardware(self.hardware_type)
        xacro_hardware_type = normalize_xacro_hardware_type(self.hardware_type)
        urdf_xacro_path = self._share_dir / "config" / "dual_arm_world.urdf.xacro"
        urdf_path = Path(gettempdir()) / f"dual_arms_{self.hardware_type}_stream_exotica.urdf"
        urdf_doc = xacro.process_file(
            str(urdf_xacro_path),
            mappings={
                "hardware_type": xacro_hardware_type,
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
            },
        )
        urdf_path.write_text(urdf_doc.toprettyxml(indent="  "), encoding="utf-8")
        return urdf_path

    def _generate_config(self) -> Path:
        urdf_path = self._generate_urdf()
        srdf_path = self._share_dir / "config" / "dual_arm_world.srdf"
        zero_state = " ".join("0" for _ in self.controlled_joint_names)
        weights = " ".join("1" for _ in self.controlled_joint_names)
        xml_text = f"""<?xml version="1.0" ?>
<DualArmIKStreamConfig>
  <IKSolver Name="dual_arms_stream_ik_solver">
    <MaxIterations>80</MaxIterations>
    <Tolerance>1e-4</Tolerance>
  </IKSolver>

  <UnconstrainedEndPoseProblem Name="dual_arms_stream_ik_problem">
    <PlanningScene>
      <Scene>
        <JointGroup>dual_arms</JointGroup>
        <URDF>{urdf_path}</URDF>
        <SRDF>{srdf_path}</SRDF>
        <CollisionScene>
          <CollisionSceneFCLLatest Name="dual_arms_stream_collision_scene"/>
        </CollisionScene>
        <AlwaysUpdateCollisionScene>1</AlwaysUpdateCollisionScene>
      </Scene>
    </PlanningScene>

    <Maps>
      <EffFrame Name="UF850_TCP">
        <EndEffector>
          <Frame Link="rg6_tcp" LinkOffset="0 0 0 0 0 0 1"/>
        </EndEffector>
      </EffFrame>
      <EffFrame Name="XARM5_TCP">
        <EndEffector>
          <Frame Link="screwdriver_tcp" LinkOffset="0 0 0 0 0 0 1"/>
        </EndEffector>
      </EffFrame>
      <JointLimit Name="Limits"/>
      <SmoothCollisionDistance Name="SelfCollision">
        <WorldMargin>0.05</WorldMargin>
        <CheckSelfCollision>1</CheckSelfCollision>
        <Linear>1</Linear>
      </SmoothCollisionDistance>
      <SmoothCollisionDistance Name="WorldCollision">
        <WorldMargin>0.05</WorldMargin>
        <CheckSelfCollision>0</CheckSelfCollision>
        <Linear>1</Linear>
      </SmoothCollisionDistance>
    </Maps>

    <Cost>
      <Task Task="UF850_TCP" Rho="1e3"/>
      <Task Task="XARM5_TCP" Rho="1e3"/>
      <Task Task="SelfCollision" Rho="50.0"/>
      <Task Task="WorldCollision" Rho="200.0"/>
      <Task Task="Limits" Rho="10.0"/>
    </Cost>

    <StartState>{zero_state}</StartState>
    <W>{weights}</W>
  </UnconstrainedEndPoseProblem>
</DualArmIKStreamConfig>
"""
        config_path = Path(gettempdir()) / f"dual_arms_{self.hardware_type}_stream_ik.xml"
        config_path.write_text(xml_text, encoding="utf-8")
        return config_path

    def _state_vector_from_joint_map(self, joint_positions: dict[str, float]):
        values = self._np.zeros(len(self.controlled_joint_names))
        for index, joint_name in enumerate(self.controlled_joint_names):
            values[index] = float(joint_positions.get(joint_name, 0.0))
        return values

    def _wrap_to_nearest(self, reference: float, angle: float) -> float:
        return reference + ((angle - reference + self._np.pi) % (2.0 * self._np.pi) - self._np.pi)

    def _project_goal_state_near_reference(self, reference_state, goal_state):
        projected = self._np.asarray(goal_state, dtype=float).copy()
        for index, joint_name in enumerate(self.controlled_joint_names):
            if joint_name not in self._CONTINUOUS_JOINTS:
                continue
            projected[index] = self._wrap_to_nearest(float(reference_state[index]), float(projected[index]))
        return projected

    def solve_dual_pose_goal_joint_positions(
        self,
        current_positions: dict[str, float],
        uf850_pose_rpy,
        xarm5_pose_rpy,
    ) -> dict[str, float] | None:
        if not self.available:
            return None

        start_state = self._state_vector_from_joint_map(current_positions)
        try:
            self._problem.start_state = start_state
            self._problem.set_goal("UF850_TCP", self._np.asarray(uf850_pose_rpy, dtype=float))
            self._problem.set_goal("XARM5_TCP", self._np.asarray(xarm5_pose_rpy, dtype=float))
            solution = self._solver.solve()
            if solution is None:
                self.last_error = "EXOTica returned no dual-arm pose solution."
                return None
            matrix = self._np.asarray(solution, dtype=float)
            if matrix.size == 0:
                self.last_error = "EXOTica returned an empty dual-arm pose solution."
                return None
            if matrix.ndim == 1:
                goal_state = self._np.asarray(matrix, dtype=float)
            else:
                goal_state = self._np.asarray(matrix[0], dtype=float)
            goal_state = self._project_goal_state_near_reference(start_state, goal_state)
            try:
                if hasattr(self._scene, "update"):
                    self._scene.update(goal_state)
                if hasattr(self._scene, "is_state_valid"):
                    if not self._scene.is_state_valid(True, 0.0) or not self._scene.is_state_valid(False, 0.0):
                        self.last_error = "EXOTica dual-arm goal state is invalid or colliding."
                        return None
            except Exception:
                pass
            self.last_error = None
            return {
                joint_name: float(goal_state[index])
                for index, joint_name in enumerate(self.controlled_joint_names)
            }
        except Exception as exc:
            self.last_error = f"EXOTica dual-arm pose solve failed: {exc}"
            return None


class ExoticaSingleArmPosePlanner:
    _setup_lock = threading.Lock()
    _setup_initialized = False

    _ARM_CONFIG = {
        "uf850_arm": {
            "joint_names": [
                "uf850_joint1",
                "uf850_joint2",
                "uf850_joint3",
                "uf850_joint4",
                "uf850_joint5",
                "uf850_joint6",
            ],
            "group_name": "uf850_arm",
            "task_name": "UF850_TCP",
            "link_name": "rg6_tcp",
        },
        "xarm5_arm_no_slide": {
            "joint_names": [
                "xarm5_joint1",
                "xarm5_joint2",
                "xarm5_joint3",
                "xarm5_joint4",
                "xarm5_joint5",
            ],
            "group_name": "xarm5_arm_no_slide",
            "task_name": "XARM5_TCP",
            "link_name": "screwdriver_tcp",
        },
    }

    _CONTINUOUS_JOINTS = {
        "uf850_joint1",
        "uf850_joint4",
        "uf850_joint6",
        "xarm5_joint1",
        "xarm5_joint5",
    }

    # Hard position limits from URDF (lower, upper) in radians.
    # Used to reject IK solutions that violate limits despite the JointLimit cost term.
    # Note uf850_joint3 and xarm5_joint3 have asymmetric limits with very small upper bounds.
    _URDF_JOINT_POSITION_LIMITS: dict[str, tuple[float, float]] = {
        "uf850_joint1": (-6.2831853, 6.2831853),
        "uf850_joint2": (-2.3038346, 2.3038346),
        "uf850_joint3": (-4.2236968, 0.061086524),   # max ≈ 3.5° — very asymmetric
        "uf850_joint4": (-6.2831853, 6.2831853),
        "uf850_joint5": (-2.1542083, 2.1542083),
        "uf850_joint6": (-6.2831853, 6.2831853),
        "xarm5_joint1": (-6.2831853, 6.2831853),
        "xarm5_joint2": (-2.0594885, 2.0943951),
        "xarm5_joint3": (-3.9269908, 0.19198622),    # max ≈ 11° — also asymmetric
        "xarm5_joint4": (-1.6929694, 3.1415927),
        "xarm5_joint5": (-6.2831853, 6.2831853),
    }
    # Tolerance beyond URDF limits before an IK solution is hard-rejected (rad).
    _LIMIT_TOLERANCE = 0.05

    def __init__(self, node, group_name: str, hardware_type: str = "fake"):
        self.node = node
        self.group_name = group_name
        self.hardware_type = hardware_type
        self.available = False
        self.last_error = None
        self._solver = None
        self._problem = None
        self._scene = None
        self._np = None
        self._joint_index = {}
        self._joint_limits = {}

        config = self._ARM_CONFIG.get(group_name)
        if config is None:
            self.last_error = f"Unsupported EXOTica single-arm group: {group_name}"
            return
        self.controlled_joint_names = list(config["joint_names"])
        self.task_name = config["task_name"]
        self.link_name = config["link_name"]

        try:
            import numpy as np
            import pyexotica as exo
        except ModuleNotFoundError as exc:
            self.last_error = f"EXOTica is not installed: {exc}"
            return
        except Exception as exc:
            self.last_error = f"Failed to import EXOTica: {exc}"
            return

        self._np = np
        self._exo = exo
        self._share_dir = Path(get_package_share_directory("dual_arm_moveit_config"))
        self._joint_limits = yaml.safe_load(
            (self._share_dir / "config" / "joint_limits.yaml").read_text(encoding="utf-8")
        )["joint_limits"]

        try:
            self._ensure_setup_initialized()
            config_path = self._generate_config()
            self._solver = self._exo.Setup.load_solver(str(config_path))
            self._problem = self._solver.get_problem()
            self._scene = self._problem.get_scene()
            scene_joint_names = list(self._scene.get_controlled_joint_names())
            self._joint_index = {
                joint_name: index for index, joint_name in enumerate(scene_joint_names)
            }
            self.available = True
            self.node.get_logger().info(
                f"EXOTica single-arm pose planner initialized for {group_name}: "
                + ", ".join(scene_joint_names)
            )
        except Exception as exc:
            self.last_error = f"Failed to initialize EXOTica single-arm planner for {group_name}: {exc}"

    @classmethod
    def _ensure_setup_initialized(cls):
        with cls._setup_lock:
            if cls._setup_initialized:
                return
            import pyexotica as exo

            exo.Setup.init_ros()
            cls._setup_initialized = True

    def _generate_urdf(self) -> Path:
        joint_commands_topic, joint_states_topic = joint_topics_for_hardware(self.hardware_type)
        xacro_hardware_type = normalize_xacro_hardware_type(self.hardware_type)
        urdf_xacro_path = self._share_dir / "config" / "dual_arm_world.urdf.xacro"
        urdf_path = Path(gettempdir()) / f"{self.group_name}_{self.hardware_type}_exotica.urdf"
        urdf_doc = xacro.process_file(
            str(urdf_xacro_path),
            mappings={
                "hardware_type": xacro_hardware_type,
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
            },
        )
        urdf_path.write_text(urdf_doc.toprettyxml(indent="  "), encoding="utf-8")
        return urdf_path

    def _generate_config(self) -> Path:
        urdf_path = self._generate_urdf()
        srdf_path = self._share_dir / "config" / "dual_arm_world.srdf"
        zero_state = " ".join("0" for _ in self.controlled_joint_names)
        xml_text = f"""<?xml version="1.0" ?>
<IKSolverDemoConfig>
  <IKSolver Name="{self.group_name}_ik_solver">
    <MaxIterations>100</MaxIterations>
    <Tolerance>1e-4</Tolerance>
  </IKSolver>

  <UnconstrainedEndPoseProblem Name="{self.group_name}_ik_problem">
    <PlanningScene>
      <Scene>
        <JointGroup>{self.group_name}</JointGroup>
        <URDF>{urdf_path}</URDF>
        <SRDF>{srdf_path}</SRDF>
        <CollisionScene>
          <CollisionSceneFCLLatest Name="{self.group_name}_collision_scene"/>
        </CollisionScene>
        <AlwaysUpdateCollisionScene>1</AlwaysUpdateCollisionScene>
      </Scene>
    </PlanningScene>

    <Maps>
      <EffFrame Name="{self.task_name}">
        <EndEffector>
          <Frame Link="{self.link_name}" LinkOffset="0 0 0 0 0 0 1"/>
        </EndEffector>
      </EffFrame>
      <JointLimit Name="Limits"/>
      <SmoothCollisionDistance Name="SelfCollision">
        <WorldMargin>0.05</WorldMargin>
        <CheckSelfCollision>1</CheckSelfCollision>
        <Linear>1</Linear>
      </SmoothCollisionDistance>
      <SmoothCollisionDistance Name="WorldCollision">
        <WorldMargin>0.05</WorldMargin>
        <CheckSelfCollision>0</CheckSelfCollision>
        <Linear>1</Linear>
      </SmoothCollisionDistance>
    </Maps>

    <Cost>
      <Task Task="{self.task_name}" Rho="1e3"/>
      <Task Task="SelfCollision" Rho="50.0"/>
      <Task Task="WorldCollision" Rho="200.0"/>
      <Task Task="Limits" Rho="1e5"/>
    </Cost>

    <StartState>{zero_state}</StartState>
    <W>{zero_state.replace('0', '1')}</W>
  </UnconstrainedEndPoseProblem>
</IKSolverDemoConfig>
"""
        config_path = Path(gettempdir()) / f"{self.group_name}_{self.hardware_type}_ik.xml"
        config_path.write_text(xml_text, encoding="utf-8")
        return config_path

    def _state_vector_from_joint_map(self, joint_positions: dict[str, float]):
        values = self._np.zeros(len(self.controlled_joint_names))
        for index, joint_name in enumerate(self.controlled_joint_names):
            values[index] = float(joint_positions.get(joint_name, 0.0))
        return values

    def _wrap_to_nearest(self, reference: float, angle: float) -> float:
        return reference + ((angle - reference + self._np.pi) % (2.0 * self._np.pi) - self._np.pi)

    def _project_goal_state_near_reference(self, reference_state, goal_state):
        projected = self._np.asarray(goal_state, dtype=float).copy()
        for index, joint_name in enumerate(self.controlled_joint_names):
            if joint_name not in self._CONTINUOUS_JOINTS:
                continue
            projected[index] = self._wrap_to_nearest(float(reference_state[index]), float(projected[index]))
        return projected

    def _check_joint_limits(self, goal_state) -> str | None:
        """Return an error string if any joint in goal_state violates URDF position limits,
        or None if all joints are within tolerance."""
        for index, joint_name in enumerate(self.controlled_joint_names):
            limits = self._URDF_JOINT_POSITION_LIMITS.get(joint_name)
            if limits is None:
                continue
            lo, hi = limits
            val = float(goal_state[index])
            if val < lo - self._LIMIT_TOLERANCE:
                return (
                    f"{joint_name}={val:.4f} violates lower limit {lo:.4f} "
                    f"(under by {lo - val:.4f} rad)"
                )
            if val > hi + self._LIMIT_TOLERANCE:
                return (
                    f"{joint_name}={val:.4f} violates upper limit {hi:.4f} "
                    f"(over by {val - hi:.4f} rad)"
                )
        return None

    def _estimate_segment_time(self, previous_row, current_row, velocity_scaling: float) -> float:
        segment_time = 0.05
        safe_scaling = max(float(velocity_scaling), 0.01)
        for index, joint_name in enumerate(self.controlled_joint_names):
            max_velocity = float(self._joint_limits.get(joint_name, {}).get("max_velocity", 1.0))
            delta = abs(float(current_row[index]) - float(previous_row[index]))
            segment_time = max(segment_time, delta / max(max_velocity * safe_scaling, 1e-3))
        return segment_time

    def _trajectory_to_robot_trajectory(self, states, velocity_scaling: float) -> RobotTrajectory:
        matrix = self._np.asarray(states, dtype=float)
        if matrix.ndim == 1:
            matrix = matrix.reshape(1, -1)

        trajectory = RobotTrajectory()
        trajectory.joint_trajectory = JointTrajectory()
        trajectory.joint_trajectory.joint_names = list(self.controlled_joint_names)

        if matrix.shape[0] == 2:
            # Single start→goal segment: use quintic time-scaling for smooth S-curve motion.
            # s(τ)   = 10τ³ − 15τ⁴ + 6τ⁵   (zero vel & accel at both endpoints)
            # ṡ(τ)   = 30τ² − 60τ³ + 30τ⁴
            # s̈(τ)   = 60τ − 180τ² + 120τ³
            q_start = matrix[0]
            q_goal = matrix[1]
            delta_q = q_goal - q_start
            # Quintic peak velocity = 1.875 * avg velocity, so scale T up to keep peak within limits.
            T = self._estimate_segment_time(q_start, q_goal, velocity_scaling) * 1.875
            n_steps = 50
            for k in range(n_steps + 1):
                tau = k / n_steps
                s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
                ds = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
                dds = 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
                q = q_start + delta_q * s
                qd = delta_q * (ds / T)
                qdd = delta_q * (dds / (T * T))
                elapsed = tau * T
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in q]
                point.velocities = [float(v) for v in qd]
                point.accelerations = [float(v) for v in qdd]
                point.time_from_start = Duration(
                    sec=int(elapsed),
                    nanosec=int((elapsed - int(elapsed)) * 1_000_000_000),
                )
                trajectory.joint_trajectory.points.append(point)
        else:
            # Multi-waypoint path: central-difference velocities and accelerations.
            n = matrix.shape[0]
            timestamps = [0.0]
            for i in range(1, n):
                timestamps.append(
                    timestamps[-1] + self._estimate_segment_time(matrix[i - 1], matrix[i], velocity_scaling)
                )
            velocities = [self._np.zeros(matrix.shape[1]) for _ in range(n)]
            for i in range(1, n - 1):
                dt = timestamps[i + 1] - timestamps[i - 1]
                if dt > 1e-6:
                    velocities[i] = (matrix[i + 1] - matrix[i - 1]) / dt
            accelerations = [self._np.zeros(matrix.shape[1]) for _ in range(n)]
            for i in range(1, n - 1):
                dt = timestamps[i + 1] - timestamps[i - 1]
                if dt > 1e-6:
                    accelerations[i] = (velocities[i + 1] - velocities[i - 1]) / dt
            for i, row in enumerate(matrix):
                elapsed = timestamps[i]
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in row]
                point.velocities = [float(v) for v in velocities[i]]
                point.accelerations = [float(v) for v in accelerations[i]]
                point.time_from_start = Duration(
                    sec=int(elapsed),
                    nanosec=int((elapsed - int(elapsed)) * 1_000_000_000),
                )
                trajectory.joint_trajectory.points.append(point)
        return trajectory

    def plan_pose_trajectory(
        self,
        current_positions: dict[str, float],
        target_pose_rpy,
        velocity_scaling: float = 0.1,
    ) -> RobotTrajectory | None:
        if not self.available:
            self.node.get_logger().error(
                f"[EXOTica/{self.group_name}] plan_pose_trajectory called but planner is not available: {self.last_error}"
            )
            return None

        # Delegate IK solving to robust resolver
        target_joints_dict = self.solve_pose_goal_joint_positions(current_positions, target_pose_rpy, max_retries=10)
        
        if not target_joints_dict:
            self.node.get_logger().error(
                f"[EXOTica/{self.group_name}] Robot trajectory planning failed: could not resolve IK."
            )
            return None

        start_state = self._state_vector_from_joint_map(current_positions)
        goal_state = self._state_vector_from_joint_map(target_joints_dict)
        
        try:
            states = self._np.vstack([start_state, goal_state])
            self.last_error = None
            return self._trajectory_to_robot_trajectory(states, velocity_scaling)
        except Exception as exc:
            self.last_error = f"EXOTica single-arm trajectory generation failed: {exc}"
            return None

    def solve_pose_goal_joint_positions(
        self,
        current_positions: dict[str, float],
        target_pose_rpy,
        max_retries: int = 10,
    ) -> dict[str, float] | None:
        if not self.available:
            return None

        import time as _time
        t0 = _time.time()
        base_start_state = self._state_vector_from_joint_map(current_positions)
        target_np = self._np.asarray(target_pose_rpy, dtype=float)

        self._problem.set_goal(self.task_name, target_np)

        best_solution = None
        best_error = float('inf')

        for attempt in range(max_retries):
            if attempt == 0:
                seed = base_start_state
            else:
                noise = self._np.random.uniform(-0.5, 0.5, size=base_start_state.shape)
                seed = base_start_state + noise

            self._problem.start_state = seed
            solution = self._solver.solve()
            
            if solution is not None:
                matrix = self._np.asarray(solution, dtype=float)
                if matrix.size > 0:
                    cand_state = matrix[0] if matrix.ndim > 1 else matrix
                    cand_state = self._np.asarray(cand_state, dtype=float)
                    cand_state = self._project_goal_state_near_reference(base_start_state, cand_state)
                    
                    if self._check_joint_limits(cand_state):
                        continue
                    
                    try:
                        if hasattr(self._scene, "update"):
                            self._scene.update(cand_state)
                        if hasattr(self._scene, "is_state_valid"):
                            if not self._scene.is_state_valid(True, 0.0):
                                continue
                    except Exception:
                        pass
                    
                    # Verify FK error
                    fk_frame = self._scene.fk(self.link_name)
                    if hasattr(fk_frame, 'get_translation'):
                        achieved_pos = fk_frame.get_translation()
                    else:
                        achieved_pos = fk_frame.flatten()[:3]
                        
                    err = self._np.linalg.norm(self._np.asarray(achieved_pos[:3]) - target_np[:3])
                    
                    if err < best_error:
                        best_error = err
                        best_solution = cand_state

                    if err < 0.05:
                        break

        if best_solution is None:
            self.last_error = "EXOTica returned no pose solution after retries."
            self.node.get_logger().error(f"[EXOTica/{self.group_name}] {self.last_error}")
            return None

        if best_error >= 0.05:
            self.last_error = f"EXOTica IK residual error {best_error:.3f}m is too large"
            self.node.get_logger().error(f"[EXOTica/{self.group_name}] {self.last_error}")
            return None
            
        solve_duration = _time.time() - t0
        log_message = f"[EXOTica/{self.group_name}] IK solved in {solve_duration:.3f}s with error {best_error:.4f}m"
        if solve_duration >= 0.2:
            self.node.get_logger().warning(log_message)
        else:
            self.node.get_logger().debug(log_message)

        return {
            joint_name: float(best_solution[index])
            for index, joint_name in enumerate(self.controlled_joint_names)
        }


class RemoteExoticaIKClient:
    """Drop-in replacement for ExoticaSingleArmPosePlanner that routes IK calls to the
    EXOTica IK server node via ROS 2 topics.

    The heavy EXOTica initialization (25-30 s) is paid only once by the server; this
    client starts up near-instantly.

    Public interface mirrors ExoticaSingleArmPosePlanner:
      available, last_error, controlled_joint_names, link_name, group_name,
      solve_pose_goal_joint_positions(), plan_pose_trajectory()
    """

    # Reuse the same arm configuration as ExoticaSingleArmPosePlanner.
    _ARM_CONFIG = ExoticaSingleArmPosePlanner._ARM_CONFIG
    _CONTINUOUS_JOINTS = ExoticaSingleArmPosePlanner._CONTINUOUS_JOINTS
    _URDF_JOINT_POSITION_LIMITS = ExoticaSingleArmPosePlanner._URDF_JOINT_POSITION_LIMITS
    _LIMIT_TOLERANCE = ExoticaSingleArmPosePlanner._LIMIT_TOLERANCE

    def __init__(
        self,
        node,
        group_name: str,
        hardware_type: str = "fake",
        timeout_s: float = 5.0,
    ):
        import uuid as _uuid

        self._uuid_mod = _uuid
        self.node = node
        self.group_name = group_name
        self.hardware_type = hardware_type
        self.available = False
        self.last_error: str | None = None

        config = self._ARM_CONFIG.get(group_name)
        if config is None:
            self.last_error = f"Unsupported group for RemoteExoticaIKClient: {group_name}"
            return

        self.controlled_joint_names: list[str] = list(config["joint_names"])
        self.link_name: str = config["link_name"]
        self.task_name: str = config["task_name"]

        # numpy and joint limits (needed for _trajectory_to_robot_trajectory)
        import numpy as np

        self._np = np

        share_dir = Path(get_package_share_directory("dual_arm_moveit_config"))
        self._joint_limits: dict = yaml.safe_load(
            (share_dir / "config" / "joint_limits.yaml").read_text(encoding="utf-8")
        )["joint_limits"]

        # Pending responses keyed by request id.
        self._pending_responses: dict[str, dict] = {}
        self._pending_lock = threading.Lock()

        # Publishers / subscribers.
        self._request_pub = node.create_publisher(
            __import__("std_msgs.msg", fromlist=["String"]).String,
            "/exotica_ik/request",
            10,
        )
        self._response_sub = node.create_subscription(
            __import__("std_msgs.msg", fromlist=["String"]).String,
            "/exotica_ik/response",
            self._on_response,
            10,
        )

        # Wait for the server's /exotica/ready signal.
        self.available = self._wait_for_server(timeout_s)

    # ------------------------------------------------------------------
    # Server readiness check
    # ------------------------------------------------------------------

    def _wait_for_server(self, timeout_s: float) -> bool:
        """Block until /exotica/ready publishes True or the timeout expires."""
        import time as _time

        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import Bool

        ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ready_flag: list[bool] = [False]
        ready_event = threading.Event()

        def _ready_cb(msg: Bool):
            if msg.data:
                ready_flag[0] = True
            ready_event.set()

        sub = self.node.create_subscription(Bool, "/exotica/ready", _ready_cb, ready_qos)

        deadline = _time.monotonic() + timeout_s
        while _time.monotonic() < deadline:
            ready_event.wait(timeout=0.1)
            if ready_event.is_set():
                break

        self.node.destroy_subscription(sub)

        if not ready_flag[0]:
            self.last_error = (
                f"EXOTica IK server not ready within {timeout_s}s "
                "(is exotica_ik_server_node running?)"
            )
            self.node.get_logger().warn(
                f"[RemoteExoticaIKClient/{self.group_name}] {self.last_error}"
            )
            return False

        self.node.get_logger().info(
            f"[RemoteExoticaIKClient/{self.group_name}] Connected to EXOTica IK server."
        )
        return True

    # ------------------------------------------------------------------
    # Response subscription callback
    # ------------------------------------------------------------------

    def _on_response(self, msg) -> None:
        """Store incoming IK responses for polling by solve_pose_goal_joint_positions."""
        try:
            data = __import__("json").loads(msg.data)
            req_id = data.get("id")
            if req_id is not None:
                with self._pending_lock:
                    self._pending_responses[req_id] = data
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------------
    # Public IK interface
    # ------------------------------------------------------------------

    def solve_pose_goal_joint_positions(
        self,
        current_positions: dict[str, float],
        target_pose_rpy,
        max_retries: int = 10,  # passed to the server; not used locally
    ) -> dict[str, float] | None:
        """Send an IK request to the server and block up to 30 s for the response."""
        import json as _json
        import time as _time

        if not self.available:
            self.last_error = "RemoteExoticaIKClient is not available."
            return None

        req_id = str(self._uuid_mod.uuid4())
        request = {
            "id": req_id,
            "group": self.group_name,
            "current": {k: float(v) for k, v in current_positions.items()},
            "pose_rpy": [float(v) for v in target_pose_rpy],
        }

        from std_msgs.msg import String as _String

        out = _String()
        out.data = _json.dumps(request)
        self._request_pub.publish(out)

        # Poll for the matching response (max 30 s).
        deadline = _time.monotonic() + 30.0
        while _time.monotonic() < deadline:
            with self._pending_lock:
                response = self._pending_responses.pop(req_id, None)
            if response is not None:
                if response.get("error"):
                    self.last_error = response["error"]
                    self.node.get_logger().error(
                        f"[RemoteExoticaIKClient/{self.group_name}] "
                        f"IK server error: {self.last_error}"
                    )
                    return None
                joints = response.get("joints")
                if joints is None:
                    self.last_error = "IK server returned null joints with no error message."
                    return None
                self.last_error = None
                return {k: float(v) for k, v in joints.items()}
            _time.sleep(0.05)

        self.last_error = f"Timed out waiting for IK response (id={req_id})"
        self.node.get_logger().error(
            f"[RemoteExoticaIKClient/{self.group_name}] {self.last_error}"
        )
        return None

    def plan_pose_trajectory(
        self,
        current_positions: dict[str, float],
        target_pose_rpy,
        velocity_scaling: float = 0.1,
    ):
        """Resolve IK via the server, then build a RobotTrajectory locally."""
        if not self.available:
            self.node.get_logger().error(
                f"[RemoteExoticaIKClient/{self.group_name}] "
                "plan_pose_trajectory called but client is not available."
            )
            return None

        target_joints_dict = self.solve_pose_goal_joint_positions(
            current_positions, target_pose_rpy, max_retries=10
        )
        if not target_joints_dict:
            self.node.get_logger().error(
                f"[RemoteExoticaIKClient/{self.group_name}] "
                "Trajectory planning failed: could not resolve IK."
            )
            return None

        np = self._np
        start_state = self._state_vector_from_joint_map(current_positions)
        goal_state = self._state_vector_from_joint_map(target_joints_dict)

        try:
            states = np.vstack([start_state, goal_state])
            self.last_error = None
            return self._trajectory_to_robot_trajectory(states, velocity_scaling)
        except Exception as exc:  # noqa: BLE001
            self.last_error = f"Trajectory generation failed: {exc}"
            return None

    # ------------------------------------------------------------------
    # Helpers — copied verbatim from ExoticaSingleArmPosePlanner
    # ------------------------------------------------------------------

    def _state_vector_from_joint_map(self, joint_positions: dict[str, float]):
        np = self._np
        values = np.zeros(len(self.controlled_joint_names))
        for index, joint_name in enumerate(self.controlled_joint_names):
            values[index] = float(joint_positions.get(joint_name, 0.0))
        return values

    def _wrap_to_nearest(self, reference: float, angle: float) -> float:
        np = self._np
        return reference + ((angle - reference + np.pi) % (2.0 * np.pi) - np.pi)

    def _project_goal_state_near_reference(self, reference_state, goal_state):
        np = self._np
        projected = np.asarray(goal_state, dtype=float).copy()
        for index, joint_name in enumerate(self.controlled_joint_names):
            if joint_name not in self._CONTINUOUS_JOINTS:
                continue
            projected[index] = self._wrap_to_nearest(
                float(reference_state[index]), float(projected[index])
            )
        return projected

    def _check_joint_limits(self, goal_state) -> str | None:
        """Return an error string if any joint violates URDF position limits, else None."""
        for index, joint_name in enumerate(self.controlled_joint_names):
            limits = self._URDF_JOINT_POSITION_LIMITS.get(joint_name)
            if limits is None:
                continue
            lo, hi = limits
            val = float(goal_state[index])
            if val < lo - self._LIMIT_TOLERANCE:
                return (
                    f"{joint_name}={val:.4f} violates lower limit {lo:.4f} "
                    f"(under by {lo - val:.4f} rad)"
                )
            if val > hi + self._LIMIT_TOLERANCE:
                return (
                    f"{joint_name}={val:.4f} violates upper limit {hi:.4f} "
                    f"(over by {val - hi:.4f} rad)"
                )
        return None

    def _estimate_segment_time(self, previous_row, current_row, velocity_scaling: float) -> float:
        segment_time = 0.05
        safe_scaling = max(float(velocity_scaling), 0.01)
        for index, joint_name in enumerate(self.controlled_joint_names):
            max_velocity = float(
                self._joint_limits.get(joint_name, {}).get("max_velocity", 1.0)
            )
            delta = abs(float(current_row[index]) - float(previous_row[index]))
            segment_time = max(segment_time, delta / max(max_velocity * safe_scaling, 1e-3))
        return segment_time

    def _trajectory_to_robot_trajectory(self, states, velocity_scaling: float):
        """Build a RobotTrajectory from a (N, J) array of joint-position waypoints."""
        from builtin_interfaces.msg import Duration
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        np = self._np
        matrix = np.asarray(states, dtype=float)
        if matrix.ndim == 1:
            matrix = matrix.reshape(1, -1)

        trajectory = RobotTrajectory()
        trajectory.joint_trajectory = JointTrajectory()
        trajectory.joint_trajectory.joint_names = list(self.controlled_joint_names)

        if matrix.shape[0] == 2:
            # Single start→goal segment: quintic time-scaling for smooth S-curve motion.
            q_start = matrix[0]
            q_goal = matrix[1]
            delta_q = q_goal - q_start
            T = self._estimate_segment_time(q_start, q_goal, velocity_scaling) * 1.875
            n_steps = 50
            for k in range(n_steps + 1):
                tau = k / n_steps
                s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
                ds = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
                dds = 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
                q = q_start + delta_q * s
                qd = delta_q * (ds / T)
                qdd = delta_q * (dds / (T * T))
                elapsed = tau * T
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in q]
                point.velocities = [float(v) for v in qd]
                point.accelerations = [float(v) for v in qdd]
                point.time_from_start = Duration(
                    sec=int(elapsed),
                    nanosec=int((elapsed - int(elapsed)) * 1_000_000_000),
                )
                trajectory.joint_trajectory.points.append(point)
        else:
            # Multi-waypoint: central-difference velocities and accelerations.
            n = matrix.shape[0]
            timestamps = [0.0]
            for i in range(1, n):
                timestamps.append(
                    timestamps[-1]
                    + self._estimate_segment_time(matrix[i - 1], matrix[i], velocity_scaling)
                )
            velocities = [np.zeros(matrix.shape[1]) for _ in range(n)]
            for i in range(1, n - 1):
                dt = timestamps[i + 1] - timestamps[i - 1]
                if dt > 1e-6:
                    velocities[i] = (matrix[i + 1] - matrix[i - 1]) / dt
            accelerations = [np.zeros(matrix.shape[1]) for _ in range(n)]
            for i in range(1, n - 1):
                dt = timestamps[i + 1] - timestamps[i - 1]
                if dt > 1e-6:
                    accelerations[i] = (velocities[i + 1] - velocities[i - 1]) / dt
            for i, row in enumerate(matrix):
                elapsed = timestamps[i]
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in row]
                point.velocities = [float(v) for v in velocities[i]]
                point.accelerations = [float(v) for v in accelerations[i]]
                point.time_from_start = Duration(
                    sec=int(elapsed),
                    nanosec=int((elapsed - int(elapsed)) * 1_000_000_000),
                )
                trajectory.joint_trajectory.points.append(point)

        return trajectory
