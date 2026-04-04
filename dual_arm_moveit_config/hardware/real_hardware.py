#!/usr/bin/env python3
import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String

try:
    from xarm.wrapper import XArmAPI
except Exception:
    XArmAPI = None

try:
    from pymodbus.client.sync import ModbusTcpClient
except Exception:
    try:
        from pymodbus.client import ModbusTcpClient
    except Exception:
        ModbusTcpClient = None


XARM_IP = "192.168.1.239"
UF850_IP = "192.168.1.195"
GRIPPER_IP = "192.168.1.1"
RG6_SLAVE_ID = 65

RAD_OPEN = -0.625
RAD_CLOSE = 0.625
MM_OPEN = 160.0
MM_CLOSE = 0.0

SLIDER_MOVEIT_MIN_M = 0.054
SLIDER_MOVEIT_MAX_M = 0.75
SLIDER_TRAVEL_MM = 700.0

MAX_RAD_JUMP = 0.4
ARM_JOINT_SPEED_RAD_S = 0.65
ARM_JOINT_ACC_RAD_S2 = 2.0
SLIDER_SPEED_MM_S = 120
DEFAULT_GRIPPER_FORCE_N = 40.0
ARM_COMMAND_ARM_DELAY_S = 8.0
SLIDER_COMMAND_ARM_DELAY_S = 8.0
GRIPPER_COMMAND_ARM_DELAY_S = 6.0
GRIPPER_WIDTH_EPS_MM = 0.5
SLIDER_POSITION_EPS_M = 0.0005
ARM_COMMAND_EPS_RAD = 0.002
ARM_JOINT_LIMIT_MARGIN_RAD = 0.01

STARTUP_STATE_TIMEOUT_S = 5.0
STATE_PUBLISH_PERIOD_S = 0.02

JOINT_ORDER = [
    "uf_slide_joint",
    "xarm5_joint1",
    "xarm5_joint2",
    "xarm5_joint3",
    "xarm5_joint4",
    "xarm5_joint5",
    "uf850_joint1",
    "uf850_joint2",
    "uf850_joint3",
    "uf850_joint4",
    "uf850_joint5",
    "uf850_joint6",
    "rg6_right_drive_joint",
]

XARM_JOINTS = ["xarm5_joint1", "xarm5_joint2", "xarm5_joint3", "xarm5_joint4", "xarm5_joint5"]
UF850_JOINTS = [
    "uf850_joint1",
    "uf850_joint2",
    "uf850_joint3",
    "uf850_joint4",
    "uf850_joint5",
    "uf850_joint6",
]

XARM_JOINT_LIMITS_RAD = {
    "xarm5_joint1": (-6.2831853, 6.2831853),
    "xarm5_joint2": (-2.0594885, 2.0943951),
    "xarm5_joint3": (-3.9269908, 0.19198622),
    "xarm5_joint4": (-1.6929694, 3.1415927),
    "xarm5_joint5": (-6.2831853, 6.2831853),
}

UF850_JOINT_LIMITS_RAD = {
    "uf850_joint1": (-6.2831853, 6.2831853),
    "uf850_joint2": (-2.3038346, 2.3038346),
    "uf850_joint3": (-4.2236968, 0.061086524),
    "uf850_joint4": (-6.2831853, 6.2831853),
    "uf850_joint5": (-2.1642083, 2.1642083),
    "uf850_joint6": (-6.2831853, 6.2831853),
}

INITIAL_POSITIONS = {
    "uf_slide_joint": 0.054,
    "xarm5_joint1": 0.0,
    "xarm5_joint2": 0.0,
    "xarm5_joint3": -1.57079632679,
    "xarm5_joint4": 1.57079632679,
    "xarm5_joint5": 0.0,
    "uf850_joint1": 0.0,
    "uf850_joint2": 0.0,
    "uf850_joint3": -1.57079632679,
    "uf850_joint4": 0.0,
    "uf850_joint5": -1.57079632679,
    "uf850_joint6": 0.0,
    "rg6_right_drive_joint": RAD_OPEN,
}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(value, upper))


def signed_16bit(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def throttle(logger_state: dict, key: str, interval_s: float) -> bool:
    now = time.time()
    last = logger_state.get(key, 0.0)
    if now - last >= interval_s:
        logger_state[key] = now
        return True
    return False


class RGBridge:
    # Poll interval for the background Modbus read thread.
    # 20 Hz is sufficient for gripper state feedback and keeps the
    # 4 sequential Modbus reads (~4-8 ms total) off the 50 Hz ROS timer.
    _POLL_INTERVAL_S = 0.05

    def __init__(self, ip: str, logger):
        self.ip = ip
        self.logger = logger
        self.client = None
        self.connected = False
        self.target_force_n = DEFAULT_GRIPPER_FORCE_N
        self.target_width_mm = MM_OPEN
        self.last_width_mm = MM_OPEN
        self.last_width_with_offset_mm = MM_OPEN
        self.fingertip_offset_mm = 0.0
        self.status_raw = 0
        self.is_moving = False
        self.object_detected = False
        self.s1_pushed = False
        self.s1_triggered = False
        self.s2_pushed = False
        self.s2_triggered = False
        self.safety_error = False
        self._log_state = {}
        self.pending_width_mm = None
        self.last_sent_width_mm = None
        self._poll_lock = threading.Lock()
        self._stop_poll = threading.Event()
        self._poll_thread = None

        if ModbusTcpClient is None:
            self.logger.warning("RG6: pymodbus client unavailable, using shadow state")
            return

        try:
            self.client = ModbusTcpClient(ip, port=502, timeout=1)
            self.connected = bool(self.client.connect())
            if not self.connected:
                self.logger.warning(f"RG6: failed to connect to {ip}, using shadow state")
            else:
                self._refresh_state_locked()
                self._poll_thread = threading.Thread(
                    target=self._poll_loop, daemon=True, name="rg6_poll"
                )
                self._poll_thread.start()
        except Exception as exc:
            self.logger.warning(f"RG6: connection failed, using shadow state: {exc}")
            self.client = None
            self.connected = False

    def stop(self):
        """Signal the background poll thread to exit."""
        self._stop_poll.set()

    def _poll_loop(self):
        while not self._stop_poll.is_set():
            start = time.time()
            with self._poll_lock:
                self._refresh_state_locked()
                self.process_pending_command()
            elapsed = time.time() - start
            sleep_time = self._POLL_INTERVAL_S - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def set_force(self, force_n: float):
        self.target_force_n = clamp(force_n, 0.0, 120.0)

    def set_width_mm(self, width_mm: float):
        self.target_width_mm = clamp(width_mm, MM_CLOSE, MM_OPEN)
        self.pending_width_mm = self.target_width_mm
        if self.client is None:
            self.last_width_with_offset_mm = self.target_width_mm
            return

    def process_pending_command(self):
        if self.client is None or self.pending_width_mm is None:
            return
        if self.is_moving:
            return
        if self.last_sent_width_mm is not None and abs(self.pending_width_mm - self.last_sent_width_mm) < GRIPPER_WIDTH_EPS_MM:
            self.pending_width_mm = None
            return

        try:
            raw_force = int(round(self.target_force_n * 10.0))
            raw_width = int(round(self.pending_width_mm * 10.0))
            response = self.client.write_registers(0, [raw_force, raw_width, 16], unit=RG6_SLAVE_ID)
            if getattr(response, "isError", lambda: False)():
                if throttle(self._log_state, "write_err", 1.0):
                    self.logger.warning(f"RG6: Modbus write returned error response: {response}")
                return
            self.last_sent_width_mm = self.pending_width_mm
            self.pending_width_mm = None
        except Exception as exc:
            if throttle(self._log_state, "write_warn", 2.0):
                self.logger.warning(f"RG6: Modbus write failed, keeping pending target: {exc}")

    def _refresh_state_locked(self):
        """Read Modbus registers. Must be called with _poll_lock held (or before thread starts)."""
        if self.client is None:
            return

        try:
            offset = self.client.read_holding_registers(258, 1, unit=RG6_SLAVE_ID)
            width = self.client.read_holding_registers(267, 1, unit=RG6_SLAVE_ID)
            status = self.client.read_holding_registers(268, 1, unit=RG6_SLAVE_ID)
            width_with_offset = self.client.read_holding_registers(275, 1, unit=RG6_SLAVE_ID)
            if not getattr(offset, "isError", lambda: False)():
                self.fingertip_offset_mm = float(signed_16bit(offset.registers[0])) / 10.0
            if not getattr(width, "isError", lambda: False)():
                self.last_width_mm = float(width.registers[0]) / 10.0
            if not getattr(width_with_offset, "isError", lambda: False)():
                self.last_width_with_offset_mm = float(signed_16bit(width_with_offset.registers[0])) / 10.0
            if not getattr(status, "isError", lambda: False)():
                self.status_raw = int(status.registers[0])
                self.is_moving = bool(self.status_raw & (1 << 0))
                self.object_detected = bool(self.status_raw & (1 << 1))
                self.s1_pushed = bool(self.status_raw & (1 << 2))
                self.s1_triggered = bool(self.status_raw & (1 << 3))
                self.s2_pushed = bool(self.status_raw & (1 << 4))
                self.s2_triggered = bool(self.status_raw & (1 << 5))
                self.safety_error = bool(self.status_raw & (1 << 6))
        except Exception as exc:
            if throttle(self._log_state, "read_warn", 2.0):
                self.logger.warning(f"RG6: failed reading status registers: {exc}")

    def refresh_state(self):
        """No-op: state is kept fresh by the background poll thread."""

    def get_state(self):
        return {
            "connected": self.connected,
            "width_mm": self.last_width_mm,
            "width_with_offset_mm": self.last_width_with_offset_mm,
            "fingertip_offset_mm": self.fingertip_offset_mm,
            "target_width_mm": self.target_width_mm,
            "target_force_n": self.target_force_n,
            "status_raw": self.status_raw,
            "is_moving": self.is_moving,
            "object_detected": self.object_detected,
            "s1_pushed": self.s1_pushed,
            "s1_triggered": self.s1_triggered,
            "s2_pushed": self.s2_pushed,
            "s2_triggered": self.s2_triggered,
            "safety_error": self.safety_error,
        }


class ArmBridge:
    def __init__(self, ip: str, name: str, joint_names, logger, stream_mode: int = 1):
        self.ip = ip
        self.name = name
        self.joint_names = list(joint_names)
        self.logger = logger
        self.stream_mode = stream_mode
        self.lock = threading.Lock()
        self.api = None
        self.connected = False
        self.shadow_positions = [INITIAL_POSITIONS[joint_name] for joint_name in self.joint_names]
        self.shadow_velocities = [0.0] * len(self.joint_names)
        self.shadow_efforts = [0.0] * len(self.joint_names)
        self.prev_positions = list(self.shadow_positions)
        self.prev_time = time.time()
        self.effort_alpha = 0.35
        self.effort_deadband = 0.15
        self.has_valid_state = False
        self._log_state = {}
        self.joint_limits = self._resolve_joint_limits()
        # True once the arm is confirmed in the correct mode/state.
        # Reset to False on any non-zero SDK command return so recovery
        # is triggered on the next send without polling on every command.
        self._mode_ok = False

        if XArmAPI is None:
            self.logger.warning(f"{name}: xArm SDK unavailable, using shadow state")
            return

        try:
            self.api = XArmAPI(ip)
            self.connected = bool(self.api.connected)
            if not self.connected:
                self.logger.warning(f"{name}: failed to connect to {ip}, using shadow state")
                self.api = None
                return
            self._initialize_arm()
        except Exception as exc:
            self.logger.warning(f"{name}: failed to connect to {ip}, using shadow state: {exc}")
            self.api = None
            self.connected = False

    def _resolve_joint_limits(self):
        if self.joint_names == XARM_JOINTS:
            return [XARM_JOINT_LIMITS_RAD[name] for name in self.joint_names]
        if self.joint_names == UF850_JOINTS:
            return [UF850_JOINT_LIMITS_RAD[name] for name in self.joint_names]
        return [(-6.2831853, 6.2831853) for _ in self.joint_names]

    def _unwrap_result(self, result):
        if isinstance(result, tuple):
            if not result:
                return None, None
            if len(result) == 1:
                return result[0], None
            return result[0], result[1]
        return result, None

    def _call_api(self, action_name: str, *args, **kwargs):
        if self.api is None:
            return None, None
        fn = getattr(self.api, action_name)
        return self._unwrap_result(fn(*args, **kwargs))

    def _initialize_arm(self):
        for action_name, args in (
            ("clean_warn", ()),
            ("clean_error", ()),
            ("motion_enable", (True,)),
            ("set_mode", (self.stream_mode,)),
            ("set_state", (0,)),
        ):
            try:
                code, _ = self._call_api(action_name, *args)
            except Exception as exc:
                self.logger.warning(f"{self.name}: {action_name}{args} raised: {exc}")
                continue
            if code not in (None, 0):
                self.logger.warning(f"{self.name}: {action_name}{args} returned SDK code {code}")
        self._mode_ok = True
        self.logger.info(f"{self.name}: SDK bridge initialized for {self.ip}")

    def _ensure_ready_for_command(self, desired_mode=None):
        if self.api is None:
            return False
        if desired_mode is None:
            desired_mode = self.stream_mode

        state_code, state_value = self._call_api("get_state")
        err_code, err_warn = self._call_api("get_err_warn_code")
        robot_error = err_warn[0] if isinstance(err_warn, (list, tuple)) and err_warn else 0
        if err_code == 0 and robot_error:
            if throttle(self._log_state, "recover_warn", 1.0):
                self.logger.warning(f"{self.name}: controller error {robot_error}, attempting recovery")
            self._call_api("clean_error")
            self._call_api("motion_enable", True)
            self._call_api("set_mode", desired_mode)
            self._call_api("set_state", 0)
        elif state_code == 0 and state_value not in (0, 1):
            self._call_api("motion_enable", True)
            self._call_api("set_mode", desired_mode)
            self._call_api("set_state", 0)
        return True

    def read_state(self):
        if self.api is None:
            return list(self.shadow_positions), list(self.shadow_velocities), list(self.shadow_efforts), False

        try:
            now = time.time()
            dt = max(now - self.prev_time, 1e-3)
            code, joint_state = self._call_api("get_joint_states", is_radian=True, num=3)
            if (
                code != 0
                or not isinstance(joint_state, (list, tuple))
                or len(joint_state) < 3
                or not isinstance(joint_state[0], (list, tuple))
            ):
                if throttle(self._log_state, "joint_state_warn", 2.0):
                    self.logger.warning(f"{self.name}: get_joint_states returned SDK code {code}, falling back to get_servo_angle/get_joints_torque")
                code_p, positions = self._call_api("get_servo_angle", is_radian=True, is_real=True)
                code_t, efforts = self._call_api("get_joints_torque")
                if (
                    code_p != 0
                    or not isinstance(positions, (list, tuple))
                    or len(positions) < len(self.joint_names)
                ):
                    if throttle(self._log_state, "read_warn", 2.0):
                        self.logger.warning(f"{self.name}: live joint read returned SDK code {code_p}")
                    return list(self.shadow_positions), list(self.shadow_velocities), list(self.shadow_efforts), False
                velocities = [
                    (float(positions[index]) - float(self.prev_positions[index])) / dt
                    for index in range(len(self.joint_names))
                ]
                efforts = list(efforts[: len(self.joint_names)]) if isinstance(efforts, (list, tuple)) else [0.0] * len(self.joint_names)
            else:
                positions = joint_state[0]
                velocities = joint_state[1]
                efforts = joint_state[2]

            if not isinstance(positions, (list, tuple)) or len(positions) < len(self.joint_names):
                if throttle(self._log_state, "read_warn", 2.0):
                    self.logger.warning(f"{self.name}: invalid joint position payload from SDK")
                return list(self.shadow_positions), list(self.shadow_velocities), list(self.shadow_efforts), False

            values = [float(value) for value in positions[: len(self.joint_names)]]
            velocity_values = [
                float(velocities[index]) if isinstance(velocities, (list, tuple)) and len(velocities) > index else 0.0
                for index in range(len(self.joint_names))
            ]
            effort_values = []
            for index in range(len(self.joint_names)):
                raw_effort = (
                    float(efforts[index])
                    if isinstance(efforts, (list, tuple)) and len(efforts) > index
                    else 0.0
                )
                filtered = 0.0 if abs(raw_effort) < self.effort_deadband else raw_effort
                smoothed = (self.effort_alpha * filtered) + ((1.0 - self.effort_alpha) * self.shadow_efforts[index])
                effort_values.append(round(smoothed, 3))
            self.shadow_positions = values
            self.shadow_velocities = velocity_values
            self.shadow_efforts = effort_values
            self.prev_positions = list(values)
            self.prev_time = now
            self.has_valid_state = True
            return values, list(self.shadow_velocities), list(self.shadow_efforts), True
        except Exception as exc:
            if throttle(self._log_state, "read_exc", 2.0):
                self.logger.warning(f"{self.name}: failed to read live joint state: {exc}")
            return list(self.shadow_positions), list(self.shadow_velocities), list(self.shadow_efforts), False

    def send_positions(self, positions):
        if self.api is None:
            self.shadow_positions = [float(value) for value in positions]
            return False

        with self.lock:
            filtered = []
            clamp_applied = False
            for index, value in enumerate(positions):
                lower, upper = self.joint_limits[index]
                safe_lower = lower + ARM_JOINT_LIMIT_MARGIN_RAD
                safe_upper = upper - ARM_JOINT_LIMIT_MARGIN_RAD
                clamped_value = clamp(float(value), safe_lower, safe_upper)
                if abs(clamped_value - float(value)) > 1e-6:
                    clamp_applied = True
                filtered.append(clamped_value)
            if clamp_applied and throttle(self._log_state, "joint_limit_clamp", 1.0):
                self.logger.warning(
                    f"{self.name}: clamped joint command to hardware-safe limits: {filtered}"
                )
            # Only run the 2-network-call recovery check when the mode is
            # known bad (after a failed command).  This removes 2 blocking
            # SDK round-trips from every 100 Hz command cycle, eliminating
            # the primary source of command-timing jitter.
            if not self._mode_ok:
                self._ensure_ready_for_command(desired_mode=1)
                self._mode_ok = True
            try:
                code, _ = self._call_api(
                    "set_servo_angle_j",
                    angles=filtered,
                    is_radian=True,
                )
                if code == 0:
                    self.shadow_positions = filtered
                    return True
                # Non-zero code → the arm may have faulted; force recovery next cycle.
                self._mode_ok = False
                if throttle(self._log_state, "command_warn", 1.0):
                    self.logger.warning(
                        f"{self.name}: set_servo_angle_j returned SDK code {code} for target {filtered}"
                    )
                return False
            except Exception as exc:
                self._mode_ok = False
                if throttle(self._log_state, "command_exc", 1.0):
                    self.logger.warning(f"{self.name}: failed to send joint targets: {exc}")
                return False

    def send_velocities(self, velocities, duration_s: float = 0.2):
        if self.api is None:
            self.shadow_velocities = [float(value) for value in velocities]
            return False

        with self.lock:
            filtered = []
            max_velocity = max(0.01, ARM_JOINT_SPEED_RAD_S * 0.75)
            for value in velocities:
                filtered.append(clamp(float(value), -max_velocity, max_velocity))
            # Velocity mode (4) differs from position mode (1); always recover
            # when switching modes, otherwise only recover on detected fault.
            if not self._mode_ok or self.stream_mode != 4:
                self._ensure_ready_for_command(desired_mode=4)
                self._mode_ok = True
            try:
                code, _ = self._call_api(
                    "vc_set_joint_velocity",
                    filtered,
                    is_radian=True,
                    is_sync=True,
                    duration=max(0.05, float(duration_s)),
                )
                if code == 0:
                    self.shadow_velocities = list(filtered)
                    return True
                self._mode_ok = False
                if throttle(self._log_state, "velocity_warn", 1.0):
                    self.logger.warning(
                        f"{self.name}: vc_set_joint_velocity returned SDK code {code} for target {filtered}"
                    )
                return False
            except Exception as exc:
                self._mode_ok = False
                if throttle(self._log_state, "velocity_exc", 1.0):
                    self.logger.warning(f"{self.name}: failed to send joint velocities: {exc}")
                return False


class SliderBridge:
    def __init__(self, arm_bridge: ArmBridge, logger):
        self.arm_bridge = arm_bridge
        self.logger = logger
        self.available = False
        self.on_zero = False
        self.shadow_position_m = INITIAL_POSITIONS["uf_slide_joint"]
        self._log_state = {}
        self.pending_target_joint_m = None
        self.last_sent_target_joint_m = None

        if self.arm_bridge.api is None:
            self.logger.warning("slider: xArm controller unavailable, using shadow state")
            return

        self._discover()

    def _discover(self):
        code, status = self.arm_bridge._call_api("get_linear_motor_status")
        if code != 0:
            self.logger.warning(
                f"slider: linear motor unavailable on {self.arm_bridge.name} controller, SDK code {code}"
            )
            return

        self.available = True
        enable_code, enabled = self.arm_bridge._call_api("get_linear_motor_is_enabled")
        if enable_code == 0 and not enabled:
            code, _ = self.arm_bridge._call_api("set_linear_motor_enable", True)
            if code not in (None, 0):
                self.logger.warning(f"slider: failed to enable linear motor, SDK code {code}")
        zero_code, on_zero = self.arm_bridge._call_api("get_linear_motor_on_zero")
        if zero_code == 0:
            self.on_zero = bool(on_zero)
            if not self.on_zero:
                self.logger.warning("slider: linear motor is not homed; attempting automatic homing")
                clean_code, _ = self.arm_bridge._call_api("clean_linear_motor_error")
                if clean_code not in (None, 0):
                    self.logger.warning(f"slider: failed to clear linear motor error, SDK code {clean_code}")
                home_code, _ = self.arm_bridge._call_api("set_linear_motor_back_origin", wait=True)
                if home_code not in (None, 0):
                    self.logger.warning(f"slider: automatic homing failed, SDK code {home_code}")
                else:
                    zero_code, on_zero = self.arm_bridge._call_api("get_linear_motor_on_zero")
                    if zero_code == 0:
                        self.on_zero = bool(on_zero)
                    if self.on_zero:
                        self.logger.info("slider: linear motor homed successfully")
                    else:
                        self.logger.warning("slider: automatic homing finished but on_zero is still false")
        speed_code, _ = self.arm_bridge._call_api("set_linear_motor_speed", SLIDER_SPEED_MM_S)
        if speed_code not in (None, 0):
            self.logger.warning(f"slider: failed to set linear motor speed, SDK code {speed_code}")
        self.logger.info(f"slider: linear motor available via {self.arm_bridge.name} controller")

    def _mm_to_joint(self, slider_mm: float) -> float:
        slider_mm = clamp(slider_mm, 0.0, SLIDER_TRAVEL_MM)
        ratio = slider_mm / SLIDER_TRAVEL_MM
        return SLIDER_MOVEIT_MIN_M + ratio * (SLIDER_MOVEIT_MAX_M - SLIDER_MOVEIT_MIN_M)

    def _joint_to_mm(self, slider_joint_m: float) -> int:
        slider_joint_m = clamp(slider_joint_m, SLIDER_MOVEIT_MIN_M, SLIDER_MOVEIT_MAX_M)
        ratio = (slider_joint_m - SLIDER_MOVEIT_MIN_M) / (SLIDER_MOVEIT_MAX_M - SLIDER_MOVEIT_MIN_M)
        return int(round(ratio * SLIDER_TRAVEL_MM))

    def read_position(self):
        if not self.available:
            return self.shadow_position_m, False

        zero_code, on_zero = self.arm_bridge._call_api("get_linear_motor_on_zero")
        if zero_code == 0:
            self.on_zero = bool(on_zero)

        code, pos_mm = self.arm_bridge._call_api("get_linear_motor_pos")
        if code != 0:
            if throttle(self._log_state, "read_warn", 2.0):
                self.logger.warning(f"slider: get_linear_motor_pos returned SDK code {code}")
            return self.shadow_position_m, False

        self.shadow_position_m = self._mm_to_joint(float(pos_mm))
        return self.shadow_position_m, self.on_zero

    def send_position(self, target_joint_m: float):
        target_joint_m = clamp(target_joint_m, SLIDER_MOVEIT_MIN_M, SLIDER_MOVEIT_MAX_M)
        if not self.available:
            self.shadow_position_m = target_joint_m
            return False
        if not self.on_zero:
            if throttle(self._log_state, "home_warn", 2.0):
                self.logger.warning("slider: refusing command because the linear motor is not homed")
            return False
        self.pending_target_joint_m = target_joint_m
        return self.process_pending_command()

    def process_pending_command(self):
        if self.pending_target_joint_m is None:
            return False
        if self.last_sent_target_joint_m is not None and abs(self.pending_target_joint_m - self.last_sent_target_joint_m) < SLIDER_POSITION_EPS_M:
            self.pending_target_joint_m = None
            return False

        target_mm = self._joint_to_mm(self.pending_target_joint_m)
        code, _ = self.arm_bridge._call_api(
            "set_linear_motor_pos",
            target_mm,
            speed=SLIDER_SPEED_MM_S,
            wait=False,
        )
        if code == 0:
            self.shadow_position_m = self._mm_to_joint(target_mm)
            self.last_sent_target_joint_m = self.pending_target_joint_m
            self.pending_target_joint_m = None
            return True

        if throttle(self._log_state, "command_warn", 1.0):
            self.logger.warning(f"slider: set_linear_motor_pos returned SDK code {code} for {target_mm} mm")
        return False


class RealHardware(Node):
    def __init__(self):
        super().__init__("real_hardware_bridge")
        self.joint_state_pub = self.create_publisher(JointState, "/robot_joint_states", 10)
        self.gripper_state_pub = self.create_publisher(String, "/rg6/state", 10)
        self.grip_detected_pub = self.create_publisher(Bool, "/rg6/grip_detected", 10)
        self.create_subscription(JointState, "/robot_joint_commands", self.handle_joint_command, 10)
        self.create_subscription(JointState, "/robot_joint_velocity_commands", self.handle_joint_velocity_command, 10)
        self.create_subscription(Float32, "/rg6/force_command", self.handle_gripper_force, 10)

        self.xarm = ArmBridge(XARM_IP, "xarm5", XARM_JOINTS, self.get_logger(), stream_mode=1)
        self.uf = ArmBridge(UF850_IP, "uf850", UF850_JOINTS, self.get_logger(), stream_mode=1)
        self.slider = SliderBridge(self.xarm, self.get_logger())
        self.rg6 = RGBridge(GRIPPER_IP, self.get_logger())

        self.slider_position = INITIAL_POSITIONS["uf_slide_joint"]
        self.last_xarm_command = None
        self.last_uf_command = None
        self.last_slider_command = None
        self.last_gripper_command = None
        self.logged_waiting_for_state = False
        self.start_time = time.time()
        self.startup_timeout_warned = False
        self._log_state = {}

        self.create_timer(STATE_PUBLISH_PERIOD_S, self.publish_state)

    def rad_to_width_mm(self, value: float) -> float:
        normalized = (clamp(value, RAD_OPEN, RAD_CLOSE) - RAD_CLOSE) / (RAD_OPEN - RAD_CLOSE)
        return MM_CLOSE + normalized * (MM_OPEN - MM_CLOSE)

    def width_mm_to_rad(self, width_mm: float) -> float:
        normalized = (clamp(width_mm, MM_CLOSE, MM_OPEN) - MM_CLOSE) / (MM_OPEN - MM_CLOSE)
        return RAD_CLOSE + normalized * (RAD_OPEN - RAD_CLOSE)

    def handle_gripper_force(self, msg: Float32):
        self.rg6.set_force(msg.data)

    def handle_joint_command(self, msg: JointState):
        command_map = {
            name: msg.position[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.position)
        }
        if not command_map:
            return

        xarm_targets = [
            float(command_map.get(name, self.xarm.shadow_positions[index]))
            for index, name in enumerate(self.xarm.joint_names)
        ]
        uf_targets = [
            float(command_map.get(name, self.uf.shadow_positions[index]))
            for index, name in enumerate(self.uf.joint_names)
        ]

        if time.time() - self.start_time >= ARM_COMMAND_ARM_DELAY_S:
            if (
                self.last_xarm_command is None
                or any(abs(target - last) > ARM_COMMAND_EPS_RAD for target, last in zip(xarm_targets, self.last_xarm_command))
            ) and all(abs(target - current) < MAX_RAD_JUMP for target, current in zip(xarm_targets, self.xarm.shadow_positions)):
                self.xarm.send_positions(xarm_targets)
                self.last_xarm_command = list(xarm_targets)

            if (
                self.last_uf_command is None
                or any(abs(target - last) > ARM_COMMAND_EPS_RAD for target, last in zip(uf_targets, self.last_uf_command))
            ) and all(abs(target - current) < MAX_RAD_JUMP for target, current in zip(uf_targets, self.uf.shadow_positions)):
                self.uf.send_positions(uf_targets)
                self.last_uf_command = list(uf_targets)
        else:
            self.last_xarm_command = list(xarm_targets)
            self.last_uf_command = list(uf_targets)

        if "uf_slide_joint" in command_map:
            slider_command = float(command_map["uf_slide_joint"])
            if time.time() - self.start_time >= SLIDER_COMMAND_ARM_DELAY_S:
                if self.last_slider_command is None or abs(slider_command - self.last_slider_command) > SLIDER_POSITION_EPS_M:
                    self.slider.send_position(slider_command)
                    self.last_slider_command = slider_command
            else:
                self.last_slider_command = slider_command

        if "rg6_right_drive_joint" in command_map:
            if time.time() - self.start_time < GRIPPER_COMMAND_ARM_DELAY_S:
                self.last_gripper_command = float(command_map["rg6_right_drive_joint"])
                return
            command = float(command_map["rg6_right_drive_joint"])
            if self.last_gripper_command is None or abs(command - self.last_gripper_command) > 1e-3:
                self.rg6.set_width_mm(self.rad_to_width_mm(command))
                self.last_gripper_command = command

    def handle_joint_velocity_command(self, msg: JointState):
        velocity_map = {
            name: msg.velocity[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.velocity)
        }
        if not velocity_map:
            return

        xarm_velocities = [
            float(velocity_map.get(name, 0.0))
            for name in self.xarm.joint_names
        ]
        uf_velocities = [
            float(velocity_map.get(name, 0.0))
            for name in self.uf.joint_names
        ]

        if time.time() - self.start_time >= ARM_COMMAND_ARM_DELAY_S:
            self.xarm.send_velocities(xarm_velocities, duration_s=0.15)
            self.uf.send_velocities(uf_velocities, duration_s=0.15)

    def publish_state(self):
        xarm_positions, xarm_velocities, xarm_efforts, xarm_valid = self.xarm.read_state()
        uf_positions, uf_velocities, uf_efforts, uf_valid = self.uf.read_state()
        slider_position, slider_valid = self.slider.read_position()
        self.slider_position = slider_position
        # rg6.refresh_state() and rg6.process_pending_command() are called
        # from the RGBridge background poll thread at 20 Hz to avoid blocking
        # this 50 Hz timer callback with sequential Modbus round-trips.
        self.slider.process_pending_command()

        live_sources = []
        if xarm_valid:
            live_sources.append("xarm5")
        if uf_valid:
            live_sources.append("uf850")
        if slider_valid:
            live_sources.append("slider")

        if xarm_valid and uf_valid:
            if self.logged_waiting_for_state:
                self.get_logger().info(
                    f"Live joint state received from {', '.join(live_sources)}; publishing /robot_joint_states"
                )
                self.logged_waiting_for_state = False
        else:
            if not self.logged_waiting_for_state:
                waiting_on = []
                if not xarm_valid:
                    waiting_on.append("xarm5")
                if not uf_valid:
                    waiting_on.append("uf850")
                if not slider_valid:
                    waiting_on.append("slider")
                self.get_logger().warning(
                    f"Waiting for valid live state from: {', '.join(waiting_on)}. "
                    "Publishing startup/shadow values for any device that is not yet reporting."
                )
                self.logged_waiting_for_state = True
            if not self.startup_timeout_warned and time.time() - self.start_time > STARTUP_STATE_TIMEOUT_S:
                self.get_logger().warning(
                    "Live state startup timeout reached. Continuing with mixed real/shadow state for "
                    "devices that are still not reporting."
                )
                self.startup_timeout_warned = True

        gripper_position = self.width_mm_to_rad(self.rg6.last_width_with_offset_mm)

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_ORDER)
        message.position = [
            self.slider_position,
            *xarm_positions,
            *uf_positions,
            gripper_position,
        ]
        message.velocity = [
            0.0,
            *xarm_velocities,
            *uf_velocities,
            0.0,
        ]
        message.effort = [
            0.0,
            *xarm_efforts,
            *uf_efforts,
            0.0,
        ]
        self.joint_state_pub.publish(message)

        gripper_state = String()
        gripper_state.data = json.dumps(self.rg6.get_state())
        self.gripper_state_pub.publish(gripper_state)
        self.grip_detected_pub.publish(Bool(data=bool(self.rg6.object_detected)))


def main(args=None):
    rclpy.init(args=args)
    node = RealHardware()
    try:
        rclpy.spin(node)
    finally:
        node.rg6.stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
