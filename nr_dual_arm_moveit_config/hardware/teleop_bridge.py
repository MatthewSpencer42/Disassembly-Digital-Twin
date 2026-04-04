#!/usr/bin/env python3
import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TeleopBridge(Node):
    def __init__(self):
        super().__init__("teleop_bridge")

        self.active_arm = "xarm"
        self.planning_frame = "base_link"
        self.linear_scale = 1.0
        self.angular_scale = 1.0
        self.joystick_alpha = 0.12
        self.deadzone = 0.08

        self.target_linear = [0.0, 0.0, 0.0]
        self.target_angular = [0.0, 0.0, 0.0]
        self.current_linear = [0.0, 0.0, 0.0]
        self.current_angular = [0.0, 0.0, 0.0]
        self.xarm_angular_scale = 0.35
        self.uf_angular_scale = 1.0

        self.xarm_pub = self.create_publisher(TwistStamped, "/xarm_servo_node/delta_twist_cmds", 10)
        self.uf_pub = self.create_publisher(TwistStamped, "/uf_servo_node/delta_twist_cmds", 10)
        self.gripper_traj_pub = self.create_publisher(JointTrajectory, "/rg6_controller/joint_trajectory", 10)
        self.xarm_traj_pub = self.create_publisher(JointTrajectory, "/xarm5_controller/joint_trajectory", 10)
        self.uf_traj_pub = self.create_publisher(JointTrajectory, "/uf850_controller/joint_trajectory", 10)
        self.slider_traj_pub = self.create_publisher(JointTrajectory, "/slider_controller/joint_trajectory", 10)
        self.tool_pub = self.create_publisher(Int8, "tool_cmd", 10)

        self.cm_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.list_controllers_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.srv_clients = {
            "xarm_start": self.create_client(Trigger, "/xarm_servo_node/start_servo"),
            "uf_start": self.create_client(Trigger, "/uf_servo_node/start_servo"),
        }

        self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.last_buttons = []
        self.gripper_is_closed = False
        self.tool_grab_active = False
        self.tool_screw_active = False
        self.tool_unscrew_active = False
        self.deadman_active = False
        self.prev_deadman_active = False
        self.pending_home = False
        self.pending_halt_publishes = 0
        self.pending_switch_to_trajectory = False
        self.control_mode = "trajectory"

        self.create_timer(0.01, self.continuous_twist_publisher)
        self.sync_timer = self.create_timer(2.0, self.initial_sync_timer_callback)
        self.home_timer = self.create_timer(0.2, self.process_pending_home)
        self.home_timer.cancel()
        self._log_mapping()

    def initial_sync_timer_callback(self):
        self.manage_controller_mode("trajectory", start_servo=False)
        self.sync_timer.cancel()

    def manage_controller_mode(self, target_mode: str, start_servo: bool = False):
        if not self.cm_client.wait_for_service(timeout_sec=1.0):
            return
        if not self.list_controllers_client.wait_for_service(timeout_sec=1.0):
            if target_mode == "servo" and start_servo:
                self.start_active_servo()
            return
        future = self.list_controllers_client.call_async(ListControllers.Request())
        future.add_done_callback(
            lambda done: self._handle_list_controllers(done, target_mode, start_servo)
        )

    def _handle_list_controllers(self, future, target_mode: str, start_servo: bool):
        try:
            response = future.result()
        except Exception:
            if target_mode == "servo" and start_servo:
                self.start_active_servo()
            return

        if target_mode == "servo":
            required_active = {
                "xarm5_servo_controller",
                "uf850_servo_controller",
                "slider_controller",
                "rg6_controller",
            }
            required_inactive = {"xarm5_controller", "uf850_controller"}
        else:
            required_active = {
                "xarm5_controller",
                "uf850_controller",
                "slider_controller",
                "rg6_controller",
            }
            required_inactive = {"xarm5_servo_controller", "uf850_servo_controller"}

        to_activate = [
            controller.name
            for controller in response.controller
            if controller.name in required_active and controller.state == "inactive"
        ]
        to_deactivate = [
            controller.name
            for controller in response.controller
            if controller.name in required_inactive and controller.state == "active"
        ]
        if not to_activate and not to_deactivate:
            self.control_mode = target_mode
            if target_mode == "servo" and start_servo:
                self.start_active_servo()
            return
        request = SwitchController.Request()
        request.activate_controllers = to_activate
        request.deactivate_controllers = to_deactivate
        request.strictness = SwitchController.Request.BEST_EFFORT
        switch_future = self.cm_client.call_async(request)
        switch_future.add_done_callback(
            lambda done: self._after_switch(done, target_mode, start_servo)
        )

    def _after_switch(self, future, target_mode: str, start_servo: bool):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Controller switch to {target_mode} failed: {exc}")
            return
        if response is None or not response.ok:
            self.get_logger().warning(f"Controller switch to {target_mode} was not acknowledged by controller_manager")
            return
        self.control_mode = target_mode
        if target_mode == "servo" and start_servo:
            self.start_active_servo()

    def start_active_servo(self):
        for key in ("xarm_start", "uf_start"):
            client = self.srv_clients[key]
            if client.wait_for_service(timeout_sec=1.0):
                client.call_async(Trigger.Request())

    def _button_edge(self, buttons, index: int) -> bool:
        previous = self.last_buttons[index] if index < len(self.last_buttons) else 0
        return buttons[index] == 1 and previous == 0

    def apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadzone else value

    def _publish_tool_cmd(self, command: int):
        msg = Int8()
        msg.data = command
        self.tool_pub.publish(msg)

    def _log_mapping(self):
        self.get_logger().info(
            "Joystick mapping: A=deadman servo, Y=home, LB=switch arm, RB=toggle gripper, "
            "B=tool grab/release, X=tool unscrew/stop, BACK=screw/stop, "
            "left stick=(X,Y), right stick X=yaw, right stick Y=Z, D-pad=UF roll/pitch"
        )

    def _queue_servo_halt(self):
        self.target_linear = [0.0, 0.0, 0.0]
        self.target_angular = [0.0, 0.0, 0.0]
        self.current_linear = [0.0, 0.0, 0.0]
        self.current_angular = [0.0, 0.0, 0.0]
        self.pending_halt_publishes = 4

    def _publish_zero_twist(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.planning_frame
        self.xarm_pub.publish(twist)
        self.uf_pub.publish(twist)

    def joy_callback(self, msg: Joy):
        buttons = list(msg.buttons)
        axes = list(msg.axes)
        while len(buttons) < 11:
            buttons.append(0)
        while len(axes) < 8:
            axes.append(0.0)
        axes = [self.apply_deadzone(value) for value in axes]

        if self._button_edge(buttons, 3):
            self.pending_home = True
            self.home_timer.reset()
            self.manage_controller_mode("trajectory", start_servo=False)

        if self._button_edge(buttons, 1):
            self.tool_grab_active = not self.tool_grab_active
            self._publish_tool_cmd(2 if self.tool_grab_active else 3)

        if self._button_edge(buttons, 2):
            self.tool_unscrew_active = not self.tool_unscrew_active
            if self.tool_unscrew_active:
                self.tool_screw_active = False
            self._publish_tool_cmd(-1 if self.tool_unscrew_active else 0)

        if self._button_edge(buttons, 6):
            self.tool_screw_active = not self.tool_screw_active
            if self.tool_screw_active:
                self.tool_unscrew_active = False
            self._publish_tool_cmd(1 if self.tool_screw_active else 0)

        if self._button_edge(buttons, 4):
            self.active_arm = "uf" if self.active_arm == "xarm" else "xarm"
            if self.deadman_active or self.control_mode == "servo":
                self.start_active_servo()

        if self._button_edge(buttons, 5):
            self.gripper_is_closed = not self.gripper_is_closed
            position = 0.625 if self.gripper_is_closed else -0.625
            self.send_traj_goal(
                self.gripper_traj_pub,
                ["rg6_right_drive_joint"],
                [position],
                duration=0.6,
            )

        if buttons[0] == 1:
            self.deadman_active = True
            z_linear = axes[4] * self.linear_scale
            if self.active_arm == "xarm":
                z_linear = -z_linear
            angular_scale = self.xarm_angular_scale if self.active_arm == "xarm" else self.uf_angular_scale
            self.target_linear = [
                axes[1] * self.linear_scale,
                axes[0] * self.linear_scale,
                z_linear,
            ]
            self.target_angular = [0.0, 0.0, axes[3] * angular_scale]
            if self.active_arm == "uf":
                self.target_angular[0] = axes[6] * angular_scale
                self.target_angular[1] = axes[7] * angular_scale
        else:
            self.deadman_active = False
            self.target_linear = [0.0, 0.0, 0.0]
            self.target_angular = [0.0, 0.0, 0.0]

        if not self.prev_deadman_active and self.deadman_active:
            self.pending_switch_to_trajectory = False
            self.manage_controller_mode("servo", start_servo=True)

        if self.prev_deadman_active and not self.deadman_active:
            self._queue_servo_halt()
            self.pending_switch_to_trajectory = True

        self.prev_deadman_active = self.deadman_active
        self.last_buttons = buttons

    def process_pending_home(self):
        if not self.pending_home:
            return
        self.pending_home = False
        self.home_timer.cancel()
        self.send_traj_goal(self.slider_traj_pub, ["uf_slide_joint"], [0.054], duration=3.0)
        self.send_traj_goal(
            self.xarm_traj_pub,
            ["xarm5_joint1", "xarm5_joint2", "xarm5_joint3", "xarm5_joint4", "xarm5_joint5"],
            [0.0, 0.0, -1.57, 1.57, 0.0],
            duration=4.0,
        )
        self.send_traj_goal(
            self.uf_traj_pub,
            ["uf850_joint1", "uf850_joint2", "uf850_joint3", "uf850_joint4", "uf850_joint5", "uf850_joint6"],
            [0.0, 0.0, -1.57, 0.0, -1.57, 0.0],
            duration=4.0,
        )

    def continuous_twist_publisher(self):
        if self.pending_halt_publishes > 0:
            self._publish_zero_twist()
            self.pending_halt_publishes -= 1
            if self.pending_halt_publishes == 0 and self.pending_switch_to_trajectory:
                self.pending_switch_to_trajectory = False
                self.manage_controller_mode("trajectory", start_servo=False)
            return

        for index in range(3):
            self.current_linear[index] = (
                self.joystick_alpha * self.target_linear[index]
                + (1.0 - self.joystick_alpha) * self.current_linear[index]
            )
            self.current_angular[index] = (
                self.joystick_alpha * self.target_angular[index]
                + (1.0 - self.joystick_alpha) * self.current_angular[index]
            )

        if not self.deadman_active:
            return

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.planning_frame
        twist.twist.linear.x = self.current_linear[0]
        twist.twist.linear.y = self.current_linear[1]
        twist.twist.linear.z = self.current_linear[2]
        twist.twist.angular.x = self.current_angular[0]
        twist.twist.angular.y = self.current_angular[1]
        twist.twist.angular.z = self.current_angular[2]
        (self.xarm_pub if self.active_arm == "xarm" else self.uf_pub).publish(twist)

    def send_traj_goal(self, publisher, joint_names, positions, duration: float = 4.0):
        message = JointTrajectory()
        message.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in positions]
        point.time_from_start = Duration(seconds=duration).to_msg()
        message.points.append(point)
        publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
