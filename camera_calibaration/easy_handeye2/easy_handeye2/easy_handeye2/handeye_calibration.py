import os
import pathlib

import yaml
from easy_handeye2_msgs.msg import HandeyeCalibration, HandeyeCalibrationParameters
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rosidl_runtime_py import set_message_fields, message_to_yaml

from . import CALIBRATIONS_DIRECTORY


def filepath_for_calibration(name) -> pathlib.Path:
    return CALIBRATIONS_DIRECTORY / f'{name}.calib'


class HandeyeCalibrationParametersProvider:
    def __init__(self, node: Node):
        self.node = node
        # Declare with empty defaults so the rqt plugin context doesn't crash.
        # When launched via the calibrate.launch.py the params file fills these in;
        # when loaded as an rqt plugin the values stay '' and read() falls back to
        # fetching them from the already-running handeye_server node.
        self.node.declare_parameter('name', '')
        self.node.declare_parameter('calibration_type', '')
        self.node.declare_parameter('robot_base_frame', '')
        self.node.declare_parameter('robot_effector_frame', '')
        self.node.declare_parameter('tracking_base_frame', '')
        self.node.declare_parameter('tracking_marker_frame', '')
        self.node.declare_parameter('freehand_robot_movement', True)

    def _fetch_from_server(self) -> HandeyeCalibrationParameters:
        """Read parameters from the handeye_server node via its parameter service."""
        import time
        import rclpy
        from rcl_interfaces.srv import GetParameters as GetParamsSrv

        param_names = ['name', 'calibration_type', 'robot_base_frame',
                       'robot_effector_frame', 'tracking_base_frame',
                       'tracking_marker_frame', 'freehand_robot_movement']

        cli = self.node.create_client(GetParamsSrv, '/handeye_server/get_parameters')
        if not cli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('handeye_server parameter service not available')

        req = GetParamsSrv.Request()
        req.names = param_names
        future = cli.call_async(req)

        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        if not future.done():
            raise RuntimeError('Timed out reading parameters from handeye_server')

        values = future.result().values  # list of ParameterValue
        pv = dict(zip(param_names, values))

        return HandeyeCalibrationParameters(
            name=pv['name'].string_value,
            calibration_type=pv['calibration_type'].string_value,
            robot_base_frame=pv['robot_base_frame'].string_value,
            robot_effector_frame=pv['robot_effector_frame'].string_value,
            tracking_base_frame=pv['tracking_base_frame'].string_value,
            tracking_marker_frame=pv['tracking_marker_frame'].string_value,
            freehand_robot_movement=pv['freehand_robot_movement'].bool_value,
        )

    def read(self) -> HandeyeCalibrationParameters:
        name = self.node.get_parameter('name').get_parameter_value().string_value
        if not name:
            # Parameters were not injected by the launch system (rqt plugin context).
            # Read them from the handeye_server that is already running.
            return self._fetch_from_server()

        return HandeyeCalibrationParameters(
            name=name,
            calibration_type=self.node.get_parameter('calibration_type').get_parameter_value().string_value,
            robot_base_frame=self.node.get_parameter('robot_base_frame').get_parameter_value().string_value,
            robot_effector_frame=self.node.get_parameter('robot_effector_frame').get_parameter_value().string_value,
            tracking_base_frame=self.node.get_parameter('tracking_base_frame').get_parameter_value().string_value,
            tracking_marker_frame=self.node.get_parameter('tracking_marker_frame').get_parameter_value().string_value,
            freehand_robot_movement=self.node.get_parameter('freehand_robot_movement').get_parameter_value().bool_value,
        )


def _normalize_legacy_calibration(data: dict, name: str) -> dict:
    if not isinstance(data, dict):
        raise ValueError(f'Calibration "{name}" must be a mapping, got {type(data).__name__}')

    if 'parameters' in data and 'transform' in data:
        parameters = dict(data['parameters'] or {})
        transform = dict(data['transform'] or {})
    else:
        parameters = {}
        transform = {}
        for key, value in data.items():
            if key in {'transform', 'translation', 'rotation'}:
                continue
            parameters[key] = value

        if 'transform' in data:
            transform = dict(data['transform'] or {})
        else:
            translation = dict(data.get('translation') or {})
            rotation = dict(data.get('rotation') or {})
            if translation or rotation:
                transform = {'translation': translation, 'rotation': rotation}

    legacy_eye_on_hand = parameters.pop('eye_on_hand', None)
    if 'calibration_type' not in parameters and legacy_eye_on_hand is not None:
        parameters['calibration_type'] = 'eye_in_hand' if legacy_eye_on_hand else 'eye_on_base'

    return {
        'parameters': parameters,
        'transform': transform,
    }


def load_calibration(name, calibration_file=None) -> HandeyeCalibration:
    filepath = pathlib.Path(calibration_file) if calibration_file else filepath_for_calibration(name)
    with open(filepath) as f:
        m = yaml.full_load(f.read())
    m = _normalize_legacy_calibration(m, name)
    ret = HandeyeCalibration()
    set_message_fields(ret, m)
    return ret


def save_calibration(calibration: HandeyeCalibration) -> pathlib.Path:
    if not os.path.exists(CALIBRATIONS_DIRECTORY):
        os.makedirs(CALIBRATIONS_DIRECTORY)
    filepath = filepath_for_calibration(calibration.parameters.name)
    with open(filepath, 'w') as f:
        f.write(message_to_yaml(calibration))
    return filepath
