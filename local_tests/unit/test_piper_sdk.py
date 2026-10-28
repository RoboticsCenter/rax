import asyncio
import ctypes
import os
import queue
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from services.piper.config import options
from services.piper.damiao import DamiaoBus, Frame, crc16, position_request
from services.piper.hand import RevoHand
from services.piper.hand_worker import HandHardware
from services.piper.sdk import PiperArm


class FakeSDK:
    def __init__(self):
        self.joints = np.deg2rad([0, 60, -60, 0, 0, 0]).tolist()
        self.commands = []
        self.connected = False
        self.fail_send = False

    def connect(self):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_fps(self):
        return 100

    def get_firmware(self, **arguments):
        return {'software_version': 'S-V1.8-2'}

    def get_joint_angles(self):
        return SimpleNamespace(msg=self.joints.copy(), timestamp=time.time())

    def get_arm_status(self):
        return SimpleNamespace(msg=SimpleNamespace(ctrl_mode=1), timestamp=time.time())

    def get_comm_error(self):
        return None

    def set_speed_percent(self, value):
        self.commands.append(('speed', value))

    def set_motion_mode(self, value):
        self.commands.append(('mode', value))

    def set_auto_set_motion_mode_enabled(self, value):
        self.commands.append(('auto_mode', value))

    def move_j(self, value):
        if self.fail_send:
            raise RuntimeError('CAN transmit failed')
        self.commands.append(('joints', value.copy()))
        self.joints = value.copy()

    def electronic_emergency_stop(self):
        self.commands.append(('damped_stop', None))


class FakeFeedback:
    def __init__(self):
        self.enabled = True
        self.closed = False
        self.error = False

    def check(self, degrees):
        if self.error:
            raise RuntimeError('stale independent feedback')
        return {'joints_deg': list(degrees), 'sdk_agrees': True, 'enabled': [self.enabled] * 6,
                'arm_status': 0, 'error': 0, 'motor_faults': [0] * 6}

    def close(self):
        self.closed = True


def driver(permit=lambda: True):
    sdk, feedback = FakeSDK(), FakeFeedback()
    result = PiperArm(options({}), sdk=sdk, feedback=feedback, interface='fake-can', permit=permit,
                      profile_resolver=lambda model, firmware: 'default')
    return result, sdk, feedback


def wait_for(predicate):
    deadline = time.monotonic() + 1.
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('Timed out waiting for driver state')
        time.sleep(.005)


def test_sdk_connection_and_read_only_close_send_no_motion_or_enable():
    arm, sdk, feedback = driver()
    value = arm.state()
    assert value['driver'] == 'pyagxarm' and value['firmware_verified']
    assert sdk.commands == []
    arm.close()
    arm.close()
    assert sdk.commands == [] and not sdk.connected and feedback.closed


def test_sdk_refuses_disabled_unarmed_or_unverified_motion():
    arm, sdk, feedback = driver(permit=lambda: False)
    try:
        with pytest.raises(RuntimeError, match='permit'):
            arm.segment(np.asarray(sdk.joints), .1)
        arm.permit = lambda: True
        feedback.enabled = False
        with pytest.raises(RuntimeError, match='enabled'):
            arm.segment(np.asarray(sdk.joints), .1)
        feedback.enabled = True
        arm.firmware_verified = False
        with pytest.raises(RuntimeError, match='verified firmware'):
            arm.segment(np.asarray(sdk.joints), .1)
        assert sdk.commands == []
    finally:
        arm.close()


def test_sdk_finite_segment_streams_and_disarm_cancels_it():
    permit = threading.Event()
    permit.set()
    arm, sdk, _ = driver(permit=permit.is_set)
    try:
        arm.segment(np.asarray(sdk.joints) + [.005, 0, 0, 0, 0, 0], .1)
        wait_for(lambda: any(name == 'joints' for name, _ in sdk.commands))
        permit.clear()
        wait_for(lambda: arm.last_stop_reason is not None)
        assert not arm.state()['busy']
        assert arm.streaming
    finally:
        arm.close()
    assert any(name == 'damped_stop' for name, _ in sdk.commands)


def test_watchdog_is_independent_of_the_backend_dispatcher():
    arm, sdk, _ = driver()
    try:
        arm.segment(np.asarray(sdk.joints) + [.005, 0, 0, 0, 0, 0], .1)
        wait_for(lambda: arm.last_stop_reason is not None)
        assert not arm.state()['busy']
        assert 'lease expired' in arm.last_stop_reason
    finally:
        arm.close()


def test_sdk_watchdog_stale_feedback_requests_damped_stop_not_disable():
    arm, sdk, feedback = driver()
    try:
        arm.segment(np.asarray(sdk.joints) + [.005, 0, 0, 0, 0, 0], .1)
        feedback.error = True
        wait_for(lambda: arm.fault is not None)
        assert 'stale' in arm.fault and not arm.streaming
        assert any(name == 'damped_stop' for name, _ in sdk.commands)
    finally:
        arm.close()


def test_sdk_adapter_never_relies_on_clamping_or_unbounded_segments():
    arm, sdk, _ = driver()
    try:
        for target, seconds in (([0] * 6, 1.), ([0, -1, -1, 0, 0, 0], .1),
                                (np.asarray(sdk.joints) + [.2, 0, 0, 0, 0, 0], .1)):
            with pytest.raises(ValueError):
                arm.segment(target, seconds)
        assert sdk.commands == []
    finally:
        arm.close()


def bus():
    result = DamiaoBus.__new__(DamiaoBus)
    result.channel, result.hand_id = 1, 127
    result.target_min, result.target_max = [100, 0, 0, 0, 0, 0], [900, 950, 950, 950, 950, 950]
    result.enabled, result.error = True, None
    result.device = object()
    result.authorized_write = None
    result.write_sent = False
    result.write_acknowledged = False
    result.transaction_error = None
    result.received = queue.Queue()
    result.frames = []
    def send(*arguments):
        result.frames.append(arguments)
        return True
    result.api = SimpleNamespace(dmcan_device_send_can=send)
    return result


def can_id(size):
    return (127 << 16) | (1 << 8) | size


def rtu(body):
    return body + crc16(body).to_bytes(2, 'little')


def test_native_frame_abi_and_modbus_crc_match_vendor_protocol():
    assert ctypes.sizeof(Frame) == 80
    assert crc16(bytes.fromhex('01030000000a')) == 0xCDC5
    request = position_request(127, [200, 300, 0, 0, 0, 0], 200)
    assert len(request) == 33 and request[2:7] == bytes.fromhex('03f2000c18')


def test_damiao_only_transmits_one_exact_authorized_finite_write():
    transport = bus()
    request = position_request(127, [200, 300, 0, 0, 0, 0], 200)
    assert not transport.send(127, can_id(len(request)), request)
    assert transport.frames == []
    transport.authorized_write = request
    assert transport.send(127, can_id(len(request)), request + bytes(15))
    frame = transport.frames[0]
    assert frame[1:7] == (1, can_id(33), True, True, False, True)
    assert frame[7] == 48 and bytes(frame[8]) == request + bytes(15)
    assert not transport.send(127, can_id(len(request)), request)
    assert len(transport.frames) == 1


def test_damiao_read_replies_require_crc_length_matching_and_freshness():
    transport = bus()
    request = rtu(bytes.fromhex('7f0407d00018'))
    assert transport.send(127, can_id(8), request)
    reply = rtu(bytes([127, 4, 48]) + bytes(48))
    transport.received.put((transport.sent_at - 1, can_id(53), reply))
    transport.received.put((time.monotonic(), can_id(53), reply[:-1] + b'\x01'))
    transport.received.put((time.monotonic(), can_id(53), reply + bytes(11)))
    assert transport.receive(127, can_id(53), 1) == (can_id(53), reply)


def test_damiao_write_ack_must_echo_the_authorized_register_range():
    transport = bus()
    request = position_request(127, [200, 300, 0, 0, 0, 0], 200)
    transport.authorized_write = request
    assert transport.send(127, can_id(33), request)
    wrong = rtu(bytes.fromhex('7f100410000c'))
    correct = rtu(request[:6])
    transport.received.put((time.monotonic(), can_id(8), wrong))
    transport.received.put((time.monotonic(), can_id(8), correct))
    assert transport.receive(127, can_id(8), 1)[1] == correct
    assert transport.write_acknowledged


def test_authorization_cannot_allow_current_mode_or_unbounded_finger_duration():
    transport = bus()
    for request in (position_request(127, [200, 300, 0, 0, 0, 0], 5000),
                    position_request(127, [0] * 6, 200), rtu(bytes.fromhex('7f10041000010201f4'))):
        transport.authorized_write = request
        assert not transport.send(127, can_id(len(request)), request)
    assert transport.frames == []


def test_sdk_async_methods_are_created_inside_the_running_event_loop():
    hardware = HandHardware.__new__(HandHardware)
    hardware.loop = asyncio.new_event_loop()
    hardware.bus = SimpleNamespace(error=None, transaction_error=None)
    def method():
        asyncio.get_running_loop()
        async def result():
            return [123]
        return result()
    try:
        assert hardware.run(method) == [123]
    finally:
        hardware.loop.close()


def test_worker_partial_response_has_a_deadline():
    read_fd, write_fd = os.pipe()
    client = RevoHand.__new__(RevoHand)
    client.buffer = bytearray()
    with os.fdopen(read_fd, 'rb', buffering=0) as stream:
        client.process = SimpleNamespace(stdout=stream)
        os.write(write_fd, b'{')
        try:
            with pytest.raises(TimeoutError):
                client._read(.03)
        finally:
            os.close(write_fd)
