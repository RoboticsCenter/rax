import asyncio
import ctypes
import json
import os
import queue
import time
import threading
from types import SimpleNamespace

import pytest

from services.revo2.client import RevoHand, RevoFault
from services.revo2.damiao import DamiaoBus, Frame, crc16, register_request
from services.revo2.worker import HandHardware


def bus(slave=127):
    result = DamiaoBus.__new__(DamiaoBus)
    result.channel, result.hand_id = 1, slave
    result.enabled, result.error = True, None
    result.device = object()
    result.authorized_write = None
    result.write_sent = result.write_acknowledged = False
    result.transaction_error = None
    result.received = queue.Queue()
    result.frames = []

    def send(*arguments):
        result.frames.append(arguments)
        return True
    result.api = SimpleNamespace(dmcan_device_send_can=send)
    return result


def can_id(slave, size):
    return (slave << 16) | (1 << 8) | size


def rtu(body):
    return body + crc16(body).to_bytes(2, 'little')


@pytest.mark.parametrize('slave', [126, 127])
@pytest.mark.parametrize('address,values', [(1010, [200, 100]*6), (1022, [200, 100]*6),
                                          (1034, [-100]*6), (1040, [200]*6)])
def test_each_native_mode_requires_exact_authorization_and_never_retries(slave, address, values):
    transport = bus(slave)
    request = register_request(slave, address, values)
    assert not transport.send(slave, can_id(slave, len(request)), request)
    assert not transport.frames
    transport.authorized_write = request
    assert transport.send(slave, can_id(slave, len(request)), request)
    assert not transport.send(slave, can_id(slave, len(request)), request)
    assert len(transport.frames) == 1
    wrong = rtu(bytes([slave, 16]) + (address+1).to_bytes(2, 'big') + len(values).to_bytes(2, 'big'))
    correct = rtu(request[:6])
    transport.transaction_error = None  # Evaluate replies to the one accepted write.
    transport.received.put((time.monotonic(), can_id(slave, 8), wrong))
    transport.received.put((time.monotonic(), can_id(slave, 8), correct))
    assert transport.receive(slave, can_id(slave, 8), 1)[1] == correct
    assert transport.write_acknowledged


def test_cross_hand_routing_and_configuration_writes_are_refused():
    transport = bus(126)
    request = register_request(127, 1040, [200]*6)
    transport.authorized_write = request
    assert not transport.send(127, can_id(127, len(request)), request)
    assert not transport.frames
    with pytest.raises(ValueError):
        register_request(126, 937, [1])  # Unit changes are not a motor operation.


def test_native_abi_and_feedback_crc_freshness():
    assert ctypes.sizeof(Frame) == 80
    assert crc16(bytes.fromhex('01030000000a')) == 0xCDC5
    transport = bus()
    request = rtu(bytes.fromhex('7f0407d00018'))
    assert transport.send(127, can_id(127, 8), request)
    reply = rtu(bytes([127, 4, 48]) + bytes(48))
    transport.received.put((transport.sent_at - 1, can_id(127, 53), reply))
    transport.received.put((time.monotonic(), can_id(127, 53), reply[:-1] + b'\x01'))
    transport.received.put((time.monotonic(), can_id(127, 53), reply + bytes(11)))
    assert transport.receive(127, can_id(127, 53), 1) == (can_id(127, 53), reply)


def test_native_futures_are_created_inside_the_running_loop():
    hardware = HandHardware.__new__(HandHardware)
    hardware.loop = asyncio.new_event_loop()
    hardware.bus = None

    def method():
        asyncio.get_running_loop()

        async def result():
            return [123]
        return result()
    try:
        assert hardware.run(method) == [123]
    finally:
        hardware.loop.close()


def test_native_worker_partial_response_cannot_block_forever():
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


def test_inflight_worker_telemetry_is_nonblocking_and_stale_feedback_is_rejected(tmp_path):
    client = RevoHand.__new__(RevoHand)
    client.root = tmp_path
    client.settings = {'handedness': 'left', 'serial': 'serial-left'}
    client.lock = threading.Lock()
    path = tmp_path/'operator'/'revo-left-state.json'
    path.parent.mkdir()
    value = {'serial': 'serial-left', 'time': time.time(), 'fresh': True, 'fault': None}
    with client.lock:
        path.write_text(json.dumps(value))
        assert client.require_ready()['fresh']
        path.write_text(json.dumps({**value, 'time': time.time()-1}))
        with pytest.raises(RevoFault, match='stale'):
            client.require_ready()
