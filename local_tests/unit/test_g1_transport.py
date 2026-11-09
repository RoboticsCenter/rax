"""Exercise the actual SDK adapter with fake DDS messages and RPC acknowledgements."""
import sys
import threading
from types import SimpleNamespace as NS

import pytest

from services.g1.transport import UnitreeTransport


def adapter(monkeypatch):
    class CRC:
        def Crc(self, message):
            return 42
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.utils.crc', NS(CRC=CRC))
    driver = UnitreeTransport.__new__(UnitreeTransport)
    driver.lock = threading.Lock()
    driver.latest = None
    driver.received = 0.
    driver.tick = None
    driver.client = NS(GetFsmId=lambda: (0, 500))
    return driver


def message(tick, crc=42):
    return NS(tick=tick, crc=crc, motor_state=[NS(q=.1, dq=.2, motorstate=0) for _ in range(35)],
              imu_state=NS(rpy=[0., 0., 0.]), mode_machine=5)


def test_feedback_rejects_corrupt_replayed_and_out_of_order_packets(monkeypatch):
    driver = adapter(monkeypatch)
    driver._receive(message(20))
    first = driver.received
    for value in (message(21, crc=0), message(20), message(19)):
        driver._receive(value)
        assert driver.received == first
    driver._receive(message(21))
    state = driver.state()
    assert len(state['joints']) == 29
    assert state['fsm_id'] == 500
    assert state['simulated'] is False
    assert driver.tick == 21


def test_tick_wrap_is_accepted(monkeypatch):
    driver = adapter(monkeypatch)
    driver._receive(message(0xffffffff))
    driver._receive(message(0))
    assert driver.tick == 0


def test_velocity_uses_acknowledged_finite_duration_api(monkeypatch):
    driver = adapter(monkeypatch)
    calls = []
    def send(*args):
        calls.append(args)
        return 0
    driver.client.SetVelocity = send
    driver.velocity(.1, 0., .2, .25)
    assert calls == [(.1, 0., .2, .25)]
    driver.client.SetVelocity = lambda *args: 3102
    with pytest.raises(RuntimeError, match='3102'):
        driver.velocity(0., 0., 0., .25)


def test_missing_feedback_and_failed_fsm_rpc_are_not_ready(monkeypatch):
    driver = adapter(monkeypatch)
    with pytest.raises(RuntimeError, match='No G1'):
        driver.state()
    driver._receive(message(1))
    driver.client.GetFsmId = lambda: (9, None)
    with pytest.raises(RuntimeError, match='GetFsmId failed'):
        driver.state()
