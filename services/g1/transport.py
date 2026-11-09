"""Stock locomotion transport. No raw joint commands or implicit mode changes."""
from copy import deepcopy
import fcntl
import hashlib
import os
from pathlib import Path
import threading
import time


class RobotLease:
    def __init__(self, identity):
        directory = Path(f'/tmp/rax-g1-{os.getuid()}')
        directory.mkdir(mode=0o700, exist_ok=True)
        stat = directory.lstat()
        if directory.is_symlink() or stat.st_uid != os.getuid() or stat.st_mode & 0o077:
            raise PermissionError('G1 lease directory must be operator-owned and private')
        self.file = (directory / (hashlib.sha256(identity.encode()).hexdigest() + '.lock')).open('a')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            self.file.close()
            raise RuntimeError('This G1 already has a RAX owner') from None

    def close(self):
        self.file.close()


class DryRunTransport:
    """Protocol exerciser only: no physics, cameras, hand control or balancing."""
    def __init__(self, cfg):
        self.commands = []
        self.fsm_id = 500
        self._state = {'joints': [0.] * 29, 'joint_velocity': [0.] * 29,
                       'rpy': [0.] * 3, 'motor_faults': [0] * 29, 'feedback_age_s': 0.}

    def state(self):
        return {**deepcopy(self._state), 'fsm_id': self.fsm_id, 'simulated': True}

    def velocity(self, vx, vy, yaw_rate, duration):
        self.commands.append((vx, vy, yaw_rate, duration))

    def close(self):
        pass


class UnitreeTransport:
    def __init__(self, cfg):
        # SDK dependencies are absent from tool discovery and dry runs.
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        self.lock = threading.Lock()
        self.latest = None
        self.received = 0.
        self.tick = None
        self.subscription = None
        self.lease = RobotLease(cfg['robot_id'])
        try:
            ChannelFactoryInitialize(cfg['dds_domain'], cfg['network_interface'])
            self.client = LocoClient()
            self.client.SetTimeout(.1)
            self.client.Init()
            self.subscription = ChannelSubscriber('rt/lowstate', LowState_)
            self.subscription.Init(self._receive, 10)
        except BaseException:
            self.close()
            raise

    def _receive(self, message):
        from unitree_sdk2py.utils.crc import CRC
        if CRC().Crc(message) != message.crc:
            return
        with self.lock:
            if self.tick is not None and not 0 < ((message.tick - self.tick) & 0xffffffff) < 0x80000000:
                return  # Replayed or out-of-order feedback cannot refresh the watchdog.
            self.tick = message.tick
            self.latest = {
                'joints': [float(m.q) for m in message.motor_state[:29]],
                'joint_velocity': [float(m.dq) for m in message.motor_state[:29]],
                'motor_faults': [int(m.motorstate) for m in message.motor_state[:29]],
                'rpy': list(message.imu_state.rpy), 'mode_machine': message.mode_machine,
            }
            self.received = time.monotonic()

    def state(self):
        with self.lock:
            value = deepcopy(self.latest)
            received = self.received
        if value is None:
            raise RuntimeError('No G1 low-state feedback received')
        code, fsm = self.client.GetFsmId()
        if code != 0:
            raise RuntimeError(f'G1 GetFsmId failed: {code}')
        value.update(feedback_age_s=time.monotonic() - received, fsm_id=fsm, simulated=False)
        return value

    def velocity(self, vx, vy, yaw_rate, duration):
        code = self.client.SetVelocity(vx, vy, yaw_rate, duration)
        if code != 0:
            raise RuntimeError(f'G1 SetVelocity failed: {code}')

    def close(self):
        try:
            if self.subscription is not None:
                self.subscription.Close()
        finally:
            self.lease.close()
