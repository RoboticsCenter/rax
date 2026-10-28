"""Minimal DM_Device CAN-FD transport for the official BrainCo SDK callbacks."""
import ctypes
import fcntl
import os
from pathlib import Path
import queue
import stat
import time


LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


class Frame(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('id_flags', ctypes.c_uint32), ('timestamp', ctypes.c_uint64),
                ('channel', ctypes.c_uint8), ('flags', ctypes.c_uint8),
                ('reserved', ctypes.c_uint16), ('payload', ctypes.c_uint8 * 64)]


class ChannelInfo(ctypes.Structure):
    _pack_ = 1
    _fields_ = [('channel', ctypes.c_uint8), ('canfd', ctypes.c_bool),
                ('can_baudrate', ctypes.c_uint32), ('canfd_baudrate', ctypes.c_uint32),
                ('can_sp', ctypes.c_float), ('canfd_sp', ctypes.c_float)]


CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.POINTER(Frame))
CONTEXTS = {}


def crc16(data):
    result = 0xFFFF
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = (result >> 1) ^ (0xA001 if result & 1 else 0)
    return result


def valid_rtu(data):
    return len(data) >= 4 and crc16(data[:-2]) == int.from_bytes(data[-2:], 'little')


class NativeContext:
    def __init__(self, library):
        self.routes = {}
        self.pointer = ctypes.c_void_p()
        parent = Path(library).parent
        for name in ('libgcc_s.so.1', 'libstdc++.so.6', 'libusb-1.0.so.0'):
            if (parent / name).is_file():
                ctypes.CDLL(str(parent / name), mode=ctypes.RTLD_GLOBAL)
        self.api = ctypes.CDLL(str(library))
        pointer = ctypes.c_void_p
        signatures = {
            'context_create': ([ctypes.POINTER(pointer)], None),
            'find_devices': ([pointer], ctypes.c_int),
            'device_get': ([pointer, ctypes.POINTER(pointer), ctypes.c_int], ctypes.c_bool),
            'device_open': ([pointer], ctypes.c_bool),
            'device_close': ([pointer], None),
            'device_get_version': ([pointer, pointer, ctypes.c_size_t], None),
            'device_enable_channel': ([pointer, ctypes.c_uint8], ctypes.c_bool),
            'device_disable_channel': ([pointer, ctypes.c_uint8], ctypes.c_bool),
            'device_get_channel_baudrate': ([pointer, ctypes.c_uint8, ctypes.POINTER(ChannelInfo)], ctypes.c_bool),
            'device_hook_recv_callback': ([pointer, CALLBACK], None),
            'device_hook_err_callback': ([pointer, CALLBACK], None),
            'device_send_can': ([pointer, ctypes.c_uint8, ctypes.c_uint32,
                                 ctypes.c_bool, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool,
                                 ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8)], ctypes.c_bool),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.api, 'dmcan_' + name)
            function.argtypes, function.restype = arguments, result
        self.receive_callback = CALLBACK(self._receive)
        self.error_callback = CALLBACK(self._error)
        self.api.dmcan_context_create(ctypes.byref(self.pointer))
        self.count = self.api.dmcan_find_devices(self.pointer)

    def _receive(self, handle, pointer):
        route = self.routes.get(handle)
        if route is None:
            return
        frame = pointer.contents
        if frame.channel != route.channel or frame.flags & 2 or not frame.flags & 1:
            return
        can_id = frame.id_flags & 0x1FFFFFFF
        if can_id >> 16 != route.hand_id:
            return
        try:
            route.received.put_nowait((time.monotonic(), can_id, bytes(frame.payload[:LENGTHS[frame.flags >> 4]])))
        except queue.Full:
            route.error = 'DaMiao receive queue overflow'

    def _error(self, handle, pointer):
        route = self.routes.get(handle)
        if route is not None and pointer.contents.channel == route.channel:
            route.error = 'DaMiao CAN/USB error callback'


def native_context(library):
    key = str(Path(library).resolve())
    if key not in CONTEXTS:
        CONTEXTS[key] = NativeContext(key)
    return CONTEXTS[key]


class DamiaoBus:
    def __init__(self, context, index, settings):
        self.context, self.api, self.index = context, context.api, index
        self.channel, self.hand_id = settings['hand_channel'], settings['hand_id']
        self.target_min, self.target_max = settings['hand_target_min'], settings['hand_target_max']
        self.device = ctypes.c_void_p()
        self.received = queue.Queue(maxsize=128)
        self.opened = self.enabled = False
        self.error = None
        self.lock_fd = None
        self.sent_at = 0.
        self.request = None
        self.authorized_write = None
        self.write_acknowledged = False
        self.write_sent = False
        self.transaction_error = None
        try:
            root = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))
            self.lock_fd = os.open(root / f'rax-damiao-{os.getuid()}-{index}.lock',
                                   os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | os.O_NOFOLLOW, 0o600)
            lock_info = os.fstat(self.lock_fd)
            if not stat.S_ISREG(lock_info.st_mode) or lock_info.st_uid != os.getuid():
                raise RuntimeError('Unsafe DaMiao ownership lock')
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if not self.api.dmcan_device_get(context.pointer, ctypes.byref(self.device), index):
                raise RuntimeError('Cannot get DaMiao adapter')
            if not self.api.dmcan_device_open(self.device):
                raise RuntimeError('Cannot open DaMiao adapter')
            self.opened = True
            version = ctypes.create_string_buffer(128)
            self.api.dmcan_device_get_version(self.device, version, len(version))
            self.version = version.value.decode('ascii')
            if self.version != settings['hand_adapter_firmware']:
                raise RuntimeError('Unexpected DaMiao adapter firmware')
            info = ChannelInfo()
            if not self.api.dmcan_device_get_channel_baudrate(self.device, self.channel, ctypes.byref(info)):
                raise RuntimeError('Cannot read DaMiao channel configuration')
            if not info.canfd or info.can_baudrate != 1000000 or info.canfd_baudrate != 5000000:
                raise RuntimeError('Expected existing 1 Mbit/s / 5 Mbit/s CAN-FD channel; no reconfiguration attempted')
            context.routes[self.device.value] = self
            self.api.dmcan_device_hook_recv_callback(self.device, context.receive_callback)
            self.api.dmcan_device_hook_err_callback(self.device, context.error_callback)
            if not self.api.dmcan_device_enable_channel(self.device, self.channel):
                raise RuntimeError('Cannot enable DaMiao communication channel')
            self.enabled = True
        except BaseException:
            self.close()
            raise

    def send(self, slave_id, can_id, data):
        try:
            logical = can_id & 255
            payload = bytes(data)
            request = payload[:logical]
            if (slave_id != self.hand_id or can_id >> 8 != (self.hand_id << 8) | 1
                    or logical > len(payload) or any(payload[logical:]) or not valid_rtu(request)
                    or request[0] != self.hand_id or not self.enabled or self.error):
                raise ValueError('Invalid BrainCo SDK CAN-FD transaction')
            function = request[1]
            if function in (3, 4):
                if len(request) != 8 or not 1 <= int.from_bytes(request[4:6], 'big') <= 32:
                    raise ValueError('Invalid BrainCo read length')
            elif (function != 16 or len(request) != 33 or request[2:7] != bytes.fromhex('03f2000c18')
                  or request != self.authorized_write):
                raise ValueError('Only the exact authorized six-motor finite position write is permitted')
            elif self.write_sent:
                raise RuntimeError('A hand write was already sent; retry is forbidden')
            else:
                for index in range(6):
                    offset = 7 + 4 * index
                    position = int.from_bytes(request[offset:offset + 2], 'big')
                    milliseconds = int.from_bytes(request[offset + 2:offset + 4], 'big')
                    if not self.target_min[index] <= position <= self.target_max[index] or not 100 <= milliseconds <= 200:
                        raise ValueError('Hand position or duration outside the transport safety envelope')
                self.write_sent = True
            self.transaction_error = None
            self.write_acknowledged = False
            self.request = request
            self.sent_at = time.monotonic()
            size = next(length for length in LENGTHS if length >= logical)
            frame = (ctypes.c_uint8 * size).from_buffer_copy(request.ljust(size, b'\0'))
            if not self.api.dmcan_device_send_can(self.device, self.channel, can_id,
                                                 True, True, False, True, size, frame):
                raise RuntimeError('DaMiao rejected CAN-FD transmission')
            return True
        except Exception as exc:
            self.transaction_error = str(exc)
            return False

    def receive(self, slave_id, expected_can_id, expected_frames):
        deadline = time.monotonic() + .4
        while time.monotonic() < deadline:
            if self.error or self.transaction_error:
                return 0, b''
            try:
                timestamp, can_id, payload = self.received.get(timeout=max(.001, deadline - time.monotonic()))
            except queue.Empty:
                break
            logical = can_id & 255
            reply = payload[:logical]
            if (slave_id != self.hand_id or timestamp < self.sent_at or can_id >> 8 != expected_can_id >> 8
                    or logical > len(payload) or not valid_rtu(reply) or self.request is None
                    or reply[:2] != self.request[:2]):
                continue
            if self.request[1] in (3, 4):
                count = int.from_bytes(self.request[4:6], 'big') * 2
                if len(reply) != count + 5 or reply[2] != count:
                    continue
            elif len(reply) != 8 or reply[2:6] != self.request[2:6]:
                continue
            else:
                self.write_acknowledged = True
            return can_id, reply
        self.transaction_error = 'BrainCo transaction timed out; no automatic write retry'
        return 0, b''

    def close(self):
        try:
            if self.opened:
                try:
                    if self.enabled:
                        self.api.dmcan_device_disable_channel(self.device, self.channel)
                finally:
                    self.enabled = False
                    self.api.dmcan_device_close(self.device)
        finally:
            self.context.routes.pop(self.device.value, None)
            self.opened = False
            if self.lock_fd is not None:
                os.close(self.lock_fd)
                self.lock_fd = None


def position_request(hand_id, positions, duration_ms):
    values = b''.join(int(value).to_bytes(2, 'big') + int(duration_ms).to_bytes(2, 'big') for value in positions)
    body = bytes([hand_id, 16]) + (1010).to_bytes(2, 'big') + (12).to_bytes(2, 'big') + bytes([24]) + values
    return body + crc16(body).to_bytes(2, 'little')
