"""Fail-closed WS2812 sink for the independent ADAS FPGA RAM variant.

Only trusted, newly appended live GUI ``display`` records grant a lease.
Start at the end of one explicit current-run log; never replay old permissions.
The producer and this process must run on the same board/perf_counter_ns clock.
Static-image logs lack a video deadline and therefore cannot grant green.
"""
import argparse
import ctypes
import json
import math
import mmap
import os
from pathlib import Path
import signal
import struct
import sys
import time

IDENTITY = 0x4c444933
STOP = 0x4c440000
DIRECTIONS = {'left': 1, 'straight': 2, 'right': 3}
REVOKE = {'control', 'task', 'task_switch', 'source_end', 'error', 'end', 'final', 'board_process_end'}
TERMINAL = {'error', 'end', 'final', 'board_process_end'}


class IdentityFault(RuntimeError):
    pass


class LogFault(RuntimeError):
    pass


def _age(value,maximum=300):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= maximum:
        raise ValueError('finite age within explicit freshness required')
    return value


def _key(value):
    if (type(value) is not dict or set(value) != {'epoch', 'frame_id', 'camera_id'} or
            any(type(v) is not int or v < 0 for v in value.values()) or value['camera_id'] > 3):
        raise ValueError('complete frame identity required')
    return value


class DisplayLease:
    def __init__(self,*,freshness_ms=300):
        if type(freshness_ms) is not int or freshness_ms not in (300,600):
            raise ValueError('explicit realtime300 or functional600 required')
        self.freshness_ms=freshness_ms
        self.mode = 0
        self.deadline_ns = 0
        self.last_submitted_ns = -1
        self.last_clock_ns = -1
        self.finished = False
        self.last_epoch = -1
        self.closed_through_epoch = -1

    def _clock(self, now_ns):
        if type(now_ns) is not int or now_ns < 0 or now_ns < self.last_clock_ns:
            self.mode = 0
            self.finished = True
            return False
        self.last_clock_ns = now_ns
        return True

    def consume(self, event, now_ns):
        if not self._clock(now_ns):
            return STOP
        if type(event) is not dict:
            self.mode = 0
            return STOP
        kind = event.get('kind')
        if kind in REVOKE:
            self.mode = 0
            self.finished = self.finished or kind in TERMINAL
            if kind == 'source_end':
                epoch=self.last_epoch
                try:epoch=max(epoch,_key(event['decision']['key'])['epoch'])
                except (KeyError,TypeError,ValueError):pass
                self.closed_through_epoch=max(self.closed_through_epoch,epoch)
                if event.get('error') or event.get('reason')=='error':self.finished=True
        elif kind == 'display':
            self.mode = 0
            if not self.finished:
                try:
                    attempts=event.get('gui_attempts')
                    if type(attempts) is not int or attempts<1:raise ValueError('actual GUI submission required')
                    decision = event['decision']
                    if type(decision) is not dict or decision['valid'] is not True:
                        raise ValueError('valid current decision required')
                    if _key(event['key']) != _key(decision['key']):
                        raise ValueError('cross-frame decision')
                    epoch=event['key']['epoch']
                    if epoch<=self.closed_through_epoch or epoch<self.last_epoch:
                        raise ValueError('closed or old source epoch')
                    submitted = event['submitted_ns']
                    if (type(submitted) is not int or submitted < 0 or submitted > now_ns or
                            submitted <= self.last_submitted_ns):
                        raise ValueError('current same-clock submission required')
                    ages = [_age(event['image_age_ms'],self.freshness_ms)]
                    nominal = event.get('nominal_source_age_ms')
                    if nominal is not None:
                        ages.append(_age(nominal,self.freshness_ms))
                    due = event.get('source_due_ns')
                    if due is not None:
                        if (type(due) is not int or due < 0 or due > submitted or nominal is None or
                                abs((submitted-due)/1e6-nominal) > .001):
                            raise ValueError('source clock/age mismatch')
                    alarms = decision['alarm_reasons']
                    if type(alarms) not in (list, tuple) or any(type(a) is not str or not a for a in alarms):
                        raise ValueError('explicit alarm reasons required')
                    deadline = submitted + int((self.freshness_ms-max(ages))*1_000_000)
                    mode = 4 if alarms else DIRECTIONS.get(decision.get('allowed_action'), 0)
                    if not alarms and decision.get('simulated_action') != decision.get('allowed_action'):
                        mode = 0
                    self.last_submitted_ns = submitted
                    self.last_epoch = epoch
                    self.deadline_ns = deadline
                    self.mode = mode
                except (KeyError, TypeError, ValueError, OverflowError):
                    self.mode = 0
        return self.word(now_ns)

    def word(self, now_ns):
        if not self._clock(now_ns) or self.finished or not self.mode:
            return STOP
        # Frame latch <=5ms, packet/reset <1ms, TTL quantization <1ms.
        # Keep 10ms of margin; no refresh may extend the original deadline.
        lease_ms = min(200, (self.deadline_ns-now_ns)//1_000_000-10)
        if lease_ms <= 0:
            self.mode = 0
            return STOP
        return STOP | (lease_ms << 8) | self.mode


class JsonlTail:
    """One append-only file; replacement/truncation/rewrite is a fatal fault."""
    def __init__(self, path):
        self.path = Path(path)
        self.buffer = b''
        self.closed = False
        try:
            with self.path.open('rb') as source:
                stat = os.fstat(source.fileno())
                self.identity = (stat.st_dev, stat.st_ino)
                self.offset = stat.st_size
                source.seek(max(0, self.offset-64))
                self.anchor = source.read()
                self.drop_fragment = bool(self.offset and not self.anchor.endswith(b'\n'))
        except OSError as exc:
            raise LogFault('current log unavailable') from exc

    def poll(self):
        if self.closed:
            raise LogFault('closed log')
        try:
            with self.path.open('rb') as source:
                stat = os.fstat(source.fileno())
                if (stat.st_dev, stat.st_ino) != self.identity or stat.st_size < self.offset:
                    raise LogFault('log replaced or truncated')
                source.seek(self.offset-len(self.anchor))
                if source.read(len(self.anchor)) != self.anchor:
                    raise LogFault('log history rewritten')
                source.seek(self.offset)
                data = source.read(1_048_577)
                if len(data) > 1_048_576 or len(self.buffer)+len(data) > 1_048_576:
                    raise LogFault('log exceeds bounded read budget')
                self.offset += len(data)
                source.seek(max(0, self.offset-64))
                self.anchor = source.read(self.offset-source.tell())
        except OSError as exc:
            raise LogFault('current log vanished or unreadable') from exc
        lines = (self.buffer+data).split(b'\n')
        self.buffer = lines.pop()
        if self.drop_fragment and lines:
            lines.pop(0)
            self.drop_fragment = False
        events = []
        try:
            for line in lines:
                event = json.loads(line.decode('utf-8'))
                if type(event) is not dict:
                    raise ValueError('object log record required')
                events.append(event)
        except (UnicodeError, ValueError) as exc:
            raise LogFault('malformed log record') from exc
        return events

    def close(self):
        self.closed = True


class PcieLed:
    """Identity-gated, aligned 32-bit writes to BAR1 byte0x240 only.

    PCI COMMAND.Memory must already be enabled by the deployment step.
    This class never changes BusMaster, DMA, capture, Flash, or PCI config.
    """
    def __init__(self, device='/sys/bus/pci/devices/0002:21:00.0'):
        self.device = Path(device)
        self.control = None
        self.write_map = None
        with (self.device/'resource0').open('rb', buffering=0) as source:
            self.read_map = mmap.mmap(source.fileno(), 8192, access=mmap.ACCESS_READ)
        if struct.unpack_from('<I', self.read_map, 0x1c00)[0] != IDENTITY:
            self.read_map.close()
            raise IdentityFault('LED identity missing; no control writes allowed')
        try:
            with (self.device/'resource1').open('r+b', buffering=0) as target:
                self.write_map = mmap.mmap(target.fileno(), 4096, access=mmap.ACCESS_WRITE)
            # ctypes emits one aligned native uint32 store. Byte-wise packed
            # writes would fail the FPGA's full 4-byte-enable requirement.
            self.control = ctypes.c_uint32.from_buffer(self.write_map, 0x240)
        except Exception:
            if self.write_map is not None:
                self.write_map.close()
            self.read_map.close()
            raise

    def readback(self):
        return struct.unpack_from('<4I', self.read_map, 0x1c00)

    def write(self, word):
        if (type(word) is not int or not 0 <= word <= 0xffffffff or word >> 16 != 0x4c44 or
                (word >> 8 & 255) > 200 or word & 0xf8 or (word >> 8 & 255) == 0 and word & 7):
            raise ValueError('only bounded LED command words are permitted')
        self.control.value = word
        return self.readback()  # BAR read also drains the posted control write.

    def close(self):
        if self.control is None:
            return
        try:
            self.write(STOP)
        finally:
            self.control = None
            self.write_map.close()
            self.read_map.close()

    def __enter__(self):
        self.write(STOP)
        return self

    def __exit__(self, *exc):
        self.close()








