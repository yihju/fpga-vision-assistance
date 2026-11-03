"""The two ordinary board keys feed the existing debounced control events."""
import ctypes
from pathlib import Path
import time

from adas.board_keys import TwoKeyEvents


class BoardKeySource:
    def __init__(self):
        self.lib=ctypes.CDLL(str(Path(__file__).with_name('libadas_keys.so')),use_errno=True)
        self.lib.adas_keys_open.restype=ctypes.c_void_p
        self.lib.adas_keys_close.argtypes=[ctypes.c_void_p];self.lib.adas_keys_close.restype=None
        self.lib.adas_keys_read.argtypes=[ctypes.c_void_p];self.lib.adas_keys_read.restype=ctypes.c_int
        self.handle=self.lib.adas_keys_open()
        if not self.handle: raise OSError(ctypes.get_errno(),'ordinary board-key inputs unavailable')
        self.decoder=TwoKeyEvents();self.last_levels=None

    def sample(self):
        if not self.handle: raise RuntimeError('closed board-key reader')
        value=self.lib.adas_keys_read(self.handle)
        if value<0:
            self.decoder.reset()
            raise OSError(ctypes.get_errno(),'board-key input failed; caller must STOP')
        now=time.perf_counter_ns()
        self.last_levels=(value&1,(value>>1)&1)
        return self.decoder.sample(value&1,(value>>1)&1,now)

    def close(self):
        if self.handle:self.lib.adas_keys_close(self.handle);self.handle=None
        self.decoder.reset()
