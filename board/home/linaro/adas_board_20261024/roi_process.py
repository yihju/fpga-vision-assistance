"""Terminate a perception worker when its owning process exits."""
import ctypes, os, signal

def bind_parent(parent_pid):
    if os.name=='posix':
        libc=ctypes.CDLL(None,use_errno=True)
        if libc.prctl(1,signal.SIGTERM,0,0,0)!=0:raise OSError(ctypes.get_errno(),'parent-death signal failed')
        if os.getppid()!=parent_pid:raise RuntimeError('worker parent has ended')
