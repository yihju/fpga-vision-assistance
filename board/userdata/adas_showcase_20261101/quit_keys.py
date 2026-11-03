"""Q/Escape input with bounded polling and reversible terminal settings."""
import os
import select
import sys


def is_quit_key(key):
    return key in ('q','Q','\x1b',113,81,65307)


class TerminalKeys:
    def __init__(self,stream=None):self.stream=stream if stream is not None else sys.stdin;self.fd=None;self.saved=None
    def __enter__(self):
        if self.stream.isatty():
            import termios,tty
            self.fd=self.stream.fileno();self.saved=termios.tcgetattr(self.fd);tty.setcbreak(self.fd)
        return self
    def poll(self):
        if self.fd is not None and select.select([self.fd],[],[],0)[0]:
            return os.read(self.fd,1).decode(errors='ignore')
        return None
    def __exit__(self,*unused):
        if self.saved is not None:
            import termios
            termios.tcsetattr(self.fd,termios.TCSADRAIN,self.saved)

