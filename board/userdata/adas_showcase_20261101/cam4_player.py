"""Q-exit wrapper; the validated capture/ordered-DMA-STOP code is unchanged."""
import os
from collections import deque
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import re
import threading
try:from .quit_keys import TerminalKeys,is_quit_key
except ImportError:from quit_keys import TerminalKeys,is_quit_key


def capture_exit_code(code,text):
    lines=[line for line in text.splitlines() if line.startswith('Stage2:')]
    if 'STOP unconfirmed' in text or not lines:return code or 1
    errors=re.search(r'\berrors=(\d+)\b',lines[-1]);count=re.search(r'\bacquired=(\d+)\b',lines[-1])
    if not errors or int(errors.group(1)) or not count or int(count.group(1))<1:return code or 1
    return 0 if code in (0,130,143,-signal.SIGTERM,-signal.SIGINT) else code


def finish_capture(camera,output=lambda:''):
    if camera.poll() is None:camera.send_signal(signal.SIGTERM)
    # A STOP timeout remains an error. Never force-kill a DMA owner.
    code=camera.wait(timeout=20)
    return capture_exit_code(code,output())


def main():
    if len(sys.argv)>1:
        return subprocess.call(['/usr/local/bin/plate-demo','video',*sys.argv[1:]])
    helper=Path(__file__).resolve().with_name('cam4_window_keys')
    if not helper.is_file() or not os.access(helper,os.X_OK):raise RuntimeError('window Q helper is missing')
    stopping=False
    def cancel(*unused):
        nonlocal stopping
        stopping=True
    previous={sig:signal.signal(sig,cancel) for sig in (signal.SIGINT,signal.SIGTERM)}
    camera=None;listener=None;drainer=None;lines=deque(maxlen=128)
    def capture_output():
        if drainer is not None:
            drainer.join(timeout=3)
            if drainer.is_alive():raise RuntimeError('capture output did not drain; STOP not confirmed')
        return ''.join(lines)
    def mirror():
        for line in camera.stdout:
            lines.append(line)
            # A detached/broken caller stdout must not prevent reading the
            # final actual capture errors/STOP result.
            try:sys.stdout.write(line);sys.stdout.flush()
            except OSError:pass
    try:
        with TerminalKeys() as terminal:
            camera=subprocess.Popen(['/usr/local/bin/plate-demo','video'],start_new_session=True,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
            drainer=threading.Thread(target=mirror,daemon=True);drainer.start()
            listener=subprocess.Popen([str(helper),str(camera.pid)],stdout=subprocess.PIPE)
            print('Four live cameras: press Q in the video window or terminal to exit.',flush=True)
            while camera.poll() is None and not stopping:
                if is_quit_key(terminal.poll()):stopping=True;break
                if listener.stdout is not None and select.select([listener.stdout],[],[],0)[0]:
                    value=os.read(listener.stdout.fileno(),16)
                    if b'q' in value:stopping=True;break
                    if listener.poll() is not None and listener.returncode:
                        raise RuntimeError('window key listener failed')
                time.sleep(.02)
            return finish_capture(camera,capture_output) if stopping else capture_exit_code(camera.returncode,capture_output())
    finally:
        try:
            if camera is not None and camera.poll() is None:finish_capture(camera,capture_output)
        finally:
            if listener is not None:
                if listener.poll() is None:listener.terminate()
                listener.wait(timeout=3)
                if listener.stdout is not None:listener.stdout.close()
            if drainer is not None:drainer.join(timeout=3)
            if camera is not None and camera.stdout is not None and (drainer is None or not drainer.is_alive()):camera.stdout.close()
            for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':
    try:raise SystemExit(main())
    except (OSError,RuntimeError,subprocess.SubprocessError) as exc:
        print(type(exc).__name__+': '+str(exc),file=sys.stderr);raise SystemExit(1)
