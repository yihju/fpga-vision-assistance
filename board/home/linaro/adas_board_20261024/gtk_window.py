"""RK's installed Gtk3 display; same frame pixels and deadline contract as PC."""
from collections import deque
import time

class ControlQueue:
    def __init__(self):
        self.keys=deque();self.quit_requested=False;self.task_switching=True
    def push(self,key):
        key=key.lower()
        if key=='\x1b':self.quit_requested=True;self.keys.clear();return
        if self.quit_requested:return
        allowed='123wslrtfc'+('ivn' if self.task_switching else '')
        if len(key)!=1 or key not in allowed:return
        if key in 'scivn':self.keys.clear()
        if not self.keys or self.keys[-1]!=ord(key):self.keys.append(ord(key))
    def poll(self):
        if self.quit_requested:return 27
        return self.keys.popleft() if self.keys else -1
    def discard_pending(self):self.keys.clear()

class GtkWindow(ControlQueue):
    def __init__(self,*,task_switching=False):
        super().__init__();self.task_switching=task_switching
        import gi
        gi.require_version('Gtk','3.0')
        from gi.repository import Gtk,Gdk,GdkPixbuf,GLib
        self.Gtk,self.Gdk,self.GdkPixbuf,self.GLib=Gtk,Gdk,GdkPixbuf,GLib
        self.root=Gtk.Window(title='RK3568 ADAS - ARM file input')
        self.root.move(0,28)  # HDMI is the top 1024x600 of the extended desktop
        self.image=Gtk.Image();self.root.add(self.image)
        self.root.connect('delete-event',self._close_event)
        self.root.connect('key-press-event',self._key)
        self.open=True;self.photo=None;self.clock=time.perf_counter_ns
        self.root.show_all();self._pump()
    def _key(self,widget,event):
        name=self.Gdk.keyval_name(event.keyval)
        self.push('\x1b' if name=='Escape' else (event.string or name or ''))
        return True
    def _close_event(self,*unused):self.close();return True
    def _pump(self):
        while self.Gtk.events_pending():self.Gtk.main_iteration_do(False)
    def poll(self):
        self._pump()
        return 27 if not self.open else super().poll()
    def discard_pending(self):self._pump();super().discard_pending()
    def is_open(self):return self.open
    def prepare(self,bgr):
        import cv2,numpy as np
        if (not self.open or not isinstance(bgr,np.ndarray) or bgr.dtype!=np.uint8 or
            bgr.ndim!=3 or bgr.shape[2]!=3 or min(bgr.shape[:2])<1):
            raise ValueError('open window and nonempty uint8 BGR required')
        h,w=bgr.shape[:2];rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        data=self.GLib.Bytes.new(rgb.tobytes())
        return self.GdkPixbuf.Pixbuf.new_from_bytes(data,self.GdkPixbuf.Colorspace.RGB,False,8,w,h,w*3)
    def commit(self,photo):
        if not self.open:raise RuntimeError('closed display')
        self.photo=photo;self.image.set_from_pixbuf(photo)
        # Queue rendering only; input is consumed only by poll().
        self.image.queue_draw()
    def show_before(self,bgr,deadline_ns):
        photo=self.prepare(bgr)
        if self.clock()>=deadline_ns:return False
        self.commit(photo)
        return self.clock()<deadline_ns
    def show(self,bgr):self.commit(self.prepare(bgr))
    def close(self):
        self.quit_requested=True;self.keys.clear()
        if self.open:
            self.open=False;self.root.destroy()
        self.photo=None

def open_window(backend,*,native=False,task_switching=False):
    if backend!='tk':raise ValueError('board adapter replaces only the Tk contract')
    return GtkWindow(task_switching=task_switching)
