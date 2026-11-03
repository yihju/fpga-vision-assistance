"""Physical keys enter the existing LiveState, never a second decision policy."""
from collections import deque
from adas.board_keys import commands_for_event

CODES={'KEY0_SHORT':'\x10','KEY1_SHORT':'\x11','KEY1_LONG_START':'\x12','KEY1_LONG_END':'\x13'}
EVENTS={value:key for key,value in CODES.items()}


def event_character(event):return CODES[event]


def state_class(base,*,retain_motion_on_stale=False):
    if type(retain_motion_on_stale) is not bool:raise ValueError('explicit motion intent option required')
    class PhysicalLiveState(base):
        def __init__(self,*args,**kwargs):
            self.physical_force_motion=False
            kwargs.setdefault('retain_motion_on_stale',retain_motion_on_stale)
            super().__init__(*args,**kwargs)

        def start(self,*args,**kwargs):
            self.physical_force_motion=False
            return super().start(*args,**kwargs)

        def fail(self,reason):
            if self.physical_force_motion:
                # The W inserted by a held FORCE is not a normal latched W.
                # Stop it BEFORE a context reset can suppress the release edge.
                self.physical_force_motion=False
                super().press('s')
            return super().fail(reason)

        def press(self,key):
            event=EVENTS.get(key)
            if event is None:
                if self.physical_force_motion and key.lower() in 'civn':
                    self.physical_force_motion=False
                    super().press('s')
                if key.lower() in ('s','\x1b'):self.physical_force_motion=False
                return super().press(key)
            # One UI poll applies the complete command tuple before any render.
            commands=commands_for_event(event,self.controls.value,self.scenario)
            if event=='KEY1_LONG_START':self.physical_force_motion=True
            elif 's' in commands:self.physical_force_motion=False
            for command in commands:
                if not super().press(command):return False
            return True
    return PhysicalLiveState


def window_class(base):
    class PhysicalWindow(base):
        def __init__(self,*args,source_factory=None,**kwargs):
            if source_factory is None:
                from board_key_source import BoardKeySource
                source_factory=BoardKeySource
            self.board_keys=source_factory();self.board_events=deque()
            try:super().__init__(*args,**kwargs)
            except Exception:self.board_keys.close();raise

        def poll(self):
            keyboard=super().poll()
            if keyboard==27:return 27
            if keyboard in tuple(map(ord,'sivn')):
                self.board_events.clear();self.board_keys.decoder.reset()
                super().discard_pending()
                return keyboard
            try:
                events=self.board_keys.sample()
                if keyboard==ord('c') and 'KEY1_LONG_END' not in events:
                    self.board_events.clear();self.board_keys.decoder.reset()
                    super().discard_pending()
                    return keyboard
                if events:
                    # A physical action supersedes older keyboard intentions.
                    super().discard_pending()
                    if 'KEY1_LONG_END' in events:
                        self.board_events.clear();events=('KEY1_LONG_END',)
                    self.board_events.extend(event_character(event) for event in events)
                if self.board_events:return ord(self.board_events.popleft())
            except Exception:
                self.board_events.clear();self.board_keys.decoder.reset()
                super().discard_pending()
                if hasattr(self,'root'):self.root.set_title('ADAS - BOARD KEY INPUT ERROR - STOP')
                return ord('s')
            return keyboard

        def discard_pending(self):
            self.board_events.clear();self.board_keys.decoder.reset()
            super().discard_pending()

        def discard_analysis_pending(self,acceptance):
            if acceptance=='error':return self.discard_pending()
            if acceptance!='stale':raise ValueError('stale/error input boundary required')
            # The LiveState has already revoked stale permission/FORCE. A slow
            # analysis is not a new input source: keep the physical down/up
            # measurement, or repeated timeouts swallow a held ordinary key.
            # Drop queued intentions; source/task/error still use the hard reset.
            self.board_events.clear();super().discard_pending()

        def close(self):
            self.board_keys.close();self.board_events.clear()
            super().close()
    return PhysicalWindow
