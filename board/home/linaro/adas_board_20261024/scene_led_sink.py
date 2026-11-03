"""LEDI4 receiver: only actual current GUI records grant a hardware lease."""
import argparse,ctypes,json,mmap,signal,struct,time
from pathlib import Path
try:
    from .led_sink import DisplayLease,PcieLed,IdentityFault,JsonlTail,STOP
except ImportError:
    from led_sink import DisplayLease,PcieLed,IdentityFault,JsonlTail,STOP

IDENTITY=0x4c444934
MODES=(0,1,2,3,4,9,10,11,12,13,14,15)


def patterns(mode):
    if mode==0:return ('off',)*3
    if mode==4:return ('red_steady',)*3
    if mode in (1,2,3):return tuple('green_steady' if i==mode else 'red_steady' for i in (1,2,3))
    return tuple('yellow_flash' if mode&(1<<i) else 'off' for i in range(3))


class SceneLease(DisplayLease):
    def consume(self,event,now_ns):
        previous=self.last_submitted_ns
        super().consume(event,now_ns)
        if type(event) is dict and event.get('kind')=='display':
            accepted=self.last_submitted_ns>previous and self.last_submitted_ns==event.get('submitted_ns')
            self.mode=0
            if accepted and not self.finished:
                try:
                    intent=event['lamp_intent'];mode=intent['mode']
                    if (type(intent) is not dict or set(intent)!={'mode','patterns','reason','junction_active','junction_muted'} or
                            type(mode) is not int or mode not in MODES or
                            type(intent['patterns']) not in (tuple,list) or tuple(intent['patterns'])!=patterns(mode) or
                            any(type(intent[k]) is not bool for k in ('junction_active','junction_muted')) or
                            type(intent['reason']) is not str):raise ValueError('malformed scene intent')
                    decision=event['decision'];reason=intent['reason']
                    if mode>=9:
                        # Task 4 has not installed a current-frame risk provider.
                        # Manual electrical tests are separate; a log alone is
                        # not allowed to invent an AI collision warning.
                        raise ValueError('validated risk provider not installed')
                    if mode in (1,2,3):
                        direction={1:'left',2:'straight',3:'right'}[mode]
                        if decision['allowed_action']!=direction or decision['simulated_action']!=direction:
                            raise ValueError('lamp direction contradicts actual decision')
                    if mode:
                        if reason=='junction':
                            if not intent['junction_active'] or intent['junction_muted']:raise ValueError('junction not armed')
                        elif reason=='lane_maneuver':
                            if intent['junction_active']:raise ValueError('lane request cannot bypass junction mute')
                        elif reason=='violation_alarm':
                            if mode!=4 or not decision['alarm_reasons']:raise ValueError('no actual violation')
                        else:raise ValueError('unrecognized current indicator source')
                    self.mode=mode
                except (KeyError,TypeError,ValueError):self.mode=0
        return self.word(now_ns)


class ScenePcieLed(PcieLed):
    def __init__(self,device='/sys/bus/pci/devices/0002:21:00.0'):
        self.device=Path(device);self.control=None;self.write_map=None
        with (self.device/'resource0').open('rb',buffering=0) as source:
            self.read_map=mmap.mmap(source.fileno(),8192,access=mmap.ACCESS_READ)
        if self.readback()[0]!=IDENTITY:
            self.read_map.close();raise IdentityFault('LEDI4 required; old LEDI3 receives no writes')
        try:
            with (self.device/'resource1').open('r+b',buffering=0) as target:
                self.write_map=mmap.mmap(target.fileno(),4096,access=mmap.ACCESS_WRITE)
            self.control=ctypes.c_uint32.from_buffer(self.write_map,0x240)
        except Exception:
            if self.write_map is not None:self.write_map.close()
            self.read_map.close();raise

    def write(self,word):
        if (type(word) is not int or not 0<=word<=0xffffffff or word>>16!=0x4c44 or
                word&0xf0 or (word&15) not in MODES or (word>>8&255)>200 or
                (word>>8&255)==0 and (word&15)!=0):raise ValueError('bounded LEDI4 command required')
        self.control.value=word
        return self.readback()








