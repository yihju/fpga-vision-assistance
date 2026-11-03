"""Active-low two-key decoder; no GPIO/PCIe/live binding or physical-test claim."""


class TwoKeyEvents:
    DEBOUNCE_NS=20000000
    LONG_NS=800000000

    def __init__(self):self.reset()

    def reset(self):
        self._raw=[1,1];self._stable=[1,1];self._changed=[0,0]
        self._down=[None,None];self._long=[False,False];self._armed=[False,False]
        self._last=None

    def sample(self,level0,level1,now_ns):
        if (any(type(v) is not int or v not in (0,1) for v in (level0,level1)) or
                type(now_ns) is not int or now_ns<0 or self._last is not None and now_ns<self._last):
            self.reset()
            raise ValueError('active-low integer levels and nondecreasing nonnegative integer clock required')
        initial=self._last is None
        self._last=now_ns;events=[]
        for index,level in enumerate((level0,level1)):
            if level!=self._raw[index]:
                self._raw[index]=level;self._changed[index]=now_ns
            if (self._stable[index]==1 and level==1 and self._down[index] is None and
                    (initial or now_ns-self._changed[index]>=self.DEBOUNCE_NS)):
                self._armed[index]=True
            if level!=self._stable[index] and now_ns-self._changed[index]>=self.DEBOUNCE_NS:
                self._stable[index]=level
                if level==0:
                    if self._armed[index]:self._down[index]=now_ns
                    self._long[index]=False
                else:
                    down=self._down[index]
                    if down is not None:
                        duration=self._changed[index]-down
                        if index==1 and (self._long[index] or duration>=self.LONG_NS):
                            events.append('KEY1_LONG_END')
                        elif duration<self.LONG_NS:events.append('KEY'+str(index)+'_SHORT')
                    self._down[index]=None;self._long[index]=False;self._armed[index]=True
            down=self._down[index]
            if (index==1 and level==0 and self._stable[index]==0 and down is not None and
                    now_ns-self._changed[index]>=self.DEBOUNCE_NS and
                    not self._long[index] and now_ns-down>=self.LONG_NS):
                self._long[index]=True;events.append('KEY1_LONG_START')
        return tuple(events)


def commands_for_event(event,control,scenario):
    """Caller applies each tuple atomically, and resets decoder on context loss."""
    if scenario not in ('intersection','lane','navigation'):raise ValueError('valid scenario required')
    if event=='KEY0_SHORT':
        if scenario=='lane':return ('s',{None:'l','left':'r','right':'t'}[control.turn])
        return ('s',{'left':'2','straight':'3','right':'1'}[control.lane])
    if event=='KEY1_SHORT':return ('s',) if control.requested_motion else ('w',)
    if event=='KEY1_LONG_START':return ('w','f')
    if event=='KEY1_LONG_END':return ('s',)
    raise ValueError('supported debounced key event required')
