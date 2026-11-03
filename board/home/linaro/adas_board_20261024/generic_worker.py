"""The existing single shared slot with the generic same-frame engine."""
import os
try:
    from .board_worker import SharedFrameWorker
    from .generic_engine import GenericRoadPerception
except ImportError:
    from board_worker import SharedFrameWorker
    from generic_engine import GenericRoadPerception


def select_engine(requested=None):
    if os.environ.get('ADAS_EXACT_LANES', '0') != '0' or os.environ.get('ADAS_LANE_PROFILE'):
        raise ValueError('this package contains the current generic road engine only')
    return requested if requested is not None else GenericRoadPerception


class GenericFrameWorker(SharedFrameWorker):
    def __init__(self,*,engine_factory=None,engine_kwargs=None):
        super().__init__(engine_factory=select_engine(engine_factory),engine_kwargs=engine_kwargs)
