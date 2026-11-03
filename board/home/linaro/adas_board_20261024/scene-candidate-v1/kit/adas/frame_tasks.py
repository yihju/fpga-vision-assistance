"""Bounded, joined work for one frame, preserving OpenCV thread-local IPP mode."""
from concurrent.futures import ThreadPoolExecutor

import cv2


def _inherit_ipp(enabled, not_exact):
    cv2.ipp.setUseIPP(enabled)
    cv2.ipp.setUseIPP_NotExact(not_exact)


def frame_tasks():
    return ThreadPoolExecutor(max_workers=2, thread_name_prefix='adas-frame', initializer=_inherit_ipp,
                              initargs=(cv2.ipp.useIPP(), cv2.ipp.useIPP_NotExact()))
