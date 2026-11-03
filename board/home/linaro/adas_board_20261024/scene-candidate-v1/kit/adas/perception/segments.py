"""Boolean run lengths without repeated temporary index concatenations."""
from itertools import chain

import numpy as np


def boolean_runs(values):
    values = np.asarray(values, dtype=bool)
    if not len(values):
        return []
    changes = np.flatnonzero(values[1:] != values[:-1])+1
    result, start = [], 0
    for end in chain(changes, (len(values),)):
        result.append((bool(values[start]), int(end-start)))
        start = end
    return result
