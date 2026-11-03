"""Perception availability is separate from action legality and model accuracy."""

VALUES = dict(signal={'red', 'green', 'yellow', 'black'},
              left_boundary={'solid', 'dashed'}, right_boundary={'solid', 'dashed'},
              path_clear={'clear', 'blocked'}, left_clear={'clear', 'blocked'},
              right_clear={'clear', 'blocked'}, navigation={'left', 'straight', 'right'})


def perception_status(observations, decision):
    fields = {}
    for observation in observations:
        if observation.key != decision.key or observation.kind in fields:
            raise ValueError('cross-frame or duplicate observation status')
        fields[observation.kind] = ('NOT_APPLICABLE' if not observation.applicable else
                                    'STALE' if not observation.fresh else
                                    'UNKNOWN' if observation.value not in VALUES.get(observation.kind, set()) else
                                    observation.value.upper())
    for required in ('signal', 'left_boundary', 'right_boundary', 'path_clear', 'left_clear', 'right_clear'):
        fields.setdefault(required, 'UNKNOWN')
    unknown_fields = sorted(kind for kind, value in fields.items() if value in ('UNKNOWN', 'STALE'))
    state = 'INVALID' if not decision.valid else 'UNKNOWN' if unknown_fields or not fields else 'KNOWN'
    return dict(state=state, fields=fields, unknown_fields=unknown_fields,
                meaning='availability_only_not_accuracy_or_permission')
