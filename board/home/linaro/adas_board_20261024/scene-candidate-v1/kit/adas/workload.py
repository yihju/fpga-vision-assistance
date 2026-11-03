"""Explicit task budgets; never infer road facts from controls or a filename."""


def validate_workload(workload, scenario):
    if type(workload) is not str or workload not in ('default', 'intersection_light'):
        raise ValueError('workload must be default or intersection_light')
    if workload == 'intersection_light' and scenario != 'intersection':
        raise ValueError('intersection_light workload requires intersection scenario')


def intersection_workload(control, scenario):
    if scenario not in ('intersection', 'lane', 'navigation'):
        raise ValueError('invalid workload scenario')
    # Selecting the intersection approach lane (1/2/3) is not a lane change.
    # An explicit L/R intent does depend on boundary and target-side evidence.
    return 'intersection_light' if scenario == 'intersection' and control.turn is None else 'default'
