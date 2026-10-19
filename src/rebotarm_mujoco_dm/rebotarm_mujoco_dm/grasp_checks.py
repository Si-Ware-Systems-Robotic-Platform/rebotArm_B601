"""Measured grasp checks; controller success alone is insufficient."""
import math

EXPECTED_OFFSET = [-.014, 0., 0.]
CENTER_TOLERANCE = .012
CLOSE_TARGET = .015


def hold_error(history, stamp, minimum_height):
    """Samples: (simulation timestamp, ball centre in TCP, world height)."""
    recent = [v for v in history if abs(stamp-v[0]) <= 1.0]
    if len(recent)<5 or abs(stamp-recent[-1][0])>.35 or recent[-1][0]-recent[0][0]<.30:
        return 'Need at least 0.3 simulation seconds of fresh ball/TCP samples'
    if any(not all(math.isfinite(x) for x in [v[0],*v[1],v[2]]) for v in recent):
        return 'Invalid ball measurement'
    if min(v[2] for v in recent)<minimum_height:
        return 'Ball has not maintained the required lift height'
    largest_offset = max(math.dist(v[1],EXPECTED_OFFSET) for v in recent)
    if largest_offset > CENTER_TOLERANCE:
        last = recent[-1][1]
        return (f'Ball is not centred between the fingers: maximum centre error '
                f'{largest_offset*1000:.1f} mm (limit {CENTER_TOLERANCE*1000:.1f} mm); '
                f'latest ball-in-TCP offset {tuple(round(v,4) for v in last)} m')
    if max(math.dist(v[1],recent[-1][1]) for v in recent)>.003:
        drift = max(math.dist(v[1],recent[-1][1]) for v in recent)
        return f'Ball is slipping or vibrating relative to the fingers: {drift*1000:.1f} mm in recent samples'
    return None


def fingers_error(left, right):
    if not all(math.isfinite(v) for v in (left,right)):
        return 'Invalid finger measurement'
    if not .019 <= left <= .024:
        return 'Finger opening does not indicate a blocked grasp of the 42.7 mm ball'
    if abs(left+right)>.0015:
        return 'Finger mimic positions are inconsistent'
    return None


def bowl_rest_error(samples, stamp):
    """Samples: (simulation timestamp, measured ball centre in base_link)."""
    recent = [v for v in samples if 0 <= stamp-v[0] <= 1.]
    if len(recent) < 5 or abs(stamp-recent[-1][0]) > .35 or recent[-1][0]-recent[0][0] < .30:
        return 'Need at least 0.3 simulation seconds of fresh bowl-ball poses'
    if any(not all(math.isfinite(x) for x in [t, *p]) for t, p in recent):
        return 'Invalid bowl-ball measurement'
    motion = max(math.dist(p, recent[-1][1]) for _, p in recent)
    if motion > .002:
        return f'Ball is still moving in the bowl: {motion*1000:.1f} mm over the last second'
    return None
