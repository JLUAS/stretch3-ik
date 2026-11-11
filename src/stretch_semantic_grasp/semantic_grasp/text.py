"""Typed command -> object phrase for the detector, plus an optional room."""
import re
from dataclasses import dataclass

VERBS = r'(?:please\s+)?(?:grab|grasp|pick\s+up|pick|get|fetch|bring(?:\s+me)?|take|find)'
ROOM = re.compile(r'\s+(?:in|on|from|at)\s+the\s+(?P<room>[a-z ]+?)\s*$')


@dataclass(frozen=True)
class Query:
    phrase: str           # what the detector looks for, e.g. "a red mug"
    room: str | None      # room name as typed, matched against rooms.yaml


def parse_command(text, rooms=()):
    """'Grab the red mug in the kitchen' -> Query('a red mug', 'kitchen').
    A trailing 'in the X' is a room only when X is a known room, so
    'the cup on the table' keeps its words for the detector."""
    t = ' '.join(text.lower().strip().rstrip('.!?').split())
    t = re.sub(rf'^{VERBS}\s+', '', t)
    room = None
    m = ROOM.search(t)
    if m and m['room'] in rooms:
        room, t = m['room'], t[:m.start()]
    t = re.sub(r'^(?:the|a|an|my|some)\b', '', t).strip()
    if not t:
        raise ValueError(f'no object in command: {text!r}')
    return Query(f'a {t}', room)
