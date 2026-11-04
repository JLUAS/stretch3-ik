"""Best accepted product evidence per persistent ID; independent of OCR execution."""

from dataclasses import dataclass

from rgbd.classification.matcher import LabelMatch, Observation


@dataclass(frozen=True)
class Identity:
    label: str
    evidence: LabelMatch
    timestamp_ns: int  # Capture time of the best reading, not latest unsuccessful attempt.


class ClassificationMemory:
    def __init__(self):
        self.labels = {}
        self.latest = {}
        self.conflicts = set()

    def clear(self):
        self.labels.clear()
        self.latest.clear()
        self.conflicts.clear()

    def prune(self, active_ids):
        active_ids = set(active_ids)
        self.labels = {i: value for i, value in self.labels.items() if i in active_ids}
        self.latest = {i: value for i, value in self.latest.items() if i in active_ids}
        self.conflicts &= active_ids

    def update(self, identity, observation: Observation, timestamp_ns):
        self.latest[identity] = observation
        previous = self.labels.get(identity)
        candidate = observation.best
        if candidate is not None and candidate.accepted and (
                previous is None or candidate.quality > previous.evidence.quality):
            self.labels[identity] = Identity(candidate.label, candidate, timestamp_ns)
        remembered = self.labels.get(identity)
        conflict = remembered is not None and (any(m.label != remembered.label for m in observation.matches)
                    or (previous is not None and previous.label != remembered.label))
        if conflict:
            self.conflicts.add(identity)
        else:
            self.conflicts.discard(identity)
        return remembered


if __name__ == '__main__':
    from rgbd.demos.detection import main
    main('classification')
