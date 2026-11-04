"""Bounded asynchronous crop OCR attached to capture-time persistent IDs."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from time import perf_counter

import numpy as np

from rgbd.classification.memory import ClassificationMemory
from rgbd.classification.ocr import OCRReader
from rgbd.classification.matcher import ProductMatcher
from rgbd.geometry import padded_box


@dataclass
class Parameters:
    refresh_seconds: float = 5.  # Capture elapsed time, never wall-clock playback time.
    padding: float = .1
    max_position_change: float = .08


class ObjectClassifier:
    """One worker, one outstanding job; model load also happens in the worker.

    poll(tracks) applies completed evidence only to retained IDs. A large move
    requires a visual verification since submission. Workers never mutate track
    or classification memory. Call poll/submit serially from the owning thread.
    """

    def __init__(self, parameters=None, *, matcher=None, memory=None, reader_factory=OCRReader):
        self.parameters = Parameters() if parameters is None else parameters
        self.matcher = ProductMatcher() if matcher is None else matcher
        self.memory = ClassificationMemory() if memory is None else memory
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='object-ocr')
        self.future = self.executor.submit(reader_factory)
        self.reader = None
        self.job = self.last_result = None
        self.last_attempt = {}
        self.epoch, self.completed_jobs = 0, 0
        self.state, self.error = 'loading', ''

    def clear(self):
        self.epoch += 1
        self.memory.clear()
        self.last_attempt.clear()

    def poll(self, tracks):
        active = {t.id: t for t in tracks if t.confirmed}
        self.memory.prune(active)
        self.last_attempt = {i: value for i, value in self.last_attempt.items() if i in active}
        if self.future is None or not self.future.done():
            return None
        future, self.future = self.future, None
        try:
            value = future.result()
        except Exception as exc:
            self.error, self.state = f'{type(exc).__name__}: {exc}', 'failed'
            return dict(kind='ocr_error', error=self.error)
        if self.reader is None:
            self.reader, self.state = value, 'idle'
            return dict(kind='ocr_ready')
        self.state, self.error = 'idle', ''
        self.completed_jobs += 1
        accepted, rejected = [], []
        job = self.job
        observations = tuple(self.matcher.identify([r for r in value['readings'] if r.region == i])
                             for i in range(len(job['seeds'])))
        value['observations'] = observations
        self.last_result = value
        for seed, observation in zip(job['seeds'], observations):
            track = active.get(seed['id'])
            reason = 'old generation' if job['epoch'] != self.epoch else 'ID expired' if track is None else None
            if reason is None:
                distance = float(np.linalg.norm(track.position-seed['anchor']))
                verified = (getattr(track, 'appearance_verified_at', -1) >= job['elapsed_seconds']
                            and getattr(track, 'association', '') in ('appearance', 'position+appearance'))
                if distance > self.parameters.max_position_change and not verified:
                    reason = 'unverified position change'
            if reason is not None:
                rejected.append(dict(id=seed['id'], reason=reason))
            else:
                self.memory.update(seed['id'], observation, job['timestamp_ns'])
                accepted.append(seed['id'])
        return dict(kind='ocr_complete', timestamp_ns=job['timestamp_ns'], accepted=accepted, rejected=rejected,
                    elapsed_ms=value['elapsed_ms'])

    def submit(self, frame, tracks, observations, elapsed_seconds, *, force=False):
        p = self.parameters
        if p.refresh_seconds < 0 or p.padding < 0 or p.max_position_change <= 0:
            raise ValueError('Invalid classification scheduling parameters')
        if self.future is not None or self.reader is None:
            return False
        seeds = []
        for track in tracks:
            index = track.detection_index
            if not track.confirmed or index is None or not 0 <= index < len(observations):
                continue
            observation = observations[index]
            if observation.box is None or (not force and elapsed_seconds-self.last_attempt.get(track.id, -np.inf) < p.refresh_seconds):
                continue
            seeds.append(dict(id=track.id, anchor=track.position.copy(),
                              box=padded_box(observation.box, frame.color.shape, p.padding)))
        if not seeds:
            return False
        self.job = dict(epoch=self.epoch, timestamp_ns=frame.timestamp_ns, elapsed_seconds=elapsed_seconds,
                        seeds=seeds, rgb=frame.color.copy())
        for seed in seeds:
            self.last_attempt[seed['id']] = elapsed_seconds
        self.future = self.executor.submit(self._read, self.job)
        self.state = 'busy'
        return True

    def _read(self, job):
        started = perf_counter()
        readings = self.reader.read(job['rgb'], [seed['box'] for seed in job['seeds']])
        return dict(timestamp_ns=job['timestamp_ns'], readings=readings,
                    elapsed_ms=(perf_counter()-started)*1000, seeds=job['seeds'], rgb=job['rgb'])

    def close(self):
        self.executor.shutdown(wait=True, cancel_futures=True)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


if __name__ == '__main__':
    from rgbd.demos.detection import main
    main('classification')
