"""Position association plus conservative recovery from trusted RGB templates."""

from dataclasses import dataclass, replace

import numpy as np

from rgbd.detection.appearance import compare, Parameters as AppearanceParameters
from rgbd.detection._memory import ObjectMemory
from rgbd.types import Track
from rgbd.geometry import reference_positions


@dataclass
class Parameters:
    match_distance: float = .12
    probation_period: float = 1.
    forget_after: float = 30.
    recovery_distance: float = .8
    recovery_margin: float = .08
    near_score: float = .45
    tight_distance: float = .03
    tight_color: float = .3
    strong_score: float = .9
    distinctive_color: float = .985
    template_interval: float = 1.
    template_distance: float = .04
    template_color: float = .75
    max_templates: int = 3
    min_size_ratios: tuple = (.25, .4, .55)
    max_size_ratios: tuple = (3.5, 2.2, 1.7)
    merge_min_color: float = .5
    continuity_distance: float = .06
    reappearance_penalty: float = .3  # Normalized association cost when a current owner exists.


@dataclass
class VisualTrack(Track):
    templates: tuple = ()
    extent: tuple = ()
    association: str = 'new'
    appearance_verified_at: float = -1.
    last_template_at: float = 0.
    recoveries: int = 0
    gallery_frozen: bool = False


def size_compatible(old, new, parameters):
    if not old or not new:
        return True
    ratio = np.array(new)/np.maximum(old, .005)
    return bool(np.all(ratio >= parameters.min_size_ratios) and np.all(ratio <= parameters.max_size_ratios))


class _AppearanceMemory(ObjectMemory):
    """Hungarian assignment; uncertain recovery never merges product instances.

    Missing confirmed tracks keep their signatures. Far/gap
    recovery needs strong appearance and mutual ambiguity margins. A hand/box
    merge can remain unassigned instead of corrupting an established track.
    """

    def __init__(self, parameters, comparison_parameters=None):
        super().__init__(parameters)
        self.comparison_parameters = AppearanceParameters() if comparison_parameters is None else comparison_parameters
        self.events, self.blocked, self.matches = [], np.zeros(0, bool), {}

    def clear(self):
        super().clear()
        self.events, self.matches = [], {}

    def update(self, positions, dt, *, observations=None):
        self.events, self.matches = [], {}
        self.blocked = np.zeros(len(positions), bool)
        return super().update(positions, dt, observations=observations)

    def _costs(self, positions, observations, previous_visible):
        p = self.parameters
        if observations is None:
            return super()._costs(positions, observations, previous_visible)
        distance = np.linalg.norm(np.array([t.position for t in self.tracks])[:, None]-positions[None, :], axis=2)
        scores = np.zeros(distance.shape)
        details = {}
        for row, track in enumerate(self.tracks):
            for col, observation in enumerate(observations):
                if observation is None or observation.signature is None or not track.templates or distance[row, col] > p.recovery_distance:
                    continue
                best = None
                for template in track.templates:
                    value = compare(template, observation.signature, self.comparison_parameters)
                    if not value.reliable and value.inliers >= self.comparison_parameters.min_matches:
                        # A partial view may be a better query than a full
                        # template containing many currently hidden features.
                        reverse = compare(observation.signature, template, self.comparison_parameters)
                        if reverse.score > value.score:
                            value = reverse
                    if best is None or value.score > best.score:
                        best = value
                    if best.score > .98 and best.reliable:
                        break
                scores[row, col], details[row, col] = best.score, best
                if track.confirmed and not size_compatible(track.extent, observation.extent, p):
                    bigger = any(observation.extent[i] > p.max_size_ratios[i]*track.extent[i] for i in (1, 2))
                    if bigger and (best.reliable or (distance[row, col] <= p.tight_distance and best.color >= p.merge_min_color)):
                        self.blocked[col] = True
        for col in np.flatnonzero(self.blocked):
            observation = observations[col]
            # A tall existing package beside a short one is not a merge.
            # Let a compatible, plausible owner protect its own detection.
            for row, track in enumerate(self.tracks):
                value = details.get((row, col))
                if (value is not None and size_compatible(track.extent, observation.extent, p)
                        and ((distance[row, col] <= p.template_distance and value.score >= p.near_score)
                             or (value.reliable and value.score >= p.strong_score))):
                    self.blocked[col] = False
                    break
        costs = np.full(distance.shape, np.inf)
        active = np.array([previous_visible[t.id] for t in self.tracks])
        continuing = np.zeros(distance.shape, bool)
        for (row, col), value in details.items():
            track, observation = self.tracks[row], observations[col]
            continuing[row, col] = (active[row] and track.confirmed
                and distance[row, col] <= p.continuity_distance and value.color >= p.tight_color
                and size_compatible(track.extent, observation.extent, p)
                and (value.score >= p.near_score or distance[row, col] <= p.tight_distance))
        for row, track in enumerate(self.tracks):
            for col, observation in enumerate(observations):
                if self.blocked[col]:
                    continue
                value = details.get((row, col))
                compatible = observation is None or size_compatible(track.extent, observation.extent, p)
                if not compatible:
                    continue
                d = distance[row, col]
                other_tracks = np.delete(scores[:, col], row)
                other_detections = np.delete(scores[row], col)
                lead = min(scores[row, col]-max(other_tracks, default=0),
                           scores[row, col]-max(other_detections, default=0))
                reliable = value is not None and value.reliable and value.score >= p.strong_score
                feature_count = lambda signature: 0 if signature.descriptors is None else len(signature.descriptors)
                featureless = (observation is not None and observation.signature is not None and track.templates
                               and feature_count(observation.signature) < self.comparison_parameters.min_matches
                               and max(feature_count(t) for t in track.templates) < self.comparison_parameters.min_matches)
                distinctive_color = value is not None and value.color >= p.distinctive_color and featureless
                recover = (track.confirmed and d <= p.recovery_distance and lead >= p.recovery_margin
                           and (reliable or distinctive_color))
                near = d <= p.match_distance
                if value is not None:
                    rivals = scores[:, col].copy()
                    rivals[row] = 0
                    if continuing[row, col]:
                        # Missing duplicates must not veto a plausible current
                        # owner. Other visible objects still override it, so
                        # genuine position swaps follow their appearance.
                        rivals[~active] = 0
                    competitor = float(rivals.max(initial=0))
                    # A clear feature match to another identity takes priority
                    # over being nearest to this one's last position.
                    claimed_elsewhere = competitor >= p.strong_score and competitor-value.score >= p.recovery_margin
                    # An occluded/newly exposed face can have weak features.
                    # Preserve ordinary positional continuity; only a large
                    # jump requires the stricter appearance recovery test.
                    near &= (value.score >= p.near_score or (d <= p.tight_distance and value.color >= p.tight_color)) and not claimed_elsewhere
                elif track.templates:
                    near &= previous_visible[track.id] and d <= p.tight_distance
                if near:
                    score = 0. if value is None else value.score
                    cost = .7*d/p.match_distance+.3*(1-score)
                    if not active[row] and continuing[:, col].any():
                        cost += p.reappearance_penalty
                elif recover:
                    cost = .05+.25*(1-value.score)+.05*d/p.recovery_distance
                else:
                    continue
                costs[row, col] = cost*p.match_distance
                self.matches[track.id, col] = (value, recover and (not previous_visible[track.id] or d > p.match_distance), d)
        if self.blocked.any():
            self.events.append(dict(kind='suspicious_segmentation', detections=np.flatnonzero(self.blocked).tolist()))
        return costs

    def _allow_new(self, index):
        return not self.blocked[index]

    def _make_track(self, position, index, observation):
        return VisualTrack(self.next_id, position.copy(), confirmed=self.parameters.probation_period == 0,
            detection_index=index, templates=() if observation is None or observation.signature is None else (observation.signature,),
            extent=() if observation is None else observation.extent, last_template_at=self.elapsed_seconds)

    def _record_match(self, track, index, position, observation, dt):
        p = self.parameters
        value, recovered, distance = self.matches.get((track.id, index), (None, False, 0.))
        super()._record_match(track, index, position, observation, dt)
        track.association = 'appearance' if recovered else ('position+appearance' if value is not None and value.reliable else 'position')
        if value is not None and value.reliable:
            track.appearance_verified_at = self.elapsed_seconds
        if recovered:
            track.recoveries += 1
            self.events.append(dict(kind='appearance_recovery', id=track.id, distance_m=float(distance),
                                    score=value.score, inliers=value.inliers, recoveries=track.recoveries))
        if observation is None or observation.signature is None:
            return
        if not track.templates:
            track.templates, track.extent = (observation.signature,), observation.extent
        elif (track.confirmed and not track.gallery_frozen and not recovered and distance <= p.template_distance
              and value is not None and value.reliable and value.color >= p.template_color
              and self.elapsed_seconds-track.last_template_at >= p.template_interval):
            # Keep the initial view; add at most two more trusted views.
            gallery = (*track.templates, observation.signature)
            track.templates = gallery if len(gallery) <= p.max_templates else (gallery[0], *gallery[-(p.max_templates-1):]) if p.max_templates > 1 else gallery[:1]
            track.last_template_at = self.elapsed_seconds


class ObjectPersistor:
    """Current detections + pose + appearance + capture dt -> ID snapshots.

    RGB appearance is supplied, not extracted here. Reference changes clear
    tracks; IDs are never reused. Missing pose ages memory without association.
    """

    def __init__(self, calibration, parameters=None, comparison_parameters=None):
        self.calibration = calibration
        self.parameters = Parameters() if parameters is None else parameters
        self.memory = _AppearanceMemory(self.parameters, comparison_parameters)
        self.reference_frame = None

    @property
    def elapsed_seconds(self):
        return self.memory.elapsed_seconds

    @property
    def events(self):
        return tuple(self.memory.events)

    def clear(self):
        self.memory.clear()
        self.reference_frame = None

    def reset(self):
        self.clear()
        self.memory.elapsed_seconds = 0.

    def freeze_templates(self, identities):
        """An external ambiguity signal can suspend gallery learning, not tracking."""
        identities = set(identities)
        for track in self.memory.tracks:
            track.gallery_frozen = track.id in identities

    def update(self, detections, pose, dt, observations=None):
        p = self.parameters
        if (not np.isfinite([p.match_distance, p.forget_after, p.probation_period, p.recovery_distance]).all()
                or min(p.match_distance, p.forget_after, p.recovery_distance, p.max_templates) <= 0
                or p.probation_period < 0 or not 0 <= p.recovery_margin <= 1):
            raise ValueError('Invalid persistence parameters')
        self.memory.parameters = p
        if observations is not None and len(observations) != len(detections):
            raise ValueError('One appearance observation is required per detection')
        if pose is None:
            positions, observations = [], None
        else:
            if self.reference_frame not in (None, pose.reference_frame):
                self.clear()
            self.reference_frame = pose.reference_frame
            positions = reference_positions(detections, self.calibration, pose)
        tracks = self.memory.update(positions, dt, observations=observations)
        return tuple(replace(t, position=t.position.copy()) for t in tracks)


if __name__ == '__main__':
    from rgbd.demos.detection import main
    main('memory')
