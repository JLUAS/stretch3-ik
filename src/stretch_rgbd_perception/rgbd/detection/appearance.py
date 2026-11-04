"""Small masked RGB signatures for object association, not product recognition."""

from dataclasses import dataclass

import cv2
import numpy as np

from rgbd.geometry import rgb_ownership


@dataclass
class Parameters:
    max_dimension: int = 240
    min_pixels: int = 40
    erosion: int = 17
    closing: int = 5
    projection_stride: int = 4
    mask_scale: float = .5  # Only ownership/morphology; RGB descriptors use native crops.
    features: int = 180
    levels: int = 4
    patch_size: int = 15
    fast_threshold: int = 10
    ratio: float = .75
    max_hamming: float = 60
    min_matches: int = 6
    ransac_pixels: float = 3.
    min_inlier_ratio: float = .65
    min_coverage: float = .08
    min_color: float = .15
    min_scale: float = .25
    max_scale: float = 4.
    target_inliers: int = 12
    ransac_iterations: int = 300


@dataclass(frozen=True)
class Signature:
    histogram: np.ndarray
    descriptors: np.ndarray | None
    points: np.ndarray
    shape: tuple
    box: tuple = ()  # Exclusive native RGB crop; points are in resized crop pixels.


@dataclass(frozen=True)
class Observation:
    signature: Signature | None
    extent: tuple
    box: tuple | None = None  # Exclusive native RGB pixels, for inspection/OCR.


@dataclass(frozen=True)
class Similarity:
    score: float
    color: float
    reliable: bool
    inliers: int = 0


def describe(rgb, mask, parameters=None, *, origin=(0, 0)):
    """Keep descriptor patches, not just keypoint centers, inside the object."""
    p = Parameters() if parameters is None else parameters
    y, x = np.where(mask)
    if len(x) < p.min_pixels:
        return None
    x0, x1 = max(0, int(x.min())-2), min(rgb.shape[1], int(x.max())+3)
    y0, y1 = max(0, int(y.min())-2), min(rgb.shape[0], int(y.max())+3)
    crop, mask = rgb[y0:y1, x0:x1], mask[y0:y1, x0:x1].astype(np.uint8)*255
    scale = p.max_dimension/max(crop.shape[:2])
    size = max(1, round(crop.shape[1]*scale)), max(1, round(crop.shape[0]*scale))
    crop = cv2.resize(crop, size)
    mask = cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)
    interior = cv2.erode(mask, np.ones((p.erosion, p.erosion), np.uint8))
    if np.count_nonzero(interior) < p.min_pixels:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
    histogram = cv2.calcHist([hsv], [0, 1], interior, [12, 4], [0, 180, 0, 256])
    histogram /= max(float(histogram.sum()), 1)
    orb = cv2.ORB_create(nfeatures=p.features, nlevels=p.levels, edgeThreshold=p.patch_size//2+1,
                         patchSize=p.patch_size, fastThreshold=p.fast_threshold)
    keypoints, descriptors = orb.detectAndCompute(cv2.cvtColor(crop, cv2.COLOR_RGB2GRAY), interior)
    points = np.float32([p.pt for p in keypoints]).reshape(-1, 2)
    for array in (histogram, descriptors, points):
        if array is not None:
            array.setflags(write=False)
    return Signature(histogram, descriptors, points, crop.shape[:2],
                     tuple(int(v) for v in (x0+origin[0], y0+origin[1], x1+origin[0], y1+origin[1])))


def compare(first, second, parameters=None):
    p = Parameters() if parameters is None else parameters
    color = float(np.clip(1-cv2.compareHist(first.histogram, second.histogram, cv2.HISTCMP_BHATTACHARYYA), 0, 1))
    # Color-only recovery is allowed only with a very high score AND a clear
    # lead over competing tracks/detections, enforced by the memory matcher.
    fallback = .8*color
    if color < p.min_color or first.descriptors is None or second.descriptors is None:
        return Similarity(fallback, color, False)
    if min(len(first.descriptors), len(second.descriptors)) < p.min_matches:
        return Similarity(fallback, color, False)
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(first.descriptors, second.descriptors, k=2)
    good = [a for pair in pairs if len(pair) == 2 for a, b in [pair] if a.distance < p.ratio*b.distance and a.distance < p.max_hamming]
    unique = {}
    for match in sorted(good, key=lambda m: m.distance):
        unique.setdefault(match.trainIdx, match)
    good = list(unique.values())
    if len(good) < p.min_matches:
        return Similarity(fallback, color, False)
    a = np.float32([first.points[m.queryIdx] for m in good])
    b = np.float32([second.points[m.trainIdx] for m in good])
    transform, inliers = cv2.estimateAffine2D(a, b, method=cv2.RANSAC, ransacReprojThreshold=p.ransac_pixels,
                                            maxIters=p.ransac_iterations, confidence=.95, refineIters=5)
    if transform is None:
        return Similarity(fallback, color, False)
    scales = np.linalg.svd(transform[:, :2], compute_uv=False)
    if np.linalg.det(transform[:, :2]) <= 0 or scales.min() < p.min_scale or scales.max() > p.max_scale:
        return Similarity(fallback, color, False)
    keep = inliers.ravel().astype(bool)
    count, ratio = int(keep.sum()), float(keep.mean())
    coverage = min(float(np.prod(np.ptp(a[keep], axis=0)))/np.prod(first.shape),
                   float(np.prod(np.ptp(b[keep], axis=0)))/np.prod(second.shape)) if count else 0
    required = max(p.min_matches, min(p.target_inliers, round(.15*min(len(first.descriptors), len(second.descriptors)))))
    reliable = count >= required and ratio >= p.min_inlier_ratio and coverage >= p.min_coverage and color >= p.min_color
    feature = min(1., count/p.target_inliers)*ratio if reliable else 0.
    score = max(fallback, .85+.15*(.75*feature+.25*color)) if reliable else fallback
    return Similarity(float(score), color, bool(reliable), count)


class RGBObjectAppearance:
    """Calibrated depth masks -> RGB silhouettes -> bounded-size signatures."""

    def __init__(self, calibration, parameters=None):
        self.calibration = calibration
        self.parameters = Parameters() if parameters is None else parameters

    def observe(self, frame, detections):
        p = self.parameters
        if min(p.projection_stride, p.max_dimension, p.erosion, p.closing, p.features) < 1:
            raise ValueError('Appearance dimensions/counts must be positive')
        if not .1 <= p.mask_scale <= 1:
            raise ValueError('mask_scale must be between .1 and 1')
        if not detections:
            return ()
        rgb_labels = rgb_ownership(frame, detections, self.calibration, p.projection_stride, p.mask_scale)
        observations = []
        for i, obj in enumerate(detections):
            y, x = np.where(rgb_labels == i)
            if len(x) < max(4, round(p.min_pixels*p.mask_scale**2)):
                observations.append(Observation(None, tuple(sorted(obj.extent))))
                continue
            # Morphology and feature extraction need only this small ROI.
            x0, x1 = max(0, int(x.min())-4), min(rgb_labels.shape[1], int(x.max())+5)
            y0, y1 = max(0, int(y.min())-4), min(rgb_labels.shape[0], int(y.max())+5)
            mask = (rgb_labels[y0:y1, x0:x1] == i).astype(np.uint8)
            # Fill the sampling grid's small holes without using the enclosing
            # rectangle as a foreground mask. Erosion happens in describe().
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((p.closing, p.closing), np.uint8))
            y, x = np.where(mask)
            if len(x) >= p.min_pixels:
                # Boxes/cylinders are convex: depth holes in a printed face
                # should not exclude its RGB lettering from the signature.
                hull = cv2.convexHull(np.column_stack((x, y)).astype(np.int32))
                cv2.fillConvexPoly(mask, hull, 1)
            owners = rgb_labels[y0:y1, x0:x1]
            mask[(owners >= 0) & (owners != i)] = 0
            factor = np.array(frame.color.shape[1::-1])/np.array(rgb_labels.shape[::-1])
            left, top = np.floor([x0, y0]*factor).astype(int)
            right, bottom = np.ceil([x1, y1]*factor).astype(int)
            mask = cv2.resize(mask, (right-left, bottom-top), interpolation=cv2.INTER_NEAREST)
            signature = describe(frame.color[top:bottom, left:right], mask, p, origin=(left, top))
            box = (int(left), int(top), int(right), int(bottom)) if signature is None else signature.box
            observations.append(Observation(signature, tuple(sorted(obj.extent)), box))
        return tuple(observations)


if __name__ == '__main__':
    from rgbd.demos.detection import main
    main('appearance')
