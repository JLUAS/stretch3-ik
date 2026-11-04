"""Inspectable, geometry-only tabletop object segmentation for RGB-D frames.

The implementation deliberately keeps the calibration and image coordinate
system supplied by the source. Open3D fits planes and clusters voxel centroids;
NumPy groups candidate voxels and maps them back to pixels without ragged traces.
Optional half-resolution processing restores masks/contours to native dimensions.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from time import perf_counter

import cv2
import numpy as np
import open3d as o3d

from rgbd.types import Camera


@dataclass
class TabletopSegmentationParameters:
    """Metric thresholds controlling the transparent segmentation pipeline."""

    min_depth: float = 0.25
    max_depth: float = 2.0
    voxel_size: float = 0.005
    plane_roi_top: float = 0.40
    object_roi_top: float = 0.50
    object_roi_left: float = 0.05
    object_roi_right: float = 0.90
    plane_distance_threshold: float = 0.010
    ransac_iterations: int = 1_000
    plane_candidate_count: int = 4
    plane_max_points: int = 12_000
    plane_stride: int = 4
    plane_min_inliers: int = 1_000
    plane_min_inlier_fraction: float = 0.25
    support_normal_y_min: float = 0.75
    object_min_height: float = 0.010
    object_max_height: float = 0.40
    min_object_peak_height: float = 0.050
    max_object_centroid_depth: float = 1.05
    cluster_eps: float = 0.015
    cluster_min_points: int = 10
    min_object_pixels: int = 500
    min_object_extent: float = 0.02
    max_object_extent: float = 0.35
    max_object_footprint: float | None = .20  # Table-plane diameter, meters; None disables.
    bottom_padding_pixels: int = 5
    border_margin_pixels: int = 2
    split_merged_clusters: bool = True
    merged_cluster_min_width: float = 0.10
    split_valley_ratio: float = 0.20
    split_min_child_pixels: int = 500
    split_min_child_width_pixels: int = 10
    random_seed: int = 0

    def validate(self) -> None:
        """Raise ``ValueError`` for parameter combinations that cannot work."""

        if self.max_object_footprint is not None and (not np.isfinite(self.max_object_footprint) or self.max_object_footprint <= 0):
            raise ValueError('max_object_footprint must be positive or None')

        if not 0 < self.min_depth < self.max_depth:
            raise ValueError("depth limits must satisfy 0 < min_depth < max_depth")
        if self.voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        if not 0 <= self.plane_roi_top < 1:
            raise ValueError("plane_roi_top must be in the range [0, 1)")
        if not 0 <= self.object_roi_top < 1:
            raise ValueError("object_roi_top must be in the range [0, 1)")
        if not 0 <= self.object_roi_left < self.object_roi_right <= 1:
            raise ValueError(
                "object ROI must satisfy 0 <= left < right <= 1"
            )
        if self.plane_distance_threshold <= 0:
            raise ValueError("plane_distance_threshold must be positive")
        if self.ransac_iterations <= 0:
            raise ValueError("ransac_iterations must be positive")
        if self.plane_candidate_count <= 0 or self.plane_min_inliers < 3:
            raise ValueError("plane candidate count and minimum inliers must be positive")
        if self.plane_max_points < 3:
            raise ValueError("plane_max_points must be at least 3")
        if self.plane_stride < 1:
            raise ValueError("plane_stride must be positive")
        if not 0 < self.plane_min_inlier_fraction <= 1:
            raise ValueError("plane_min_inlier_fraction must be in the range (0, 1]")
        if not 0 <= self.support_normal_y_min <= 1:
            raise ValueError("support_normal_y_min must be in the range [0, 1]")
        if not 0 <= self.object_min_height < self.object_max_height:
            raise ValueError(
                "object heights must satisfy 0 <= min_height < max_height"
            )
        if not self.object_min_height <= self.min_object_peak_height <= self.object_max_height:
            raise ValueError(
                "min_object_peak_height must lie inside the object height range"
            )
        if not self.min_depth < self.max_object_centroid_depth <= self.max_depth:
            raise ValueError("object centroid depth must lie inside the depth range")
        if self.cluster_eps <= 0:
            raise ValueError("cluster_eps must be positive")
        if self.cluster_min_points <= 0 or self.min_object_pixels <= 0:
            raise ValueError("cluster point thresholds must be positive")
        if (not np.isfinite([self.min_object_extent, self.max_object_extent]).all()
                or not 0 <= self.min_object_extent < self.max_object_extent):
            raise ValueError("object extent limits must be finite and satisfy 0 <= min < max")
        if not isinstance(self.bottom_padding_pixels, int) or self.bottom_padding_pixels < 0:
            raise ValueError("bottom_padding_pixels must be a nonnegative integer")
        if self.border_margin_pixels < -1:
            raise ValueError("border_margin_pixels must be -1 or greater")
        if self.merged_cluster_min_width <= 0:
            raise ValueError("merged_cluster_min_width must be positive")
        if not 0 <= self.split_valley_ratio <= 1:
            raise ValueError("split_valley_ratio must be in the range [0, 1]")
        if self.split_min_child_pixels <= 0 or self.split_min_child_width_pixels <= 0:
            raise ValueError("split child thresholds must be positive")


@dataclass(frozen=True)
class DetectedObject:
    """Geometric summary for one accepted point cluster."""

    label: int
    pixel_count: int
    voxel_count: int
    centroid_xyz: tuple[float, float, float]
    min_bound_xyz: tuple[float, float, float]
    max_bound_xyz: tuple[float, float, float]
    extent_xyz: tuple[float, float, float]
    max_height_above_plane: float
    centroid_depth_pixel: tuple[float, float]
    contour_depth_pixels: np.ndarray  # Enclosing padded rectangle, 4x2 depth (x, y).
    footprint_diameter: float = 0.0  # Measured diameter projected onto the support plane.


@dataclass(frozen=True)
class TabletopSegmentationResult:
    """Plane, pixel masks, object geometry, and performance diagnostics."""

    plane_model: tuple[float, float, float, float]
    table_mask: np.ndarray
    object_labels: np.ndarray
    objects: tuple[DetectedObject, ...]
    valid_pixel_count: int
    downsampled_point_count: int  # Voxels in the sampled plane-fitting region.
    table_voxel_count: int  # Estimated inliers before the RANSAC point cap.
    candidate_voxel_count: int
    noise_voxel_count: int
    timings_ms: dict[str, float]
    bottom_padding_pixels: int
    rejections: dict[str, int]
    processing_scale: float = 1.0


def footprint_diameter(points, normal):
    """Rotation-invariant visible footprint; height above the table is removed."""
    if len(points) < 2:
        return 0.
    normal = np.array(normal, dtype=float, copy=True)
    normal /= np.linalg.norm(normal)
    u = np.cross(normal, np.eye(3)[np.argmin(np.abs(normal))])
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    xy = np.column_stack((points @ u, points @ v)).astype(np.float32)
    hull = cv2.convexHull(xy).reshape(-1, 2)
    return float(np.sqrt(np.max(np.sum((hull[:, None]-hull[None, :])**2, axis=2))))


def bounding_contour(pixels: np.ndarray, shape: tuple[int, int], bottom_padding: int = 0) -> np.ndarray:
    """Enclose all input pixels; extend only the bottom, clipped to the image."""
    x, y, width, height = cv2.boundingRect(np.asarray(pixels, dtype=np.int32))
    right = x + width - 1
    bottom = min(y + height - 1 + bottom_padding, shape[0] - 1)
    return np.array(((x, y), (right, y), (right, bottom), (x, bottom)), dtype=np.int32)


def _scaled_intrinsics(
    intrinsics: Camera,
    shape: tuple[int, int],
) -> tuple[float, float, float, float]:
    height, width = shape
    calibrated_aspect = intrinsics.width / intrinsics.height
    actual_aspect = width / height
    if not np.isclose(actual_aspect, calibrated_aspect, rtol=1e-3):
        raise ValueError(
            f"depth shape {shape} is incompatible with calibrated size "
            f"({intrinsics.height}, {intrinsics.width})"
        )

    scale_x = width / intrinsics.width
    scale_y = height / intrinsics.height
    fx = intrinsics.K[0, 0] * scale_x
    fy = intrinsics.K[1, 1] * scale_y
    cx = (intrinsics.K[0, 2] + 0.5) * scale_x - 0.5
    cy = (intrinsics.K[1, 2] + 0.5) * scale_y - 0.5
    return fx, fy, cx, cy


def _as_float_tuple(values: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(value) for value in values)  # type: ignore[return-value]


def _split_at_horizontal_valley(
    member_indices: np.ndarray,
    pixel_x: np.ndarray,
    points: np.ndarray,
    params: TabletopSegmentationParameters,
) -> tuple[np.ndarray, ...]:
    """Split a suspiciously wide cluster at a strong vertical image gap.

    DBSCAN can join adjacent cartons through a thin chain of depth samples. A
    true two-carton merge has two dense horizontal lobes with a sparse valley;
    a single wide carton generally does not. The conservative tests below make
    this a no-op unless that structure is clearly present.
    """

    if not params.split_merged_clusters:
        return (member_indices,)
    member_points = points[member_indices]
    if float(np.ptp(member_points[:, 0])) < params.merged_cluster_min_width:
        return (member_indices,)

    member_x = pixel_x[member_indices]
    min_x = int(member_x.min())
    max_x = int(member_x.max())
    width = max_x - min_x + 1
    child_width = params.split_min_child_width_pixels
    if width < 2 * child_width + 1:
        return (member_indices,)

    column_counts = np.bincount(member_x - min_x, minlength=width).astype(np.float64)
    smoothed = np.convolve(column_counts, np.ones(3) / 3.0, mode="same")
    cumulative = np.cumsum(column_counts)
    candidate_columns = np.arange(child_width, width - child_width)
    enough_pixels = (
        (cumulative[candidate_columns] >= params.split_min_child_pixels)
        & (
            cumulative[-1] - cumulative[candidate_columns]
            >= params.split_min_child_pixels
        )
    )
    candidate_columns = candidate_columns[enough_pixels]
    if not len(candidate_columns):
        return (member_indices,)

    valley_index = int(candidate_columns[np.argmin(smoothed[candidate_columns])])
    left_peak = float(smoothed[:valley_index].max(initial=0.0))
    right_peak = float(smoothed[valley_index + 1 :].max(initial=0.0))
    reference_peak = min(left_peak, right_peak)
    if (
        reference_peak <= 0
        or float(smoothed[valley_index])
        > params.split_valley_ratio * reference_peak
    ):
        return (member_indices,)

    cut_x = min_x + valley_index
    left = member_indices[member_x <= cut_x]
    right = member_indices[member_x > cut_x]
    if (
        len(left) < params.split_min_child_pixels
        or len(right) < params.split_min_child_pixels
    ):
        return (member_indices,)
    return left, right


def _voxelize(points, size):
    """Return voxel centroids and one voxel index per input point."""
    if not len(points):
        return np.empty((0, 3), np.float64), np.empty(0, np.intp)
    origin = points.min(axis=0) - size*0.5
    cells = np.floor((points-origin)/size).astype(np.int64)
    dimensions = cells.max(axis=0)+1
    keys = (cells[:, 0]*dimensions[1]+cells[:, 1])*dimensions[2]+cells[:, 2]
    _, owners = np.unique(keys, return_inverse=True)
    counts = np.bincount(owners)
    centroids = np.column_stack([
        np.bincount(owners, weights=points[:, axis])/counts for axis in range(3)
    ])
    return centroids, owners


def _segment_half_resolution(depth, intrinsics, params, plane_model=None):
    """Reduce depth only; restore native pixel coordinates and original padding."""
    started = perf_counter()
    params.validate()
    height, width = depth.shape
    if height % 2 or width % 2:
        raise ValueError('Half-resolution processing requires even image dimensions')
    # Decimate measured pixels (1,3,5,...); use their exact rays, rather than
    # resize-center rays that lie half a native pixel away from each sample.
    reduced = depth[1::2, 1::2]
    K = intrinsics.K.copy()
    K[:2] /= 2
    K[:2, 2] -= .5
    reduced_camera = Camera(width//2, height//2, K, intrinsics.D, intrinsics.distortion_model)
    coarse_params = replace(params,
        plane_min_inliers=max(3, round(params.plane_min_inliers/4)),
        min_object_pixels=max(1, round(params.min_object_pixels/4)),
        split_min_child_pixels=max(1, round(params.split_min_child_pixels/4)),
        split_min_child_width_pixels=max(1, round(params.split_min_child_width_pixels/2)),
        border_margin_pixels=-1 if params.border_margin_pixels < 0 else round(params.border_margin_pixels/2),
        bottom_padding_pixels=0,
    )
    result = segment_tabletop_objects(reduced, reduced_camera, coarse_params, plane_model=plane_model)
    restore_started = perf_counter()
    labels = cv2.resize(result.object_labels, (width, height), interpolation=cv2.INTER_NEAREST)
    table = cv2.resize(result.table_mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)
    valid = np.isfinite(depth) & (depth >= params.min_depth) & (depth <= params.max_depth)
    labels[~valid], table[~valid] = -1, False
    counts = np.bincount(labels[labels >= 0], minlength=len(result.objects))
    scale = np.array([width/reduced.shape[1], height/reduced.shape[0]])
    objects = []
    for obj in result.objects:
        left, top = np.floor(obj.contour_depth_pixels[0]*scale).astype(int)
        right, bottom = np.ceil((obj.contour_depth_pixels[2]+1)*scale).astype(int)-1
        bottom = min(height-1, bottom+params.bottom_padding_pixels)
        contour = np.int32([[left, top], [min(width-1, right), top],
                            [min(width-1, right), bottom], [left, bottom]])
        center = np.asarray(obj.centroid_depth_pixel)*scale+1
        objects.append(replace(obj, pixel_count=int(counts[obj.label]),
                               centroid_depth_pixel=tuple(map(float, center)),
                               contour_depth_pixels=contour))
    timings = result.timings_ms.copy()
    timings["restore_native"] = (perf_counter()-restore_started)*1000
    timings["total"] = (perf_counter()-started)*1000
    return replace(result, objects=tuple(objects), object_labels=labels, table_mask=table,
                   valid_pixel_count=int(valid.sum()), bottom_padding_pixels=params.bottom_padding_pixels,
                   processing_scale=.5, timings_ms=timings)


def segment_tabletop_objects(
    depth_meters: np.ndarray,
    intrinsics: Camera,
    parameters: TabletopSegmentationParameters | None = None,
    *, processing_scale: float = 1.0, plane_model=None,
) -> TabletopSegmentationResult:
    """Separate a dominant support plane and cluster objects above it.

    The table normal is oriented toward the camera origin. Consequently,
    positive signed distances point from the table toward the camera, which is
    the expected side for objects resting on a visible tabletop.
    Plane fitting uses a sparse grid; output masks and object measurements
    retain native depth resolution. An optional validated depth-camera plane
    bypasses plane fitting; the caller owns temporal tracking and validation.
    """

    params = parameters or TabletopSegmentationParameters()
    params.validate()

    depth = np.asarray(depth_meters, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError(f"depth_meters must be a 2-D array, got {depth.shape}")
    if processing_scale not in (0.5, 1.0):
        raise ValueError("processing_scale must be 0.5 or 1.0")
    if processing_scale == 0.5:
        return _segment_half_resolution(depth, intrinsics, params, plane_model=plane_model)

    total_started = perf_counter()
    timings: dict[str, float] = {}

    stage_started = perf_counter()
    valid_mask = (
        np.isfinite(depth)
        & (depth >= params.min_depth)
        & (depth <= params.max_depth)
    )
    flat_pixel_indices = np.flatnonzero(valid_mask.ravel())
    if len(flat_pixel_indices) < 3:
        raise ValueError("fewer than three depth pixels survive the depth limits")

    height, width = depth.shape
    pixel_y, pixel_x = np.unravel_index(flat_pixel_indices, depth.shape)
    z = depth.ravel()[flat_pixel_indices]
    fx, fy, cx, cy = _scaled_intrinsics(intrinsics, depth.shape)
    points = np.column_stack(
        (
            (pixel_x.astype(np.float32) - cx) * z / fx,
            (pixel_y.astype(np.float32) - cy) * z / fy,
            z,
        )
    ).astype(np.float32, copy=False)
    timings["unproject"] = (perf_counter() - stage_started) * 1000

    stage_started = perf_counter()
    # Fit geometry on a sparse image grid, retaining all original pixels for
    # object masks and measurements. The wall above the ROI is never voxelized.
    if plane_model is not None:
        plane = np.asarray(plane_model, dtype=float).copy()
        if plane.shape != (4,) or not np.isfinite(plane).all() or np.linalg.norm(plane[:3]) == 0:
            raise ValueError('Require a finite plane with nonzero normal')
        plane /= np.linalg.norm(plane[:3])
        if plane[3] < 0:
            plane = -plane
        down_points = np.empty((0, 3))
        table_voxel_count = 0  # No plane-fitting cloud was built.
        timings['downsample'] = timings['plane_ransac'] = 0.
    else:
        plane_samples = (
            (pixel_y >= params.plane_roi_top * height)
            & (pixel_x % params.plane_stride == 0)
            & (pixel_y % params.plane_stride == 0)
        )
        plane_cloud = o3d.geometry.PointCloud()
        plane_cloud.points = o3d.utility.Vector3dVector(points[plane_samples].astype(np.float64))
        down_cloud = plane_cloud.voxel_down_sample(params.voxel_size)
        down_points = np.asarray(down_cloud.points)
        if len(down_points) < 3:
            raise ValueError("fewer than three points remain after voxel downsampling")
        timings["downsample"] = (perf_counter() - stage_started) * 1000

        stage_started = perf_counter()
        plane_fit_indices = np.arange(len(down_points))
        if len(plane_fit_indices) < 3:
            raise ValueError("fewer than three points lie inside the plane-fitting ROI")
        sample_fraction = min(1.0, params.plane_max_points / len(plane_fit_indices))
        if len(plane_fit_indices) > params.plane_max_points:
            rng = np.random.default_rng(params.random_seed)
            plane_fit_indices = rng.choice(plane_fit_indices, params.plane_max_points, replace=False)
        plane_fit_cloud = down_cloud.select_by_index(plane_fit_indices.tolist())

        # A scene can contain several parallel horizontal surfaces. In particular,
        # some recordings show both the medicine table and a large structure at
        # the bottom of the image. Peel off several RANSAC planes, keep substantial
        # planes whose normal points mostly along the rotated image's vertical
        # axis, and choose the nearest credible one rather than blindly choosing
        # the globally largest plane.
        candidates: list[tuple[np.ndarray, int]] = []
        remaining_plane_cloud = plane_fit_cloud
        for candidate_index in range(params.plane_candidate_count):
            if len(remaining_plane_cloud.points) < 3:
                break
            o3d.utility.random.seed(params.random_seed + candidate_index)
            raw_plane, inliers = remaining_plane_cloud.segment_plane(
                distance_threshold=params.plane_distance_threshold,
                ransac_n=3,
                num_iterations=params.ransac_iterations,
            )
            if len(inliers) < 3:
                break
            candidate = np.asarray(raw_plane, dtype=np.float64)
            normal_length = float(np.linalg.norm(candidate[:3]))
            if np.isfinite(normal_length) and normal_length > 0:
                candidate /= normal_length
                # The camera is at the origin. Positive d orients the normal from
                # the support surface toward the camera.
                if candidate[3] < 0:
                    candidate = -candidate
                if (
                    len(inliers) >= params.plane_min_inliers * sample_fraction
                    and abs(float(candidate[1])) >= params.support_normal_y_min
                ):
                    candidates.append((candidate, round(len(inliers) / sample_fraction)))
            remaining_plane_cloud = remaining_plane_cloud.select_by_index(
                inliers, invert=True
            )

        if not candidates:
            raise RuntimeError("RANSAC found no credible support-plane candidate")
        largest_candidate = max(inlier_count for _, inlier_count in candidates)
        substantial_candidates = [
            item
            for item in candidates
            if item[1] >= largest_candidate * params.plane_min_inlier_fraction
        ]
        plane, table_voxel_count = min(
            substantial_candidates,
            key=lambda item: float(item[0][3]),
        )
        timings["plane_ransac"] = (perf_counter() - stage_started) * 1000

    stage_started = perf_counter()
    signed_full_distances = points @ plane[:3] + plane[3]
    candidate_indices = np.flatnonzero(
        (pixel_x >= params.object_roi_left * width)
        & (pixel_x <= params.object_roi_right * width)
        & (pixel_y >= params.object_roi_top * height)
        & (signed_full_distances >= params.object_min_height)
        & (signed_full_distances <= params.object_max_height)
    )
    candidate_cloud = o3d.geometry.PointCloud()
    candidate_points = points[candidate_indices].astype(np.float64)
    voxel_points, owners = _voxelize(candidate_points, params.voxel_size)
    candidate_cloud.points = o3d.utility.Vector3dVector(voxel_points)
    candidate_voxel_count = len(candidate_cloud.points)
    timings["height_filter"] = (perf_counter() - stage_started) * 1000

    stage_started = perf_counter()
    if candidate_voxel_count == 0:
        raw_candidate_labels = np.empty(0, dtype=np.int32)
    else:
        raw_candidate_labels = np.asarray(
            candidate_cloud.cluster_dbscan(
                eps=params.cluster_eps,
                min_points=params.cluster_min_points,
                print_progress=False,
            ),
            dtype=np.int32,
        )
    timings["dbscan"] = (perf_counter() - stage_started) * 1000

    stage_started = perf_counter()
    # Direct indexing replaces thousands of per-voxel Python/pybind conversions.
    # Values remain -1 for rejected points and DBSCAN noise.
    point_raw_labels = np.full(len(points), -1, dtype=np.int32)
    point_voxels = np.full(len(points), -1, dtype=np.int32)
    if len(owners):
        labels = raw_candidate_labels[owners]
        keep = labels >= 0
        source_indices = candidate_indices[keep]
        point_raw_labels[source_indices] = labels[keep]
        point_voxels[source_indices] = owners[keep]

    accepted: list[tuple[np.ndarray, float, int, float]] = []
    rejections = {}
    def reject(reason):
        rejections[reason] = rejections.get(reason, 0)+1
    clustered_indices = np.flatnonzero(point_raw_labels >= 0)
    ordered = clustered_indices[np.argsort(point_raw_labels[clustered_indices])]
    boundaries = np.flatnonzero(np.diff(point_raw_labels[ordered])) + 1
    for member_indices in np.split(ordered, boundaries):
        if not len(member_indices):
            continue
        if len(member_indices) < params.min_object_pixels:
            reject('too_few_pixels')
            continue
        if float(signed_full_distances[member_indices].max()) < params.min_object_peak_height:
            reject('too_low')
            continue
        if params.border_margin_pixels >= 0:
            margin = params.border_margin_pixels
            member_x = pixel_x[member_indices]
            member_y = pixel_y[member_indices]
            if (
                int(member_x.min()) <= margin
                or int(member_x.max()) >= width - 1 - margin
                or int(member_y.min()) <= margin
                or int(member_y.max()) >= height - 1 - margin
            ):
                reject('image_border')
                continue
        for part_indices in _split_at_horizontal_valley(
            member_indices, pixel_x, points, params
        ):
            if float(points[part_indices, 2].mean()) > params.max_object_centroid_depth:
                reject('too_far')
                continue
            # Filter measured camera-aligned bounds after splitting, so a merged
            # pair is not discarded before it can become two valid objects.
            extent = float(np.ptp(points[part_indices], axis=0).max())
            if not params.min_object_extent <= extent <= params.max_object_extent:
                reject('extent')
                continue
            footprint = footprint_diameter(points[part_indices], plane[:3])
            if params.max_object_footprint is not None and footprint > params.max_object_footprint:
                reject('footprint')
                continue
            part_voxel_count = len(np.unique(point_voxels[part_indices]))
            mean_pixel_x = float(pixel_x[part_indices].mean())
            accepted.append((part_indices, mean_pixel_x, part_voxel_count, footprint))
    accepted.sort(key=lambda item: item[1])

    point_labels = np.full(len(points), -1, dtype=np.int32)
    objects: list[DetectedObject] = []
    for label, (member_indices, _, part_voxel_count, footprint) in enumerate(accepted):
        point_labels[member_indices] = label
        member_points = points[member_indices]
        min_xyz = member_points.min(axis=0)
        max_xyz = member_points.max(axis=0)
        objects.append(
            DetectedObject(
                label=label,
                pixel_count=len(member_indices),
                voxel_count=part_voxel_count,
                centroid_xyz=_as_float_tuple(member_points.mean(axis=0)),
                min_bound_xyz=_as_float_tuple(min_xyz),
                max_bound_xyz=_as_float_tuple(max_xyz),
                extent_xyz=_as_float_tuple(max_xyz - min_xyz),
                max_height_above_plane=float(
                    signed_full_distances[member_indices].max()
                ),
                centroid_depth_pixel=(
                    float(pixel_x[member_indices].mean()),
                    float(pixel_y[member_indices].mean()),
                ),
                contour_depth_pixels=bounding_contour(np.column_stack(
                    (pixel_x[member_indices], pixel_y[member_indices])
                ), depth.shape, params.bottom_padding_pixels),
                footprint_diameter=footprint,
            )
        )

    point_table_mask = np.abs(signed_full_distances) <= params.plane_distance_threshold
    table_mask = np.zeros(depth.size, dtype=bool)
    table_mask[flat_pixel_indices] = point_table_mask
    table_mask = table_mask.reshape(depth.shape)

    object_labels = np.full(depth.size, -1, dtype=np.int32)
    object_labels[flat_pixel_indices] = point_labels
    object_labels = object_labels.reshape(depth.shape)
    timings["label_mapping"] = (perf_counter() - stage_started) * 1000
    timings["total"] = (perf_counter() - total_started) * 1000

    return TabletopSegmentationResult(
        plane_model=tuple(float(value) for value in plane),  # type: ignore[arg-type]
        table_mask=table_mask,
        object_labels=object_labels,
        objects=tuple(objects),
        valid_pixel_count=len(points),
        downsampled_point_count=len(down_points),
        table_voxel_count=table_voxel_count,
        candidate_voxel_count=candidate_voxel_count,
        noise_voxel_count=int(np.count_nonzero(raw_candidate_labels < 0)),
        timings_ms=timings,
        bottom_padding_pixels=params.bottom_padding_pixels,
        rejections=rejections,
    )
