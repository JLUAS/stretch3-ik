"""DB3 tabletop demo."""
import cv2
import numpy as np
from rgbd.detection.tabletop import TabletopSegmenter, Parameters

def main():
    import json
    from dataclasses import asdict
    import cv2
    from rgbd.pose.board import BoardPoseEstimator
    from rgbd.pose.filter import PoseFilter
    from rgbd.sources.db3 import RealSenseReader
    from rgbd.demos.common import demo_parser, resize, run_demo

    parser = demo_parser(__doc__)
    parser.add_argument('--no-pose', action='store_true', help='stateless depth-only baseline')
    parser.add_argument('--processing-scale', type=float, choices=(.5, 1.), default=.5)
    parser.add_argument('--max-footprint', type=float, default=.2, help='meters; 0 disables')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    with RealSenseReader(args.bag, rotations=args.rotations) as source:
        estimator, smoother = BoardPoseEstimator(source.calibration.color), PoseFilter()
        segmenter = TabletopSegmenter(source.calibration, Parameters(max_object_footprint=args.max_footprint or None),
                                     processing_scale=args.processing_scale)
        if args.output:
            args.output.mkdir(parents=True, exist_ok=True)
            config = dict(bag=str(args.bag.resolve()), rotations=args.rotations, no_pose=args.no_pose,
                          processing_scale=segmenter.processing_scale, parameters=asdict(segmenter.parameters),
                          plane_parameters=asdict(segmenter.plane_parameters), calibration=asdict(source.calibration))
            (args.output/'config.json').write_text(json.dumps(config, default=lambda value: value.tolist(), indent=2))
        previous_index = None
        last_result = None
        colors = np.array([[60, 220, 60], [220, 100, 50], [60, 180, 240], [200, 80, 200],
                           [220, 210, 50], [120, 80, 240]], np.uint8)

        def process(frame, index):
            nonlocal previous_index, last_result
            if previous_index is not None and abs(index-previous_index) != 1:
                segmenter.reset()
                smoother.reset()
            previous_index = index
            pose = None if args.no_pose else smoother.update(estimator.estimate(frame))
            result = segmenter.segment(frame.depth, pose)
            last_result = result
            depth = cv2.applyColorMap(np.uint8(np.clip(np.nan_to_num(frame.depth)/2, 0, 1)*255), cv2.COLORMAP_BONE)
            depth[~np.isfinite(frame.depth) | (frame.depth <= 0)] = 0
            depth[result.table_mask] = depth[result.table_mask]//2 + [30, 20, 0]
            rgb = cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)
            for i, detection in enumerate(result.detections):
                color = colors[i % len(colors)]
                depth[detection.mask] = depth[detection.mask]//2+color//2
                x0, y0, x1, y1 = detection.box
                cv2.rectangle(depth, (x0, y0), (x1, y1), color.tolist(), 2)
                cv2.putText(depth, str(i), (x0, max(12, y0-4)), cv2.FONT_HERSHEY_SIMPLEX, .45, color.tolist(), 1)
                # Project measured object pixels only; no dense RGB-D map is needed.
                y, x = np.nonzero(detection.mask)
                z, k = frame.depth[y, x], source.calibration.depth.K
                points = np.column_stack(((x-k[0, 2])*z/k[0, 0], (y-k[1, 2])*z/k[1, 1], z))
                transform = source.calibration.depth_to_color
                points = points @ transform[:3, :3].T + transform[:3, 3]
                points = points[points[:, 2] > 0]
                if len(points):
                    pixels = cv2.projectPoints(points, np.zeros(3), np.zeros(3),
                                              source.calibration.color.K, source.calibration.color.D)[0].reshape(-1, 2)
                    left, top = np.floor(pixels.min(axis=0)).astype(int)
                    right, bottom = np.ceil(pixels.max(axis=0)).astype(int)
                    cv2.rectangle(rgb, (left, top), (right, bottom), color.tolist(), 2)
                    cv2.putText(rgb, str(i), (left, top-4), cv2.FONT_HERSHEY_SIMPLEX, .6, color.tolist(), 2)
            depth, rgb = resize(depth, 600), resize(rgb, 600)
            image = np.hstack((depth, rgb))
            lines = [f'{len(result.detections)} objects | plane {result.plane_status} | {result.timings_ms["total"]:.1f} ms',
                     'Rejected: '+str(result.rejections), result.error]
            for row, line in enumerate(lines):
                cv2.putText(image, line, (8, 20+22*row), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(image, line, (8, 20+22*row), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1, cv2.LINE_AA)
            return image, dict(status=result.plane_status, objects=len(result.detections),
                plane=None if result.plane is None else result.plane.tolist(), error=result.error,
                reference_plane=None if segmenter.reference_plane is None else segmenter.reference_plane.tolist(),
                rejections=result.rejections, timings_ms=result.timings_ms, noise_voxels=result.noise_voxels,
                support_count=result.support_count, plane_rms_mm=result.plane_rms_mm,
                detections=[dict(box=d.box, centroid=d.centroid.tolist(), extent=d.extent.tolist(),
                                 footprint=d.footprint, height=d.height, pixels=int(d.mask.sum())) for d in result.detections])

        run_demo(source, process, args, title='Depth segmentation | RGB boxes')
        if args.output and last_result is not None:
            np.savez_compressed(args.output/'last_masks.npz', object_labels=last_result.object_labels,
                                table_mask=last_result.table_mask)


if __name__ == "__main__":
    main()
