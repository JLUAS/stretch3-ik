"""Native live-camera test, with bounded capture and optional background OCR."""

import argparse
from collections import deque
from contextlib import ExitStack
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import signal
from time import perf_counter

import cv2
import numpy as np

from rgbd.pose.board import BoardPoseEstimator
from rgbd.pose.filter import PoseFilter
from rgbd.detection.pipeline import DetectionPipeline
from rgbd.detection.tabletop import TabletopSegmenter, Parameters
from rgbd.demos.common import resize
from rgbd.demos.detection import draw


def main():
    from rgbd.sources.realsense import RealSenseLive
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true')
    parser.add_argument('--serial')
    parser.add_argument('--max-dimension', type=int, default=600)
    parser.add_argument('--features', type=int, default=120)
    parser.add_argument('--scene-depth', type=float, default=1.1)
    parser.add_argument('--plane-min-inliers', type=int, default=300)
    parser.add_argument('--no-ocr', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--frames', type=int, default=0, help='0 runs until stopped')
    parser.add_argument('--output', type=Path, default=Path('outputs/live')/datetime.now().strftime('%Y%m%d_%H%M%S'))
    args = parser.parse_args()
    if args.max_dimension <= 0 or args.features <= 0 or args.frames < 0:
        parser.error('Require positive dimensions/features and nonnegative frames')
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'config.json').write_text(json.dumps(vars(args), default=str, indent=2))
    cv2.setNumThreads(1)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    for name in ('SIGHUP', 'SIGTERM'):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), interrupt)
    finished, core_times = deque(maxlen=100), deque(maxlen=100)
    window, count, last_print = 'Live medicine detection', 0, 0.
    last_frame = last_image = last_result = None
    print(f'Output: {args.output.resolve()}', flush=True)
    print('Ctrl+C or close this terminal to stop. Esc/Q quits; G saves a snapshot.', flush=True)
    try:
        with ExitStack() as stack:
            source = stack.enter_context(RealSenseLive(serial=args.serial))
            print(f'Camera: {source.name} ({source.serial})', flush=True)
            calibration = source.calibration
            (args.output/'calibration.json').write_text(json.dumps(asdict(calibration), default=lambda v: v.tolist(), indent=2))
            estimator, smoother = BoardPoseEstimator(calibration.color), PoseFilter()
            segmenter = TabletopSegmenter(calibration, Parameters(max_depth=args.scene_depth,
                                           plane_min_inliers=args.plane_min_inliers))
            pipeline = DetectionPipeline(calibration, segmenter=segmenter)
            pipeline.extractor.parameters.features = args.features
            classifier = None
            if not args.no_ocr:
                from rgbd.classification.worker import ObjectClassifier
                classifier = stack.enter_context(ObjectClassifier())
            log = stack.enter_context((args.output/'frames.jsonl').open('a', buffering=1))
            while not args.frames or count < args.frames:
                started = perf_counter()
                frame = source.read()
                captured = perf_counter()
                pose = smoother.update(estimator.estimate(frame))
                result = pipeline.process(frame, pose)
                processed = perf_counter()
                event, labels, state = None, None, 'off'
                if classifier is not None:
                    event = classifier.poll(result.tracks)
                    pipeline.persistor.freeze_templates(classifier.memory.conflicts)
                    classifier.submit(frame, result.tracks, result.observations, pipeline.persistor.elapsed_seconds)
                    labels, state = classifier.memory.labels, classifier.state
                count += 1
                core_times.append((processed-captured)*1000)
                fps = 0. if len(finished) < 2 else (len(finished)-1)/(finished[-1]-finished[0])
                visible = [t.id for t in result.tracks if t.visible and t.confirmed]
                status = f'{fps:.1f} FPS | pose {"OK" if pose else "LOST"} | table {result.segmentation.plane_status} | OCR {state}'
                image = draw(frame, result.observations, tracks=result.tracks, labels=labels)
                image = resize(image, args.max_dimension)
                cv2.putText(image, status, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(image, status, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 255, 255), 1, cv2.LINE_AA)
                key = -1
                if not args.headless:
                    cv2.imshow(window, image)
                    key = cv2.waitKey(1) & 255
                now = perf_counter()
                finished.append(now)
                row = dict(frame=count, timestamp_ns=frame.timestamp_ns, fps=fps, read_ms=(captured-started)*1000,
                    core_ms=(processed-captured)*1000, loop_ms=(now-started)*1000, timings_ms=result.timings_ms,
                    skipped=source.skipped, pair_delta_ms=source.pair_delta_ms, pose_ok=pose is not None,
                    plane_status=result.segmentation.plane_status, error=result.segmentation.error,
                    visible_ids=visible, objects=len(result.segmentation.detections), events=list(result.events),
                    tracks=[dict(id=t.id, index=t.detection_index, confirmed=t.confirmed,
                        position=t.position.tolist(), missing_seconds=t.missing_seconds,
                        association=t.association, templates=len(t.templates)) for t in result.tracks],
                    labels={} if labels is None else {str(i): asdict(v) for i, v in labels.items()},
                    ocr_state=state, ocr_event=event, ocr_error='' if classifier is None else classifier.error)
                log.write(json.dumps(row)+'\n')
                if now-last_print >= 1:
                    print(f'Frame {count}: {status} | core {np.median(core_times):.1f} ms median / '
                          f'{np.percentile(core_times,95):.1f} p95 | IDs {visible}', flush=True)
                    if event is not None:
                        print(event, flush=True)
                    last_print = now
                last_frame, last_image, last_result = frame, image, result
                if key in (27, ord('q')):
                    break
                if key == ord('g'):
                    folder = args.output/f'snapshot_{count:05d}'
                    folder.mkdir(exist_ok=True)
                    cv2.imwrite(str(folder/'rgb.png'), cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR))
                    cv2.imwrite(str(folder/'annotations.png'), image)
                    np.save(folder/'depth.npy', frame.depth)
                    (folder/'report.json').write_text(json.dumps(row, indent=2))
                    np.savez_compressed(folder/'masks.npz', labels=result.segmentation.object_labels,
                                        table=result.segmentation.table_mask)
                    print(f'Saved {folder}', flush=True)
                if not args.headless and cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
    except KeyboardInterrupt:
        print('Stopping and releasing the camera...', flush=True)
    finally:
        if last_image is not None:
            cv2.imwrite(str(args.output/'last_frame.png'), last_image)
        cv2.destroyAllWindows()
        print(f'Stopped after {count} frames. Logs: {args.output.resolve()}', flush=True)


if __name__ == '__main__':
    main()
