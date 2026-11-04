"""Shared DB3 harness for appearance, persistence, and pipeline component demos."""

from dataclasses import asdict, replace
from contextlib import ExitStack
import json
from time import perf_counter
from time import sleep

import cv2
import numpy as np

from rgbd.detection.appearance import RGBObjectAppearance, compare
from rgbd.pose.board import BoardPoseEstimator
from rgbd.pose.filter import PoseFilter
from rgbd.sources.db3 import RealSenseReader
from rgbd.demos.common import demo_parser, run_demo
from rgbd.detection.tabletop import TabletopSegmenter


def draw(frame, observations, *, tracks=(), labels=None, features=False, selected=None):
    image = cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR)
    owners = {t.detection_index: t for t in tracks if t.detection_index is not None}
    for i, observation in enumerate(observations):
        if observation.box is None:
            continue
        x0, y0, x1, y1 = observation.box
        track = owners.get(i)
        color = (80, 220, 80) if track is None or track.confirmed else (0, 180, 255)
        cv2.rectangle(image, (x0, y0), (x1-1, y1-1), color, 2)
        signature = observation.signature
        count = 0 if signature is None else len(signature.points)
        text = f'{i}: {count} features' if track is None else f'ID {track.id}'+(' ?' if not track.confirmed else '')
        if labels and track is not None and track.id in labels:
            text += ' '+labels[track.id].label
        text_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)[0][0]
        text_x = max(0, min(x0, image.shape[1]-text_width))
        cv2.putText(image, text, (text_x, max(15, y0-6)), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
        if features and signature is not None and (selected is None or selected == i):
            points = (signature.points+.5)*np.array([x1-x0, y1-y0])/np.array(signature.shape[::-1])-.5+[x0, y0]
            for point in np.rint(points).astype(int):
                cv2.circle(image, tuple(point), 2, (255, 80, 255), -1)
    return image


def main(stage='appearance'):
    parser = demo_parser(__doc__)
    if stage == 'full':
        parser.add_argument('--live', action='store_true', help='run the native live RealSense demo')
        parser.add_argument('--no-ocr', action='store_true', help='run detection without classification')
    parser.add_argument('--object', type=int, help='highlight features/compare this current detection index')
    parser.add_argument('--reference-object', type=int, help='compare against this detection on the initial frame')
    parser.add_argument('--features', type=int, default=180)
    parser.add_argument('--probation', type=float, help='capture seconds before confirming a track')
    parser.add_argument('--wait-seconds', type=float, default=60., help='headless OCR drain timeout')
    args = parser.parse_args()
    if stage == 'full':
        stage = 'detection' if args.no_ocr else 'classification'
    if args.probation is None:
        args.probation = 0. if stage == 'classification' else 1.
    cv2.setNumThreads(1)
    with ExitStack() as stack:
        source = stack.enter_context(RealSenseReader(args.bag, rotations=args.rotations))
        estimator, smoother = BoardPoseEstimator(source.calibration.color), PoseFilter()
        segmenter, extractor = TabletopSegmenter(source.calibration), RGBObjectAppearance(source.calibration)
        extractor.parameters.features = args.features
        persistor = None
        if stage == 'memory':
            from rgbd.detection.persistence import ObjectPersistor, Parameters
            persistor = ObjectPersistor(source.calibration, Parameters(probation_period=args.probation))
        pipeline = None
        if stage in ('detection', 'classification'):
            from rgbd.detection.pipeline import DetectionPipeline
            pipeline = DetectionPipeline(source.calibration, segmenter=segmenter, extractor=extractor, allow_reverse=True)
            pipeline.persistor.parameters.probation_period = args.probation
        previous_index, reference = None, None
        previous_time = None
        classifier = None
        last_frame = last_result = last_event = None
        if stage == 'classification':
            from rgbd.classification.worker import ObjectClassifier
            classifier = stack.enter_context(ObjectClassifier())
        reader = matcher = None
        if stage in ('ocr', 'products'):
            from rgbd.classification.ocr import OCRReader
            reader = OCRReader()
            if stage == 'products':
                from rgbd.classification.matcher import ProductMatcher
                matcher = ProductMatcher()

        def classification_step():
            nonlocal last_event
            if classifier is None or last_result is None:
                return None
            event = classifier.poll(last_result.tracks)
            pipeline.persistor.freeze_templates(classifier.memory.conflicts)
            submitted = classifier.submit(last_frame, last_result.tracks, last_result.observations,
                                          pipeline.persistor.elapsed_seconds)
            if event is not None:
                last_event = event
                if event['kind'] == 'ocr_complete' and args.output:
                    job = classifier.last_result
                    folder = args.output/f'ocr_{job["timestamp_ns"]}'
                    folder.mkdir(parents=True, exist_ok=True)
                    (folder/'readings.json').write_text(json.dumps(dict(event=event,
                        seeds=[dict(id=s['id'], box=s['box'], anchor=s['anchor'].tolist()) for s in job['seeds']],
                        readings=[asdict(r) for r in job['readings']], products=[asdict(o) for o in job['observations']]), indent=2))
                    for seed in job['seeds']:
                        x0, y0, x1, y1 = seed['box']
                        cv2.imwrite(str(folder/f'id_{seed["id"]}.png'), cv2.cvtColor(job['rgb'][y0:y1, x0:x1], cv2.COLOR_RGB2BGR))
            if event is not None or submitted:
                return classification_view()
            return None

        def classification_view():
            image = draw(last_frame, last_result.observations, tracks=last_result.tracks, labels=classifier.memory.labels)
            cv2.putText(image, f'OCR {classifier.state}', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
            return image, dict(ocr_state=classifier.state, ocr_error=classifier.error,
                classifications={str(i): asdict(value) for i, value in classifier.memory.labels.items()},
                ocr_event=last_event)

        def process(frame, index):
            nonlocal previous_index, reference, previous_time, last_frame, last_result
            if previous_index is not None and abs(index-previous_index) != 1:
                segmenter.reset()
                smoother.reset()
            previous_index = index
            pose_started = perf_counter()
            pose = smoother.update(estimator.estimate(frame))
            pose_ms = (perf_counter()-pose_started)*1000
            dt = 0 if previous_time is None else abs(frame.timestamp_ns-previous_time)/1e9
            previous_time = frame.timestamp_ns
            timings = {}
            if pipeline is None:
                segmentation = segmenter.segment(frame.depth, pose)
                started = perf_counter()
                observations = extractor.observe(frame, segmentation.detections)
                appearance_ms = (perf_counter()-started)*1000
                tracks = () if persistor is None else persistor.update(segmentation.detections, pose, dt, observations)
                events = () if persistor is None else persistor.events
            else:
                result = pipeline.process(frame, pose)
                last_frame, last_result = frame, result
                segmentation, observations, tracks = result.segmentation, result.observations, result.tracks
                appearance_ms, timings, events = result.timings_ms['appearance'], result.timings_ms, result.events
            if reference is None:
                reference = observations
            comparisons = []
            for i, current in enumerate(observations if stage == 'appearance' else ()):
                if current.signature is None or args.object not in (None, i):
                    continue
                for j, initial in enumerate(reference):
                    if initial.signature is not None and args.reference_object in (None, j):
                        comparisons.append(dict(current=i, reference=j, **asdict(compare(initial.signature, current.signature))))
            readings, products = (), []
            if reader is not None:
                from rgbd.geometry import padded_box
                regions = [(i, padded_box(o.box, frame.color.shape)) for i, o in enumerate(observations)
                           if o.box is not None and args.object in (None, i)]
                readings = tuple(replace(r, region=regions[r.region][0]) for r in
                                 reader.read(frame.color, [box for _, box in regions]))
                if matcher is not None:
                    products = [asdict(matcher.identify([r for r in readings if r.region == i])) for i in range(len(observations))]
            image = draw(frame, observations, tracks=tracks, features=stage == 'appearance', selected=args.object)
            for i, observation in enumerate(observations):
                if reader is None or observation.box is None:
                    continue
                texts = sorted((r for r in readings if r.region == i), key=lambda r: r.confidence, reverse=True)
                text = texts[0].text if texts else 'unknown'
                if matcher is not None:
                    product = matcher.identify(texts).best
                    text = 'unknown' if product is None else product.label
                cv2.putText(image, text, (observation.box[0], observation.box[3]+18),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
            info = dict(
                status=segmentation.plane_status, objects=len(observations), appearance_ms=appearance_ms, pose_ms=pose_ms,
                features=[0 if o.signature is None else len(o.signature.points) for o in observations],
                comparisons=comparisons,
                tracks=[dict(id=t.id, index=t.detection_index, confirmed=t.confirmed, missing=t.missing_seconds,
                             position=t.position.tolist(), association=t.association, recoveries=t.recoveries,
                             templates=len(t.templates), gallery_frozen=t.gallery_frozen) for t in tracks],
                events=list(events), timings_ms=timings, readings=[asdict(r) for r in readings], products=products)
            if classifier is not None:
                classification_step()
                image, extra = classification_view()
                info.update(extra)
            return image, info

        rows = run_demo(source, process, args, title='RGB appearance features' if stage == 'appearance' else stage.capitalize(),
                        refresh=classification_step if classifier is not None else None)
        if classifier is not None and args.headless:
            deadline = perf_counter()+args.wait_seconds
            while perf_counter() < deadline:
                classification_step()
                eligible = any(t.confirmed and t.visible for t in last_result.tracks)
                if classifier.state == 'failed':
                    raise RuntimeError(classifier.error)
                if classifier.future is None and (classifier.completed_jobs or not eligible):
                    break
                sleep(.02)
            else:
                raise RuntimeError('OCR worker did not finish before --wait-seconds')
            image, extra = classification_view()
            rows[-1].update(extra)
            if args.output:
                (args.output/'measurements.json').write_text(json.dumps(rows, indent=2))
                cv2.imwrite(str(args.output/'last_frame.png'), image)


if __name__ == '__main__':
    main()
