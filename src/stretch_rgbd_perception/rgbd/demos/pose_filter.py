"""DB3 pose_filter demo."""
import cv2
import numpy as np
from time import perf_counter
from rgbd.pose.filter import PoseFilter, Parameters

def main():
    import cv2
    from rgbd.pose.board import Board, BoardPoseEstimator
    from rgbd.sources.db3 import RealSenseReader
    from rgbd.demos.common import demo_parser, run_demo

    parser = demo_parser(__doc__)
    parser.add_argument("--window", type=int, default=3)
    parser.add_argument("--max-gap-seconds", type=float, default=.5)
    parser.add_argument("--board-scale", type=float, default=.96)
    args = parser.parse_args()
    with RealSenseReader(args.bag, rotations=args.rotations) as source:
        estimator = BoardPoseEstimator(source.calibration.color, Board(scale=args.board_scale))
        smoother = PoseFilter(Parameters(args.window, args.max_gap_seconds))
        previous_index = None

        def process(frame, index):
            nonlocal previous_index
            if previous_index is not None and abs(index-previous_index) != 1:
                smoother.reset()
            previous_index = index
            raw = estimator.estimate(frame)
            started = perf_counter()
            filtered = smoother.update(raw)
            filter_ms = (perf_counter()-started)*1000
            panes = []
            for label, pose in (("Raw", raw), ("Filtered", filtered)):
                pane = estimator.annotate(frame.color, pose)
                lines = [label, estimator.observation.status]
                if pose is not None:
                    lines.append("XYZ (m): "+", ".join(f"{v:+.3f}" for v in pose.camera_to_reference[:3, 3]))
                for row, line in enumerate(lines):
                    cv2.putText(pane, line, (8, 24+row*24), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(pane, line, (8, 24+row*24), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
                panes.append(pane)
            return np.hstack(panes), dict(status=estimator.observation.status, filter_ms=filter_ms,
                samples=len(smoother.history), raw_camera_to_reference=None if raw is None else raw.camera_to_reference.tolist(),
                filtered_camera_to_reference=None if filtered is None else filtered.camera_to_reference.tolist())

        run_demo(source, process, args, title="Raw | filtered camera pose")


if __name__ == "__main__":
    main()
