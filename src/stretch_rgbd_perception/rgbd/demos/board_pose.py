"""DB3 board_pose demo."""
import cv2
from rgbd.pose.board import Board, BoardPoseEstimator, Parameters

def main():
    from rgbd.sources.db3 import RealSenseReader
    from rgbd.demos.common import demo_parser, run_demo

    parser = demo_parser(__doc__)
    parser.add_argument("--board-scale", type=float, default=.96)
    parser.add_argument("--min-markers", type=int, default=2)
    parser.add_argument("--max-reprojection-error", type=float, default=3.)
    args = parser.parse_args()
    with RealSenseReader(args.bag, rotations=args.rotations) as source:
        estimator = BoardPoseEstimator(source.calibration.color, Board(scale=args.board_scale),
                                       Parameters(args.min_markers, args.max_reprojection_error))
        def process(frame, index):
            pose = estimator.estimate(frame)
            observation = estimator.observation
            position = None if pose is None else pose.camera_to_reference[:3, 3].tolist()
            image = estimator.annotate(frame.color, pose)
            lines = [f"{observation.status} | {len(observation.ids)} markers | {observation.processing_ms:.1f} ms"]
            if observation.reprojection_error is not None:
                lines.append(f"RMS {observation.reprojection_error:.2f} px")
            if position is not None:
                lines.append("Camera XYZ (m): "+", ".join(f"{v:+.3f}" for v in position))
            for row, line in enumerate(lines):
                cv2.putText(image, line, (8, 24+24*row), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(image, line, (8, 24+24*row), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1, cv2.LINE_AA)
            return image, dict(status=observation.status, markers=observation.ids.ravel().tolist(),
                              rms_px=observation.reprojection_error, pose_ms=observation.processing_ms,
                              camera_xyz=position, reference_to_camera=None if pose is None else pose.reference_to_camera.tolist())

        run_demo(source, process, args, title="Board pose")


if __name__ == "__main__":
    main()
