"""DB3 RGB/depth viewer."""
import numpy as np
from rgbd.sources.db3 import RealSenseReader

def main():
    import cv2
    from rgbd.demos.common import demo_parser, resize, run_demo

    args = demo_parser(__doc__).parse_args()
    with RealSenseReader(args.bag, rotations=args.rotations) as source:
        def show(frame, index):
            valid = np.isfinite(frame.depth) & (frame.depth > 0)
            values = np.where(valid, frame.depth, 0)
            depth = cv2.applyColorMap(np.uint8(np.clip((values-.25)/1.75, 0, 1)*255), cv2.COLORMAP_TURBO)
            depth[~valid] = 0
            color = resize(cv2.cvtColor(frame.color, cv2.COLOR_RGB2BGR), args.max_dimension)
            depth = cv2.resize(depth, (round(depth.shape[1]*color.shape[0]/depth.shape[0]), color.shape[0]))
            return np.hstack((color, depth)), dict(valid_depth_pixels=int(valid.sum()))

        run_demo(source, show, args, title="RGB | depth (0.25-2 m)")


if __name__ == "__main__":
    main()
