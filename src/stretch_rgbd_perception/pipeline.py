"""Project entry point: source-independent detection and asynchronous classification.

Run python pipeline.py for the combined DB3 demo, or add --no-ocr for detection.
Import the pipeline classes here; component implementations live under rgbd/.
"""

from rgbd.detection.pipeline import DetectionPipeline
from rgbd.classification.worker import ObjectClassifier

__all__ = ['DetectionPipeline', 'ObjectClassifier']


if __name__ == '__main__':
    import sys
    if '--live' in sys.argv:
        from rgbd.demos.live import main
        main()
    else:
        from rgbd.demos.detection import main
        main('full')
