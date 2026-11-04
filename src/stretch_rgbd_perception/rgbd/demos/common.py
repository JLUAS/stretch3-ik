"""Shared DB3 playback for component main-guard demos; no processing logic."""

import argparse
import os
import json
from pathlib import Path
from time import perf_counter

import cv2



PLAYBACK_HELP = "A reverse | S pause | D play | Q previous | E next | W restart | Esc quit"
DEFAULT_BAG = Path(os.environ.get("RGBD_BAG", "recordings/20261005_195417.db3"))


class Playback:
    def __init__(self, source, start_frame=0):
        if not 0 <= start_frame < len(source):
            raise ValueError(f"start-frame must be between 0 and {len(source)-1}")
        self.count = len(source)
        self.index = self.start_frame = start_frame
        self.interval = 1 / (source.fps or 30)
        self.direction, self.next_frame, self.quit = 0, 0., False

    def update(self, key=-1, *, now=None):
        now = perf_counter() if now is None else now
        command = chr(key & 255).lower() if key >= 0 else ""
        if command == "\x1b":
            self.quit = True
        elif command in ("a", "d"):
            direction = -1 if command == "a" else 1
            if direction != self.direction:
                self.direction, self.next_frame = direction, now+self.interval
        elif command == "s":
            self.direction = 0
        elif command in ("q", "e", "w"):
            self.index = self.start_frame if command == "w" else self.index+(-1 if command == "q" else 1)
            self.index = min(max(self.index, 0), self.count-1)
            self.direction = 0
        if not self.quit and self.direction and now >= self.next_frame:
            self.index = min(max(self.index+self.direction, 0), self.count-1)
            self.next_frame = now+self.interval
            if self.index in (0, self.count-1):
                self.direction = 0
        return command


def resize(image, max_dimension):
    height, width = image.shape[:2]
    scale = max_dimension/max(height, width)
    return cv2.resize(image, (round(width*scale), round(height*scale)),
                      interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)


def demo_parser(description):
    parser = argparse.ArgumentParser(description=description, epilog=PLAYBACK_HELP)
    parser.add_argument("bag", nargs="?", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--rotations", type=int, choices=range(4), default=1)
    parser.add_argument("--max-dimension", type=int, default=600)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--frames", type=int, default=60, help="frames to process in headless mode")
    parser.add_argument("--output", type=Path, help="save measurements.json and the final annotated frame")
    return parser


def run_demo(source, process, args, *, title, refresh=None):
    """process(frame, index) -> (BGR display, JSON-compatible measurements)."""
    if args.max_dimension <= 0 or args.frames <= 0:
        raise ValueError("max-dimension and frames must be positive")
    playback = Playback(source, args.start_frame)
    rows, displayed = [], None
    stop = min(len(source), args.start_frame+args.frames)
    print(PLAYBACK_HELP)
    try:
        while not playback.quit:
            index = playback.index
            if index != displayed:
                frame = source[index]
                started = perf_counter()
                image, info = process(frame, index)
                info = dict(frame=index, timestamp_ns=frame.timestamp_ns,
                            processing_ms=(perf_counter()-started)*1000, **info)
                rows.append(info)
                print(f"Frame {index}: {info.get('status', 'OK')} | {info['processing_ms']:.1f} ms")
                if not args.headless:
                    shown = resize(image, args.max_dimension)
                    cv2.putText(shown, f"Frame {index}/{len(source)-1}", (8, shown.shape[0]-12),
                                cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1, cv2.LINE_AA)
                    cv2.imshow(title, shown)
                displayed = index
            elif refresh is not None:
                refreshed = refresh()
                if refreshed is not None:
                    image, info = refreshed
                    rows[-1].update(info)
                    if not args.headless:
                        cv2.imshow(title, resize(image, args.max_dimension))
            if args.headless:
                playback.index += 1
                if playback.index >= stop:
                    break
            else:
                playback.update(cv2.waitKey(15))
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    break
        if args.output:
            args.output.mkdir(parents=True, exist_ok=True)
            (args.output/"measurements.json").write_text(json.dumps(rows, indent=2))
            cv2.imwrite(str(args.output/"last_frame.png"), image)
    finally:
        if not args.headless:
            cv2.destroyAllWindows()
    return rows
