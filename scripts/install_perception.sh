#!/usr/bin/env bash
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
venv="${STRETCH_PERCEPTION_VENV:-$HOME/venvs/stretch_perception}"
python3 -m venv --system-site-packages "$venv"
"$venv/bin/python3" -m pip install -r "$repo/src/stretch_rgbd_perception/requirements-ocr.txt"
"$venv/bin/python3" -c 'import cv2, open3d, scipy, rapidocr_onnxruntime; print("Perception dependencies ready")'
printf 'Launch with perception_python:=%s/bin/python3\n' "$venv"
