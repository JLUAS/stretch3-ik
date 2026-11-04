#!/bin/bash
# Run ON the robot (needs internet). Installs SAM 2 for CPU in its own
# virtualenv, because SAM 2 needs torch >= 2.5 and the system has 2.4. ROS
# Python packages stay visible through --system-site-packages.
# Ends with a CPU benchmark at the D435i low-resolution size.
set -euo pipefail
V=${SAM2_VENV:-$HOME/venvs/sam2}
CKPT=$V/sam2.1_hiera_tiny.pt
python3 -m venv --system-site-packages "$V"
"$V/bin/pip" install --upgrade pip
# Explicit minimums: the system torch 2.4 is visible and would satisfy a bare
# "torch", after which sam2 would pull the multi-GB CUDA build from PyPI.
"$V/bin/pip" install --upgrade "torch>=2.5.1" "torchvision>=0.20.1" \
  --index-url https://download.pytorch.org/whl/cpu
# No build isolation: an isolated build would download its own CUDA torch.
SAM2_BUILD_CUDA=0 "$V/bin/pip" install --no-build-isolation sam2
[ -f "$CKPT" ] || curl -L -o "$CKPT" \
  https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
"$V/bin/python" - "$CKPT" <<'EOF'
import sys, time
import numpy as np, torch
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
torch.set_num_threads(4)
p = SAM2ImagePredictor(build_sam2('configs/sam2.1/sam2.1_hiera_t.yaml', sys.argv[1], device='cpu'))
img = np.zeros((240, 424, 3), np.uint8)
img[80:160, 180:240] = 200
for _ in range(2):  # first call warms up
    t = time.monotonic()
    p.set_image(img)
    m, s, _ = p.predict(point_coords=np.array([[210, 120]], np.float32),
                        point_labels=np.array([1], np.int32), multimask_output=True)
    print(f'click -> mask: {time.monotonic() - t:.2f} s, best score {s.max():.2f}')
EOF
echo "Launch with: sam2_python:=$V/bin/python3 sam2_checkpoint:=$CKPT"
