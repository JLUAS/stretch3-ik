"""Open-vocabulary detection with OWLv2 (transformers, CUDA when available)."""
import numpy as np

MODEL = 'google/owlv2-base-patch16-ensemble'


class Detector:
    def __init__(self, model=MODEL, device=None):
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.torch = torch
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.proc = Owlv2Processor.from_pretrained(model)
        self.model = Owlv2ForObjectDetection.from_pretrained(model).to(self.device).eval()

    def detect(self, rgb, phrase, threshold=0.1):
        """rgb: HxWx3 uint8 upright image. Returns [(score, (u0, v0, u1, v1))]
        best first, in rgb pixels."""
        h, w = rgb.shape[:2]
        inputs = self.proc(text=[[phrase]], images=rgb, return_tensors='pt').to(self.device)
        with self.torch.no_grad():
            out = self.model(**inputs)
        # OWLv2 pads to a square: boxes come back in the padded frame.
        side = max(h, w)
        r = self.proc.post_process_object_detection(
            out, threshold=threshold,
            target_sizes=self.torch.tensor([[side, side]], device=self.device))[0]
        dets = []
        for s, b in zip(r['scores'].tolist(), r['boxes'].tolist()):
            u0, v0, u1, v1 = np.clip(b, 0, [w - 1, h - 1, w - 1, h - 1])
            if u1 > u0 and v1 > v0:
                dets.append((s, (u0, v0, u1, v1)))
        return sorted(dets, reverse=True)
