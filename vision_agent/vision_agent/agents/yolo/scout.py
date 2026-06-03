#!/usr/bin/env python3
"""YOLOv11l-seg Global Scout Agent.

Drop-in replacement for the RF-DETR ScoutAgent.  Returns the same dict
structure so GlobalVisionNode requires zero changes to its processing logic.

Key improvement over RF-DETR version:
  - `segments` field now contains the real polygon mask vertices (not fake
    rectangle corners), enabling accurate 3-D depth projection via fillPoly.

Per-class confidence thresholds
---------------------------------
Some classes (e.g. pcb_screw) are structurally harder for the model — small,
partially occluded, or few training examples — and typically predict below
the global threshold.  PER_CLASS_CONF_OVERRIDE lets you lower the bar for
specific labels without opening the floodgates for everything else.

The model is always run at BASE_CONF (lowest of any override or default) so
YOLO's own NMS pre-filters obvious noise.  Then each detection is re-checked
against the per-class threshold before being returned.
"""
import cv2
import numpy as np

# ── Per-class confidence overrides ────────────────────────────────────────
# Keys must exactly match the model's class names (case-sensitive).
# Classes NOT listed here use the default conf_threshold passed to __init__.
PER_CLASS_CONF_OVERRIDE: dict[str, float] = {
    "pcb_screw": 0.30,   # small / hard class — relaxed threshold
    "case":      0.55,   # large object — slightly relaxed (live-cam vs. val)
}
# Absolute floor used for the YOLO predict() call — must be ≤ all overrides.
_BASE_CONF = min(PER_CLASS_CONF_OVERRIDE.values()) if PER_CLASS_CONF_OVERRIDE else 0.25


class ScoutYolo:
    def __init__(self, model_path: str, conf_threshold: float = 0.70):
        from ultralytics import YOLO  # imported here to avoid top-level dep

        print(f"[ScoutYolo] Loading YOLOv11l-seg global model: {model_path}")
        self.model = YOLO(model_path)
        self.conf = conf_threshold          # default threshold for unlisted classes
        self._base_conf = min(_BASE_CONF, conf_threshold)
        self.imgsz = 1088
        self.class_names: list[str] = self.model.names  # dict {int: str}

        print(f"[ScoutYolo] Default conf={conf_threshold:.2f} | base conf={self._base_conf:.2f}")
        for cls, thr in PER_CLASS_CONF_OVERRIDE.items():
            print(f"[ScoutYolo]   override: {cls} → {thr:.2f}")

        # Warm-up: compile CUDA kernels on main thread to avoid cold-start
        # hang when the first predict() call arrives via ThreadPoolExecutor.
        try:
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self.model.predict(
                source=dummy,
                imgsz=self.imgsz,
                conf=self._base_conf,
                verbose=False,
            )
            print("[ScoutYolo] CUDA warm-up complete.")
        except Exception as exc:
            print(f"[ScoutYolo] Warm-up skipped ({type(exc).__name__}: {exc}).")

    # ------------------------------------------------------------------
    def _class_threshold(self, label: str) -> float:
        """Return the effective confidence threshold for a given class label."""
        return PER_CLASS_CONF_OVERRIDE.get(label, self.conf)

    # ------------------------------------------------------------------
    # Public interface (same as RF-DETR ScoutAgent)
    # ------------------------------------------------------------------
    def scan(self, frame, draw_debug: bool = True):
        """Run segmentation inference on a BGR frame.

        Returns
        -------
        detections : list[dict]
            Each dict contains:
              label, confidence, box [x1,y1,x2,y2], bbox (alias),
              center [cx,cy], segments [[x,y],...]  (real mask polygon)
        debug_frame : np.ndarray | None
        """
        if frame is None:
            return [], None

        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self._base_conf,   # run at lowest possible conf; filter below
            iou=0.60,               # tighter NMS — suppresses duplicate overlapping boxes
            verbose=False,
        )
        result = results[0]

        processed = []
        debug_frame = frame.copy() if draw_debug else None

        boxes = result.boxes
        masks = result.masks  # may be None if no detections

        if boxes is None or len(boxes) == 0:
            return [], debug_frame

        for i in range(len(boxes)):
            x1, y1, x2, y2 = boxes.xyxy[i].tolist()
            conf   = float(boxes.conf[i])
            cls_id = int(boxes.cls[i])
            label  = self.class_names[cls_id] if cls_id in self.class_names else f"cls_{cls_id}"

            # Per-class confidence gate — drop if below class-specific threshold.
            if conf < self._class_threshold(label):
                continue

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            box = [int(x1), int(y1), int(x2), int(y2)]

            # Real segmentation polygon — scaled to image dimensions
            if masks is not None and i < len(masks.xy):
                poly = masks.xy[i]  # (N, 2) float32, already in pixel coords
                segments = poly.astype(np.int32).tolist()
            else:
                # Fallback to bounding-box rectangle if mask missing
                segments = [
                    [int(x1), int(y1)],
                    [int(x2), int(y1)],
                    [int(x2), int(y2)],
                    [int(x1), int(y2)],
                ]

            obj = {
                "label":      label,
                "confidence": round(conf, 3),
                "box":        box,
                "bbox":       box,           # alias for compatibility
                "center":     [cx, cy],
                "segments":   segments,
            }
            processed.append(obj)

            if draw_debug:
                # Use different colour for override classes so they're obvious.
                colour = (0, 165, 255) if label in PER_CLASS_CONF_OVERRIDE else (0, 255, 0)
                cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), colour, 2)
                cv2.putText(
                    debug_frame,
                    f"{label} {conf:.2f}",
                    (int(x1), max(int(y1) - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 2,
                )
                if len(segments) > 2:
                    pts = np.array(segments, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(debug_frame, [pts], True, (0, 200, 255), 1)

        return processed, debug_frame
