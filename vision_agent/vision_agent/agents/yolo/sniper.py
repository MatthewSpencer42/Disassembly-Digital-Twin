#!/usr/bin/env python3
"""YOLOv11s-seg Local Sniper Agent.

Drop-in replacement for the RF-DETR SniperAgent.  Returns the same dict
structure so LocalVisionNode requires zero changes to its processing logic.

Classes (must match training data.yaml):
  0: hole
  1: screw
  2: screw_head
  3: tool_head
"""
import cv2
import numpy as np


class SniperYolo:
    def __init__(self, model_path: str, conf_threshold: float = 0.70):
        from ultralytics import YOLO

        print(f"[SniperYolo] Loading YOLOv11s-seg local model: {model_path}")
        self.model = YOLO(model_path)
        self.conf = conf_threshold
        self.imgsz = 640
        self.class_names: dict = self.model.names  # {int: str}

        # Warm-up
        try:
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self.model.predict(source=dummy, imgsz=self.imgsz, conf=self.conf, verbose=False)
            print("[SniperYolo] CUDA warm-up complete.")
        except Exception as exc:
            print(f"[SniperYolo] Warm-up skipped ({type(exc).__name__}: {exc}).")

    # ------------------------------------------------------------------
    # Public interface (same as RF-DETR SniperAgent)
    # ------------------------------------------------------------------
    def target(self, frame) -> dict:
        """Run segmentation inference on a BGR frame.

        Returns
        -------
        dict with keys: screws, screw_heads, tool_tips, holes
        Each entry is a list of dicts:
          label, conf, confidence, box [x1,y1,x2,y2],
          centroid [cx,cy], center [cx,cy], contact_point (tool_head only)
        """
        empty = {"screws": [], "screw_heads": [], "tool_tips": [], "holes": []}
        if frame is None:
            return empty

        h, w = frame.shape[:2]
        if w != self.imgsz or h != self.imgsz:
            inf_frame = cv2.resize(frame, (self.imgsz, self.imgsz))
            sx, sy = w / self.imgsz, h / self.imgsz
        else:
            inf_frame = frame
            sx, sy = 1.0, 1.0

        results = self.model.predict(
            source=inf_frame,
            imgsz=self.imgsz,
            conf=self.conf,
            iou=0.60,           # tighter NMS — suppresses duplicate overlapping boxes
            verbose=False,
        )
        result = results[0]
        data = {"screws": [], "screw_heads": [], "tool_tips": [], "holes": []}

        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return data

        for i in range(len(boxes)):
            x1, y1, x2, y2 = [v * s for v, s in zip(boxes.xyxy[i].tolist(), [sx, sy, sx, sy])]
            conf     = float(boxes.conf[i])
            cls_id   = int(boxes.cls[i])
            label    = self.class_names[cls_id] if cls_id in self.class_names else "unknown"

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            box = [int(x1), int(y1), int(x2), int(y2)]

            obj = {
                "label":      label,
                "conf":       round(conf, 3),
                "confidence": round(conf, 3),
                "box":        box,
                "centroid":   [cx, cy],
            }

            lk = label.lower()
            if lk == "screw":
                obj["center"] = [cx, cy]
                data["screws"].append(obj)
            elif lk == "screw_head":
                obj["center"] = [cx, cy]
                data["screw_heads"].append(obj)
            elif lk == "tool_head":
                obj["contact_point"] = [cx, cy]
                data["tool_tips"].append(obj)
            elif lk == "hole":
                data["holes"].append(obj)

        return data
