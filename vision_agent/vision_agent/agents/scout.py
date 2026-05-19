from rfdetr import RFDETRLarge
from PIL import Image
import cv2
import numpy as np

class ScoutAgent:
    def __init__(self, model_path):
        print(f"Loading Global Scout (RF-DETR Large): {model_path}")
        # resolution=1088 must match the value used at training / JIT-trace time.
        self.model = RFDETRLarge(pretrain_weights=model_path, resolution=1088)
        try:
            self.model.optimize_for_inference()
            # Warmup: compile CUDA kernels now (main thread) so the ThreadPoolExecutor
            # worker doesn't hit a cold-start hang on the first predict() call.
            dummy_pil = Image.fromarray(np.zeros((1088, 1088, 3), dtype=np.uint8))
            self.model.predict(dummy_pil, threshold=0.5)
            print("Global Scout: JIT optimization + CUDA warmup complete.")
        except Exception as exc:
            print(f"Global Scout: optimization skipped ({type(exc).__name__}: {exc}), using eager inference.")
        raw = self.model.class_names or {}
        if isinstance(raw, dict):
            self.class_names = [str(raw[k]) for k in sorted(raw.keys())]
        else:
            self.class_names = [str(c) for c in raw]

    def scan(self, frame, draw_debug=True):
        """Returns list of detected objects with bounding boxes and debug image."""
        if frame is None:
            return [], None
            
        # Convert BGR (cv2) to RGB (PIL)
        pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        # Inference
        detections = self.model.predict(pil_img, threshold=0.25)
        
        processed_detections = []
        debug_frame = frame.copy() if draw_debug else None
        
        for i in range(len(detections.xyxy)):
            x1, y1, x2, y2 = detections.xyxy[i]
            conf = float(detections.confidence[i])
            cls_id = int(detections.class_id[i])
            label = self.class_names[cls_id] if cls_id < len(self.class_names) else f"Unknown_{cls_id}"
            
            bbox_int = [int(x1), int(y1), int(x2), int(y2)]
            
            obj = {
                "label": label,
                "confidence": round(conf, 3),
                "box": bbox_int,
                "bbox": bbox_int,
                "center": [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                # Mock segments using box corners for compatibility with depth processing logic
                "segments": [
                    [int(x1), int(y1)],
                    [int(x2), int(y1)],
                    [int(x2), int(y2)],
                    [int(x1), int(y2)]
                ]
            }
            processed_detections.append(obj)
            
            # Draw debug visuals
            if draw_debug:
                cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(debug_frame, f"{label} {conf:.2f}", (int(x1), int(y1)-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        return processed_detections, debug_frame
