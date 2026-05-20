from rfdetr import RFDETRSmall
from PIL import Image
import cv2

class SniperAgent:
    def __init__(self, model_path):
        print(f"🎯 Loading Local Sniper (RF-DETR): {model_path}")
        # Match the training-time inference resolution used for the local model.
        self.model = RFDETRSmall(pretrain_weights=model_path, resolution=640)
        try:
            self.model.optimize_for_inference()
        except RuntimeError as exc:
            if "No CUDA GPUs are available" not in str(exc):
                raise
            print("⚠️ Local Sniper running without CUDA optimization; falling back to eager inference.")
        raw = self.model.class_names or {}
        if isinstance(raw, dict):
            self.class_names = [str(raw[k]) for k in sorted(raw.keys())]
        else:
            self.class_names = [str(c) for c in raw]

    def target(self, frame):
        """
        Returns structured dict with screws, screw_heads, tool_tips, and holes.
        """
        if frame is None:
            return {"screws": [], "screw_heads": [], "tool_tips": [], "holes": []}
            
        pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        detections = self.model.predict(pil_img, threshold=0.6)
        
        data = {
            "screws": [],
            "screw_heads": [],
            "tool_tips": [],
            "holes": []
        }
        
        for i in range(len(detections.xyxy)):
            x1, y1, x2, y2 = detections.xyxy[i]
            conf = float(detections.confidence[i])
            cls_id = int(detections.class_id[i])
            label = self.class_names[cls_id] if cls_id < len(self.class_names) else "Unknown"
            
            box = [int(x1), int(y1), int(x2), int(y2)]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            
            obj = {
                "label": label,
                "conf": round(conf, 3),
                "confidence": round(conf, 3),
                "box": box,
                "centroid": [cx, cy],
            }
            
            label_key = label.lower()

            if label_key == "screw":
                obj["center"] = [cx, cy]
                data["screws"].append(obj)
            elif label_key == "screw_head":
                obj["center"] = [cx, cy]
                data["screw_heads"].append(obj)
            elif label_key == "tool_head":
                obj["contact_point"] = [cx, cy]
                data["tool_tips"].append(obj)
            elif label_key == "hole":
                data["holes"].append(obj)
                
        return data
