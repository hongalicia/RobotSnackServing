from ultralytics import YOLO

class PeanutNumClassification:
    def __init__(self):
        self.model = YOLO("PeanutNumberClassification/model/yolo11n-cls_peanut_20251106.pt")  # load the model for classification

    def classify(self, image):
        results_cls = self.model(image, verbose=False)  # predict on an image
        r_cls = results_cls[0]
        top1_id = int(r_cls.probs.top1)
        names = getattr(r_cls, "names", getattr(self.model, "names", {}))
        class_name = names.get(top1_id, str(top1_id))
        return class_name