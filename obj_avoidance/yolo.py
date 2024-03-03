"""
YOLOv8
Github: https://github.com/ultralytics/ultralytics
Docs: https://github.com/ultralytics/ultralytics

https://docs.ultralytics.com/modes/predict/#key-features-of-predict-mode
https://github.com/ultralytics/ultralytics?tab=readme-ov-file#python
"""

from ultralytics import YOLO

# Load a model
model = YOLO("yolov8n.pt")  # pretrained YOLOv8n model

# Run batched inference on a list of images
# results = model(["im1.jpg", "im2.jpg"])  # return a list of Results objects
results = model("https://ultralytics.com/images/bus.jpg")  # predict on an image

print("RESULTS:", results)  # display to screen

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    result.show()  # display to screen
    result.save(filename="result.jpg")  # save to disk
