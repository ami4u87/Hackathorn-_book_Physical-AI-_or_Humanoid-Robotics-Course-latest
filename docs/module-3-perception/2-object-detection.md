# Object Detection and Classification

Object detection enables robots to identify and locate objects in their environment. This lesson covers traditional computer vision methods and modern deep learning approaches for object detection.

## Detection Approaches

### 1. Traditional Methods (Classical CV)
- Color-based segmentation
- Template matching
- Feature detection (SIFT, ORB, SURF)
- Cascade classifiers

### 2. Deep Learning Methods (Modern)
- YOLO (You Only Look Once)
- SSD (Single Shot Detector)
- Faster R-CNN
- EfficientDet

## Color-Based Detection

Simple but effective for controlled environments.

```python
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.detection_pub = self.create_publisher(
            Image, '/detection/image', 10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect red objects
        detections = self.detect_color(cv_image, color='red')

        # Draw bounding boxes
        output_image = self.draw_detections(cv_image, detections)

        # Publish result
        output_msg = self.bridge.cv2_to_imgmsg(output_image, 'bgr8')
        self.detection_pub.publish(output_msg)

    def detect_color(self, image, color='red'):
        """
        Detect objects of specific color

        Returns:
            List of bounding boxes [(x, y, w, h), ...]
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define color ranges
        color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],  # Red wraps around
            'red2': [(170, 100, 100), (180, 255, 255)],
            'blue': [(100, 100, 100), (130, 255, 255)],
            'green': [(40, 50, 50), (80, 255, 255)],
            'yellow': [(20, 100, 100), (30, 255, 255)],
        }

        # Create mask
        if color == 'red':
            # Red needs two ranges
            mask1 = cv2.inRange(hsv, np.array(color_ranges['red'][0]),
                               np.array(color_ranges['red'][1]))
            mask2 = cv2.inRange(hsv, np.array(color_ranges['red2'][0]),
                               np.array(color_ranges['red2'][1]))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            lower, upper = color_ranges[color]
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Get bounding boxes
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)

            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                detections.append((x, y, w, h))

        return detections

    def draw_detections(self, image, detections):
        """Draw bounding boxes on image"""
        output = image.copy()

        for (x, y, w, h) in detections:
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Calculate center
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)

            # Add label
            label = f"Object ({w}x{h})"
            cv2.putText(output, label, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return output
```

## Feature-Based Detection

Use SIFT/ORB for object recognition.

```python
class FeatureDetector:
    def __init__(self, template_image_path):
        """
        Initialize with template image to match

        Args:
            template_image_path: Path to reference image
        """
        self.template = cv2.imread(template_image_path)
        self.template_gray = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)

        # Initialize ORB detector (SIFT is patented, use ORB)
        self.detector = cv2.ORB_create(nfeatures=1000)

        # Detect template keypoints
        self.template_kp, self.template_desc = self.detector.detectAndCompute(
            self.template_gray, None
        )

        # Matcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    def detect_object(self, scene_image, min_matches=10):
        """
        Detect template object in scene

        Returns:
            Homography matrix and corners if found, else None
        """
        scene_gray = cv2.cvtColor(scene_image, cv2.COLOR_BGR2GRAY)

        # Detect scene keypoints
        scene_kp, scene_desc = self.detector.detectAndCompute(scene_gray, None)

        if scene_desc is None:
            return None

        # Match features
        matches = self.matcher.knnMatch(self.template_desc, scene_desc, k=2)

        # Apply Lowe's ratio test
        good_matches = []
        for pair in matches:
            if len(pair) == 2:
                m, n = pair
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)

        if len(good_matches) < min_matches:
            return None

        # Find homography
        src_pts = np.float32([self.template_kp[m.queryIdx].pt for m in good_matches])
        dst_pts = np.float32([scene_kp[m.trainIdx].pt for m in good_matches])

        H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

        if H is None:
            return None

        # Get template corners
        h, w = self.template_gray.shape
        template_corners = np.float32([
            [0, 0],
            [w, 0],
            [w, h],
            [0, h]
        ]).reshape(-1, 1, 2)

        # Transform to scene coordinates
        scene_corners = cv2.perspectiveTransform(template_corners, H)

        return H, scene_corners

    def draw_detection(self, scene_image, detection):
        """Draw detected object outline"""
        if detection is None:
            return scene_image

        H, corners = detection
        output = scene_image.copy()

        # Draw outline
        corners_int = np.int32(corners)
        cv2.polylines(output, [corners_int], True, (0, 255, 0), 3)

        return output
```

## Deep Learning Object Detection

### YOLO with ONNX Runtime

```python
import onnxruntime as ort

class YOLODetector:
    def __init__(self, model_path, conf_threshold=0.5, iou_threshold=0.4):
        """
        Initialize YOLO detector

        Args:
            model_path: Path to ONNX model file
            conf_threshold: Confidence threshold
            iou_threshold: IoU threshold for NMS
        """
        self.session = ort.InferenceSession(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold

        # Input shape
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape

        # COCO class names
        self.classes = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

    def preprocess(self, image):
        """Preprocess image for YOLO"""
        # Resize to input shape
        input_h, input_w = self.input_shape[2:4]
        resized = cv2.resize(image, (input_w, input_h))

        # Normalize to [0, 1]
        input_tensor = resized.astype(np.float32) / 255.0

        # HWC to CHW
        input_tensor = np.transpose(input_tensor, (2, 0, 1))

        # Add batch dimension
        input_tensor = np.expand_dims(input_tensor, axis=0)

        return input_tensor

    def postprocess(self, outputs, original_shape):
        """Post-process YOLO outputs"""
        predictions = outputs[0][0]  # [num_boxes, 85] for COCO

        boxes = []
        scores = []
        class_ids = []

        img_h, img_w = original_shape[:2]
        input_h, input_w = self.input_shape[2:4]

        for prediction in predictions:
            confidence = prediction[4]

            if confidence > self.conf_threshold:
                # Get class scores
                class_scores = prediction[5:]
                class_id = np.argmax(class_scores)
                class_score = class_scores[class_id]

                final_score = confidence * class_score

                if final_score > self.conf_threshold:
                    # Box coordinates (cx, cy, w, h) normalized
                    cx, cy, w, h = prediction[0:4]

                    # Convert to pixel coordinates
                    cx = cx * img_w / input_w
                    cy = cy * img_h / input_h
                    w = w * img_w / input_w
                    h = h * img_h / input_h

                    # Convert to (x, y, w, h)
                    x = cx - w / 2
                    y = cy - h / 2

                    boxes.append([x, y, w, h])
                    scores.append(final_score)
                    class_ids.append(class_id)

        # Apply NMS
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_threshold, self.iou_threshold)

        detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                detections.append({
                    'box': boxes[i],
                    'score': scores[i],
                    'class_id': class_ids[i],
                    'class_name': self.classes[class_ids[i]]
                })

        return detections

    def detect(self, image):
        """Run detection on image"""
        # Preprocess
        input_tensor = self.preprocess(image)

        # Inference
        outputs = self.session.run(None, {self.input_name: input_tensor})

        # Postprocess
        detections = self.postprocess(outputs, image.shape)

        return detections

    def draw_detections(self, image, detections):
        """Draw detections on image"""
        output = image.copy()

        for det in detections:
            x, y, w, h = [int(v) for v in det['box']]
            score = det['score']
            class_name = det['class_name']

            # Draw box
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 0), 2)

            # Draw label
            label = f"{class_name}: {score:.2f}"
            label_size, baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)

            # Background rectangle
            cv2.rectangle(output,
                         (x, y - label_size[1] - 10),
                         (x + label_size[0], y),
                         (0, 255, 0), -1)

            # Text
            cv2.putText(output, label, (x, y - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        return output
```

### ROS 2 Integration

```python
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', 'yolov5s.onnx')
        self.declare_parameter('confidence_threshold', 0.5)

        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('confidence_threshold').value

        # Initialize detector
        self.detector = YOLODetector(model_path, conf_threshold)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10
        )
        self.image_pub = self.create_publisher(
            Image, '/detection/image', 10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect
        detections = self.detector.detect(cv_image)

        # Publish detection messages
        detection_msg = self.create_detection_msg(detections, msg.header)
        self.detection_pub.publish(detection_msg)

        # Publish annotated image
        output_image = self.detector.draw_detections(cv_image, detections)
        output_msg = self.bridge.cv2_to_imgmsg(output_image, 'bgr8')
        self.image_pub.publish(output_msg)

    def create_detection_msg(self, detections, header):
        """Convert detections to Detection2DArray message"""
        msg = Detection2DArray()
        msg.header = header

        for det in detections:
            detection = Detection2D()

            # Bounding box
            x, y, w, h = det['box']
            detection.bbox.center.position.x = x + w / 2
            detection.bbox.center.position.y = y + h / 2
            detection.bbox.size_x = w
            detection.bbox.size_y = h

            # Hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = det['score']

            detection.results.append(hypothesis)
            detection.id = det['class_name']

            msg.detections.append(detection)

        return msg
```

## Instance Segmentation

For pixel-level object masks, use Mask R-CNN or similar.

```python
class MaskRCNNDetector:
    """Simplified Mask R-CNN interface"""

    def __init__(self, model_path):
        import torch
        from torchvision.models.detection import maskrcnn_resnet50_fpn

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Load model
        self.model = maskrcnn_resnet50_fpn(pretrained=True)
        self.model.to(self.device)
        self.model.eval()

    def detect(self, image):
        """Run instance segmentation"""
        import torch
        from torchvision import transforms

        # Preprocess
        transform = transforms.Compose([transforms.ToTensor()])
        input_tensor = transform(image).unsqueeze(0).to(self.device)

        # Inference
        with torch.no_grad():
            predictions = self.model(input_tensor)[0]

        # Extract results
        masks = predictions['masks'].cpu().numpy()
        boxes = predictions['boxes'].cpu().numpy()
        labels = predictions['labels'].cpu().numpy()
        scores = predictions['scores'].cpu().numpy()

        return {
            'masks': masks,
            'boxes': boxes,
            'labels': labels,
            'scores': scores
        }
```

## 3D Object Detection

Combine RGB and depth for 3D bounding boxes.

```python
def detect_3d_objects(rgb_image, depth_image, camera_intrinsics, detections_2d):
    """
    Convert 2D detections to 3D bounding boxes

    Args:
        rgb_image: Color image
        depth_image: Depth map (meters)
        camera_intrinsics: CameraIntrinsics object
        detections_2d: List of 2D detections

    Returns:
        List of 3D bounding boxes
    """
    detections_3d = []

    for det in detections_2d:
        x, y, w, h = det['box']

        # Get depth at center
        cx = int(x + w / 2)
        cy = int(y + h / 2)

        # Median depth in box (more robust than single pixel)
        roi = depth_image[int(y):int(y+h), int(x):int(x+w)]
        roi_valid = roi[roi > 0]

        if len(roi_valid) == 0:
            continue

        z = np.median(roi_valid)

        # Back-project to 3D
        fx, fy, cx_cam, cy_cam = camera_intrinsics.fx, camera_intrinsics.fy, \
                                  camera_intrinsics.cx, camera_intrinsics.cy

        x_3d = (cx - cx_cam) * z / fx
        y_3d = (cy - cy_cam) * z / fy
        z_3d = z

        # Estimate 3D size (rough approximation)
        width_3d = w * z / fx
        height_3d = h * z / fy
        depth_3d = 0.2  # Assume fixed depth for simplicity

        detections_3d.append({
            'class_name': det['class_name'],
            'score': det['score'],
            'position': (x_3d, y_3d, z_3d),
            'size': (width_3d, height_3d, depth_3d)
        })

    return detections_3d
```

## Performance Optimization

### Model Quantization

```python
# Convert PyTorch model to ONNX with quantization
import torch

model = ...  # Your trained model
dummy_input = torch.randn(1, 3, 640, 640)

torch.onnx.export(
    model,
    dummy_input,
    "model_quantized.onnx",
    opset_version=13,
    do_constant_folding=True,
    input_names=['input'],
    output_names=['output'],
    dynamic_axes={
        'input': {0: 'batch_size'},
        'output': {0: 'batch_size'}
    }
)

# Further quantize with ONNX Runtime
from onnxruntime.quantization import quantize_dynamic

quantize_dynamic("model.onnx", "model_int8.onnx")
```

### GPU Acceleration

```bash
# Install ONNX Runtime with GPU support
pip install onnxruntime-gpu

# Use CUDA execution provider
session = ort.InferenceSession(
    model_path,
    providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
)
```

## Next Steps

In the next lesson, you'll learn how to estimate object poses and publish them to the TF tree.

## Further Reading

- [YOLOv5 Documentation](https://github.com/ultralytics/yolov5)
- [OpenCV DNN Module](https://docs.opencv.org/master/d2/d58/tutorial_table_of_content_dnn.html)
- [vision_msgs ROS Package](https://github.com/ros-perception/vision_msgs)
- [ONNX Runtime](https://onnxruntime.ai/)
