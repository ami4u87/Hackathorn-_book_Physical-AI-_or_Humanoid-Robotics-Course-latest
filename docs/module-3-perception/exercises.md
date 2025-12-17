# Module 3 Exercises

Practice perception and control integration with these hands-on exercises.

## Exercise 1: Color-Based Object Tracking

**Objective**: Track a colored object and publish its 3D position.

**Tasks**:
1. Use RGB-D camera (simulated or real)
2. Detect object by color (choose red, blue, or green)
3. Calculate 3D position using depth
4. Publish TF transform for the object
5. Visualize in RViz2

**Deliverables**:
- `color_tracker.py` node
- Launch file with camera and RViz2
- Video showing tracking

---

## Exercise 2: ArUco Marker Pose Estimation

**Objective**: Detect ArUco markers and publish their poses to TF.

**Tasks**:
1. Print ArUco markers (use 6x6_250 dictionary)
2. Implement detection node
3. Estimate 6DOF pose for each marker
4. Publish to TF tree
5. Visualize axes in RViz2

**Validation**:
```bash
ros2 run tf2_tools view_frames
# Should show aruco_marker_X frames
```

---

## Exercise 3: YOLO Object Detection

**Objective**: Implement real-time object detection.

**Tasks**:
1. Download YOLOv5 ONNX model
2. Create detection node
3. Publish Detection2DArray messages
4. Draw bounding boxes on output image
5. Measure and report FPS

**Requirements**:
- Achieve > 10 FPS on your hardware
- Detect at least 3 object classes
- Handle cases with no detections

---

## Exercise 4: 3D Object Localization

**Objective**: Combine 2D detection with depth to localize objects in 3D.

**Tasks**:
1. Use YOLO for 2D detection
2. Use depth camera for distance
3. Publish 3D bounding boxes as markers
4. Filter false positives
5. Track objects over time

**Bonus**: Implement simple tracking (e.g., nearest neighbor association)

---

## Exercise 5: Visual Servoing

**Objective**: Control robot to center an object in camera view.

**Tasks**:
1. Detect target object
2. Calculate error from image center
3. Implement P or PD controller
4. Publish velocity commands
5. Test in simulation

**Success Criteria**:
- Object stays centered within ±20 pixels
- Smooth motion (no oscillations)
- Handles target loss gracefully

---

## Exercise 6: Reactive Navigation

**Objective**: Navigate while avoiding obstacles.

**Tasks**:
1. Use laser scan or depth camera
2. Detect obstacles
3. Implement bug algorithm or potential fields
4. Navigate to goal while avoiding collisions
5. Visualize planned path

---

## Exercise 7: State Machine Controller

**Objective**: Implement multi-state behavior.

**States**:
1. SEARCH: Rotate to find target
2. APPROACH: Move toward target
3. INSPECT: Stop and analyze
4. RETREAT: Back away

**Transitions**:
- SEARCH → APPROACH: Target detected
- APPROACH → INSPECT: Close enough
- INSPECT → RETREAT: Analysis complete
- RETREAT → SEARCH: Far enough

---

## Exercise 8: Point Cloud Processing

**Objective**: Process and segment point clouds.

**Tasks**:
1. Subscribe to PointCloud2
2. Downsample using voxel grid
3. Segment ground plane (RANSAC)
4. Cluster remaining points
5. Publish segmented clouds

---

## Exercise 9: Multi-Sensor Fusion

**Objective**: Combine RGB, depth, and IMU data.

**Tasks**:
1. Synchronize RGB and depth images
2. Incorporate IMU orientation
3. Publish fused pose estimate
4. Compare to ground truth
5. Measure accuracy (RMSE)

---

## Exercise 10: Perception Pipeline Integration

**Objective**: Build complete perception system.

**Pipeline**:
1. Camera input
2. Object detection (YOLO)
3. 3D localization
4. TF publishing
5. Reactive control
6. RViz2 visualization

**Success Criteria**:
- All components run simultaneously
- < 100ms end-to-end latency
- Smooth robot behavior
- Clear visualizations

---

## Bonus Challenges

### Challenge A: Real-Time SLAM
Implement visual SLAM using ORB-SLAM3 or similar.

### Challenge B: Human Pose Estimation
Detect and track human skeleton keypoints.

### Challenge C: Semantic Segmentation
Classify each pixel (road, sidewalk, objects, etc.).

### Challenge D: Multi-Object Tracking
Track multiple objects across frames with unique IDs.

---

## Submission Guidelines

For each exercise:
1. **Code**: Clean, commented, with launch files
2. **README**: Setup instructions and dependencies
3. **Results**: Screenshots/videos showing it working
4. **Analysis**: Performance metrics and challenges faced

### Evaluation Criteria
- **Functionality** (40%): Does it work as specified?
- **Code Quality** (20%): Clean, modular, documented?
- **Performance** (20%): Efficient, real-time capable?
- **Visualization** (10%): Clear RViz2 setup?
- **Documentation** (10%): Clear README and comments?
