# Color-Sorted Pick-and-Place in ROS

A compact ROS-Gazebo manipulation setup that performs perception-driven pick-and-place by detecting colored blocks in image space, projecting them into 3D world coordinates, and executing sorting through a simple state-machine pipeline. The system combines RGB-based color segmentation, depth-assisted geometric reasoning, camera-to-world coordinate transformation, and controller-driven manipulation to place red, green, and blue blocks into their corresponding bins. The perception node publishes structured object detections, while the downstream state machine repeatedly selects available objects and commands pick-and-place execution until the workspace is cleared. The implementation is lightweight, interpretable, and simulation-first, making it a clean demonstration of practical robot perception feeding into action.

---

## Overview

We study a classic robotics pipeline in which **perception informs manipulation**. A simulated RGB-D camera observes a tabletop workspace containing colored blocks and bins. The perception module identifies block pixels using color segmentation, estimates geometric properties from contours, retrieves depth values, and converts 2D detections into 3D world-frame object locations. These detections are then published as ROS messages for downstream use. A state machine consumes those detections, selects one object at a time, invokes the manipulation controller, and repeats until no blocks remain. This creates a complete perception-to-action loop inside ROS and Gazebo.

At its core, the work demonstrates five practical robotics ideas:

1. **Image-space object detection** using HSV masking and contour extraction.  
2. **Depth-aware geometric reasoning** for estimating object position and size.  
3. **Camera-to-world transformation** using homogeneous transforms.  
4. **Structured ROS communication** via custom detection messages.  
5. **Task-level autonomy** via a simple but effective pick-and-place state machine.

## Main Idea

The main idea is to treat colored block sorting as a compact end-to-end robotic autonomy problem:

- **Perception** answers: *What objects exist, where are they, and what color are they?*
- **Manipulation logic** answers: *Which object should be picked next, and where should it go?*
- **Control** answers: *How does the arm physically move the object into the correct bin?*

Instead of relying on learned object detectors or heavy perception stacks, the setup uses a transparent, geometry-aware classical pipeline. RGB image data isolates color classes, depth data provides metric information, and camera intrinsics plus rigid transforms map detections into the world frame. A state machine then orchestrates repeated sorting behavior until the task is complete. This makes the system interpretable, reproducible, and ideal for demonstrating how perception and planning couple in a manipulation pipeline.

## System Pipeline

The full pipeline is:

```text
RGB-D Camera Frame
        ↓
ROS Image Callback
        ↓
BGR → HSV Conversion
        ↓
Color-wise Masking (Red / Green / Blue)
        ↓
Contour Extraction
        ↓
Centroid + Bounding Box + Area Filtering
        ↓
Depth Lookup
        ↓
2D Pixel → 3D Camera Ray Projection
        ↓
Camera Frame → World Frame Transform
        ↓
DetectedObjectsStamped Publication
        ↓
State Machine Object Selection
        ↓
Controller-Driven Pick and Place
        ↓
Correct Bin Placement
```

This flow is directly reflected in the implementation: the callback converts the image, extracts contours for each target color, computes centroids and dimensions, and publishes world-frame detections. The state machine then transitions through Home, SelectingObject, PickingAndPlacing, and Done states until all blocks have been sorted.

## Mathematical Model

Although lightweight, this work contains a meaningful geometric perception backbone.

### 1. Color Segmentation in HSV Space

Given an RGB image converted to HSV, color-specific binary masks are created using thresholding:

$$
M_c(u,v) =
\begin{cases}
1, & \text{if } (H,S,V) \in \mathcal{R}_c \\
0, & \text{otherwise}
\end{cases}
$$

where $\mathcal{R}_c$ is the HSV range for color $c \in \{\text{red, green, blue}\}$.

Contours are extracted from these masks, and only regions above a chosen area threshold are retained as valid block candidates. This reduces noise and isolates block-like connected components.

### 2. Centroid Computation

For each valid contour, the pixel centroid is computed using image moments:

$$
c_x = \frac{M_{10}}{M_{00}}, \qquad c_y = \frac{M_{01}}{M_{00}}
$$

where $M_{ij}$ are the standard contour moments. This centroid becomes the reference pixel for depth lookup and 3D back-projection.

### 3. Pixel-to-3D Projection

Using the camera intrinsics from a pinhole camera model, each object centroid is projected into a 3D ray:

$$
\mathbf{p}_c =
z \, K^{-1}
\begin{bmatrix}
u\\
v\\
1
\end{bmatrix}
$$

where $K$ is the intrinsic calibration matrix. The ray is then scaled using the measured depth \( z \) from the depth image to estimate the 3D point in the camera frame:

$$
\mathbf{p}_c = z \cdot \mathbf{r}(u,v)
$$

### 4. Homogeneous Coordinate Transform

To convert a 3D point from the camera frame to the world frame, the project applies a rigid-body coordinate transformation. In standard vector form,

$$
\mathbf{p}_w = R\,\mathbf{p}_c + t
$$

where $\mathbf{p}_c = [x_c\ y_c\ z_c]^T$ is the point in the camera frame, $\mathbf{p}_w = [x_w\ y_w\ z_w]^T$ is the corresponding point in the world frame, $R \in \mathbb{R}^{3\times3}$ is the rotation matrix, and $t \in \mathbb{R}^{3}$ is the translation vector.

Equivalently, in homogeneous coordinates,

$$
\mathbf{p}_w^h = T_{c\to w}\.\mathbf{p}_c^h
$$

where $p_{c^h}$ = $[x_c\ y_c\ z_c\ 1]^T$ is the point in homogeneous camera-frame coordinates, $p_{w^h}$ = $[x_w\ y_w\ z_w\ 1]^T$ is the corresponding point in homogeneous world-frame coordinates, and $T_{c→w}$ is the rigid transformation from camera frame to world frame.

Here, $R \in \mathbb{R}^{3 \times 3}$ is the rotation matrix and $t \in \mathbb{R}^{3}$ is the translation vector from camera frame to world frame.

### 5. Metric Box Dimension Estimation

Object width and length are estimated by back-projecting image-space box corners into 3D and measuring Euclidean distances:

$$
w_{obj} = \|\mathbf{p}_1 - \mathbf{p}_2\|, \qquad
l_{obj} = \|\mathbf{p}_1 - \mathbf{p}_3\|
$$

where $\mathbf{p}_1, \mathbf{p}_2, \mathbf{p}_3$ are selected 3D corner points from the box footprint.

### 6. State-Machine Task Logic

The pick-and-place execution follows a finite-state machine:

$$
\text{Home} \rightarrow \text{SelectingObject} \rightarrow \text{PickingAndPlacing} \rightarrow \text{Home}
$$

with terminal condition:

$$
\text{Home} \rightarrow \text{Done} \quad \text{if no objects remain}
$$

The arm resets in Home, selects an object if any remain, executes pick-and-place, and returns to Home until completion.

## ROS Components

### `object_detector.py`

This is the perception front-end. It subscribes to `/camera/color/image_raw`, converts ROS image messages into OpenCV format, applies HSV masking for red, green, and blue objects, extracts contours, filters them by area, estimates geometric attributes, and publishes all detected objects through a structured `DetectedObjectsStamped` message. It also uses depth data and the camera model to recover real 3D object positions.

### `pick_and_place_state_machine.py`

This is the task-level execution layer. It starts in Home, checks whether objects remain, selects a target object, calls the manipulation controller, and cycles until no objects remain. The state transitions are intentionally minimal, which keeps the behavior interpretable and robust for a structured sorting task.

### `controller.py`

This provides the manipulation interface used by the state machine. It abstracts arm motions and object transfer, allowing the task logic to remain separate from lower-level motion execution.

### Custom Messages

The package defines:

- `DetectedObject.msg`
- `DetectedObjectsStamped.msg`

These support structured communication of object detections between perception and downstream task logic.

## Repository Structure

```text
color-sorted-pick-place-ros/
├── assets/
│   └── demo.mp4
├── CMakeLists.txt
├── launch/
│   └── panda_world.launch
├── LICENSE
├── models/
│   ├── bin_blue/
│   ├── bin_green/
│   ├── bin_red/
│   ├── block_blue/
│   ├── block_green/
│   ├── block_red/
│   ├── kinect/
│   ├── underbin_bench/
│   └── workbench/
├── msg/
│   ├── DetectedObject.msg
│   └── DetectedObjectsStamped.msg
├── package.xml
├── requirements.txt
├── scripts/
│   ├── controller.py
│   ├── __init__.py
│   ├── object_detector.py
│   └── pick_and_place_state_machine.py
├── setup.py
└── worlds/
    └── pick_and_place.world
```

### Folder Notes

- `scripts/` contains the core node logic for perception, control, and state-machine execution.
- `msg/` defines the custom ROS messages used to communicate detections.
- `launch/` contains the launch file for spawning the simulation environment.
- `worlds/` stores the Gazebo world definition.
- `models/` includes the simulation models for blocks, bins, camera, and workbench assets.
- `assets/demo.mp4` contains a sample execution of the completed pipeline.

## How to Reproduce

### Prerequisites

- ROS1 catkin workspace.
- Gazebo simulation support.
- The required dependencies listed in `package.xml`.
- Python dependencies from `requirements.txt`.

### Build / Workspace Setup

If using an existing catkin workspace:

```bash
cd ~/catkin_ws
source devel/setup.bash
```

If building from a fresh catkin workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Execution

Run the following in **four separate terminals**, all sourced to the workspace:

#### Terminal 1

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch pick_and_place panda_world.launch
```

#### Terminal 2

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch panda_sim_moveit sim_move_group.launch
```

#### Terminal 3

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun pick_and_place object_detector.py
```

#### Terminal 4

```bash
cd ~/catkin_ws/src/pick_and_place/scripts
source ~/catkin_ws/devel/setup.bash
python3 pick_and_place_state_machine.py
```

This execution flow follows the intended launch pipeline for the perception, planning, and manipulation stack.

## Detection and Task Flow

### Perception Flow

When an image arrives:

1. ROS delivers the image to `image_callback(msg)`.
2. The image is converted from ROS to OpenCV BGR.
3. The BGR image is transformed into HSV.
4. A mask is created for each target color.
5. Contours are extracted from each masked image.
6. Small noisy contours are rejected.
7. Bounding boxes, centroids, and object dimensions are estimated.
8. Each detection is projected into 3D world coordinates.
9. A `DetectedObjectsStamped` message is published.

### Task Flow

1. The arm enters a neutral Home state.
2. If objects remain, a target object is selected.
3. The controller executes a pick-and-place motion.
4. The system returns to Home.
5. The loop repeats until no objects remain.
6. The state machine exits with a completion signal.

## Results and Discussion

The final system successfully demonstrates a full simulation-based sorting loop in which colored tabletop blocks are detected in image space, localized in 3D, and placed into their corresponding bins through repeated pick-and-place execution. The perception pipeline is effective because the task is visually structured: colors are distinct, object geometry is simple, and the environment is controlled. This makes HSV-based segmentation a practical choice instead of a heavier learned detector.

A strong point of this work is its **interpretability**. Each stage is easy to inspect:

- masks can be visualized,
- contours can be validated,
- centroids and boxes are explicit,
- 3D coordinates follow from known geometry,
- state transitions are transparent and finite.

The work also highlights a classic robotics lesson: **simple pipelines can work very well when the environment is well constrained**. Rather than over-engineering the solution, the implementation uses just enough perception and logic to complete the task reliably in simulation.

That said, the system naturally inherits several limitations:

- HSV thresholding is sensitive to lighting variation and color ambiguity.
- Depth accuracy can degrade near edges or reflective regions.
- The state machine is simple and does not optimize task order.
- The work is strongly simulation-oriented and not meant as a robust real-world deployment pipeline.

Even so, for its intended scope, the implementation is successful: it shows how image processing, geometry, ROS1 messaging, and control can be composed into a functional manipulation behavior.

## Conclusion

This work demonstrates a clean end-to-end robotic perception and manipulation workflow in ROS1 and Gazebo. A perception node detects colored objects using classical computer vision, estimates their metric location through depth and camera geometry, and communicates those detections to a downstream state machine that repeatedly performs sorting actions. The result is a compact but complete robotics system connecting sensing, reasoning, and action.

## Demo

A demonstration video is included in:

```text
assets/demo.mp4
```

This video shows the manipulator performing block sorting behavior in simulation.

## Academic Context

This work was completed in an academic context as part of **Dr. Nakul Gopalan’s CSE 598: Perception in Robotics** course at **Arizona State University**, where it originally appeared as a pick-and-place ROS assignment involving an environment, planning setup, perception pipeline, and state machine. This repository reframes that work as a compact robotics work while preserving the underlying technical ideas and implementation structure.

## License

> **MIT License**

---
