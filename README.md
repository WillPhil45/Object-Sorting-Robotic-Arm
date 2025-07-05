# Intelligent Object Sorting Robotic Arm

A sophisticated robotic simulation system that demonstrates intelligent object sorting capabilities using computer vision and inverse kinematics. The system autonomously detects, grasps, and sorts objects based on their color characteristics using a 6-DOF industrial robotic arm.

## Project Overview

This project simulates an intelligent robotics system capable of automated object sorting in the Webots robotics simulation environment. The system combines computer vision, motion planning, and robotic manipulation to create a fully autonomous sorting solution that can identify objects by color and place them into corresponding bins.

**Key Capabilities:**
- **Autonomous Object Detection**: Real-time visual recognition using camera-based perception
- **Color-Based Classification**: Intelligent sorting based on RGB color analysis
- **Precision Manipulation**: Accurate pick-and-place operations using inverse kinematics
- **Adaptive Motion Planning**: Dynamic path planning for optimal object retrieval

## System Architecture

### Hardware Components (Simulated)
- **IRB 4600/40 Industrial Robot**: 6-degree-of-freedom robotic arm providing extensive workspace coverage
- **ROBOTIQ 3f Gripper**: Three-finger adaptive gripper for secure object manipulation
- **End-Effector Camera**: Integrated vision system with GPS positioning for accurate object localization
- **Sorting Bins**: Three designated bins for red, green, and blue object classification

### Software Framework
- **Webots Simulation Environment**: Physics-based robotics simulation platform
- **IKPy Library**: Inverse kinematics calculations for motion planning
- **Computer Vision Pipeline**: Object detection and color analysis system
- **Coordinate Transformation**: Camera-to-world frame coordinate mapping

## Technical Implementation

### Computer Vision System
The vision system uses Webots' built-in object recognition capabilities to:
- Detect objects in real-time camera feed
- Extract RGB color values for classification
- Calculate object position relative to camera frame
- Transform coordinates to world reference frame

### Motion Planning Algorithm
```python
# Coordinate transformation from camera frame to world frame
xw = xcamera + (r11 · xc + r12 · yc + r13 · zc)
yw = ycamera + (r21 · xc + r22 · yc + r23 · zc)  
zw = zcamera + (r31 · xc + r32 · yc + r33 · zc)
```

The system employs inverse kinematics to calculate joint angles required for precise end-effector positioning, enabling smooth and accurate object manipulation.

### Pick-and-Place Workflow
1. **Object Detection**: Scan environment for objects using camera system
2. **Coordinate Transformation**: Convert object position to robot coordinate system
3. **Approach Planning**: Calculate optimal approach trajectory
4. **Precision Grasping**: Execute controlled gripper closure
5. **Safe Transport**: Lift object to collision-free height
6. **Color Classification**: Determine target bin based on object color
7. **Placement**: Deliver object to appropriate sorting bin
8. **Home Return**: Return to scanning position for next object

## Getting Started

### Prerequisites
- **Webots R2023b** or later
- **Python 3.8+**
- **Required Libraries**:
  ```bash
  pip install ikpy numpy
  ```

### Installation & Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/WillPhil45/Object-Sorting-Robotic-Arm.git
   cd Object-Sorting-Robotic-Arm
   ```

2. Open Webots and load the simulation world file

3. Ensure the URDF file path in the controller matches your local setup:
   ```python
   filename = r'path/to/your/Irb4600-40.urdf'
   ```

4. Place objects in the simulation environment

5. Run the simulation to watch autonomous sorting in action

### Configuration
- **Bin Positions**: Modify `BIN_POSITIONS` array to adjust sorting locations
- **Color Thresholds**: Tune color detection parameters in `determine_bin_position()`
- **Motion Parameters**: Adjust `POSITION_TOLERANCE` and `GRIPPER_TOLERANCE` for precision tuning

## Performance Evaluation

### Test Scenarios
The system has been evaluated across multiple scenarios:

| Test Condition | Performance | Notes |
|----------------|-------------|-------|
| **Normal Conditions** | ✅ Excellent | Objects on table surface - optimal performance |
| **Floor Placement** | ✅ Excellent | Demonstrates height adaptability |
| **Scattered Objects** | ⚠️ Limited | Field of view constraints affect detection |
| **Clustered Objects** | ⚠️ Partial | Collision detection limitations |
| **Enclosed Spaces** | ❌ Failed | Requires collision avoidance system |

### Key Strengths
- **High Precision**: Accurate object positioning and placement
- **Robust Color Detection**: Reliable RGB-based classification
- **Smooth Motion**: Fluid robotic arm movements
- **Modular Design**: Easy to extend and modify

### Known Limitations
- **No Collision Detection**: Cannot handle obstacles or enclosed spaces
- **Fixed Camera Position**: Limited field of view affects object detection range
- **Basic Grasping**: Simplified gripper control without force feedback
- **Simulation Dependency**: Heavy reliance on Webots-specific features


