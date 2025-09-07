# Mobile Robotics Project – Thymio Navigation  

## 📌 Project Overview  
This project focuses on enabling a Thymio robot to autonomously navigate a dynamic environment with obstacles, using a combination of **computer vision, path planning, motion control, local navigation, and sensor fusion**. The system integrates multiple sources of information (camera feed and onboard sensors) to achieve robust and adaptive navigation.  

## Video Demo
The video of the demo can be watched on [YouTube](https://youtu.be/gpMigWjB38A)

## 🧠 Methods  

### 1. Vision System  
- **Marker Detection**: The environment includes ArUco markers to detect the robot, goal, and obstacles.  
- **Grayscale Conversion & Thresholding**: Frames are preprocessed into binary representations to segment the environment.  
- **2D Grid Representation**: A cell-based grid map is constructed to represent free space and obstacles.  
- **Dynamic Updates**: The map updates in real time, allowing recovery from events like robot displacement (kidnapping).  

---

### 2. Global Navigation  
- **Dijkstra’s Algorithm**: Used to compute the shortest path from the Thymio to the goal on the 2D grid.  
- **Path Discretization**: The path is converted into intermediate waypoints to enable smooth traversal.  
- **Integration with Vision**: The path planning system updates dynamically as new vision input is received.  

---

### 3. Motion Control  
- **Direction Calculation**: Determines the heading relative to the next waypoint.  
- **Rotation & Movement Commands**: The robot adjusts orientation before moving forward, ensuring precise navigation.  
- **Movement Schematic**: Alternates between rotation and translation steps for controlled movement.  
- **Handling Orientation Angle Challenges**: Special handling for discontinuities and wrap-around issues with angular measurements.  

---

### 4. Local Navigation (Obstacle Avoidance)  
- **Proximity Sensors**: The Thymio uses onboard sensors to detect obstacles not visible to the global vision system.  
- **Severity Judgement**: Based on sensor input, the system classifies obstacles by threat level.  
- **Rotation & Circling**: If blocked, the robot performs evasive maneuvers to navigate around obstacles.  
- **Global Navigation Resumption**: Once clear, the robot rejoins its global path.  

---

### 5. Kalman Filter (Sensor Fusion)  
- **Wheel Speed Measurements**: Used to estimate position and velocity.  
- **Camera Input**: Provides absolute localization from vision.  
- **Prediction and Update Step**: Combines odometry and vision to reduce noise and drift.  
- **Handling Orientation Jumps**: Ensures stable estimation even with abrupt orientation changes.  
- **Vision Cut-Off Robustness**: When the camera feed is obstructed, the system continues with odometry predictions until vision is restored.  
- **Choice of Matrices**: Transition, observation, and noise matrices are tuned to balance responsiveness and stability.  

---

## 🎯 Key Features  
- **Robust Navigation**: Handles dynamic changes such as displacement of the robot or goal.  
- **Sensor Fusion**: Integrates vision and odometry for reliable position estimation.  
- **Adaptive Local Navigation**: Reacts in real time to unexpected obstacles.  
- **Scalable Approach**: The modular integration of vision, planning, control, and filtering makes it adaptable to other robotic platforms.  

---

## 📊 Results  
- Successful navigation across obstacle-filled maps.  
- Ability to recover from disturbances such as kidnapping or teleportation of the goal.  
- Reliable trajectory following even during temporary loss of vision.  
