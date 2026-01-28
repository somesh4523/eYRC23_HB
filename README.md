# Hi there, I'm Somesh! 👋

I am a PhD Scholar specializing in Control and Automation.

## 🛠️ Tech Stack
* **Languages:** Python, C++, MATLAB
* **Tools:** Simulink, ROS2, Git, Linux
* **Interests:** Control Theory, Robotics, Automation

---

## 🔐 Research Projects
*Below is a summary of my projects.*

### 🚀 Hologlyph Bots
**eYantra_25-26**
**ROS2, Gazebo, OpenCV, EasyEDA**
Using multiple nonholonomic robots to create large-scale designs or "geoglyphs," involving coordinated swarm robotics for efficiency in tasks like complex pattern formation.
* 1. Holonomic Navigation & Kinematics Control
This task relies on ROS 2 Humble nodes to manage the complex movement of a 3-Wheel Omnidirectional (Holonomic) Drive. Unlike differential drive robots, these bots utilize Inverse Kinematics (IK) matrices to resolve the desired chassis velocity vector $(v_x, v_y, \omega_z)$ into individual wheel angular velocities. The core navigation logic involves implementing a PID Controller to minimize the error between the robot's current pose and the target trajectory points. Velocity commands are published to the /cmd_vel topic using geometry_msgs/Twist messages, enabling simultaneous translation and rotation for smooth, drift-free motion.
* 2. Global Perception & State Estimation Perception is handled using a global "Eye-to-Hand" vision system with OpenCV within a ROS 2 package. The pipeline relies on an Overhead Camera to track the robots' positions in real-time using ArUco Markers. The system employs Perspective Transformation (cv2.getPerspectiveTransform and cv2.warpPerspective) to map the skewed camera feed onto a corrected 2D coordinate system (Homography) of the arena. This provides precise (x, y, \theta) state estimation for each robot, which is published as odometry data to close the feedback loop, compensating for wheel slippage and drift.
* 3. Trajectory Generation & Swarm Coordination The "manipulation" task involves the precise execution of mathematical path planning to "draw" shapes (Glyphs) such as Lissajous figures or geometric polygons. The technical core requires a Trajectory Planner that generates high-resolution waypoints based on parametric equations. This process relies on Multi-Robot Coordination algorithms to synchronize the movement of multiple bots (Swarm) within the shared Gazebo or physical arena, ensuring they maintain formation and avoid collisions while tracing their assigned segments of the composite image.
 
** Video link:

<p align="center">
  <a href="https://youtu.be/t-eeVHw8XE8">
    <img src="https://img.youtube.com/vi/t-eeVHw8XE8/0.jpg" alt="Video">
  </a>
</p>

<p align="center">
  <em>
    [Hardware implementation of the Hologlyph Theme]
  </em>
</p>

---

## 📫 Connect with Me
* [LinkedIn](www.linkedin.com/in/somesh-swami-8b7684219)
* Email: somesh.swami398@gmail.com
