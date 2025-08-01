# Autonomous Drone for Waypoint Navigation and Object Identification

This repository contains the code and documentation for an autonomous drone project developed as part of the **e-Yantra competition** at **IIT Bombay**. The project's core mission was to create a drone capable of **autonomous waypoint navigation**, **real-time ground element identification**, and **systematic data reporting**.

---

## 🚀 Project Overview

The project successfully delivered a complete autonomous system. It features a custom object detection pipeline for recognizing ground elements, a fine-tuned PID controller for stable and accurate navigation, and a well-defined scouring strategy to explore and map its environment. The solution was validated in both a ROS2-Gazebo simulation and a real-world physical deployment.

The solution was validated in both a **ROS2-Gazebo simulation** and a **real-world physical deployment**.

---

## 🔑 Key Features

### 🎯 Object Detection
- Developed a robust object detection pipeline using **Python** and **OpenCV**.
- Accurately identifies and computes centroids of specific ground elements from the drone's live camera feed.

### 🧭 Waypoint Navigation
- Implemented and extensively tuned a **PID controller** for precise waypoint navigation.
- Validated performance in **ROS2-Gazebo** simulation before physical deployment.

### 📡 Scouring Strategy
- Designed a methodical **scouring algorithm** for systematic area exploration.
- Ensures identification and location reporting of all unknown elements to a control center.

---

## 🧰 Technical Stack

- **Programming Language:** Python  
- **Robotics Frameworks:** ROS2, Gazebo  
- **Computer Vision Library:** OpenCV  
- **Control Systems:** PID Controller  

---
