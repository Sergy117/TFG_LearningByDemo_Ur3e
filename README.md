UR3e Robotic Arm Control via Kinematic Mapping from Human Pose Estimation

📖 Project Overview

This repository contains the source code, documentation, and auxiliary tools for a Final Degree Project (TFG) focused on the teleoperation of a UR3e robotic arm in joint space. The system translates the movements of a human operator, captured by a stereo vision system with MediaPipe, into joint state commands for the robot. This approach deliberately avoids real-time Inverse Kinematics (IK) to prioritize low latency and computational efficiency.

The primary goal is to develop and validate a robust kinematic mapping pipeline that serves as a foundation for Learning from Demonstration (LfD) tasks. This is achieved by using Dynamic Movement Primitives (DMPs) to enable the robot to learn and reproduce complex tasks demonstrated by a human operator.

(Suggestion: Add a GIF here showing the final result, such as the Matplotlib comparison or an RViz simulation!)

✨ Key Features

    Stereo Vision System: Utilizes two Intel RealSense cameras for accurate 3D scene reconstruction.

    3D Human Pose Estimation: Implements MediaPipe Pose for real-time detection of 33 body landmarks.

    Custom Object Detection: Integrates a custom-trained YOLOv8 model, exported to the ONNX format, for detecting task-relevant objects.

    Human-to-Robot Kinematic Mapping: A sophisticated "translator" that converts the human operator's pose into robot joint angles, resolving morphological and coordinate system differences.

    Learning from Demonstration (LfD): Employs Dynamic Movement Primitives (DMPs) to learn complex trajectories from multiple human demonstrations.

    Robot Control in ROS: Integrates with ROS Noetic and MoveIt! for safe control of the UR3e robot, both in simulation (Gazebo/RViz) and on the real hardware.

🏗️ System Architecture

The project follows a modular architecture where each component has a clear responsibility. The data flows as follows:

Stereo Cameras → 1. Vision Node (3D Capture & Triangulation) → 2. Offline Learning Pipeline (Translation & DMP Training) → 3. Robot Control Node (DMP Loading & Execution) → UR3e Robot

    Vision System (vision_system/): Captures images, detects human landmarks and objects, and saves the 3D trajectories of the demonstration into .csv files.

    Learning Pipeline (learning_and_validation/): Loads the demonstrations, translates them into the robot's joint space using the methodology from the technical report, and trains the DMP models, saving them as .pkl files.

    Robot Control (robot_control/): Loads the learned DMPs and uses them to generate and execute trajectories on the robot via MoveIt.

📁 Repository Structure

TFG_UR3e_Teleoperation/ 
|       
├── 📄 README.md              # <-- This main README file.      
├── 📄 requirements.txt        # <-- Main Python dependencies.      
|   
├── 🤖 robot_control/   
│   └── src/                 # ROS scripts for controlling the robot (e.g., dmpplayer.py).  
|                  
├── 👁️ vision_system/   
│   └── src/                 # Scripts for capturing demonstrations (e.g., main_script_final.py).   
|                               
├── 🎓 learning_and_validation/                     
│   └── src/                 # Scripts to validate data and train DMPs (e.g., learn_dmp.py).
|                                       
├── 🛠️ auxiliary_tools/                    
│   ├── 1_camera_calibration/  # Scripts and README for stereo calibration.             
│   ├── 2_hand_eye_calibration/ # Scripts and README for camera-robot calibration.              
│   └── 3_yolo_training/     # Scripts and README for the YOLO model training pipeline.
|                                                                                               
├── 📂 data/    
│   ├── calibration_files/   # .npy files from the calibrations.                    
│   └── learned_dmps/        # .pkl files of the learned DMPs.          
│                                                                           
└── 📝 documentation/       
    └── (Images, GIFs, and other resources for the TFG document).   

🚀 Getting Started

To replicate this project, follow the steps in order. Each subfolder in auxiliary_tools and the main directories contain their own README.md with detailed instructions.

    Clone the Repository:
    Bash

    git clone [URL-to-your-repository]
    cd TFG_UR3e_Teleoperation

    Install Dependencies: Install the required libraries as specified in the various requirements.txt files within the subdirectories.

    Calibration (See auxiliary_tools/):

        Step 1: Run the script in 1_camera_calibration to calibrate the stereo system.

        Step 2: Run the scripts in 2_hand_eye_calibration to get the transformation between the camera and the robot.

    Object Detector Training (See auxiliary_tools/):

        Follow the instructions in the 3_yolo_training/README.md to capture images, annotate them, and train your .onnx model.

    Create Demonstration Dataset (See vision_system/):

        Run the main vision script to record a set of task demonstrations.

    Learn the Model (See learning_and_validation/):

        Use the validation script to inspect the quality of your demonstrations.

        Run the learn_dmp.py script to process the dataset and train the DMP models.

    Execute on Robot (See robot_control/):

        Launch the simulation in RViz or the connection to the real robot.

        Run the robot_player_from_dmp.py script to have the robot reproduce the learned task.

👤 Author

Sergio González Rodríguez

    Email: meanssergy@gmail.com

    GitHub: Sergy117

Final Degree Project for the University of Santiago de Compostela (USC) - 2025.