
 README - Multi-Camera Vision System for Action Segmentation (LfD)


Developed by: Sergio González Rodriguez (meanssergy@gmail.com)
Last Updated: 2025-05-03 (Please update as needed)
Licensed by: Santiago de Compostela University

![demoj_Hpm9b2og(2)](https://github.com/user-attachments/assets/72f40a9d-7ff2-4484-9ad2-afc4bc897b5c)

--------------------
1. Introduction
--------------------
This project implements a computer vision system using two Intel RealSense cameras
to capture human demonstrations for Learning from Demonstration (LfD). It tracks
human pose (shoulders, elbows, wrists) and detailed hand landmarks, alongside
detecting and locating a target object using a custom-trained YOLOv8 model (via ONNX).

The system analyzes the spatial relationship (distance) and hand orientation (wrist rotation)
relative to the target object to automatically segment the captured motion into distinct
action phases (e.g., Idle, Grasping, Rotating, Moving Away).

The primary output is segmented 3D trajectory data for selected human joints, saved as
CSV files, intended as input for training robotic policies (e.g., for a UR3 arm performing
the demonstrated task, such as demolding a doll head).

The system utilizes the following key libraries:
- pyrealsense2: For RealSense camera interfacing.
- OpenCV (cv2): For image processing, 3D triangulation, Optical Flow, and ONNX inference.
- MediaPipe: For robust human pose and hand landmark detection.
- NumPy: For numerical operations.
- Open3D: For real-time 3D visualization.
- Ultralytics YOLO: For training the object detector (inference uses OpenCV DNN).

--------------------
2. Prerequisites
--------------------

Before running the main script (`main_script_clean.py`), ensure the following prerequisites are met:
 
2.1. Hardware:
    - Two Intel RealSense Depth Cameras (tested with D435, ensure correct serial numbers are set in the script).
    - A capable computer (CPU/RAM). A dedicated NVIDIA GPU with CUDA correctly installed is highly recommended for real-time performance, especially for MediaPipe and potentially ONNX inference.
    - The target object (e.g., the specific toy doll) used for training the YOLO model.
 
2.2. Software / Libraries:
    - Python (e.g., 3.10 or compatible version).
    - It is strongly recommended to use a Python virtual environment (like `venv` or `conda`).
    - Install required libraries:
      ```bash
      pip install opencv-python numpy mediapipe pyrealsense2 open3d onnx onnxruntime # or onnxruntime-gpu
      # You might need specific versions depending on your system/CUDA.
      # Ultralytics is needed ONLY for re-training YOLO, not for running the main script.
      # pip install ultralytics
      ```

2.3. MANDATORY: Camera Calibration:
    - **This step is essential for accurate 3D reconstruction.** You MUST calibrate the stereo camera pair before running the main script.
    - **Refer to the `calibration/` directory** (Please update this path if needed: /home/sergio/Escritorio/PracticasTFG/Deprecated/Multi-Camera_ActionRecognition/cal/) provided alongside this project.
    - Inside that directory, you will find Python scripts and instructions (likely in comments or a separate README within that folder) on how to capture calibration images (e.g., using a chessboard pattern) and compute the necessary parameters.
    - **Expected Output Files:** The calibration process should generate the following `.npy` files within the `calibration/data/` subfolder (or the path specified in `main_script_clean.py`):
        - `camera_matrix_cam1.npy`, `dist_coeffs_cam1.npy`
        - `camera_matrix_cam2.npy`, `dist_coeffs_cam2.npy`
        - `R.npy` (Rotation matrix from Cam1 to Cam2)
        - `T.npy` (Translation vector from Cam1 to Cam2)
    - The main script (`main_script_clean.py`) loads these files to calculate the projection matrices (P1, P2) needed for `cv2.triangulatePoints`. Ensure the path in the main script points to the correct location of these `.npy` files.

--------------------
3. Configuration
--------------------

Before running `main_script_clean.py`, verify the following configuration variables near the beginning of the script:

* `ONNX_MODEL_PATH`: **Crucial.** Path to the trained `.onnx` object detection model file (e.g., `.../weights/best.onnx`).
* `CLASSES_PATH`: **Crucial.** Path to the text file containing the object class names (one per line, in the order corresponding to training indices 0, 1, ...). Example: `./models/doll.names`.
* `SERIAL_CAM1`, `SERIAL_CAM2`: Serial numbers of your two RealSense cameras.
* `TARGET_CLASS_FOR_INTERACTION`: The string name (must match an entry in `CLASSES_PATH`) of the object part used for distance/interaction calculations (e.g., `"doll_body"`).
* `HAND_TO_MONITOR`: Which hand ("Left" or "Right") triggers the state machine transitions based on its interaction with the target object.
* `GRASP_DISTANCE_THRESHOLD`: Distance (in meters, based on triangulated coordinates) below which the monitored hand is considered to be "grasping" the target object. Needs tuning.
* `ROTATION_THRESHOLD_DEGREES`: Angle difference (in degrees) required to transition from "Grasping" to "Rotating" state. Needs tuning.
* `CONFIDENCE_THRESHOLD_YOLO`, `NMS_THRESHOLD_YOLO`: Confidence and Non-Maximum Suppression thresholds for the YOLO object detector.
* `calibration_dir`: Path to the directory containing the camera calibration `.npy` files.

--------------------
4. Running the Main Script
--------------------

1.  Ensure all prerequisites (Hardware, Software, **Calibration**) are met.
2.  Verify the configuration variables in `main_script_clean.py`.
3.  Activate your Python virtual environment.
4.  Navigate to the script's directory in your terminal.
5.  Run the script:
    ```bash
    python main_script_clean.py
    ```
6.  Two OpenCV windows will appear showing the live feeds from Camera 1 and Camera 2, with overlays for pose, hands, and object detections.
7.  An Open3D window will appear showing the live 3D visualization of the pose, hands, and the target object's bounding box.
8.  The console will print status messages, including state transitions ("Idle" -> "Grasping", etc.) and when trajectory segments are saved.
9.  Perform the demonstrated action (e.g., grasp, rotate, move away) with the monitored hand relative to the target object part.
10. Press 'q' (with one of the OpenCV windows active) to stop the script. Any data in the active trajectory buffer will be saved as a final segment.

--------------------
5. Understanding the Output
--------------------

* **Live Visualizations:** The 2D and 3D windows provide real-time feedback on what the system is detecting and tracking.
* **Console Output:** Shows state transitions and saving events. Check for any error messages.
* **Segmented Trajectories:** The primary output is saved in a timestamped folder (e.g., `action_segments_YYYYMMDD_HHMMSS/`). Inside this folder, you will find CSV files for each recorded segment.
    * **Filename Format:** `[joint_name]_segment_[segment_number]_[phase_name]_[timestamp].csv`
        * `joint_name`: e.g., "right_hand", "left_elbow".
        * `segment_number`: A sequential number (001, 002, ...) indicating the order of segments in the run.
        * `phase_name`: Indicates the action phase that *just completed* when this file was saved (e.g., "Initial-Idle", "Grasping-to-Rotating", "Rotating-MovingAway", "Final-Idle").
        * `timestamp`: Time when the segment was saved.
    * **CSV Content:** Each file contains the 3D coordinates (x, y, z) of the specified joint over time during that action segment, with one row per frame and a header 'x,y,z'.

-------------------------------------------------
6. Optional: Fine-tuning YOLO for Custom Objects
-------------------------------------------------

This system uses a fine-tuned YOLOv8 model (exported to ONNX) for object detection. If you need to detect a different object or improve detection for the current one, you need to re-train/fine-tune the model.

**Utility:** This allows the vision system to be adapted to new tasks and objects required for robot demonstrations.

**Steps:**

1.  **Data Collection:**
    * Gather numerous images and/or short video clips of your **new target object(s)**.
    * **Crucially, use the same RealSense cameras** you will use for the main script, capturing data in realistic conditions (varying angles, lighting, backgrounds, distances).
    * Include images where the object is partially occluded by hands or a robot gripper if that's expected during the task.
    * Aim for at least 100-200+ diverse images per object class.

2.  **Data Annotation:**
    * Use an annotation tool (e.g., CVAT, LabelImg, Roboflow).
    * Define your new class names (e.g., `new_object_part1`, `new_object_part2`). Remember which index corresponds to which name (0, 1, ...).
    * Draw tight bounding boxes around every instance of your target object(s) in all collected images.
    * **Export annotations in YOLO `.txt` format.** Each image `image.png` will have a corresponding `image.txt` file. Each line in the `.txt` file is `class_index center_x center_y width height` (normalized coordinates 0-1).

3.  **Dataset Organization:**
    * Split your annotated data (images AND corresponding `.txt` label files) into `train` and `valid` sets (e.g., 80% train, 20% valid).
    * Organize them into the following structure:
      ```
      <your_dataset_folder>/
      ├── images/
      │   ├── train/  (contains train images .png/.jpg)
      │   └── valid/  (contains validation images .png/.jpg)
      └── labels/
          ├── train/  (contains train labels .txt)
          └── valid/  (contains validation labels .txt)
      ```

4.  **Create `data.yaml` File:**
    * Create a YAML text file (e.g., `new_object_dataset.yaml`) describing your dataset:
      ```yaml
      # Path to the main dataset folder created above
      path: /path/to/your_dataset_folder

      # Relative paths to image directories
      train: images/train
      val: images/valid

      # Class information
      nc: <number_of_classes> # e.g., 2 if detecting two parts
      names: ['<class_name_for_index_0>', '<class_name_for_index_1>'] # MUST match indices in .txt files!
      ```
    * Replace paths and class info accordingly.

5.  **Environment Setup:**
    * Ensure you have `pytorch` and `ultralytics` installed (`pip install ultralytics`). A GPU is highly recommended.

6.  **Run Training (Fine-tuning):**
    * Use the Ultralytics training interface (CLI or Python). Start from a pre-trained model (e.g., `yolov8n.pt`).
    * **CLI Example:**
      ```bash
      yolo train data=/path/to/new_object_dataset.yaml model=yolov8n.pt epochs=100 imgsz=640 batch=8 device=0 name=new_object_run1
      ```
      (Adjust `epochs`, `batch`, `device`, `name` as needed).
    * **Python Example:** Refer to the `train_doll_yolo.py` script structure provided previously, adapting paths and parameters.
    * Training results will be saved in `runs/detect/<name>/`. Look for `weights/best.pt`.

7.  **Export to ONNX:**
    * Export the best model found during training. Using `simplify=True` is highly recommended.
    * **CLI Example:**
      ```bash
      yolo export model=runs/detect/new_object_run1/weights/best.pt format=onnx imgsz=640 simplify=True opset=12
      ```
    * **Python Example:** Refer to the `export_onnx.py` script provided previously.
    * This creates the `best.onnx` file (or similar).

8.  **Update Main Script:**
    * In `main_script_clean.py`, update the following configuration variables:
        * `ONNX_MODEL_PATH`: Path to your newly exported `.onnx` file.
        * `CLASSES_PATH`: Path to a new `.names` file containing your new class names (one per line, matching the order in the YAML `names:` list).
        * `CLASS_NAMES_YOLO` (loaded from `CLASSES_PATH`): Ensure this reflects the new classes.
        * `TARGET_CLASS_FOR_INTERACTION`: Update this string to the specific class name you want to use for distance calculations and triangulation in the state machine.
        * Update `COLORS_YOLO` generation if the number of classes changed.

--------------------
7. Utilities
--------------------

* **`vision_utils.py`:** Contains the core helper functions for detection, tracking, geometry, etc.
* **`calibration/` Directory:** Contains scripts and instructions for performing the mandatory stereo camera calibration.

---
End of README
---
