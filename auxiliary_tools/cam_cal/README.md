# Camera Calibration for stereo
This document outlines the purpose, requirements, and usage of the CalibrationStereo.py script, designed to perform a full stereo calibration for a pair of Intel RealSense cameras.
The script consist on 3 steps:
1. Capturing images
    * 20 pair of images captured
    * Images of each camera saved on /images
2. Individual Calibration
    * Intrinsic parameters for each camera calculated using captured images
3. Stereo Calibration
    * Using parameters saved, extrinsic parameters calculated
    * Rotation and Traslation Matrix between cameras saved on /data as .npy files
---
## Hardware
Cameras: 2 x Intel RealSense cameras.
Calibration Pattern: A physical chessboard with the following specifications:
* **Board Size**: 9x6 internal corners.
* **Square Size**: 2.5 cm (0.025 meters).
---
## Software
Python 3.x
Required Python libraries: numpy, opencv-python, pyrealsense2.
For installing this libraries:
```sh
    pip install numpy opencv-python pyrealsense2
```

Matrix data of individual parameters of each camera and Stereo parameters saved in /data
---
Before using the script CalibrationStereo.py:
1. Connect your cameras to the pc.
2. Prepare your chessboard, preferly printed on a carboard or rigid surface.
3. Be sure of visibility of the chessboard in both cameras. Failed images are discarded and shown to user. 'Q' to pass them when prompted.
4. Modify parameters of your chessboard on the first lines of the script, adapt rows, columns, and square size
```sh
    chessboard_size = (9, 6)  # Number of squares in each row and column
    square_size = 0.025  # Size of each square in the chessboard
```
5. Modify serial numbers of your own realsense cameras.
    * Serial number of each camera
```sh
    * Serial_Cam1 = '153222070290'
    * Serial_Cam2 = '153222070548'
```
--- 
For runing the script:
---
python3 CalibrationStereo.py

Follow the on-screen prompts. The script will loop 20 times. For each iteration:

You will have 5 seconds to position the chessboard in front of both cameras.

Important: For a high-quality calibration, show the board in a wide variety of positions, distances, and angles, making sure it's fully visible in both camera views.

The script will automatically capture and save the images.
---
The script will generate two folders (images/ and data/) if they don't exist. The final calibration parameters will be saved in the data/ folder:

camera_matrix_camX.npy: The intrinsic matrix for the camera. Relates 3D world points to 2D image pixels.

dist_coeffs_camX.npy: The distortion coefficients for the camera's lens. Used to correct optical aberrations.

R.npy: The Rotation Matrix (3x3). Describes the orientation of Camera 2 relative to Camera 1's coordinate system.

T.npy: The Translation Vector (3x1). Describes the position of Camera 2 relative to Camera 1's coordinate system, in meters.

These files are the essential inputs for the 3D triangulation and computer vision tasks in the main project.
---
