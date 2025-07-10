"""
This code belong to https://github.com/Sergy117/TFG_LearningByDemo_Ur3e
Developed by Sergio Gonzalez Rodríguez for Unversity of Santiago de Compostela
owner: meanssergy@gmail.com
date: 2025
"""


import numpy as np
import cv2
import os
import pyrealsense2 as rs
import glob

# Chessboard parameters
chessboard_size = (9, 6)  # Number of squares in each row and column
square_size = 0.025  # Size of each square in the chessboard

"""
Initailization of the pipelines for using realsense cameras
"""
pipeline_1 = rs.pipeline()
pipeline_2 = rs.pipeline()

config_1 = rs.config()
config_2 = rs.config()

#Serial number of each camera
Serial_Cam1 = '153222070290'
Serial_Cam2 = '153222070548'
config_1.enable_device(Serial_Cam1)
config_2.enable_device(Serial_Cam2)

config_1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config_1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config_2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline_1.start(config_1)
pipeline_2.start(config_2)



print("------------------------------------------ Own parameters given by realsense library  ---------------------------------------------")
# Get info of the first camera
profile_1 = pipeline_1.get_active_profile()

# Get color and depth information from the profile
depth_sensor_1 = profile_1.get_device().first_depth_sensor()
depth_scale_1 = depth_sensor_1.get_depth_scale()
# Get extrinsic from color and depth
depth_to_color_extrinsics_1 = profile_1.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(profile_1.get_stream(rs.stream.color))
print("Rotation matrix (3x3) camera 1 pipeline:")
print(depth_to_color_extrinsics_1.rotation)
print("\nTraslation matrix (3x1)camera 1 pipeline:")
print(depth_to_color_extrinsics_1.translation)

#Same process for second camera
profile_2 = pipeline_2.get_active_profile()
depth_sensor_2 = profile_2.get_device().first_depth_sensor()
depth_scale_2 = depth_sensor_2.get_depth_scale()
depth_to_color_extrinsics_2 = profile_2.get_stream(rs.stream.depth).as_video_stream_profile().get_extrinsics_to(profile_2.get_stream(rs.stream.color))
print("Rotation matrix (3x3) camara 2 pipeline:")
print(depth_to_color_extrinsics_2.rotation)
print("\nTraslation matrix (3x1)camara 2 pipeline:")
print(depth_to_color_extrinsics_2.translation)

"""
-------------------------------------------- Capturing and saving of the images with chessboard pattern for manual calibration -----------------------------
"""

print("-------------------------------------------- Capturing and saving of the images with chessboard pattern for manual calibration -----------------------------")

# Create directories to store images
img_dir_1 = "./images/cam1"
img_dir_2 = "./images/cam2"
if not os.path.exists(img_dir_1):
    os.makedirs(img_dir_1)
if not os.path.exists(img_dir_2):
    os.makedirs(img_dir_2)

img = 0
while img < 20: # 20 images for calibration, as mentioned in the official cv2 library advise
    print("Prepare for taking images, positionate the chessboard")
    key = cv2.waitKey(5000)  # Wait 5 seconds
    print("GO")
    #Get frames from both cameras
    frames_1 = pipeline_1.wait_for_frames()
    frames_2 = pipeline_2.wait_for_frames()
    color_frame_1 = frames_1.get_color_frame()
    color_frame_2 = frames_2.get_color_frame()
    #Convert frames into numpy arrays
    color_image_1 = np.asanyarray(color_frame_1.get_data())
    color_image_2 = np.asanyarray(color_frame_2.get_data())
    #Show images
    cv2.imshow("img1", color_image_1)
    cv2.imshow("img2", color_image_2)

    key = cv2.waitKey(1000)  # Wait 1 second
    if key == ord('q'):  # Quit if you press 'q'
        break
    else:
        #Save the images in the directories
        cv2.imwrite(f'{img_dir_1}/{img}.png', color_image_1)
        cv2.imwrite(f'{img_dir_2}/{img}.png', color_image_2)
        print(f"Images saved as {img_dir_1}/{img}.png")
        img += 1

"""
-------------------------------------------- Use the images with the chessboard for calibration-----------------------------
"""
print ("--------------------------------------------Use the images with the chessboard for calibration------------------------------")
# Directory containing the calibration images
image_files_1 = [os.path.join(img_dir_1, f) for f in os.listdir(img_dir_1) if f.endswith(".png")]
# Arrays to store object points and image points
objpoints_1 = []  # 3D points in real world space
imgpoints_1 = []  # 2D points in image plane
# Prepare object points (3D points in real world space)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size
# Loop through the images and detect chessboard corners
for image_file in image_files_1:
    print("Procesando imagenes cam 1")
    # Load the image
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        # Refine corner locations
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        # Add object points and image points
        objpoints_1.append(objp)
        imgpoints_1.append(corners_refined)

        # Draw and display the corners
        cv2.drawChessboardCorners(image, chessboard_size, corners_refined, ret)
        cv2.imshow("Chessboard Corners", image)
        cv2.waitKey(500)  # Pause to show the image
    else:
        print(f"Chessboard not found in {image_file}. Displaying image for debugging...")
        cv2.imshow("Failed Image", image)
        cv2.waitKey(0)  # Wait for a key press to continue

cv2.destroyAllWindows()

# Calibrate the camera
if len(objpoints_1) > 0:
    print("Calibracion camara 1")
    ret, camera_matrix_1, dist_coeffs_1, rvecs_1, tvecs_1 = cv2.calibrateCamera(objpoints_1, imgpoints_1, gray.shape[::-1], None, None)

    if ret:
        print("Camera matrix (intrinsics):")
        print(camera_matrix_1)

        print("\nDistortion coefficients:")
        print(dist_coeffs_1)

        # Compute extrinsic parameters for the last image
        rvec_1 = rvecs_1[-1]  # Rotation vector
        tvec_1 = tvecs_1[-1]  # Translation vector

        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec_1)

        # Extrinsic matrix
        extrinsic_matrix_1 = np.hstack((R, tvec_1))
        extrinsic_matrix_1 = np.vstack((extrinsic_matrix_1, [0, 0, 0, 1]))

        print("\nExtrinsic matrix:")
        print(extrinsic_matrix_1)
        np.save("data/camera_matrix_cam1.npy", camera_matrix_1)
        np.save("data/dist_coeffs_cam1.npy", dist_coeffs_1)
        np.save("data/extrinsic_matrix_cam1.npy", extrinsic_matrix_1)
        print("Calibration parameters saved.")
    else:
        print("Calibration failed.")
else:
    print("No valid images for calibration.")

# Directory containing the calibration images
image_files_2 = [os.path.join(img_dir_2, f) for f in os.listdir(img_dir_2) if f.endswith(".png")]
# Arrays to store object points and image points
objpoints_2 = []  # 3D points in real world space
imgpoints_2 = []  # 2D points in image plane

# Loop through the images and detect chessboard corners
for image_file in image_files_2:
    print("Procesando imagenes cam 2")
    # Load the image
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret:
        # Refine corner locations
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        # Add object points and image points
        objpoints_2.append(objp)
        imgpoints_2.append(corners_refined)

        # Draw and display the corners
        cv2.drawChessboardCorners(image, chessboard_size, corners_refined, ret)
        cv2.imshow("Chessboard Corners", image)
        cv2.waitKey(500)  # Pause to show the image
    else:
        print(f"Chessboard not found in {image_file}. Displaying image for debugging...")
        cv2.imshow("Failed Image", image)
        cv2.waitKey(0)  # Wait for a key press to continue

cv2.destroyAllWindows()

# Calibrate the camera
if len(objpoints_2) > 0:
    print("Calibracion camara 2")
    ret, camera_matrix_2, dist_coeffs_2, rvecs_2, tvecs_2 = cv2.calibrateCamera(objpoints_2, imgpoints_2, gray.shape[::-1], None, None)

    if ret:
        print("Camera matrix (intrinsics):")
        print(camera_matrix_2)

        print("\nDistortion coefficients:")
        print(dist_coeffs_2)

        # Compute extrinsic parameters for the last image
        rvec = rvecs_2[-1]  # Rotation vector
        tvec = tvecs_2[-1]  # Translation vector

        # Convert rotation vector to rotation matrix
        R, _ = cv2.Rodrigues(rvec)

        # Extrinsic matrix
        extrinsic_matrix = np.hstack((R, tvec))
        extrinsic_matrix = np.vstack((extrinsic_matrix, [0, 0, 0, 1]))

        print("\nExtrinsic matrix:")
        print(extrinsic_matrix)
        np.save("data/camera_matrix_cam2.npy", camera_matrix_2)
        np.save("data/dist_coeffs_cam2.npy", dist_coeffs_2)
        np.save("data/extrinsic_matrix_cam2.npy", extrinsic_matrix)
        print("Calibration parameters saved.")
    else:
        print("Calibration failed.")
else:
    print("No valid images for calibration.")

"""------------------------------------------------------ Individual calibration done, starting Stereo Calibration -----------------------"""

print("------------------------------------------------------Individual calibration done, starting Stereo Calibration ------------------------")

objpoints = []  # 3D points
imgpoints_1 = []  # 2D points for camera 1
imgpoints_2 = []  # 2D points for camera 2
# load images
images_left = glob.glob('images/cam1/*.png')
images_right = glob.glob('images/cam2/*.png')

for img_left, img_right in zip(images_left, images_right):
    imgL = cv2.imread(img_left, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(img_right, cv2.IMREAD_GRAYSCALE)

    retL, cornersL = cv2.findChessboardCorners(imgL, chessboard_size, None)
    retR, cornersR = cv2.findChessboardCorners(imgR, chessboard_size, None)

    if retL and retR:
        objpoints.append(objp)
        imgpoints_1.append(cornersL)
        imgpoints_2.append(cornersR)

# Load intrinsic parameters for each camera
M1 = camera_matrix_1
dist1 = dist_coeffs_1

M2 = camera_matrix_2
dist2 = dist_coeffs_2

# Stereo calibration using cv2
retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_1, imgpoints_2, M1, dist1, M2, dist2,
    imgL.shape[::-1], criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
    flags=cv2.CALIB_FIX_INTRINSIC
)

# Saving data 
np.save('data/R.npy', R)
np.save('data/T.npy', T)
#Optional prints of the matrix saved
print("Rotación (R):\n", R)
print("Traslación (T):\n", T)
#Stopping cameras
pipeline_1.stop()
pipeline_2.stop()