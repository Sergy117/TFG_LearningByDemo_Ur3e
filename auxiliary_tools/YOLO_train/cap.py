# -*- coding: utf-8 -*-
"""
This code belong to https://github.com/Sergy117/TFG_LearningByDemo_Ur3e
Developed by Sergio Gonzalez Rodríguez for Unversity of Santiago de Compostela
owner: meanssergy@gmail.com
date: 2025
"""
"""
Script to capture images simultanously from two Realsense cameras.
's' to save images
'q' to exit
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

# --- Configuration ---
#Directories config
IMG_WIDTH = 640
IMG_HEIGHT = 480
FPS = 30

BASE_OUTPUT_FOLDER = "doll_images"
output_folder_path = os.path.join(BASE_OUTPUT_FOLDER)

os.makedirs(output_folder_path, exist_ok=True)
print(f"Saving images in : {output_folder_path}")
img_counter = 0
#Camera configuration
SERIAL_CAM1 = '153222070290'  
SERIAL_CAM2 = '153222070548' 
pipeline1 = rs.pipeline()
config1 = rs.config()
config1.enable_device(SERIAL_CAM1)
config1.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)

pipeline2 = rs.pipeline()
config2 = rs.config()
config2.enable_device(SERIAL_CAM2)
config2.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
#Camera start
try:
    print("Starting cameras...")
    profile1 = pipeline1.start(config1)
    profile2 = pipeline2.start(config2)
    print("Cameras started")
    for _ in range(30):
        pipeline1.wait_for_frames()
        pipeline2.wait_for_frames()
    print("Ready for capture.")

except Exception as e:
    print(f"Failed to start the cameras {e}")
    print("Verify connection and serial numbers")
    exit()

# --- Capturing loop ---
try:
    while True:
        frames1 = pipeline1.wait_for_frames()
        frames2 = pipeline2.wait_for_frames()

        if not frames1 or not frames2:
            print("Failed frames")
            continue

        # Get color frames
        color_frame1 = frames1.get_color_frame()
        color_frame2 = frames2.get_color_frame()

        if not color_frame1 or not color_frame2:
            print("Color frames not valid")
            continue

        # numpy arrays (format BGR)
        image1 = np.asanyarray(color_frame1.get_data())
        image2 = np.asanyarray(color_frame2.get_data())
        cv2.imshow('Camera 1 - Press S to Save, Q to Quit', image1)
        cv2.imshow('Camera 2 - Press S to Save, Q to Quit', image2)
        key = cv2.waitKey(1) & 0xFF #Wait to key pressed

        #  's' for saving images
        if key == ord('s'):
            img_counter += 1
            ts_file = datetime.now().strftime("%H%M%S_%f")[:-3] # Using timestamp for securing unique name for files
            filename1 = f"img_{img_counter:04d}_cam1_{ts_file}.png"
            filename2 = f"img_{img_counter:04d}_cam2_{ts_file}.png"

            filepath1 = os.path.join(output_folder_path, filename1)
            filepath2 = os.path.join(output_folder_path, filename2)

            # Save images in the paths
            cv2.imwrite(filepath1, image1)
            cv2.imwrite(filepath2, image2)

            print(f"Saved! Pair #{img_counter}: {filename1}, {filename2}")

        # If 'q' pressed, exit
        elif key == ord('q'):
            print("Exit...")
            break

finally:
    print("Stopping cámaras...")
    try:
        pipeline1.stop()
    except RuntimeError as e:
        print(f"Error pipeline 1: {e}")
    try:
        pipeline2.stop()
    except RuntimeError as e:
        print(f"Error  pipeline 2: {e}")
    cv2.destroyAllWindows()
    print("Finished")