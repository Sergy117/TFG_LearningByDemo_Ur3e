import cv2
import numpy as np
import os
import time
import pyrealsense2 as rs
import onnxruntime # <<< Import ONNX Runtime
"""
This code belong to https://github.com/Sergy117/TFG_LearningByDemo_Ur3e
Developed by Sergio Gonzalez Rodríguez for Unversity of Santiago de Compostela
owner: meanssergy@gmail.com
date: 2025
"""
"""
This script is used to test onnx model on live video
"""
# --- Configuration ---
ONNX_MODEL_PATH = 'best.onnx' 

SERIAL_CAM1 = '153222070290'  
SERIAL_CAM2 = '153222070548'  
IMG_WIDTH = 640
IMG_HEIGHT = 480
FPS = 30

CONFIDENCE_THRESHOLD = 0.3  # threshold to detect doll
NMS_THRESHOLD = 0.4         # threshold Non-Maximum Suppression
INPUT_WIDTH = 640           # These sould match with imgsz used in exporting the model
INPUT_HEIGHT = 640          # 

#Names of the classes to detect, the order should match the data.yaml order and the training order
CLASS_NAMES = ['doll_head', 'doll_body']
# Colors for classes
COLORS = [(0, 255, 0), (0, 0, 255)] # green for head, blue for body

#Function for post processing the same one for the output of onnx
def postprocess_yolo_output(outputs, original_width, original_height, conf_threshold, nms_threshold):
    """Process raw output of model YOLOv8 ONNX."""

    #The ouput shoud be a list, we take the first element that should have shape [batch, num_props, num_detections], ej [1, 6, 8400]
    outputs = outputs[0].T # Traspose to [num_detections, num_props]
    boxes, confidences, class_ids = [], [], []
    for row in outputs:
        box_probs = row[4:] #Confidence of classes start at column 4
        class_id = np.argmax(box_probs)
        confidence = box_probs[class_id]
        if confidence > conf_threshold:
            cx, cy, w, h = row[:4] # Normalize center
            # Desnormalice coordinates to original image
            center_x = int(cx * original_width)
            center_y = int(cy * original_height)
            width = int(w * original_width)
            height = int(h * original_height)
            x1 = int(center_x - width / 2)
            y1 = int(center_y - height / 2)
            boxes.append([x1, y1, width, height])
            confidences.append(float(confidence))
            class_ids.append(class_id)
    # Non-Maximum Suppression
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    final_boxes, final_confidences, final_class_ids = [], [], []
    if len(indices) > 0:
        for i in indices.flatten():
            final_boxes.append(boxes[i])
            final_confidences.append(confidences[i])
            final_class_ids.append(class_ids[i])
    return final_boxes, final_confidences, final_class_ids

if __name__ == "__main__":
    #Load ONNX Model
    if not os.path.exists(ONNX_MODEL_PATH):
        print(f"Error: No se encuentra el archivo ONNX en {ONNX_MODEL_PATH}")
        exit()
    try:
        # Try to use CUA if aviable, if not, uses cpu
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        ort_session = onnxruntime.InferenceSession(ONNX_MODEL_PATH, providers=providers)
        # Verify what is actually using
        selected_provider = ort_session.get_providers()[0]
        print(f"ONNX loaded, using:  {selected_provider}")
        # Obtain name of the model 'images'
        input_name = ort_session.get_inputs()[0].name
        print(f"model name {input_name}")
    except Exception as e:
        print(f"Failed loading of ONNX with ONNX Runtime: {e}")
        print("Verify having 'onnxruntime' or 'onnxruntime-gpu' instaled.")
        print("If using GPU, verify compatibility of CUDA/cuDNN with onnxruntime-gpu.")
        exit()

    #Realsense cameras for video testing
    pipeline1 = rs.pipeline()
    config1 = rs.config()
    config1.enable_device(SERIAL_CAM1)
    config1.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
    pipeline2 = rs.pipeline()
    config2 = rs.config()
    config2.enable_device(SERIAL_CAM2)
    config2.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
    try:
        print("Initiating Realsense Cameras")
        profile1 = pipeline1.start(config1)
        profile2 = pipeline2.start(config2)
        print("Cameras started correctly")
        print("\nPress 'q' for exit")
    except Exception as e:
        print(f"Error RealSense: {e}")
        exit()

    try:
        while True:
            start_time = time.time()

            try:
                frames1 = pipeline1.wait_for_frames()
                frames2 = pipeline2.wait_for_frames()
                if not frames1 or not frames2: continue
                color_frame1 = frames1.get_color_frame()
                color_frame2 = frames2.get_color_frame()
                if not color_frame1 or not color_frame2: continue
                image1 = np.asanyarray(color_frame1.get_data())
                image2 = np.asanyarray(color_frame2.get_data())
            except RuntimeError as e:
                 print(f"Error : {e}")
                 time.sleep(0.5)
                 continue

            processed_image1 = image1.copy() # Copy for drawing on top of the image
            processed_image2 = image2.copy()

            # Camera 1
            original_h1, original_w1 = processed_image1.shape[:2]
            # Preprocess
            blob1 = cv2.dnn.blobFromImage(processed_image1, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
            # Inference with ONNX
            ort_inputs1 = {input_name: blob1}
            try:
                ort_outputs1 = ort_session.run(None, ort_inputs1) # Execute inference
                # Postprocess
                boxes1, confidences1, class_ids1 = postprocess_yolo_output(ort_outputs1[0], original_w1, original_h1, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
                # Draw results
                for i in range(len(boxes1)):
                    box = boxes1[i]; x1, y1, w, h = box[0], box[1], box[2], box[3]
                    conf = confidences1[i]; class_id = class_ids1[i]
                    class_name = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else "???"
                    color = COLORS[class_id] if 0 <= class_id < len(COLORS) else (255, 255, 255)
                    cv2.rectangle(processed_image1, (x1, y1), (x1 + w, y1 + h), color, 2)
                    label = f"{class_name}: {conf:.2f}"
                    (lw, lh), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(processed_image1, (x1, y1 - lh - base), (x1 + lw, y1), color, cv2.FILLED)
                    cv2.putText(processed_image1, label, (x1, y1 - base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            except Exception as e:
                 print(f"Error ONNX Runtime (Cam 1): {e}")

            # Camera 2
            original_h2, original_w2 = processed_image2.shape[:2]
            # Preprocess
            blob2 = cv2.dnn.blobFromImage(processed_image2, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
            # Inference with ONNX
            ort_inputs2 = {input_name: blob2}
            try:
                ort_outputs2 = ort_session.run(None, ort_inputs2)
                # Postprocess
                boxes2, confidences2, class_ids2 = postprocess_yolo_output(ort_outputs2[0], original_w2, original_h2, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
                # Draw results
                for i in range(len(boxes2)):
                    box = boxes2[i]; x1, y1, w, h = box[0], box[1], box[2], box[3]
                    conf = confidences2[i]; class_id = class_ids2[i]
                    class_name = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else "???"
                    color = COLORS[class_id] if 0 <= class_id < len(COLORS) else (255, 255, 255)
                    cv2.rectangle(processed_image2, (x1, y1), (x1 + w, y1 + h), color, 2)
                    label = f"{class_name}: {conf:.2f}"
                    (lw, lh), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(processed_image2, (x1, y1 - lh - base), (x1 + lw, y1), color, cv2.FILLED)
                    cv2.putText(processed_image2, label, (x1, y1 - base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            except Exception as e:
                 print(f"Error ONNX Runtime (Cam 2): {e}")


            # FPS
            end_time = time.time()
            fps = 1 / (end_time - start_time) if (end_time - start_time) > 0 else 0
            cv2.putText(processed_image1, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("RealSense Cam 1 - ONNX Runtime Test (Q to Quit)", processed_image1)
            cv2.imshow("RealSense Cam 2 - ONNX Runtime Test (Q to Quit)", processed_image2)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        
        print("Stopping cameras...")
        try: pipeline1.stop()
        except RuntimeError as e: print(f"Error pipeline 1: {e}")
        try: pipeline2.stop()
        except RuntimeError as e: print(f"Error pipeline 2: {e}")
        cv2.destroyAllWindows()
        print("Finish.")