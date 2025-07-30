---
Custom YOLO Model Training for Doll Detection
---
## Description

This folder contains the complete pipeline and all necessary scripts to train a custom YOLOv8 object detection model. The model is fine-tuned to detect two specific classes required for the main robotic task: doll_head and doll_body.

The final output of this process is a best.onnx file, a lightweight and framework-independent model format used for real-time inference in the main vision system.

## Methodology / Workflow

* The end-to-end process for creating the custom detector follows these steps:

    * **Data Collection**: Use the cap.py script to capture images of the target object with the stereo camera setup.

    * **Annotation**: Upload the captured images to an annotation tool like CVAT.ai. Manually draw bounding boxes for doll_head and doll_body objects.

    * **Export Annotations**: Download the annotations from CVAT in the YOLO 1.1 format. This typically results in images and labels folders, along with a classes.txt file.

    * **Dataset Splitting**: Use the dataset_yolo_split.py script to automatically divide the annotated data into training and validation sets, * creating the directory structure required by Ultralytics YOLO.

    * **Training & Exporting**: Run the main train_doll_yolo.py script. It handles the fine-tuning of a pre-trained YOLOv8 model and automatically exports the best-performing checkpoint to the ONNX format.

    * **Testing (Optional)**: Use the test_onnx_inference.py script to perform a quick inference test on the exported .onnx model to ensure it loads and works correctly.

## Dependencies

All required Python libraries are listed in the requirements.txt file. You can install them using pip:
Bash
```sh
pip install -r requirements.txt
```
This will typically install libraries such as:

    ultralytics

    torch & torchvision

    opencv-python

    numpy

## File Structure

This directory contains the following key files:

* **cap.py**: A simple script to capture and save images from the stereo cameras, used to build the initial image dataset.

* **dataset_yolo_split.py**: A utility script that splits the annotated dataset into train/ and valid/ subdirectories.

* **train_doll_yolo.py**: The main script for training. It loads a base YOLO model, fine-tunes it on the custom dataset, and exports the best model to ONNX.

* **test_onnx_inference.py**: A utility script to load the final .onnx model and test its inference on a live video test.

* **data.yaml**: The crucial dataset configuration file for YOLO. It specifies the paths to the training/validation sets and the class names.

* **requirements.txt**: Lists all Python dependencies for this training environment.

## Step-by-Step Instructions

* **Step 1**: Data Collection (cap.py)

Run the script to capture images of the doll in various positions, orientations, and lighting conditions.
Bash
```sh
python3 cap.py
```
Place all captured images in a common source folder (e.g., dataset/source_images/).

* **Step 2**: Annotation (cvat.ai)

    * Create a new project in CVAT.

    * Upload the images from dataset/source_images/.

    * Define the labels: doll_head and doll_body.

    * Annotate all images by drawing bounding boxes around the objects.

    * Export the project annotations in YOLO 1.1 format.

* **Step 3**: Prepare Dataset (dataset_yolo_split.py)

    * Place the exported images and labels folders inside a main directory, for example, YOLO_train/raw_dataset/.

    * Run the split script. It will create a new directory (e.g., doll_dataset/) containing train/ and valid/ subfolders, properly structured for training.
```sh
python3 dataset_yolo_split.py
```
* **Step 4**: Configure Training (data.yaml)

* Edit the data.yaml file to ensure the paths point to your new train and valid image folders and that the class names are correct.
    YAML
    * train: /path/to/your/project/YOLO_train/doll_dataset/train/images
    * val: /path/to/your/project/YOLO_train/doll_dataset/valid/images

    * nc: 2
    * names: ['doll_head', 'doll_body']

* **Step 5**: Train the Model (train_doll_yolo.py)

Before running, you can adjust training parameters like num_epochs, batch_size, etc., at the top of the train_doll_yolo.py script. Then, start the training:
```sh
python3 train_doll_yolo.py
```
The process will create a new run folder inside runs/detect/. 
After training is complete, the best model (best.pt) will be automatically converted to best.onnx and saved in the same location.

* **Step 6**: Test the Final Model (test_onnx_inference.py)

This is an optional but recommended final check.

Copy the generated best.onnx file to the location where your main vision system expects it.

Run the test script to load the ONNX model and perform detection on a test image.
```sh
    python3 test_onnx_inference.py
``` 
## Final Output

The primary artifact of this entire process is the best.onnx model file. This is the final, optimized model that should be used by the main teleoperation project for real-time object detection.