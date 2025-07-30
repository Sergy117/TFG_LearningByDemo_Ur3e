from ultralytics import YOLO
import os
import torch
"""
This code belong to https://github.com/Sergy117/TFG_LearningByDemo_Ur3e
Developed by Sergio Gonzalez Rodríguez for Unversity of Santiago de Compostela
owner: meanssergy@gmail.com
date: 2025
"""
"""
This script is used to retrain a pretrained model for better detection on a custom object, in this project, to detect better the doll
"""

#  Verify path
yaml_file_path = 'data.yaml' # Asumiendo que está aquí

# Use 'best.pt' that was trained for the doll
base_model_weights = 'best.pt' 

# Adjust epochs
num_epochs = 50

# Change name for avoiding override
run_name = 'doll_yolov8n_run2'

# Keep coherence with other codes
image_size = 640
batch_size = 8
device_to_use = 0 if torch.cuda.is_available() else 'cpu'


# Verify 
if not os.path.exists(yaml_file_path):
    print(f"Critic error: YAML '{yaml_file_path}' not found.")
elif not os.path.exists(base_model_weights):
    print(f"Critic error: base model '{base_model_weights}' not found.")
else:
    print(f" YAML found: {yaml_file_path}")
    print(f"Training from: {base_model_weights}")
    print(f"Using device for training: {device_to_use}")

    # Load model
    model = YOLO(base_model_weights)

    # re-training
    try:
        results = model.train(
            data=yaml_file_path,
            epochs=num_epochs,
            imgsz=image_size,
            batch=batch_size,
            device=device_to_use,
            name=run_name,
            patience=25 #You could adjust parameters
        )

        print("-" * 30)
        print("Completed retraining!")
        best_model_path = os.path.join(results.save_dir, 'weights', 'best.pt')
        print(f"New model 'best.pt' saved in: {best_model_path}")
        print("-" * 30)

        # Exporting to ONNX for posterior use
        if os.path.exists(best_model_path):
            print("\nExporting ONNX...")
            model_to_export = YOLO(best_model_path) 
            try:
                # check simplify=True
                onnx_path = model_to_export.export(format='onnx', imgsz=image_size, simplify=True, opset=12)
                print(f"Exported to ONNX!")
                print(f"ONNX saved in : {onnx_path}")
                print("-" * 30)
            except Exception as export_error:
                print(f"\nError during export to  ONNX: {export_error}")
                print("Re-training completed but aborted export")
                print("Try exporting manually using :")
                print(f"yolo export model={best_model_path} format=onnx imgsz={image_size} simplify=True opset=12")
        else:
             print("Warning 'best.pt' not found")

    except Exception as e:
        print(f"\nRetraining not completed, failed:  {e}")