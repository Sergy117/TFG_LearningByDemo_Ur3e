from ultralytics import YOLO
import os
import torch # To checck CUDA compatibility

# --- Training config ---
yaml_file_path = 'data.yaml'

# Base model()'yolov8n.pt', 'yolov8s.pt', etc.)
base_model_weights = 'yolov8n.pt'

# Training Parameters
num_epochs = 75  
image_size = 640 # Check img size used on export and inference on the model
batch_size = 8 
device_to_use = 0 if torch.cuda.is_available() else 'cpu' #USe GPU if aviable, CPU if not
run_name = 'doll_yolov8n_run1' # Name of the result model

# Verify YAML 
if not os.path.exists(yaml_file_path):
    print(f"YAML '{yaml_file_path}' not found")
    print("Verify path.")
else:
    print(f"YAML found: {yaml_file_path}")
    print(f"Starting fine-tuning from: {base_model_weights}")
    print(f"Using : {device_to_use}")

    # Load base model
    model = YOLO(base_model_weights)

    # Start training
    best_model_path = None # Variable to store path for the best model
    try:
        results = model.train(
            data=yaml_file_path,
            epochs=num_epochs,
            imgsz=image_size,
            batch=batch_size,
            device=device_to_use,
            name=run_name,
            patience=25
        )

        print("-" * 30)
        print("Training completed!")
        best_model_path = os.path.join(results.save_dir, 'weights', 'best.pt')
        print(f"Model 'best.pt' saved at: {best_model_path}")
        print("-" * 30)

        # <<< Export to ONNX --- >>>
        if os.path.exists(best_model_path):
            print("\nIniciando exportación a ONNX...")
            #export the best model
            model_to_export = YOLO(best_model_path)

            # Exportar
            try:
                onnx_path = model_to_export.export(format='onnx', imgsz=image_size) # Usa el mismo imgsz
                print(f"Model exported to ONNX!")
                print(f"ONNX saved in: {onnx_path}")
                print("-" * 30)
            except Exception as export_error:
                print(f"\nError during eport to ONNX: {export_error}")
                print("The training finished but export failed.")
                print("Export manually using :")
                print(f"yolo export model={best_model_path} format=onnx imgsz={image_size}")

        else:
             print("Warning not found 'best.pt'.")


    except Exception as e:
        print(f"\nError during training: {e}")
        print("(reduce 'batch_size'),")
        print("Check dataset routes")