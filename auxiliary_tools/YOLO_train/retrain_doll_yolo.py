from ultralytics import YOLO
import os
import torch

# --- Configuración del Reentrenamiento ---
# 1. Verifica que apunta al YAML correcto
yaml_file_path = 'data.yaml' # Asumiendo que está aquí

# 2. ¡¡IMPORTANTE!! Apunta al 'best.pt' ANTERIOR
base_model_weights = 'best.pt' 

# 3. Ajusta las épocas (probablemente menos)
num_epochs = 50

# 4. Cambia el nombre de la ejecución
run_name = 'doll_yolov8n_run2' # Nuevo nombre

# Otros parámetros (mantener consistencia)
image_size = 640
batch_size = 8
device_to_use = 0 if torch.cuda.is_available() else 'cpu'
# --- Fin Configuración ---

# Verificar archivos
if not os.path.exists(yaml_file_path):
    print(f"Error Crítico: El archivo YAML '{yaml_file_path}' no se encuentra.")
elif not os.path.exists(base_model_weights):
    print(f"Error Crítico: El modelo base '{base_model_weights}' no se encuentra.")
else:
    print(f"Archivo YAML encontrado: {yaml_file_path}")
    print(f"Continuando entrenamiento desde: {base_model_weights}")
    print(f"Usando dispositivo: {device_to_use}")

    # Cargar el modelo entrenado anteriormente
    model = YOLO(base_model_weights)

    # Iniciar el re-entrenamiento
    try:
        results = model.train(
            data=yaml_file_path,
            epochs=num_epochs,
            imgsz=image_size,
            batch=batch_size,
            device=device_to_use,
            name=run_name,
            patience=25 # Puedes ajustar la paciencia
        )

        print("-" * 30)
        print("¡Re-entrenamiento completado exitosamente!")
        best_model_path = os.path.join(results.save_dir, 'weights', 'best.pt')
        print(f"Nuevo modelo 'best.pt' guardado en: {best_model_path}")
        print("-" * 30)

        # <<< Exportación Automática a ONNX del NUEVO modelo >>>
        if os.path.exists(best_model_path):
            print("\nIniciando exportación a ONNX...")
            model_to_export = YOLO(best_model_path) # Cargar el nuevo best.pt
            try:
                # ¡Asegúrate de incluir simplify=True!
                onnx_path = model_to_export.export(format='onnx', imgsz=image_size, simplify=True, opset=12)
                print(f"¡Nuevo modelo exportado exitosamente a ONNX!")
                print(f"Archivo ONNX guardado en: {onnx_path}")
                print("-" * 30)
            except Exception as export_error:
                print(f"\nError durante la exportación a ONNX: {export_error}")
                print("El re-entrenamiento finalizó, pero la exportación falló.")
                print("Puedes intentar exportar manualmente usando:")
                print(f"yolo export model={best_model_path} format=onnx imgsz={image_size} simplify=True opset=12")
        else:
             print("Advertencia: No se encontró el nuevo archivo 'best.pt'. No se puede exportar a ONNX.")

    except Exception as e:
        print(f"\nError durante el re-entrenamiento: {e}")