import cv2
import numpy as np
import os
import time
import pyrealsense2 as rs
import onnxruntime # <<< Import ONNX Runtime

# --- Configuration ---
ONNX_MODEL_PATH = 'best.onnx' 

SERIAL_CAM1 = '153222070290'  
SERIAL_CAM2 = '153222070548'  
IMG_WIDTH = 640
IMG_HEIGHT = 480
FPS = 30

CONFIDENCE_THRESHOLD = 0.3  # Umbral (puedes empezar con 0.3 o 0.4)
NMS_THRESHOLD = 0.4         # Umbral para Non-Maximum Suppression
INPUT_WIDTH = 640           # Debe coincidir con 'imgsz' usado en export
INPUT_HEIGHT = 640          # Debe coincidir con 'imgsz' usado en export

# Nombres de las clases (¡el orden DEBE coincidir con tu data.yaml y entrenamiento!)
CLASS_NAMES = ['doll_head', 'doll_body']
# Colores para los bounding boxes (opcional)
COLORS = [(0, 255, 0), (0, 0, 255)] # Verde para cabeza, Azul para cuerpo (ajusta según orden)
# --- Fin Configuración ---

# --- Función de Post-procesamiento (La misma que antes para salida ONNX) ---
def postprocess_yolo_output(outputs, original_width, original_height, conf_threshold, nms_threshold):
    """Procesa la salida raw del modelo YOLOv8 ONNX."""
    # La salida de ONNX Runtime suele ser una lista, tomamos el primer elemento
    # que debe tener shape [batch, num_props, num_detections], ej [1, 6, 8400]
    outputs = outputs[0].T # Transponer a [num_detections, num_props]
    boxes, confidences, class_ids = [], [], []
    for row in outputs:
        box_probs = row[4:] # Confianzas de clase empiezan en la columna 4
        class_id = np.argmax(box_probs)
        confidence = box_probs[class_id]
        if confidence > conf_threshold:
            cx, cy, w, h = row[:4] # Centro y dimensiones normalizadas
            # Desnormalizar a coordenadas de imagen original
            center_x = int(cx * original_width)
            center_y = int(cy * original_height)
            width = int(w * original_width)
            height = int(h * original_height)
            x1 = int(center_x - width / 2)
            y1 = int(center_y - height / 2)
            boxes.append([x1, y1, width, height])
            confidences.append(float(confidence))
            class_ids.append(class_id)
    # Aplicar Non-Maximum Suppression
    indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    final_boxes, final_confidences, final_class_ids = [], [], []
    if len(indices) > 0:
        for i in indices.flatten():
            final_boxes.append(boxes[i])
            final_confidences.append(confidences[i])
            final_class_ids.append(class_ids[i])
    return final_boxes, final_confidences, final_class_ids

# --- Ejecución Principal ---
if __name__ == "__main__":
    # 1. Cargar el modelo ONNX con ONNX Runtime
    if not os.path.exists(ONNX_MODEL_PATH):
        print(f"Error: No se encuentra el archivo ONNX en {ONNX_MODEL_PATH}")
        exit()
    try:
        # Intentar usar CUDA si está disponible, si no, CPU
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        ort_session = onnxruntime.InferenceSession(ONNX_MODEL_PATH, providers=providers)
        # Verificar qué proveedor se está usando realmente
        selected_provider = ort_session.get_providers()[0]
        print(f"Modelo ONNX cargado. Usando proveedor: {selected_provider}")
        # Obtener nombre de la entrada del modelo (usualmente 'images')
        input_name = ort_session.get_inputs()[0].name
        print(f"Nombre de entrada del modelo: {input_name}")
    except Exception as e:
        print(f"Error cargando modelo ONNX con ONNX Runtime: {e}")
        print("Asegúrate de tener 'onnxruntime' o 'onnxruntime-gpu' instalado.")
        print("Si usas GPU, verifica la compatibilidad de CUDA/cuDNN con tu onnxruntime-gpu.")
        exit()

    # 2. Iniciar cámaras RealSense (igual que antes)
    pipeline1 = rs.pipeline()
    config1 = rs.config()
    config1.enable_device(SERIAL_CAM1)
    config1.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
    pipeline2 = rs.pipeline()
    config2 = rs.config()
    config2.enable_device(SERIAL_CAM2)
    config2.enable_stream(rs.stream.color, IMG_WIDTH, IMG_HEIGHT, rs.format.bgr8, FPS)
    try:
        print("Iniciando cámaras RealSense...")
        profile1 = pipeline1.start(config1)
        profile2 = pipeline2.start(config2)
        print("Cámaras iniciadas correctamente.")
        print("\nPresiona 'q' en cualquiera de las ventanas para salir.")
    except Exception as e:
        print(f"Error al iniciar cámaras RealSense: {e}")
        exit()

    # --- Bucle de Procesamiento ---
    try:
        while True:
            start_time = time.time()

            # --- Obtener frames ---
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
                 print(f"Error de Runtime al obtener frames: {e}")
                 time.sleep(0.5)
                 continue

            processed_image1 = image1.copy() # Copiar para dibujar encima
            processed_image2 = image2.copy()

            # --- Procesar Cámara 1 ---
            original_h1, original_w1 = processed_image1.shape[:2]
            # Preprocesar
            blob1 = cv2.dnn.blobFromImage(processed_image1, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
            # Inferir con ONNX Runtime
            ort_inputs1 = {input_name: blob1}
            try:
                ort_outputs1 = ort_session.run(None, ort_inputs1) # Ejecutar inferencia
                # Postprocesar
                boxes1, confidences1, class_ids1 = postprocess_yolo_output(ort_outputs1[0], original_w1, original_h1, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
                # Dibujar resultados
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
                 print(f"Error en inferencia/postproceso ONNX Runtime (Cam 1): {e}")

            # --- Procesar Cámara 2 ---
            original_h2, original_w2 = processed_image2.shape[:2]
            # Preprocesar
            blob2 = cv2.dnn.blobFromImage(processed_image2, 1/255.0, (INPUT_WIDTH, INPUT_HEIGHT), swapRB=True, crop=False)
             # Inferir con ONNX Runtime
            ort_inputs2 = {input_name: blob2}
            try:
                ort_outputs2 = ort_session.run(None, ort_inputs2)
                # Postprocesar
                boxes2, confidences2, class_ids2 = postprocess_yolo_output(ort_outputs2[0], original_w2, original_h2, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
                # Dibujar resultados
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
                 print(f"Error en inferencia/postproceso ONNX Runtime (Cam 2): {e}")


            # Calcular y mostrar FPS
            end_time = time.time()
            fps = 1 / (end_time - start_time) if (end_time - start_time) > 0 else 0
            cv2.putText(processed_image1, f"FPS: {fps:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # --- Mostrar Frames ---
            cv2.imshow("RealSense Cam 1 - ONNX Runtime Test (Q to Quit)", processed_image1)
            cv2.imshow("RealSense Cam 2 - ONNX Runtime Test (Q to Quit)", processed_image2)

            # --- Salir ---
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # --- Limpieza Final ---
        print("Deteniendo cámaras...")
        try: pipeline1.stop()
        except RuntimeError as e: print(f"Error deteniendo pipeline 1: {e}")
        try: pipeline2.stop()
        except RuntimeError as e: print(f"Error deteniendo pipeline 2: {e}")
        cv2.destroyAllWindows()
        print("Recursos liberados.")