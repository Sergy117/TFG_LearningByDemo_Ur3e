import os
import glob
import random
import shutil
import argparse # Para aceptar argumentos desde línea de comandos

# --- Configuración ---
# Puedes modificar estos valores por defecto o pasarlos como argumentos
DEFAULT_IMAGE_SOURCE_DIR = "/home/sergio/Escritorio/PracticasTFG/Trayectoria/doll_images" # Directorio con TODAS tus imágenes (.png, .jpg)
DEFAULT_LABEL_SOURCE_DIR = "/home/sergio/Escritorio/PracticasTFG/Trayectoria/DollHeadBody/labels/train"   # Directorio con TODOS tus .txt de YOLO
DEFAULT_DEST_DIR = "dataset_yolo_split" # Carpeta donde se creará la estructura train/valid
DEFAULT_TRAIN_RATIO = 0.8 # 80% para entrenamiento, el resto para validación
DEFAULT_IMAGE_EXT = ".png" # Extensión de tus imágenes (incluye el punto)

# --- Funciones Auxiliares ---
def create_dirs(base_path):
    """Crea la estructura de directorios necesaria."""
    os.makedirs(os.path.join(base_path, "images", "train"), exist_ok=True)
    os.makedirs(os.path.join(base_path, "images", "valid"), exist_ok=True)
    os.makedirs(os.path.join(base_path, "labels", "train"), exist_ok=True)
    os.makedirs(os.path.join(base_path, "labels", "valid"), exist_ok=True)
    print(f"Directorios creados/verificados en: {base_path}")

def split_data(image_dir, label_dir, dest_dir, train_ratio, img_ext):
    """Encuentra archivos, los baraja y los copia a las carpetas train/valid."""

    # Validar directorios de origen
    if not os.path.isdir(image_dir):
        print(f"Error: El directorio de imágenes '{image_dir}' no existe.")
        return
    if not os.path.isdir(label_dir):
        print(f"Error: El directorio de etiquetas '{label_dir}' no existe.")
        return

    print(f"Buscando imágenes ('*{img_ext}') en: {image_dir}")
    print(f"Buscando etiquetas ('.txt') en: {label_dir}")

    # Encontrar todas las imágenes
    image_files = glob.glob(os.path.join(image_dir, f'*{img_ext}'))
    if not image_files:
        print(f"Error: No se encontraron imágenes con extensión '{img_ext}' en '{image_dir}'.")
        return

    print(f"Se encontraron {len(image_files)} imágenes.")

    # Barajar aleatoriamente
    random.shuffle(image_files)

    # Calcular punto de división
    total_images = len(image_files)
    num_train = int(total_images * train_ratio)
    num_valid = total_images - num_train

    print(f"Dividiendo: {num_train} para entrenamiento, {num_valid} para validación.")

    # Crear directorios de destino
    create_dirs(dest_dir)

    # Procesar y copiar archivos
    copied_train_count = 0
    copied_valid_count = 0
    missing_label_count = 0

    print("Copiando archivos...")

    # Copiar a Entrenamiento
    for i in range(num_train):
        img_path = image_files[i]
        base_filename = os.path.splitext(os.path.basename(img_path))[0]
        label_filename = base_filename + ".txt"
        label_path = os.path.join(label_dir, label_filename)

        dest_img_path_train = os.path.join(dest_dir, "images", "train", os.path.basename(img_path))
        dest_label_path_train = os.path.join(dest_dir, "labels", "train", label_filename)

        if os.path.exists(label_path):
            shutil.copy2(img_path, dest_img_path_train) # copy2 preserva metadatos
            shutil.copy2(label_path, dest_label_path_train)
            copied_train_count += 1
        else:
            print(f"Advertencia: No se encontró la etiqueta '{label_filename}' para la imagen '{os.path.basename(img_path)}'. Se omitirá este par.")
            missing_label_count += 1

    # Copiar a Validación
    for i in range(num_train, total_images):
        img_path = image_files[i]
        base_filename = os.path.splitext(os.path.basename(img_path))[0]
        label_filename = base_filename + ".txt"
        label_path = os.path.join(label_dir, label_filename)

        dest_img_path_valid = os.path.join(dest_dir, "images", "valid", os.path.basename(img_path))
        dest_label_path_valid = os.path.join(dest_dir, "labels", "valid", label_filename)

        if os.path.exists(label_path):
            shutil.copy2(img_path, dest_img_path_valid)
            shutil.copy2(label_path, dest_label_path_valid)
            copied_valid_count += 1
        else:
            print(f"Advertencia: No se encontró la etiqueta '{label_filename}' para la imagen '{os.path.basename(img_path)}'. Se omitirá este par.")
            missing_label_count += 1

    print("\n--- Resumen ---")
    print(f"Imágenes procesadas: {total_images}")
    print(f"Pares copiados a 'train': {copied_train_count}")
    print(f"Pares copiados a 'valid': {copied_valid_count}")
    if missing_label_count > 0:
        print(f"Advertencia: Se omitieron {missing_label_count} pares por falta de archivo de etiqueta.")
    print(f"Dataset dividido creado en: {dest_dir}")

# --- Ejecución Principal ---
if __name__ == "__main__":
    # Configurar argumentos de línea de comandos
    parser = argparse.ArgumentParser(description="Divide un dataset de imágenes y etiquetas YOLO en conjuntos de entrenamiento y validación.")
    parser.add_argument("--img_dir", type=str, default=DEFAULT_IMAGE_SOURCE_DIR,
                        help="Directorio que contiene las imágenes originales.")
    parser.add_argument("--lbl_dir", type=str, default=DEFAULT_LABEL_SOURCE_DIR,
                        help="Directorio que contiene los archivos .txt de etiquetas YOLO originales.")
    parser.add_argument("--out_dir", type=str, default=DEFAULT_DEST_DIR,
                        help="Directorio base donde se creará la estructura 'images/train', 'labels/valid', etc.")
    parser.add_argument("--ratio", type=float, default=DEFAULT_TRAIN_RATIO,
                        help="Proporción de datos para el conjunto de entrenamiento (ej. 0.8 para 80%).")
    parser.add_argument("--ext", type=str, default=DEFAULT_IMAGE_EXT,
                        help="Extensión de los archivos de imagen (ej. '.png', '.jpg'). Incluir el punto.")

    args = parser.parse_args()

    # Validar ratio
    if not 0 < args.ratio < 1:
        print("Error: El ratio de entrenamiento debe estar entre 0 y 1 (exclusivo).")
    else:
        # Ejecutar la función principal
        split_data(args.img_dir, args.lbl_dir, args.out_dir, args.ratio, args.ext)