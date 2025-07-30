import os
import glob
import random
import shutil
import argparse 
"""
This code belong to https://github.com/Sergy117/TFG_LearningByDemo_Ur3e
Developed by Sergio Gonzalez Rodríguez for Unversity of Santiago de Compostela
owner: meanssergy@gmail.com
date: 2025
"""
"""
This script is used to split the dataset tagged and downloaded from cvat.ai
"""

Name_folder_download_Cvat = "DollHeadBody"
DEFAULT_IMAGE_SOURCE_DIR = f"{Name_folder_download_Cvat}/images/train" #path to ALL the images of the doll
DEFAULT_LABEL_SOURCE_DIR = f"{Name_folder_download_Cvat}/labels/train" #path to ALL the .txt from YOLO that has the labels and position structure:
#For example the .txt being img1_3012321.txt
"""
0 0.729953 0.465354 0.110406 0.130167
1 0.582594 0.532875 0.211688 0.209250
"""
#in images/train there should be a img1_3012321.png being that the image that was labeled with cvat.ai 

DEFAULT_DEST_DIR = "dataset_structure" # Folder that will be created with the correct format for training train/valid
DEFAULT_TRAIN_RATIO = 0.8 # Define 80% of images for training 20% for validation
DEFAULT_IMAGE_EXT = ".png" # Images format

# --- Auxiliar functions ---
def create_dirs(base_path):
    """Creating necessary data structure for training"""
    os.makedirs(os.path.join(base_path, "images", "train"), exist_ok=True)
    os.makedirs(os.path.join(base_path, "images", "valid"), exist_ok=True)
    os.makedirs(os.path.join(base_path, "labels", "train"), exist_ok=True)
    os.makedirs(os.path.join(base_path, "labels", "valid"), exist_ok=True)
    print(f"Directories created/verified in: {base_path}")

def split_data(image_dir, label_dir, dest_dir, train_ratio, img_ext):
    """Find archives randomize then and copy them into the new folders"""

    if not os.path.isdir(image_dir):
        print(f"Error: Images directory '{image_dir}' doesnt exist.")
        return
    if not os.path.isdir(label_dir):
        print(f"Error: Labels directory '{label_dir}' doesnt exist.")
        return
    # Finding all images
    image_files = glob.glob(os.path.join(image_dir, f'*{img_ext}'))
    if not image_files:
        print(f"Error: Not '{img_ext}' in '{image_dir}'.")
        return

    print(f"found {len(image_files)} images.")

    # Shuffle images for randomizing valid and train images
    random.shuffle(image_files)

    # Calculate division
    total_images = len(image_files)
    num_train = int(total_images * train_ratio)
    num_valid = total_images - num_train

    print(f"Dividing: {num_train} for training, {num_valid} for validation.")

    # Destiny directories
    create_dirs(dest_dir)

    # Process and copy 
    copied_train_count = 0
    copied_valid_count = 0
    missing_label_count = 0

    # Copy to train
    for i in range(num_train):
        img_path = image_files[i]
        base_filename = os.path.splitext(os.path.basename(img_path))[0]
        label_filename = base_filename + ".txt"
        label_path = os.path.join(label_dir, label_filename)

        dest_img_path_train = os.path.join(dest_dir, "images", "train", os.path.basename(img_path))
        dest_label_path_train = os.path.join(dest_dir, "labels", "train", label_filename)

        if os.path.exists(label_path):
            shutil.copy2(img_path, dest_img_path_train) # copy2 preserv metadata
            shutil.copy2(label_path, dest_label_path_train)
            copied_train_count += 1
        else:
            print(f"Warning: No label for'{label_filename}' for image '{os.path.basename(img_path)}'. This pair is omited.")
            missing_label_count += 1

    # Copy to validate
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
            print(f"Warning: No label for'{label_filename}' for image '{os.path.basename(img_path)}'. This pair is omited.")
            missing_label_count += 1

    print("\n--- Resume ---")
    print(f"Processed images: {total_images}")
    print(f"Pairs copied to 'train': {copied_train_count}")
    print(f"Pairs copied to 'valid': {copied_valid_count}")
    if missing_label_count > 0:
        print(f"Warning: omited {missing_label_count} pairs for missing label")
    print(f"Dataset divided on: {dest_dir}")

# --- Main ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Divide a dataset of images and labels of YOLO in train and validation.")
    parser.add_argument("--img_dir", type=str, default=DEFAULT_IMAGE_SOURCE_DIR,
                        help="Directory of images.")
    parser.add_argument("--lbl_dir", type=str, default=DEFAULT_LABEL_SOURCE_DIR,
                        help="Directory with .txt of YOLO labels.")
    parser.add_argument("--out_dir", type=str, default=DEFAULT_DEST_DIR,
                        help="Directory to create 'images/train', 'labels/valid', etc.")
    parser.add_argument("--ratio", type=float, default=DEFAULT_TRAIN_RATIO,
                        help="Proportion of train/valid for example 80% to train = 0.8.")
    parser.add_argument("--ext", type=str, default=DEFAULT_IMAGE_EXT,
                        help="Images format ('.png', '.jpg'). ")

    args = parser.parse_args()


    if not 0 < args.ratio < 1:
        print("Error: ratio should be between 0 and 1")
    else:
        split_data(args.img_dir, args.lbl_dir, args.out_dir, args.ratio, args.ext)