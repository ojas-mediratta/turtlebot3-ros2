import os
import cv2
import numpy as np
import csv
import argparse
import shutil
import random
from sklearn.model_selection import train_test_split
from collections import Counter

# Import the shared cropping function
from utils import crop_sign

# --- Augmentation Functions ---

def apply_augmentations(image, label):
    """
    Applies a set of augmentations to a single image.
    Returns a list of augmented images and a list of corresponding labels.
    """
    augmented_images = []
    augmented_labels = []

    # 1. Add the original (cropped) image
    augmented_images.append(image)
    augmented_labels.append(label)

    # 2. Brightness/Contrast
    # Apply to all classes
    contrast = random.uniform(0.8, 1.2)
    brightness = random.uniform(-20, 20)
    bc_img = cv2.convertScaleAbs(image, alpha=contrast, beta=brightness)
    augmented_images.append(bc_img)
    augmented_labels.append(label)

    # 3. Horizontal Flip
    # Apply to all, but swap labels for 1 and 2
    flipped_img = cv2.flip(image, 1)
    flipped_label = label
    if label == 1:
        flipped_label = 2
    elif label == 2:
        flipped_label = 1
    augmented_images.append(flipped_img)
    augmented_labels.append(flipped_label)

    # 4. Rotation
    # Apply only to rotation-invariant classes
    if label in [0, 1, 2, 3, 4, 5]:
        angle = random.uniform(-15, 15)
        (h, w) = image.shape[:2]
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_img = cv2.warpAffine(image, M, (w, h), borderValue=(0,0,0))
        augmented_images.append(rotated_img)
        augmented_labels.append(label)

    # --- DISABLING for now to reduce overfitting ---

    # # 5. Gaussian Blur
    # # Simulates an out-of-focus camera
    # blur_img = cv2.GaussianBlur(image, (5, 5), 0)
    # augmented_images.append(blur_img)
    # augmented_labels.append(label)
    
    # # 6. Gaussian Noise
    # # Simulates camera sensor noise
    # # We add noise to the int16 version, then clip back to uint8
    # noise = np.random.normal(0, 10, image.shape) # 10 std dev
    # noise_img = np.clip(image.astype(np.int16) + noise.astype(np.int16), 0, 255).astype(np.uint8)
    # augmented_images.append(noise_img)
    # augmented_labels.append(label)
    
    # 7. Random Crop (Zoom)
    # Simulates the sign being slightly off-center
    scale = random.uniform(0.85, 1.0) # Zoom in by 0% to 15%
    (h, w) = image.shape[:2]
    crop_h, crop_w = int(h * scale), int(w * scale)
    y_start = random.randint(0, h - crop_h)
    x_start = random.randint(0, w - crop_w)
    crop_img = image[y_start:y_start+crop_h, x_start:x_start+crop_w]
    crop_img = cv2.resize(crop_img, (w, h)) # Resize back to standard size
    augmented_images.append(crop_img)
    augmented_labels.append(label)
    
    # 8. Perspective Shear
    # Simulates viewing the sign from a slight angle
    (h, w) = image.shape[:2]
    pts1 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    # Shear horizontally by up to 10%
    shear_val = random.uniform(-0.1, 0.1)
    pts2 = np.float32([
        [int(w * shear_val), 0],  # Top-left
        [w, 0],                  # Top-right
        [0, h],                  # Bottom-left
        [int(w * (1 + shear_val)), h] # Bottom-right (shifted)
    ])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    shear_img = cv2.warpPerspective(image, M, (w, h), borderValue=(0,0,0))
    augmented_images.append(shear_img)
    augmented_labels.append(label)
    
    return augmented_images, augmented_labels

# --- Core Data Processing ---

def load_all_data(data_dir):
    """
    Loads all image paths and labels from subdirectories.
    """
    all_data = []
    print(f"Loading data from '{data_dir}'...")
    
    if not os.path.isdir(data_dir):
        print(f"Error: Directory not found: {data_dir}")
        return []
        
    for subfolder in os.listdir(data_dir):
        subfolder_path = os.path.join(data_dir, subfolder)
        if os.path.isdir(subfolder_path):
            labels_file = os.path.join(subfolder_path, 'labels.txt')
            if not os.path.isfile(labels_file):
                print(f"Warning: No 'labels.txt' found in {subfolder_path}. Skipping.")
                continue
            
            print(f"  Processing subfolder: {subfolder}")
            with open(labels_file, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row: continue # skip empty lines
                    try:
                        img_name = row[0].strip()
                        label = int(row[1].strip())
                        
                        img_path = os.path.join(subfolder_path, f"{img_name}.png")
                        if os.path.isfile(img_path):
                            all_data.append((img_path, label))
                        else:
                            print(f"Warning: Image file not found: {img_path}")
                    except Exception as e:
                        print(f"Warning: Skipping malformed row in {labels_file}: {row}. Error: {e}")
                        
    print(f"Found {len(all_data)} total images.")
    return all_data

def process_and_save_data(data_list, output_dir, do_augmentations):
    """
    Processes a list of (path, label) tuples and saves them to the output dir.
    Applies cropping and (optionally) augmentations.
    """
    if not data_list:
        print(f"No data to process for {output_dir}.")
        return

    labels_filepath = os.path.join(output_dir, 'labels.txt')
    image_counter = 0

    with open(labels_filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        
        for (img_path, label) in data_list:
            image = cv2.imread(img_path)
            if image is None:
                print(f"Warning: Failed to read image {img_path}. Skipping.")
                continue
            
            # --- 1. Preprocessing: Crop ---
            # This is applied to BOTH train and test
            cropped_image = crop_sign(image)
            if cropped_image is None:
                print(f"Warning: Cropping failed for {img_path}. Skipping.")
                continue

            # --- 2. Augmentation ---
            if do_augmentations:
                images_to_save, labels_to_save = apply_augmentations(cropped_image, label)
            else:
                # For test set, just save the single cropped image
                images_to_save = [cropped_image]
                labels_to_save = [label]

            # --- 3. Save ---
            for img, lab in zip(images_to_save, labels_to_save):
                img_filename = f"{image_counter:05d}.png"
                img_savename = f"{image_counter:05d}"
                img_savepath = os.path.join(output_dir, img_filename)
                
                cv2.imwrite(img_savepath, img)
                writer.writerow([img_savename, lab])
                image_counter += 1

    print(f"Saved {image_counter} images to {output_dir}.")

def main():
    parser = argparse.ArgumentParser(description="Data Preparation Script for Sign Classification")
    parser.add_argument('--data_dir', type=str, default='data', help="Root directory containing data subfolders (e.g., 'f1', 'f2')")
    parser.add_argument('--output_dir', type=str, default='SPLIT', help="Directory to save the TRAIN and TEST splits")
    parser.add_argument('--test_split', type=float, default=0.3, help="Fraction of data to use for the test set (e.g., 0.3 for 30%)")
    parser.add_argument('--seed', type=int, default=42, help="Random seed for reproducible splits")
    
    args = parser.parse_args()
    
    random.seed(args.seed)
    np.random.seed(args.seed)

    # 1. Setup Output Directories
    train_dir = os.path.join(args.output_dir, 'TRAIN')
    test_dir = os.path.join(args.output_dir, 'TEST')
    
    if os.path.isdir(args.output_dir):
        print(f"Output directory '{args.output_dir}' already exists. Removing it.")
        shutil.rmtree(args.output_dir)
        
    print(f"Creating new output directories...")
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(test_dir, exist_ok=True)

    # 2. Load all data
    all_data = load_all_data(args.data_dir)
    if not all_data:
        print("No data found. Exiting.")
        return
        
    X_paths = [item[0] for item in all_data]
    y_labels = [item[1] for item in all_data]
    
    print(f"Original data distribution: {Counter(y_labels)}")

    # 3. Create Stratified Split
    # MODIFIED: Handle test_split = 0.0
    if args.test_split > 0.0:
        print(f"Splitting data ({1.0 - args.test_split} train / {args.test_split} test)...")
        try:
            train_data, test_data = train_test_split(
                all_data,
                test_size=args.test_split,
                stratify=y_labels,
                random_state=args.seed
            )
        except ValueError as e:
            print(f"Warning: Could not perform stratified split. Using regular split. Error: {e}")
            train_data, test_data = train_test_split(
                all_data,
                test_size=args.test_split,
                random_state=args.seed
            )
        
        print(f"Test set size: {len(test_data)}")
        print(f"Test distribution: {Counter([label for _, label in test_data])}")
    
    else:
        print("Test split is 0.0. Using 100% of data for training.")
        train_data = all_data
        test_data = [] # Create an empty list for the test set
        
    print(f"Train set size: {len(train_data)}")
    print(f"Train distribution: {Counter([label for _, label in train_data])}")

    # 4. Process and Save TRAIN data (with augmentations)
    print("\nProcessing TRAINING data (cropping + augmentations)...")
    process_and_save_data(train_data, train_dir, do_augmentations=True)

    # 5. Process and Save TEST data (cropping only)
    print("\nProcessing TESTING data (cropping only)...")
    process_and_save_data(test_data, test_dir, do_augmentations=False)

    print("\nData preparation complete!")
    print(f"Train data saved to: {train_dir}")
    print(f"Test data saved to: {test_dir}")

if __name__ == "__main__":
    main()