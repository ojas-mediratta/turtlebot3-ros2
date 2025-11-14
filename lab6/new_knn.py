#!/usr/bin/env python3
import cv2
import argparse
import csv
import math
import pickle
import numpy as np
import random
import os
import shutil
import albumentations as A
from skimage.feature import hog
import matplotlib.pyplot as plt

# --- NEW: Imports for scikit-learn ---
from sklearn.neighbors import KNeighborsClassifier
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline

# --- Resize dimensions ---
# We will resize the CROPPED sign to this standard size
RESIZE_DIM = (64, 64)

def preprocess_and_crop(image, label=None):
    """
    Finds the sign in the image, crops it, and resizes it.
    This is the most important step for high accuracy.
    For class 0 (empty walls), skip cropping and just resize.
    """
    if image is None or image.size == 0:
        return None
    
    # Class 0 (empty wall) - don't crop, just resize
    if label is not None and int(label) == 0:
        return cv2.resize(image, RESIZE_DIM, interpolation=cv2.INTER_AREA)

    # 1. Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 2. Define color masks for all sign colors
    # Red (Stop, Do Not Enter)
    mask1 = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
    mask2 = cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
    mask_red = cv2.bitwise_or(mask1, mask2)
    
    # Blue (Left Arrows) - Relaxed saturation threshold
    mask_blue = cv2.inRange(hsv, (90, 80, 50), (130, 255, 255))
    
    # Green/Yellow (Right Arrows) - Extended range to capture yellowish-green
    mask_green = cv2.inRange(hsv, (25, 40, 50), (95, 255, 255))
    
    # Orange (U-Turn, Goal)
    mask_orange = cv2.inRange(hsv, (10, 100, 100), (25, 255, 255))

    # Combine all masks
    mask = cv2.bitwise_or(mask_red, mask_blue)
    mask = cv2.bitwise_or(mask, mask_green)
    mask = cv2.bitwise_or(mask, mask_orange)
    
    # Apply morphological operations to fill gaps and connect components
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 3. Find the largest contour
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        # If no contour found, fallback to resizing the whole image
        return cv2.resize(image, RESIZE_DIM, interpolation=cv2.INTER_AREA)

    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)

    # 4. Crop to bounding box
    if area > 100: # Filter out tiny noise
        x, y, w, h = cv2.boundingRect(largest_contour)
        # Give a 10% padding for better context
        pad_x = int(w * 0.10)
        pad_y = int(h * 0.10)
        
        # Crop with padding, ensuring we stay within image bounds
        y_min = max(0, y - pad_y)
        y_max = min(image.shape[0], y + h + pad_y)
        x_min = max(0, x - pad_x)
        x_max = min(image.shape[1], x + w + pad_x)
        
        cropped_image = image[y_min:y_max, x_min:x_max]

        if cropped_image.size > 0:
            return cv2.resize(cropped_image, RESIZE_DIM, interpolation=cv2.INTER_AREA)

    # Fallback if contour is too small or crop fails
    return cv2.resize(image, RESIZE_DIM, interpolation=cv2.INTER_AREA)

def extract_features(image):
    """
    Extract a high-quality, concise feature vector from the
    preprocessed (cropped and resized) image.
    
    We use ONLY HOG and an HSV Color Histogram.
    We REMOVE raw pixels and Canny edges.
    """
    if image is None or image.size == 0:
        raise ValueError("Invalid image")
    
    image = image.astype(np.uint8)
    features = []
    
    # 1. Color Histogram (HSV) - 16 bins (Hue) + 8 bins (Sat) = 24 features
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h_hist = cv2.calcHist([hsv], [0], None, [16], [0, 180])
    s_hist = cv2.calcHist([hsv], [1], None, [8], [0, 256])
    
    cv2.normalize(h_hist, h_hist)
    cv2.normalize(s_hist, s_hist)
    
    hist_features = np.concatenate((h_hist.flatten(), s_hist.flatten())).astype(np.float32)
    features.append(hist_features)
    
    # 2. HOG features - ~7200 features on 64x64
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hog_features = hog(gray, orientations=9, pixels_per_cell=(8, 8),
                           cells_per_block=(2, 2), visualize=False, multichannel=False)
        hog_features = hog_features.astype(np.float32)
        features.append(hog_features)
    except Exception as e:
        # Fallback if HOG fails
        fallback_size = 7524 # Calculated from a 64x64 image
        hog_features = np.zeros(fallback_size, dtype=np.float32)
        features.append(hog_features)
    
    # Concatenate all features
    all_features = np.concatenate(features).astype(np.float32)
    
    return all_features

def train_model(data_path, train_lines, image_type, model_filename, save_model, k_value):
    """
    Loads, preprocesses, and trains a scikit-learn KNN model.
    Saves a pipeline containing the StandardScaler and the Classifier.
    """
    train_features_list = []
    train_labels = []
    
    # Create directory for cropped training images
    cropped_train_dir = os.path.join(os.getcwd(), 'cropped_images', 'TRAIN')
    os.makedirs(cropped_train_dir, exist_ok=True)

    for rel_path, label in train_lines:
        img_path = os.path.join(data_path, rel_path + image_type)
        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: Could not load image {img_path}")
            continue
        
        # 1. Preprocess (Crop and Resize)
        proc_img = preprocess_and_crop(img, label)
        if proc_img is None:
            continue
        
        # Save cropped image
        cropped_subdir = os.path.join(cropped_train_dir, os.path.dirname(rel_path))
        os.makedirs(cropped_subdir, exist_ok=True)
        cropped_path = os.path.join(cropped_subdir, os.path.basename(rel_path) + '_cropped' + image_type)
        cv2.imwrite(cropped_path, proc_img)

        # 2. Extract features
        features = extract_features(proc_img)
        train_features_list.append(features)
        train_labels.append(int(label))
    
    # Convert to numpy array
    train_data = np.array(train_features_list, dtype=np.float32)
    train_labels = np.array(train_labels, dtype=np.int32)

    # --- Train scikit-learn classifier ---
    print(f"Training scikit-learn KNN with k={k_value}...")
    
    # 1. Create a StandardScaler
    scaler = StandardScaler()
    
    # 2. Create KNN classifier optimized for TurtleBot3
    # Use ball_tree for faster inference on embedded systems
    knn = KNeighborsClassifier(n_neighbors=k_value, weights='distance', algorithm='ball_tree')
    
    # 3. Create a pipeline
    model_pipeline = make_pipeline(scaler, knn)
    
    # 4. Fit the pipeline (scales and then trains)
    model_pipeline.fit(train_data, train_labels)

    print("scikit-learn model created!")
    print(f"Training data shape: {train_data.shape}")
    print(f"Feature dimensionality: {train_data.shape[1]} (optimized for TurtleBot3)")

    if save_model:
        # Save the *entire pipeline* (scaler + model)
        with open(model_filename + '.pkl', 'wb') as f:
            pickle.dump(model_pipeline, f)
        print(f"scikit-learn pipeline saved to {model_filename}.pkl")
    
    return model_pipeline

def test_model(data_path, test_lines, image_type, model_pipeline, show_img):
    """
    Tests the trained scikit-learn model pipeline.
    """
    if(show_img):
        cv2.namedWindow( 'Tested Image', cv2.WINDOW_AUTOSIZE )

    correct = 0.0
    confusion_matrix = np.zeros((6,6))
    
    # Create directory for cropped test images
    cropped_test_dir = os.path.join(os.getcwd(), 'cropped_images', 'TEST')
    os.makedirs(cropped_test_dir, exist_ok=True)

    for rel_path, test_label in test_lines:
        img_path = os.path.join(data_path, rel_path + image_type)
        original_img = cv2.imread(img_path)
        if original_img is None:
            print(f"Warning: Could not load image {img_path}")
            continue
        
        test_label = int(test_label)

        # 1. Preprocess (Crop and Resize)
        proc_img = preprocess_and_crop(original_img, test_label)
        if proc_img is None:
            continue
        
        # Save cropped test image
        cropped_subdir = os.path.join(cropped_test_dir, os.path.dirname(rel_path))
        os.makedirs(cropped_subdir, exist_ok=True)
        cropped_path = os.path.join(cropped_subdir, os.path.basename(rel_path) + '_cropped' + image_type)
        cv2.imwrite(cropped_path, proc_img)
        
        # 2. Extract features
        test_features = extract_features(proc_img)
        test_features = test_features.reshape(1, -1).astype(np.float32)
        
        if(show_img):
            cv2.imshow('Tested Image', proc_img)
            key = cv2.waitKey(0)
            if key==27:    # Esc key to stop
                break

        # 3. Predict using the pipeline (automatically scales and predicts)
        ret = model_pipeline.predict(test_features)[0]
        ret = int(ret)

        if test_label == ret:
            print(str(rel_path) + " Correct, " + str(ret))
            correct += 1
        else:
            print(str(rel_path) + " Wrong, " + str(test_label) + " classified as " + str(ret))
        
        confusion_matrix[test_label][ret] += 1
    
    total_accuracy = correct/len(test_lines) if len(test_lines) > 0 else 0.0
    print("\n\nTotal accuracy: " + str(total_accuracy))
    print("Confusion Matrix:")
    print(confusion_matrix)
    
    # Calculate per-class metrics
    num_classes = 6
    class_names = ['Empty Wall', 'Left', 'Right', 'Do Not Enter', 'Stop', 'Goal']
    precisions = []
    recalls = []
    f1_scores = []
    
    for c in range(num_classes):
        tp = confusion_matrix[c, c]
        fp = confusion_matrix[:, c].sum() - tp
        fn = confusion_matrix[c, :].sum() - tp
        
        prec = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        rec = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        f1 = (2 * prec * rec) / (prec + rec) if (prec + rec) > 0 else 0.0
        
        precisions.append(prec)
        recalls.append(rec)
        f1_scores.append(f1)
    
    macro_f1 = sum(f1_scores) / num_classes
    
    print("\nPer-class metrics:")
    for c in range(num_classes):
        print(f"Class {c} ({class_names[c]}): Precision={precisions[c]:.3f}, Recall={recalls[c]:.3f}, F1={f1_scores[c]:.3f}")
    print(f"Macro F1: {macro_f1:.3f}")
    
    # Plot F1 scores
    plt.figure(figsize=(10, 6))
    bars = plt.bar(range(num_classes), f1_scores, color=['#e74c3c', '#c0392b', '#3498db', '#2ecc71', '#f39c12', '#9b59b6'])
    plt.xlabel('Class', fontsize=12)
    plt.ylabel('F1 Score', fontsize=12)
    plt.title('F1 Score per Class', fontsize=14, fontweight='bold')
    plt.xticks(range(num_classes), [f"{i}\n{class_names[i]}" for i in range(num_classes)], fontsize=10)
    plt.ylim(0, 1.0)
    plt.grid(axis='y', alpha=0.3, linestyle='--')
    
    # Add value labels on bars
    for i, (bar, f1) in enumerate(zip(bars, f1_scores)):
        plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02, 
                f'{f1:.3f}', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # Add macro F1 line
    plt.axhline(y=macro_f1, color='black', linestyle='--', linewidth=2, label=f'Macro F1: {macro_f1:.3f}')
    plt.legend(loc='upper right', fontsize=11)
    
    plt.tight_layout()
    plt.savefig('f1_scores.png', dpi=300, bbox_inches='tight')
    print("\nF1 score plot saved to 'f1_scores.png'")
    plt.show()

# --- Other functions (check_split_value_range, check_k_value, data loading/splitting, augmentation) ---
# --- (These functions from your 'example_knn.py' are excellent and can be pasted in here) ---
# --- (Pasting them here for completeness) ---

def check_split_value_range(val):
    try:
        float_val = float(val)
        if float_val < 0 or float_val > 1:
            raise argparse.ArgumentTypeError(f"Invalid ratio {float_val}. Must be in [0, 1]!")
        return float_val
    except ValueError:
        raise argparse.ArgumentTypeError(f"Received '{val}' which is not a valid float!")

def check_k_value(val):
    try:
        int_val = int(val)
        if float(val) != int_val:
            raise argparse.ArgumentTypeError(f"Received '{val}' which is a float not an integer.")
        if int_val % 2 == 0 or int_val < 1:
            raise argparse.ArgumentTypeError(f"Received '{val}'. Must be a positive, odd integer!")
        return int_val
    except ValueError:
        raise argparse.ArgumentTypeError(f"Received '{val}' which is not a valid integer!")

def create_augmentation_pipeline(label):
    """
    Create class-specific augmentation pipeline.
    Classes 0, 3, 4, 5: horizontal flip, vertical flip, rotation
    Classes 1, 2: vertical flip only (arrows should not be horizontally flipped)
    """
    label = int(label)
    
    # Base augmentations for all classes
    base_transforms = [
        # A.RandomBrightnessContrast(brightness_limit=0.3, contrast_limit=0.3, p=0.8),
        A.Affine(scale=(0.85, 1.15), p=0.6),
        A.ElasticTransform(alpha=20, sigma=5, p=0.5),
        A.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=20, val_shift_limit=10, p=0.6),
        A.GaussNoise(p=0.3),
        # A.GaussianBlur(blur_limit=3, p=0.2),
        A.RandomScale(scale_limit=0.2, p=0.6),  # Scale by ±20%
        A.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1, p=0.7),  # Color jittering
        A.RandomResizedCrop(size=(64, 64), scale=(0.8, 1.0), ratio=(0.9, 1.1), p=0.5),  # Random crop and resize
    ]
    
    # Class-specific flipping and rotation
    if label in [0, 3, 4, 5]:  # Stop, Do Not Enter, U-Turn, Goal - can flip both ways and rotate
        class_specific = [
            A.HorizontalFlip(p=0.5),
            A.VerticalFlip(p=0.5),
            A.Rotate(limit=15, p=0.7),  # Random rotation up to ±15 degrees
        ]
    elif label in [1, 2]:  # Left and Right arrows - vertical flip only (not horizontal)
        class_specific = [
            A.VerticalFlip(p=0.5),
            A.Rotate(limit=10, p=0.5),  # Smaller rotation for directional signs
        ]
    else:
        class_specific = []
    
    return A.Compose(base_transforms + class_specific)

def apply_augmentation(image, aug_pipeline, num_augmented=1):
    augmented_images = []
    for _ in range(num_augmented):
        aug_img = aug_pipeline(image=image)['image']
        augmented_images.append(aug_img)
    return augmented_images

def generate_augmented_data(data_path, train_lines, split_train, image_type, augmentation_multiplier):
    if augmentation_multiplier <= 0:
        return []
    
    aug_train_lines = []
    aug_per_image = max(1, int(augmentation_multiplier))
    max_aug_num = 0
    
    # Find max existing aug number
    for root, dirs, files in os.walk(split_train):
        for fname in files:
            if fname.startswith('aug_') and (fname.endswith(image_type)):
                try:
                    parts = fname.replace('aug_', '').replace(image_type, '').split('_')
                    max_aug_num = max(max_aug_num, int(parts[-1]))
                except (ValueError, IndexError): pass
    
    aug_counter = max_aug_num + 1
    
    for rel_path, label in train_lines:
        src = os.path.join(data_path, rel_path + image_type)
        if not os.path.isfile(src): continue
        img = cv2.imread(src)
        if img is None: continue
        
        # Create class-specific augmentation pipeline
        aug_pipeline = create_augmentation_pipeline(label)
        augmented_imgs = apply_augmentation(img, aug_pipeline, aug_per_image)
        
        for aug_img in augmented_imgs:
            subdir = os.path.join(split_train, os.path.dirname(rel_path))
            if not os.path.exists(subdir):
                os.makedirs(subdir, exist_ok=True)
            
            aug_basename = f"aug_{os.path.basename(rel_path).replace(image_type, '')}_{aug_counter}"
            aug_rel_path = os.path.join(os.path.dirname(rel_path), aug_basename)
            aug_full_path = os.path.join(subdir, aug_basename + image_type)
            
            cv2.imwrite(aug_full_path, aug_img)
            aug_train_lines.append([aug_rel_path, label])
            aug_counter += 1
            
    return aug_train_lines

def load_from_prepared_split(train_dir, test_dir, image_type='.png'):
    """
    Load training and testing data from pre-prepared split directories.
    Assumes directories have labels.txt files already created.
    """
    train_lines = []
    test_lines = []
    
    # Load training data
    train_dir = os.path.normpath(train_dir)
    root_labels = os.path.join(train_dir, 'labels.txt')
    if os.path.isfile(root_labels):
        with open(root_labels, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2: train_lines.append([row[0].strip(), row[1].strip()])
    
    for entry in os.listdir(train_dir):
        subdir = os.path.join(train_dir, entry)
        if os.path.isdir(subdir):
            labels_file = os.path.join(subdir, 'labels.txt')
            if os.path.isfile(labels_file):
                with open(labels_file, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if len(row) >= 2: train_lines.append([os.path.join(entry, row[0].strip()), row[1].strip()])
    
    # Load testing data
    test_dir = os.path.normpath(test_dir)
    root_labels = os.path.join(test_dir, 'labels.txt')
    if os.path.isfile(root_labels):
        with open(root_labels, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2: test_lines.append([row[0].strip(), row[1].strip()])
    
    for entry in os.listdir(test_dir):
        subdir = os.path.join(test_dir, entry)
        if os.path.isdir(subdir):
            labels_file = os.path.join(subdir, 'labels.txt')
            if os.path.isfile(labels_file):
                with open(labels_file, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if len(row) >= 2: test_lines.append([os.path.join(entry, row[0].strip()), row[1].strip()])
    
    print(f"Loaded {len(train_lines)} training samples from {train_dir}")
    print(f"Loaded {len(test_lines)} testing samples from {test_dir}")
    
    return train_lines, test_lines, train_dir, test_dir

def load_and_split_data(data_path, split_ratio, image_type='.png', augmentation_multiplier=0.0):
    lines = []
    data_path = os.path.normpath(data_path)
    
    # Check root dir
    root_labels = os.path.join(data_path, 'labels.txt')
    if os.path.isfile(root_labels):
        with open(root_labels, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2: lines.append([row[0].strip(), row[1].strip()])
    
    # Check one level deep
    for entry in os.listdir(data_path):
        subdir = os.path.join(data_path, entry)
        if os.path.isdir(subdir):
            labels_file = os.path.join(subdir, 'labels.txt')
            if os.path.isfile(labels_file):
                with open(labels_file, 'r') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if len(row) >= 2: lines.append([os.path.join(entry, row[0].strip()), row[1].strip()])
    
    # Create split folders
    cwd = os.path.abspath(os.getcwd())
    split_root = os.path.join(cwd, 'split')
    split_train = os.path.join(split_root, 'TRAIN')
    split_test = os.path.join(split_root, 'TEST')
    if os.path.exists(split_root): shutil.rmtree(split_root)
    os.makedirs(split_train, exist_ok=True)
    os.makedirs(split_test, exist_ok=True)

    # Stratified split
    groups = {}
    for item in lines: groups.setdefault(int(item[1]), []).append(item)
    train_lines, test_lines = [], []
    for lbl, items in groups.items():
        random.shuffle(items)
        n_train = math.floor(len(items) * split_ratio)
        train_lines.extend(items[:n_train])
        test_lines.extend(items[n_train:])
    
    # Function to copy files and create label maps
    def copy_items(items, dest_root):
        labels_map = {}
        for rel_path, label in items:
            src = os.path.join(data_path, rel_path + image_type)
            dest = os.path.join(dest_root, rel_path + image_type)
            dest_dir = os.path.dirname(dest)
            rel_subdir = os.path.relpath(dest_dir, dest_root) if dest_dir != dest_root else ''
            if not os.path.exists(dest_dir): os.makedirs(dest_dir, exist_ok=True)
            
            if os.path.isfile(src):
                shutil.copyfile(src, dest)
                base = os.path.splitext(os.path.basename(dest))[0]
                labels_map.setdefault(rel_subdir, []).append((base, int(label)))
            else:
                print(f"Warning: source image not found: {src}")
        return labels_map
    
    train_labels_map = copy_items(train_lines, split_train)
    test_labels_map = copy_items(test_lines, split_test)
    
    # Function to write labels.txt
    def write_labels_map(labels_map, dest_root):
        for subdir, entries in labels_map.items():
            labels_path = os.path.join(dest_root, subdir, 'labels.txt')
            os.makedirs(os.path.dirname(labels_path), exist_ok=True)
            with open(labels_path, 'w', newline='') as f:
                for name, lbl in entries: f.write(f"{name}, {lbl}\n")

    write_labels_map(train_labels_map, split_train)
    write_labels_map(test_labels_map, split_test)
    
    # Augmentation
    aug_train_lines = generate_augmented_data(data_path, train_lines, split_train, image_type, augmentation_multiplier)
    if aug_train_lines:
        aug_labels_map = {}
        for aug_rel_path, label in aug_train_lines:
            rel_subdir = os.path.dirname(aug_rel_path)
            aug_labels_map.setdefault(rel_subdir, []).append((os.path.basename(aug_rel_path), int(label)))
        
        for subdir, entries in aug_labels_map.items():
            train_labels_map.setdefault(subdir, []).extend(entries)
        
        write_labels_map(train_labels_map, split_train) # Rewrite labels with aug
        train_lines.extend(aug_train_lines)

    return train_lines, test_lines, split_train, split_test

def main():
    parser = argparse.ArgumentParser(description="High-Accuracy KNN Trainer for 7785 Lab 6!")
    parser.add_argument("-p","--data_path", type=str, required=False, help="Path to the valid dataset directory (for splitting mode)")
    parser.add_argument("-r","--data_split_ratio", type=check_split_value_range, required=False, default=0.8, help="Ratio of data for training (e.g., 0.8 for 80/20 split).")
    parser.add_argument("-k","--knn-value", type=check_k_value, required=False, default=5, help="KNN value (positive, odd integer).")
    parser.add_argument("-i","--image_type", type=str, required=False, default=".png", help="Extension of the image files (e.g. .png, .jpg)")
    parser.add_argument("-s","--save_model_bool", action='store_true', required=False, help="Flag to save the model.")
    parser.add_argument("-n","--model_filename", type=str, required=False, default="knn_model_pipeline", help="Filename of the saved KNN model (will be .pkl).")
    parser.add_argument("-t","--dont_test_model_bool", action='store_false', required=False, help="Flag to *not* test the model on the split testing set.")
    parser.add_argument("-d","--show_img", action='store_true', required=False, help="Flag to show images as they are classified.")
    parser.add_argument("-a","--augmentation_multiplier", type=float, required=False, default=2.0, help="Multiplier for augmented data (e.g., 2.0 = 2 new images per original).")
    
    # New arguments for direct training mode
    parser.add_argument("--train_dir", type=str, required=False, help="Path to pre-prepared TRAIN directory (skips data splitting/augmentation)")
    parser.add_argument("--test_dir", type=str, required=False, help="Path to pre-prepared TEST directory (skips data splitting/augmentation)")
    
    args = parser.parse_args()

    # Check if using direct training mode or data preparation mode
    if args.train_dir and args.test_dir:
        # Direct training mode - use pre-prepared directories
        print("\n=== DIRECT TRAINING MODE ===")
        print(f"Loading from pre-prepared directories:")
        print(f"  Train: {args.train_dir}")
        print(f"  Test: {args.test_dir}\n")
        
        train_lines, test_lines, train_path, test_path = load_from_prepared_split(
            args.train_dir, args.test_dir, args.image_type
        )
    elif args.data_path:
        # Data preparation mode - split and augment
        print("\n=== DATA PREPARATION MODE ===")
        print(f"Processing dataset from: {args.data_path}\n")
        
        train_lines, test_lines, train_path, test_path = load_and_split_data(
            args.data_path, args.data_split_ratio, args.image_type, args.augmentation_multiplier
        )
    else:
        parser.error("Either --data_path OR (--train_dir AND --test_dir) must be provided")
    
    # Train model
    model_pipeline = train_model(
        train_path, train_lines, args.image_type, args.model_filename, args.save_model_bool, args.knn_value
    )
    
    # Test model
    if args.dont_test_model_bool:
        print("\n--- Testing Model on Validation Set ---")
        test_model(test_path, test_lines, args.image_type, model_pipeline, args.show_img)

if __name__ == "__main__":
    main()