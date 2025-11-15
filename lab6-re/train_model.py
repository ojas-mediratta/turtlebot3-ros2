import os
import cv2
import numpy as np
import csv
import argparse
import pickle
from collections import Counter
from sklearn.model_selection import train_test_split
from sklearn.neighbors import KNeighborsClassifier
from sklearn.svm import SVC
from sklearn.metrics import classification_report, accuracy_score

# Import our shared processing functions
from supplemental.utils import crop_sign, extract_features, plot_confusion_matrix, plot_f1_scores

def load_data_paths(data_dir):
    """
    Loads all image paths and labels from the data directory.
    Assumes data_dir contains 'labels.txt' and image files.
    """
    all_data = []
    labels_file = os.path.join(data_dir, 'labels.txt')
    if not os.path.isfile(labels_file):
        print(f"Error: 'labels.txt' not found in {data_dir}")
        return []
        
    with open(labels_file, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row: continue
            try:
                img_name = f"{row[0].strip()}.png"
                label = int(row[1].strip())
                img_path = os.path.join(data_dir, img_name)
                
                if os.path.isfile(img_path):
                    all_data.append((img_path, label))
                else:
                    print(f"Warning: Image file not found: {img_path}")
            except Exception as e:
                print(f"Warning: Skipping malformed row: {row}. Error: {e}")
                
    print(f"Found {len(all_data)} images in {data_dir}.")
    return all_data

def preprocess_data(data_list):
    """
    Loads images, crops, extracts features, and returns feature/label arrays.
    
    --- V4 Change ---
    This now SKIPS images where crop_sign returns None (i.e., Class 0).
    The model will only be trained on data from classes 1-5.
    """
    X_features = []
    y_labels = []
    
    print("Processing images and extracting features...")
    for (img_path, label) in data_list:
        image = cv2.imread(img_path)
        if image is None:
            print(f"Warning: Failed to read {img_path}. Skipping.")
            continue
            
        # 1. Crop the image (using shared function)
        cropped_image = crop_sign(image)
        
        # --- V4 Change ---
        if cropped_image is None:
            # This is (most likely) a Class 0 image.
            # We will NOT include it in the SVM training data.
            continue
            
        # 2. Extract features (using shared function)
        features = extract_features(cropped_image)
        
        X_features.append(features)
        y_labels.append(label)
        
    print(f"Successfully processed {len(y_labels)} images.")
    return np.array(X_features), np.array(y_labels)

def main():
    parser = argparse.ArgumentParser(description="Train a sign classifier (KNN or SVM)")
    parser.add_argument('--data_dir', type=str, default='SPLIT/TRAIN',
                        help="Directory containing training data (from prepare_data.py)")
    parser.add_argument('--model_type', type=str, required=True, choices=['knn', 'svm'],
                        help="Type of model to train (knn or svm)")
    parser.add_argument('--val_split', type=float, default=0.2,
                        help="Fraction of training data to use for validation (e.g., 0.2 for 20%)")
    parser.add_argument('--save_path', type=str, default='model.pkl',
                        help="Path to save the trained model file")
    # CM plot argument
    parser.add_argument('--cm_path', type=str, default='validation_cm.png',
                        help="Path to save the validation confusion matrix image")
    # NEW ARGUMENT for the F1 plot
    parser.add_argument('--f1_path', type=str, default='validation_f1.png',
                        help="Path to save the validation F1-scores image")
    
    # KNN arguments
    parser.add_argument('--k', type=int, default=5,
                        help="Number of neighbors (K) for KNN")
    
    # SVM arguments
    parser.add_argument('--svm_c', type=float, default=1.0,
                        help="Regularization parameter (C) for SVM")
    parser.add_argument('--svm_kernel', type=str, default='rbf',
                        choices=['linear', 'poly', 'rbf', 'sigmoid'],
                        help="Kernel type for SVM")
    # NEW: Added gamma argument for SVM
    parser.add_argument('--svm_gamma', type=str, default='scale',
                        help="Kernel coefficient (gamma) for RBF/poly/sigmoid SVM. Can be 'scale', 'auto', or a float value.")

    args = parser.parse_args()

    # 1. Load data paths and labels
    all_data = load_data_paths(args.data_dir)
    if not all_data:
        return

    # 2. Preprocess all data (crop + extract features)
    X, y = preprocess_data(all_data)
    
    print(f"Total features extracted: {X.shape[0]} (from {len(all_data)} images)")
    print(f"Feature vector length: {X.shape[1]}")
    print(f"Class distribution (of data fed to SVM): {Counter(y)}")

    # Define class names based on PDF (0-5)
    class_names = [
        "0: empty", 
        "1: left", 
        "2: right", 
        "3: do not enter", 
        "4: stop", 
        "5: goal"
    ]
    
    # --- V4 Change ---
    # The class names for the *model* are now only 1-5
    model_class_names = [
        "1: left", 
        "2: right", 
        "3: do not enter", 
        "4: stop", 
        "5: goal"
    ]
    model_labels = [1, 2, 3, 4, 5]


    # 3. Create the train/validation split
    # MODIFIED: Handle val_split = 0.0
    if args.val_split > 0.0:
        print(f"Splitting data into training and validation ({1.0 - args.val_split} / {args.val_split})...")
        X_train, X_val, y_train, y_val = train_test_split(
            X, y,
            test_size=args.val_split,
            stratify=y,
            random_state=42
        )
        print(f"Train samples: {len(y_train)}, Validation samples: {len(y_val)}")
    else:
        print("Validation split is 0.0. Training on 100% of the data. Skipping validation.")
        X_train, y_train = X, y
        X_val, y_val = None, None # Set validation data to None
        print(f"Train samples: {len(y_train)}")

    # 4. Initialize the model
    if args.model_type == 'knn':
        print(f"Initializing KNN model with k={args.k}...")
        model = KNeighborsClassifier(n_neighbors=args.k)
        
    elif args.model_type == 'svm':
        # MODIFIED: Handle gamma value (can be float, 'scale', or 'auto')
        try:
            gamma_val = float(args.svm_gamma)
        except ValueError:
            gamma_val = args.svm_gamma

        print(f"Initializing SVM model with C={args.svm_c}, kernel='{args.svm_kernel}', gamma='{gamma_val}'...")
        model = SVC(
            C=args.svm_c, 
            kernel=args.svm_kernel, 
            gamma=gamma_val,
            probability=True, 
            random_state=42
        )

    # 5. Train the model
    print(f"Training {args.model_type.upper()} model...")
    model.fit(X_train, y_train)
    print("Training complete.")

    # 6. Evaluate on the validation set
    # MODIFIED: Only run validation if X_val exists
    if X_val is not None and y_val is not None:
        print("\n--- Validation Results ---")
        y_pred = model.predict(X_val)
        accuracy = accuracy_score(y_val, y_pred)
        # Ensure labels match class_names for the report
        
        # --- V4 Change ---
        # The report should now only cover classes 1-5
        print(classification_report(y_val, y_pred, target_names=model_class_names, labels=model_labels))
        
        plot_confusion_matrix(y_val, y_pred, model_class_names, save_path=args.cm_path)
        
        plot_f1_scores(y_val, y_pred, model_class_names, save_path=args.f1_path)
    else:
        print("\nSkipping validation.")
    
    # 7. Save the model
    print(f"\nSaving trained model to {args.save_path}...")
    with open(args.save_path, 'wb') as f:
        pickle.dump(model, f)
    print("Model saved successfully.")

if __name__ == "__main__":
    main()