import os
import cv2
import numpy as np
import csv
import argparse
import pickle
from collections import Counter
from sklearn.metrics import classification_report, accuracy_score, confusion_matrix

# Import our shared processing and plotting functions
from utils import (
    crop_sign, 
    extract_features, 
    plot_confusion_matrix, 
    plot_f1_scores
)

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

def main():
    parser = argparse.ArgumentParser(description="Evaluate a classifier on the TEST set with plots")
    parser.add_argument('--data_path', type=str, required=True,
                        help="Path to the test dataset directory (e.g., SPLIT/TEST)")
    parser.add_argument('--model_path', type=str, required=True,
                        help="Path to the trained .pkl model file")
    parser.add_argument('--cm_path', type=str, default='test_cm.png',
                        help="Path to save the test confusion matrix image")
    parser.add_argument('--f1_path', type=str, default='test_f1.png',
                        help="Path to save the test F1-scores image")
    # --- NEW ARGUMENT ---
    parser.add_argument('--threshold', type=float, default=0.0,
                        help="Confidence threshold for overriding to Class 0. Set to 0.0 to disable. (e.g., 0.5)")
    args = parser.parse_args()

    # 1. Load the Model
    print(f"Loading model from {args.model_path}...")
    try:
        with open(args.model_path, 'rb') as f:
            model = pickle.load(f)
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    print("Model loaded.")
    
    # 2. Load Test Data
    print(f"Loading data from {args.data_path}...")
    test_data = load_data_paths(args.data_path)
    if not test_data:
        print(f"No test data found in {args.data_path}. Exiting.")
        return
        
    # --- V4 Change: We still need the *full* list of names for the final report ---
    FULL_CLASS_NAMES = [
        "0: empty", 
        "1: left", 
        "2: right", 
        "3: do not enter", 
        "4: stop", 
        "5: goal"
    ]
    
    # 3. Process and Predict
    print("Running predictions on test set...")
    if args.threshold > 0.0:
        print(f"--- Using confidence threshold: {args.threshold} ---")
        
    # --- NEW: Initialize lists ---
    all_y_true = []
    all_y_pred = []
    
    for (img_path, y_true) in test_data:
        image = cv2.imread(img_path)
        if image is None:
            print(f"Warning: Failed to read {img_path}. Skipping.")
            continue
        
        # Run the same preprocessing pipeline
        cropped_image = crop_sign(image)
        
        # --- V4: 2-STAGE PREDICTION LOGIC ---
        
        # --- MODIFIED: More robust check ---
        if cropped_image is None or cropped_image.size == 0:
            # Stage 1: No sign found or crop failed. Predict 0.
            y_pred = 0
        else:
            # Stage 2: A sign was found. Let the SVM decide.
            features = extract_features(cropped_image)
            features_2d = features.reshape(1, -1)
            
            # Get probabilities
            probabilities = model.predict_proba(features_2d)[0]
            max_prob = np.max(probabilities)
            
            if args.threshold > 0.0 and max_prob < args.threshold:
                # SVM is not confident, override to 0
                y_pred = 0
            else:
                # SVM is confident, use its prediction.
                # model.predict() will return 1, 2, 3, 4, or 5
                y_pred = model.predict(features_2d)[0]
                
        # --- END V4 LOGIC ---
        
        all_y_true.append(y_true)
        all_y_pred.append(int(y_pred))

    print("Evaluation complete.")

    # 4. Show Results
    print("\n--- Test Set Results ---")
    accuracy = accuracy_score(all_y_true, all_y_pred)
    print(f"Test Accuracy: {accuracy * 100:.2f}%")
    
    # V4: Use the full class names for the report
    labels_for_report = [0, 1, 2, 3, 4, 5]
    
    print("\nClassification Report:")
    print(classification_report(all_y_true, all_y_pred, target_names=FULL_CLASS_NAMES, labels=labels_for_report))
    
    # 5. Generate Plots
    print("Generating plots...")
    plot_confusion_matrix(all_y_true, all_y_pred, FULL_CLASS_NAMES, save_path=args.cm_path)
    plot_f1_scores(all_y_true, all_y_pred, FULL_CLASS_NAMES, save_path=args.f1_path)
    
    print("\nDone. Plots have been saved and displayed.")

if __name__ == "__main__":
    main()