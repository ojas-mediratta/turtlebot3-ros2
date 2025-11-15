#!/usr/bin/env python3
import os
import sys
import argparse
import csv
import cv2
import numpy as np


# --- 1. MODIFIED: imports ---

import pickle
from supplemental.utils import crop_sign, extract_features

# ------------------------------------------------------------------------------
#                  DO NOT MODIFY FUNCTION NAMES OR ARGUMENTS
# ------------------------------------------------------------------------------

PREDICTION_THRESHOLD = 0.41 # Default threshold; can be modified in predict()

def initialize_model(model_path=None):
    """
    Initialize and return your trained model.
    You MUST modify this function to load and/or construct your model.
    DO NOT change the function name or its input/output.
    
    Args:
        model_path: The path to your pretrained model file (if one is needed).
    Returns:
        model: Your trained model.
    """

    # --- 2. MODIFIED: initialize_model ---
    # Load the trained model from the .pkl file
    
    if model_path is None:
        raise ValueError("Error: --model_path is required to load a .pkl model.")
    
    print(f"Loading model from: {model_path}")
    
    try:
        with open(model_path, 'rb') as f:
            model = pickle.load(f)
    except Exception as e:
        print(f"Error: Failed to load model from {model_path}. {e}")
        sys.exit(1)
        
    print("Model loaded successfully.")
    
    return model

def predict(model, image):
    """
    Run inference on a single image using your model.
    You MUST modify this function to perform prediction.
    DO NOT change the function signature.
    
    Args:
        model: The model object returned by initialize_model().
        image: The input image (as a NumPy array) to classify.
    
    Returns:
        int: The predicted class label.
    """
    # --- 3. MODIFIED: predict (V4 2-STAGE LOGIC) ---
    
    # 1. Crop the image
    # crop_sign() will return None if no sign is found
    cropped_image = crop_sign(image)
    
    if cropped_image is None:
        # Stage 1: No sign found by color detection.
        # Predict Class 0.
        return 0
        
    # Stage 2: A sign was found. Let the SVM (Classes 1-5) decide.
    
    # 2. Extract features (HOG + Color)
    features = extract_features(cropped_image)
    
    # 3. Reshape features for the model
    features_2d = features.reshape(1, -1)
    
    # 4. Predict with confidence threshold
    probabilities = model.predict_proba(features_2d)[0]
    max_prob = np.max(probabilities)
    
    final_prediction = 0 # Default to 0
    
    if PREDICTION_THRESHOLD > 0.0 and max_prob < PREDICTION_THRESHOLD:
        # SVM is not confident *which* sign it is,
        # so it's probably noise. Override to 0.
        final_prediction = 0
    else:
        # SVM is confident. Use its prediction (1-5).
        final_prediction = model.predict(features_2d)[0]
    
    # Return the prediction as a single integer
    return int(final_prediction)
# ------------------------------------------------------------------------------
#                      DO NOT MODIFY ANY CODE BELOW THIS LINE
# ------------------------------------------------------------------------------

def load_validation_data(data_path):
    """
    Load validation images and labels from the given directory.
    Expects a 'labels.txt' file in the directory and images in .png format.
    
    Args:
        data_path (str): Path to the validation dataset.
    
    Returns:
        list of tuples: Each tuple contains (image_path, true_label)
    """
    labels_file = os.path.join(data_path, "labels.txt")
    data = []
    with open(labels_file, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            # Assumes row[0] is the image filename (without extension) and row[1] is the label.
            image_file = os.path.join(data_path, row[0] + ".png")  # Modify if images use a different extension.
            data.append((image_file, int(row[1])))
    return data

def evaluate_model(model, validation_data):
    """
    Evaluate the model on the validation dataset.
    Computes and prints the confusion matrix and overall accuracy.
    
    Args:
        model: The model object.
        validation_data (list): List of tuples (image_path, true_label).
    """
    num_classes = 6  # Number of classes (adjust if needed)
    confusion_matrix = np.zeros((num_classes, num_classes), dtype=np.int32)
    correct = 0
    total = len(validation_data)
    
    for image_path, true_label in validation_data:
        # Read the image
        image = cv2.imread(image_path)
        if image is None:
            print("Warning: Could not load image:", image_path)
            continue
        # Get the predicted label using the student's implementation.
        predicted_label = predict(model, image)
        
        if predicted_label == true_label:
            correct += 1
        confusion_matrix[true_label][predicted_label] += 1
        print(f"Image: {os.path.basename(image_path)} - True: {true_label}, Predicted: {predicted_label}")
    
    accuracy = correct / total if total > 0 else 0
    print("\nTotal accuracy:", accuracy)
    print("Confusion Matrix:")
    print(confusion_matrix)

def main():
    parser = argparse.ArgumentParser(description="Model Grader for Lab 6")
    parser.add_argument("--data_path", type=str, required=True,
                        help="Path to the validation dataset directory (must contain labels.txt and images)")
    parser.add_argument("--model_path", type=str, required=False,
                        help="Path to the trained model file (if applicable)")
    args = parser.parse_args()
    
    # Path to the validation dataset directory from command line argument.
    VALIDATION_DATASET_PATH = args.data_path

    # Path to the trained model file from command line argument.
    MODEL_PATH = args.model_path
    
    # Load validation data.
    validation_data = load_validation_data(VALIDATION_DATASET_PATH)
    
    # Initialize the model using the student's implementation.
    model = initialize_model(MODEL_PATH) if MODEL_PATH else initialize_model()
    
    # Evaluate the model on the validation dataset.
    evaluate_model(model, validation_data)

if __name__ == "__main__":
    main()
