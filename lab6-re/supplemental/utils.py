import cv2
import numpy as np
from skimage.feature import hog
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, f1_score
import pandas as pd

# --- Configuration ---
# This is the standard size all images will be resized to after cropping.
IMG_SIZE = (128, 128)

# --- Cropping Function ---

def crop_sign(image):
    """
    Finds the largest sign-like object in the image using color segmentation,
    crops it, and resizes it.
    
    This function is SHARED between data prep, training, and prediction.
    
    Returns:
        A resized image (IMG_SIZE, IMG_SIZE, 3) or None if no contour is found.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV color ranges for the signs - BROADENED for better detection
    # Note: Red wraps around in HSV, so we need two ranges
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    
    # Blue range - broadened
    lower_blue = np.array([90, 100, 40])
    upper_blue = np.array([130, 255, 255])
    
    # Yellow/Green range - broadened
    lower_green_yellow = np.array([20, 40, 50])
    upper_green_yellow = np.array([100, 255, 255])

    # Brown range - broadened for goal sign
    lower_brown = np.array([10, 30, 30])
    upper_brown = np.array([30, 255, 220])
    
    # Orange range (for some red-orange signs)
    lower_orange = np.array([11, 50, 50])
    upper_orange = np.array([25, 255, 255])

    # Create masks for SIGNS
    mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_gy = cv2.inRange(hsv, lower_green_yellow, upper_green_yellow)
    mask_br = cv2.inRange(hsv, lower_brown, upper_brown)
    mask_or = cv2.inRange(hsv, lower_orange, upper_orange)
    
    # Combine *sign* masks
    combined_sign_mask = mask_r1 | mask_r2 | mask_b | mask_gy | mask_br | mask_or
    
    # Apply morphological operations to clean up the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    combined_sign_mask = cv2.morphologyEx(combined_sign_mask, cv2.MORPH_CLOSE, kernel)
    combined_sign_mask = cv2.morphologyEx(combined_sign_mask, cv2.MORPH_OPEN, kernel)

    # --- V5: ADD A CONTOUR AREA THRESHOLD ---
    # We will ignore any "sign" that is smaller than this many pixels
    # to filter out noise, shadows, and reflections.
    MIN_CONTOUR_AREA = 300 # Further lowered to capture smaller signs
    
    # Find contours for SIGNS
    all_sign_contours, _ = cv2.findContours(combined_sign_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter out contours that are too small
    sign_contours = [
        c for c in all_sign_contours 
        if cv2.contourArea(c) > MIN_CONTOUR_AREA
    ]
    
    # --- End V5 Change ---
    
    if not sign_contours:
        # No *large enough* sign was found. This is Class 0.
        # We must return None here.
        return None

    # A sign was found (Class 1-5). Find the largest sign.
    largest_contour = max(sign_contours, key=cv2.contourArea)

    # Get the bounding box for the largest_contour
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    # Add some padding (e.g., 10% of width/height)
    pad_w = int(w * 0.10)
    pad_h = int(h * 0.10)
    
    # Clamp coordinates to be within the image bounds
    y1 = max(0, y - pad_h)
    y2 = min(image.shape[0], y + h + pad_h)
    x1 = max(0, x - pad_w)
    x2 = min(image.shape[1], x + w + pad_w)
    
    cropped_image = image[y1:y2, x1:x2]
    
    if cropped_image.size == 0:
        # Fallback in case of bad crop
        print("Warning: Bad crop. Returning None.")
        return None
        
    # Resize the cropped image to our standard size
    return cv2.resize(cropped_image, IMG_SIZE)

# --- Feature Extraction Function ---

def extract_features(cropped_image):
    """
    Extracts optimized feature vector for TurtleBot3 real-time performance.
    Balances discriminative power with computational efficiency.
    
    Args:
        cropped_image: A (128, 128, 3) BGR image from crop_sign().
    
    Returns:
        A 1D numpy array (feature vector).
    """
    # Apply histogram equalization for lighting robustness
    gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    gray_image = cv2.equalizeHist(gray_image)
    
    # HOG features - balanced parameters for speed and accuracy
    hog_features = hog(gray_image, 
                       orientations=9,
                       pixels_per_cell=(16, 16),    # Balanced cell size
                       cells_per_block=(2, 2),
                       visualize=False,
                       feature_vector=True)
    
    # Color histogram features
    hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
    
    hist_h = cv2.calcHist([hsv], [0], None, [16], [0, 180])
    hist_s = cv2.calcHist([hsv], [1], None, [12], [0, 256])
    hist_v = cv2.calcHist([hsv], [2], None, [12], [0, 256])
    
    cv2.normalize(hist_h, hist_h)
    cv2.normalize(hist_s, hist_s)
    cv2.normalize(hist_v, hist_v)
    
    color_features = np.hstack([hist_h.flatten(), hist_s.flatten(), hist_v.flatten()])
    
    # Edge density features - helps distinguish sign shapes
    edges = cv2.Canny(gray_image, 50, 150)
    edge_density = np.sum(edges > 0) / (edges.shape[0] * edges.shape[1])
    
    # Simple shape features
    moments = cv2.moments(edges)
    hu_moments = cv2.HuMoments(moments).flatten()
    
    return np.hstack([hog_features, color_features, [edge_density], hu_moments])

# --- Evaluation Functions ---

def plot_confusion_matrix(y_true, y_pred, class_names, save_path):
    """
    Calculates, plots, and saves a colored confusion matrix.
    
    Args:
        y_true (list or np.array): True labels.
        y_pred (list or np.array): Predicted labels.
        class_names (list of str): Names of the classes for labels.
        save_path (str): Path to save the PNG image (e.g., "cm.png").
    """
    print(f"Generating confusion matrix at {save_path}...")
    cm = confusion_matrix(y_true, y_pred, labels=np.arange(len(class_names)))
    
    # Normalize the confusion matrix to show percentages (from 0 to 1)
    # This is more informative than raw counts, especially with imbalanced classes
    cm_normalized = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
    cm_normalized = np.nan_to_num(cm_normalized) # Handle divide-by-zero for classes with 0 samples
    
    plt.figure(figsize=(10, 8))
    sns.heatmap(
        cm_normalized, 
        annot=True,     # Show the percentages in each cell
        fmt=".2f",      # Format as a 2-decimal-place float
        cmap="Blues",   # Color scheme
        xticklabels=class_names,
        yticklabels=class_names
    )
    plt.title("Normalized Confusion Matrix")
    plt.ylabel("True Label")
    plt.xlabel("Predicted Label")
    plt.tight_layout()
    plt.savefig(save_path)
    print(f"Confusion matrix saved to {save_path}")
    plt.show() # <-- ADDED: Show the plot
    plt.close() # Free up memory

# --- NEW: F1-Score Plot Function ---

def plot_f1_scores(y_true, y_pred, class_names, save_path):
    """
    Calculates, plots, and saves a bar chart of per-class F1-scores.
    
    Args:
        y_true (list or np.array): True labels.
        y_pred (list or np.array): Predicted labels.
        class_names (list of str): Names of the classes for labels.
        save_path (str): Path to save the PNG image (e.g., "f1.png").
    """
    print(f"Generating F1-Score plot at {save_path}...")
    # Calculate F1-scores for each class
    scores = f1_score(y_true, y_pred, average=None, labels=np.arange(len(class_names)))
    
    # Create a pandas DataFrame for easy plotting with seaborn
    f1_data = pd.DataFrame({
        'Class': class_names,
        'F1-Score': scores
    })
    
    plt.figure(figsize=(10, 6))
    sns.barplot(
        x='Class', 
        y='F1-Score', 
        data=f1_data,
        palette='viridis', # Use a color palette
        hue='Class',       # FIX: Assign hue as per warning
        legend=False       # FIX: Disable the legend
    )
    
    # Add text labels (the scores) on top of each bar
    for index, row in f1_data.iterrows():
        plt.text(index, row['F1-Score'] + 0.01, f"{row['F1-Score']:.2f}", 
                 color='black', ha="center")
                 
    plt.title("Per-Class F1-Scores")
    plt.ylim(0, 1.05) # Set y-axis limit to 0-1 (plus a little padding)
    plt.tight_layout()
    plt.savefig(save_path)
    print(f"F1-Score plot saved to {save_path}")
    plt.show() # <-- ADDED: Show the plot
    plt.close() # Free up memory