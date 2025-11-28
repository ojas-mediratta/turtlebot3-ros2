import cv2
import numpy as np
from skimage.feature import hog

# --- Configuration ---
# Standard size for resized crops
IMG_SIZE = (128, 128)


# --- Cropping Function ---
def crop_sign(image):
    """Finds the largest sign-like object, crops, and resizes it.

    Returns a resized image (IMG_SIZE) or None if no sign found.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # HSV ranges for candidate sign colors
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_blue = np.array([90, 100, 40])
    upper_blue = np.array([130, 255, 255])
    lower_green_yellow = np.array([20, 40, 50])
    upper_green_yellow = np.array([100, 255, 255])
    lower_brown = np.array([10, 30, 30])
    upper_brown = np.array([30, 255, 220])
    lower_orange = np.array([11, 50, 50])
    upper_orange = np.array([25, 255, 255])

    mask_r1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_r2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_b = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_gy = cv2.inRange(hsv, lower_green_yellow, upper_green_yellow)
    mask_br = cv2.inRange(hsv, lower_brown, upper_brown)
    mask_or = cv2.inRange(hsv, lower_orange, upper_orange)

    combined_sign_mask = mask_r1 | mask_r2 | mask_b | mask_gy | mask_br | mask_or

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    combined_sign_mask = cv2.morphologyEx(combined_sign_mask, cv2.MORPH_CLOSE, kernel)
    combined_sign_mask = cv2.morphologyEx(combined_sign_mask, cv2.MORPH_OPEN, kernel)

    MIN_CONTOUR_AREA = 0
    all_sign_contours, _ = cv2.findContours(combined_sign_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sign_contours = [c for c in all_sign_contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]

    if not sign_contours:
        return None

    largest_contour = max(sign_contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    pad_w = int(w * 0.10)
    pad_h = int(h * 0.10)
    y1 = max(0, y - pad_h)
    y2 = min(image.shape[0], y + h + pad_h)
    x1 = max(0, x - pad_w)
    x2 = min(image.shape[1], x + w + pad_w)
    cropped_image = image[y1:y2, x1:x2]
    if cropped_image.size == 0:
        return None
    return cv2.resize(cropped_image, IMG_SIZE)


# --- Feature Extraction ---
def extract_features(cropped_image):
    """Return a 1D numpy feature vector for a cropped image.
    Uses HOG, color histograms, edge density, and Hu moments.
    """
    gray_image = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    gray_image = cv2.equalizeHist(gray_image)

    hog_features = hog(gray_image,
                       orientations=9,
                       pixels_per_cell=(16, 16),
                       cells_per_block=(2, 2),
                       visualize=False,
                       feature_vector=True)

    hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
    hist_h = cv2.calcHist([hsv], [0], None, [16], [0, 180])
    hist_s = cv2.calcHist([hsv], [1], None, [12], [0, 256])
    hist_v = cv2.calcHist([hsv], [2], None, [12], [0, 256])
    cv2.normalize(hist_h, hist_h)
    cv2.normalize(hist_s, hist_s)
    cv2.normalize(hist_v, hist_v)
    color_features = np.hstack([hist_h.flatten(), hist_s.flatten(), hist_v.flatten()])

    edges = cv2.Canny(gray_image, 50, 150)
    edge_density = np.sum(edges > 0) / (edges.shape[0] * edges.shape[1])

    moments = cv2.moments(edges)
    hu_moments = cv2.HuMoments(moments).flatten()

    return np.hstack([hog_features, color_features, [edge_density], hu_moments])
