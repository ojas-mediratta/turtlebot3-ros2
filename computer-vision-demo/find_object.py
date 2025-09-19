"""
find_object.py

Description:
  Tracks a chosen object in real time from the laptop webcam using OpenCV.
  Approach: HSV color thresholding + morphology + contour filtering.
  Outputs:
    - Prints pixel coordinates (cx, cy) of detected object center.
    - Overlays a marker/bounding circle and label on the video stream.
Usage:
  python3 find_object.py
  (Press 'q' to quit. Use the on-screen sliders to tune color thresholds.)
Notes:
  - Works across distances (not fixed to a single distance).
  - Tested on red notebook.
  - Uses HSV color space, which works better with different lighting conditions.
"""

import sys, time
import cv2
import numpy as np

# HSV band set for red (two bands since red wraps around 180)
H_LOW,  S_LOW,  V_LOW  = 0,   120,  150
H_HIGH, S_HIGH, V_HIGH = 12, 255, 255
H_LOW2,  S_LOW2,  V_LOW2  = 170,  120,  150
H_HIGH2, S_HIGH2, V_HIGH2 = 179, 255, 255

# Area filter as fraction of frame (skip tiny specks / whole-frame blobs)
MIN_AREA_RATIO = 0.04
MAX_AREA_RATIO = 0.5
PRINT_INTERVAL = 0.2  

# ---------------- Main ----------------
def main():
    cap = cv2.VideoCapture(0, cv2.CAP_ANY)
    
    last_print = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            h, w = frame.shape[:2]
            min_area = max(50.0, MIN_AREA_RATIO * (w * h))
            max_area = MAX_AREA_RATIO * (w * h)

            # Blur and convert to HSV (blurring with a 5x5 kernel)
            hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)

            # Build mask with two bands
            mask1 = cv2.inRange(hsv, (H_LOW, S_LOW, V_LOW), (H_HIGH, S_HIGH, V_HIGH))
            mask2 = cv2.inRange(hsv, (H_LOW2, S_LOW2, V_LOW2), (H_HIGH2, S_HIGH2, V_HIGH2))
            mask = cv2.bitwise_or(mask1, mask2)

            cv2.imshow("HSV", hsv)
            # cv2.imshow("Mask1", mask1)
            # cv2.imshow("Mask2", mask2)
            cv2.imshow("Original Mask", mask)

            # Opening to reduce noise and specks (5x5 kernel)
            kernel = np.ones((6, 6), np.uint8)

            # Test 1
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
            # cv2.imshow("Mask Open", mask)

            # Test 2
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
            cv2.imshow("Closed Mask", mask)
            
            # Find contours
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detected = False
            if cnts:
                # Pick largest contour within reasonable area
                for c in sorted(cnts, key=cv2.contourArea, reverse=True):
                    area = cv2.contourArea(c)
                    if min_area <= area <= max_area:
                        # Draw the contour
                        cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                        
                        # Calculate center using moments
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            
                            # Draw center point and coordinates
                            cv2.putText(frame, f"({cx}, {cy})", (cx + 10, cy - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                            
                            # Print coordinates (throttled)
                            t = time.time()
                            if t - last_print >= PRINT_INTERVAL:
                                print(f"{cx} {cy}")
                                last_print = t
                                
                            detected = True
                            break

            # UI text
            cv2.putText(frame, "Target: " + ("DETECTED" if detected else "NOT DETECTED"),
                        (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("find_object", frame)

            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # let HighGUI process window teardown

if __name__ == "__main__":
    main()