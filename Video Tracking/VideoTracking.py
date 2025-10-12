# VideoTracking.py
# This script tracks a yellow tennis (foam training) ball (diameter: 9 cm) in a video using OpenCV's tracking algorithms.
import cv2
import numpy as np
import matplotlib.pyplot as plt
from tools import *

# Define the video file path / Name the mp4 file you want to track a yellow ball in
video_path = './IMG_5481_1046AM_Cut.mp4'

# Define the video capture object
video_capture = cv2.VideoCapture(video_path)
window_name = "task 1"
cv2.namedWindow(window_name)

# Get frame count, frame rate, and duration
frame_count = int(video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
frame_rate = video_capture.get(cv2.CAP_PROP_FPS)
duration = int(1000 / frame_rate)

# Read the first frame from the video
ret, frame = video_capture.read()

# Check if frame was read successfully
if not ret:
    print("Error: Could not read the first frame from the video")
    video_capture.release()
    cv2.destroyAllWindows()
    exit()

# Display the first frame in the window
cv2.imshow(window_name, frame)

# Measure ball size for calibration
print("First, let's calibrate by measuring the tennis ball size...")
print("Click and drag to select the diameter of the tennis ball")
ball_bbox = cv2.selectROI("Ball Size Calibration", frame, False)
ball_diameter_pixels = max(ball_bbox[2], ball_bbox[3])  # Use the larger dimension
cv2.destroyWindow("Ball Size Calibration")

# Ball diameter is 9 cm
ball_diameter_cm = 9.0
pixels_per_cm = ball_diameter_pixels / ball_diameter_cm
print(f"Ball diameter: {ball_diameter_pixels} pixels = {ball_diameter_cm} cm")
print(f"Scale: {pixels_per_cm:.2f} pixels per cm")

# Select a region of interest (ROI) to track
print("Now select the BALL ONLY to track...")
print("IMPORTANT: Select only the tennis ball, avoid including the hand!")
print("Try to select the ball when it's separated from the hand")
bbox = cv2.selectROI(window_name, frame, False)

# Check if a valid region was selected
if bbox == (0, 0, 0, 0) or bbox[2] <= 0 or bbox[3] <= 0:
    print("No valid region selected. Exiting...")
    video_capture.release()
    cv2.destroyAllWindows()
    exit()

# Initialize CSRT tracker (now available in OpenCV 4.7.0!)
tracker = None
try:
    # CSRT tracker is the best for this application - it's now available!
    tracker = cv2.TrackerCSRT_create()
    print("✅ Using CSRT tracker (excellent for tennis ball tracking)")
except (AttributeError, cv2.error):
    try:
        # Try legacy CSRT if main namespace fails
        if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerCSRT_create'):
            tracker = cv2.legacy.TrackerCSRT_create()
            print("Using legacy CSRT tracker")
        else:
            raise AttributeError("CSRT not found")
    except (AttributeError, cv2.error):
        try:
            # Try Nano tracker as backup (good for small objects)
            tracker = cv2.TrackerNano_create()
            print("Using Nano tracker (CSRT not available)")
        except (AttributeError, cv2.error):
            try:
                # Try MIL tracker as last resort
                tracker = cv2.TrackerMIL_create()
                print("Using MIL tracker (fallback)")
            except AttributeError:
                print("❌ No compatible tracker found")
                video_capture.release()
                cv2.destroyAllWindows()
                exit()

# Initialize the tracker
success = tracker.init(frame, bbox)
# Handle None return (some OpenCV versions)
if success is None:
    success = True

if not success:
    print("Failed to initialize tracker")
    video_capture.release()
    cv2.destroyAllWindows()
    exit()

# List to store frames for gif creation
frames_for_gif = []

# Variables for displacement tracking
previous_center = None
total_displacement_pixels = 0
total_displacement_cm = 0
frame_number = 0

# Variables for peak detection
highest_point_y = float('inf')  # Start with infinite (lowest y-coordinate is highest point)
peak_frame = 0
peak_time_sec = 0
time_per_frame = 1.0 / frame_rate

while video_capture.isOpened():
    # Read a new frame from the video
    ret, frame = video_capture.read()

    if not ret:
        break

    # Validate frame before processing
    if frame is None or frame.size == 0:
        print(f"Warning: Invalid frame at frame number {frame_number + 1}")
        continue
    
    # Ensure frame has proper dimensions
    if len(frame.shape) != 3 or frame.shape[2] != 3:
        print(f"Warning: Unexpected frame shape {frame.shape} at frame {frame_number + 1}")
        continue

    frame_number += 1

    # Enhanced preprocessing specifically for yellow ball vs WHITE background
    frame_enhanced = frame.copy()
    
    # Convert to HSV for better color processing
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Optimized yellow detection for tennis ball against white background
    yellow_lower = np.array([18, 120, 180])
    yellow_upper = np.array([32, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    
    # Create mask for white/bright background areas (to suppress them)
    white_lower = np.array([0, 0, 200])
    white_upper = np.array([180, 30, 255])
    white_mask = cv2.inRange(hsv, white_lower, white_upper)
    
    # Create mask for very bright areas that might interfere
    bright_lower = np.array([0, 0, 230])
    bright_upper = np.array([180, 50, 255])
    bright_mask = cv2.inRange(hsv, bright_lower, bright_upper)
    
    # Combine white and bright masks
    background_mask = cv2.bitwise_or(white_mask, bright_mask)
    
    # Morphological operations
    kernel = np.ones((5,5), np.uint8)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
    yellow_mask = cv2.dilate(yellow_mask, kernel, iterations=1)
    background_mask = cv2.dilate(background_mask, kernel, iterations=2)
    
    # Create enhanced frame optimized for yellow ball vs white background
    frame_enhanced = cv2.convertScaleAbs(frame, alpha=1.1, beta=10)
    
    # Validate mask and frame compatibility before enhancement
    if (yellow_mask.shape[:2] == frame_enhanced.shape[:2] and 
        background_mask.shape[:2] == frame_enhanced.shape[:2]):
        
        # Enhance yellow areas safely
        try:
            yellow_indices = yellow_mask > 0
            if np.any(yellow_indices):
                frame_enhanced[yellow_indices] = cv2.convertScaleAbs(
                    frame_enhanced[yellow_indices], alpha=2.2, beta=50)
        except Exception as e:
            print(f"Warning: Could not enhance yellow areas: {e}")
        
        # Darken white background areas safely
        try:
            background_indices = background_mask > 0
            if np.any(background_indices):
                frame_enhanced[background_indices] = cv2.convertScaleAbs(
                    frame_enhanced[background_indices], alpha=0.7, beta=-30)
        except Exception as e:
            print(f"Warning: Could not darken background areas: {e}")
    else:
        print(f"Warning: Mask shape mismatch - Frame: {frame_enhanced.shape}, Yellow mask: {yellow_mask.shape}, Background mask: {background_mask.shape}")
        # Use original frame if there's a shape mismatch
        frame_enhanced = frame.copy()

    # Update the tracker with enhanced frame
    success, bbox = tracker.update(frame_enhanced)
    
    # Use full-frame yellow detection to guide tracking
    full_contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    yellow_bbox = None
    if full_contours:
        largest_full_contour = max(full_contours, key=cv2.contourArea)
        largest_full_area = cv2.contourArea(largest_full_contour)
        
        if largest_full_area > 50:
            yellow_x, yellow_y, yellow_w, yellow_h = cv2.boundingRect(largest_full_contour)
            yellow_bbox = (yellow_x, yellow_y, yellow_w, yellow_h)
    
    # Decide which bbox to use
    if yellow_bbox is not None and success:
        (x, y, w, h) = [int(v) for v in bbox]
        (yellow_x, yellow_y, yellow_w, yellow_h) = yellow_bbox
        
        # Calculate overlap between CSRT and yellow detection
        overlap_x1 = max(x, yellow_x)
        overlap_y1 = max(y, yellow_y)
        overlap_x2 = min(x + w, yellow_x + yellow_w)
        overlap_y2 = min(y + h, yellow_y + yellow_h)
        
        if overlap_x2 > overlap_x1 and overlap_y2 > overlap_y1:
            overlap_area = (overlap_x2 - overlap_x1) * (overlap_y2 - overlap_y1)
            tracker_area = w * h
            overlap_ratio = overlap_area / tracker_area
            
            if overlap_ratio > 0.3:
                # Blend the two
                blend_factor = 0.6
                x = int(x * (1 - blend_factor) + yellow_x * blend_factor)
                y = int(y * (1 - blend_factor) + yellow_y * blend_factor)
                w = int(w * (1 - blend_factor) + yellow_w * blend_factor)
                h = int(h * (1 - blend_factor) + yellow_h * blend_factor)
                bbox = (x, y, w, h)
            else:
                # Use yellow detection
                x, y, w, h = yellow_bbox
                bbox = yellow_bbox
                success = True
        else:
            # Use yellow detection
            x, y, w, h = yellow_bbox
            bbox = yellow_bbox
            success = True
    
    elif yellow_bbox is not None:
        # Only yellow detection available
        x, y, w, h = yellow_bbox
        bbox = yellow_bbox
        success = True
    
    elif success:
        # Only CSRT available
        (x, y, w, h) = [int(v) for v in bbox]
    
    if success:
        # Draw the tracking box
        (x, y, w, h) = [int(v) for v in bbox]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Calculate and draw the centroid as a red dot
        center_x = int(x + w/2)
        center_y = int(y + h/2)
        current_center = (center_x, center_y)
        cv2.circle(frame, current_center, 5, (0, 0, 255), -1)
        
        # Check if this is the highest point (lowest y-coordinate)
        if center_y < highest_point_y:
            highest_point_y = center_y
            peak_frame = frame_number
            peak_time_sec = frame_number * time_per_frame
        
        # Calculate displacement if we have a previous position
        if previous_center is not None:
            displacement_pixels = np.sqrt((current_center[0] - previous_center[0])**2 + 
                                        (current_center[1] - previous_center[1])**2)
            displacement_cm = displacement_pixels / pixels_per_cm
            total_displacement_pixels += displacement_pixels
            total_displacement_cm += displacement_cm
            
            # Display current frame displacement
            cv2.putText(frame, f"Frame displacement: {displacement_cm:.2f} cm", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Display total displacement
        cv2.putText(frame, f"Total displacement: {total_displacement_cm:.2f} cm", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Display current position in cm from origin
        if frame_number == 1:
            origin_x, origin_y = current_center
        pos_x_cm = (current_center[0] - origin_x) / pixels_per_cm
        pos_y_cm = (origin_y - current_center[1]) / pixels_per_cm  # Invert Y for upward positive
        cv2.putText(frame, f"Position: ({pos_x_cm:.1f}, {pos_y_cm:.1f}) cm", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # Display current time and peak information
        current_time = frame_number * time_per_frame
        cv2.putText(frame, f"Time: {current_time:.2f} sec", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        if peak_frame > 0:
            cv2.putText(frame, f"Peak at: {peak_time_sec:.3f} sec", 
                       (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # Draw a line at the peak height
            cv2.line(frame, (0, highest_point_y), (frame.shape[1], highest_point_y), (0, 255, 0), 1)
        
        previous_center = current_center
    else:
        # Tracking failed - show warning
        cv2.putText(frame, "Tracking failed", (50, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Display the resulting frame
    cv2.imshow(window_name, frame)
    
    # Store frame for gif creation (convert BGR to RGB and ensure uint8 format)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    frames_for_gif.append(frame_rgb.astype(np.uint8))

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close windows
cv2.waitKey(0)
cv2.destroyWindow(window_name)
cv2.waitKey(1)
video_capture.release()

# Print final displacement summary
print(f"\n=== TRACKING ANALYSIS SUMMARY ===")
print(f"Total frames processed: {frame_number}")
print(f"Video duration: {frame_number * time_per_frame:.2f} seconds")
print(f"Frame rate: {frame_rate} FPS")
print(f"Total displacement: {total_displacement_cm:.2f} cm")
print(f"Average displacement per frame: {total_displacement_cm/max(frame_number-1, 1):.2f} cm")
print(f"Scale used: {pixels_per_cm:.2f} pixels per cm")
print(f"\n=== PEAK ANALYSIS ===")
if peak_frame > 0:
    print(f"Highest point reached at frame {peak_frame}")
    print(f"Time to peak: {peak_time_sec:.3f} seconds")
    peak_height_cm = (origin_y - highest_point_y) / pixels_per_cm
    print(f"Peak height: {peak_height_cm:.2f} cm above starting position")
else:
    print("No peak detected in trajectory")

# Save frames as gif
if len(frames_for_gif) > 0:
    frames_array = np.array(frames_for_gif, dtype=np.uint8)
    duration_seconds = duration / 1000.0
    
    # Generate output filename from input video path
    import os
    video_name = os.path.splitext(os.path.basename(video_path))[0]
    gif_name = f"{video_name}_tracked.gif"
    
    to_gif(frames_array, duration_seconds, f"./{gif_name}")
    print(f"GIF saved as {gif_name}")
else:
    print("No frames captured for gif creation")