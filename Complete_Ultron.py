import cv2
import numpy as np
import serial
from cvzone.HandTrackingModule import HandDetector
import time

# Serial communication setup
ser = serial.Serial('COM4', 115200, timeout=1)  # Change to your Arduino's COM port
time.sleep(2)  # Allow time for serial connection to establish

def map_range(value, left_min, left_max, right_min, right_max):
    """
    Maps a value from one range to another while constraining it within bounds
    Args:
        value: Input value to map
        left_min/max: Original range bounds
        right_min/max: Target range bounds
    Returns:
        Mapped integer value
    """
    # Constrain value within original range
    value = max(min(value, left_max), left_min)
    # Calculate ranges
    left_span = left_max - left_min
    right_span = right_max - right_min
    # Convert to 0-1 range then to target range
    value_scaled = float(value - left_min) / float(left_span)
    return int(right_min + (value_scaled * right_span))

# Initialize video capture
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open video device")
    exit()

# Hand detector with confidence threshold
detector = HandDetector(detectionCon=0.8, maxHands=1)

# Load face detection classifier
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

# Tracking variables
prev_finger_str = ""  # Stores last sent finger state
prev_servo_pos = (75, 95)  # Default center position (x,y)
last_face_time = time.time()  # Track when face was last detected
face_lost_threshold = 1.0  # Seconds before returning to center

try:
    while True:
        # Read frame from camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to capture frame")
            break

        # Mirror the frame for more intuitive control
        frame = cv2.flip(frame, 1)
        height, width = frame.shape[:2]
        
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Face detection with improved parameters
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,  # Reduced from 1.3 for better close-range detection
            minNeighbors=5,
            minSize=(100, 100)  # Minimum face size to reduce false positives
        )

        face_detected = len(faces) > 0
        
        if face_detected:
            # Reset face timer
            last_face_time = time.time()
            
            # Get primary face (largest in frame)
            (x, y, w, h) = max(faces, key=lambda f: f[2]*f[3])  # Sort by area
            
            # Calculate face center
            center_x, center_y = x + w//2, y + h//2
            
            # Map face position to servo values with constrained ranges
            # X: 50 (left) to 100 (right)
            # Y: 60 (up) to 120 (down)
            servo_x = map_range(center_x, 0, width, 100, 50)
            servo_y = map_range(center_y, 0, height, 60, 120)
            
            # Only send commands if position changed significantly
            if (abs(servo_x - prev_servo_pos[0]) > 2 or 
                abs(servo_y - prev_servo_pos[1]) > 2):
                command = f"SERVO,{servo_x},{servo_y}\n"
                ser.write(command.encode())
                print(command.encode())
                prev_servo_pos = (servo_x, servo_y)
            
            # Draw face tracking visuals
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.putText(
                frame, f'X:{servo_x} Y:{servo_y}', (x, y-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
            )
        else:
            # Return to center position if face is lost for too long
            if time.time() - last_face_time > face_lost_threshold:
                if prev_servo_pos != (75, 95):
                    ser.write(b"SERVO,75,95\n")
                    prev_servo_pos = (75, 95)

        # Hand detection
        hands, _ = detector.findHands(frame)
        if hands:
            # Get finger states (1=open, 0=closed)
            fingers = detector.fingersUp(hands[0])
            finger_str = ','.join(map(str, fingers))
            
            # Only send if finger state changed
            if finger_str != prev_finger_str:
                ser.write(f"{finger_str}\n".encode())
                print(f"{finger_str}\n".encode())
                prev_finger_str = finger_str
            
            # Display active fingers
            finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
            active_fingers = [
                name for name, state in zip(finger_names, fingers) if state
            ]
            status_text = f'Fingers: {", ".join(active_fingers) or "None"}'
            cv2.putText(
                frame, status_text, (20, height-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2
            )

        # Display frame
        cv2.imshow('Face & Hand Tracking', frame)
        
        # Exit on 'x' key press
        if cv2.waitKey(1) == ord('x'):
            break

finally:
    # Cleanup resources
    cap.release()
    cv2.destroyAllWindows()
    ser.close()
    print("Resources released")