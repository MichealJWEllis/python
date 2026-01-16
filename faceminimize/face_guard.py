"""
Face Guard - Minimizes Chrome when an unrecognized face is detected.

Usage:
    python face_guard.py enroll    # Take reference photos of your face
    python face_guard.py           # Run the guard
"""

import cv2
import win32gui
import win32con
import time
import sys
import os
import numpy as np

WINDOW_TARGET = "chrome"
COOLDOWN_SECONDS = 3
FACE_DIR = "my_face"
MODEL_FILE = "face_model.yml"
CONFIDENCE_THRESHOLD = 45  # Lower = stricter matching


def find_window(partial_title: str) -> int:
    """Find window handle by partial title match."""
    result = []
    
    def callback(hwnd, _):
        if win32gui.IsWindowVisible(hwnd):
            title = win32gui.GetWindowText(hwnd).lower()
            if partial_title.lower() in title:
                result.append(hwnd)
        return True
    
    win32gui.EnumWindows(callback, None)
    return result[0] if result else 0


def minimize_window(hwnd: int) -> None:
    """Minimize window by handle."""
    win32gui.ShowWindow(hwnd, win32con.SW_MINIMIZE)
    print(f"[!] Unknown face detected - minimized window")


def get_camera():
    """Open camera with DirectShow backend."""
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap


def get_face_cascade():
    """Load Haar cascade for face detection."""
    cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        raise RuntimeError("Failed to load Haar cascade")
    return face_cascade


def enroll():
    """Capture reference photos of your face."""
    os.makedirs(FACE_DIR, exist_ok=True)
    
    cap = get_camera()
    face_cascade = get_face_cascade()
    
    print("[*] Enrollment mode")
    print("[*] Press SPACE to capture a photo (take 5-10 from different angles)")
    print("[*] Press 'q' when done")
    
    count = len([f for f in os.listdir(FACE_DIR) if f.endswith(".jpg")])
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("Failed to read from camera")
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.3,
                minNeighbors=8,
                minSize=(100, 100)
            )
            
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            cv2.putText(frame, f"Photos: {count}", (10, 30), 
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Enrollment", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and len(faces) == 1:
                x, y, w, h = faces[0]
                face_img = gray[y:y+h, x:x+w]
                filepath = os.path.join(FACE_DIR, f"face_{count:03d}.jpg")
                cv2.imwrite(filepath, face_img)
                count += 1
                print(f"[+] Captured {filepath}")
            elif key == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    
    if count < 3:
        print("[!] Need at least 3 photos. Run enroll again.")
        return
    
    train()


def train():
    """Train the recognizer on enrolled faces."""
    print("[*] Training model...")
    
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    
    faces = []
    labels = []
    
    for filename in os.listdir(FACE_DIR):
        if not filename.endswith(".jpg"):
            continue
        filepath = os.path.join(FACE_DIR, filename)
        img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
        faces.append(img)
        labels.append(0)  # 0 = owner
    
    recognizer.train(faces, np.array(labels))
    recognizer.write(MODEL_FILE)
    
    print(f"[+] Model trained on {len(faces)} images")


def guard():
    """Run the face guard (headless)."""
    if not os.path.exists(MODEL_FILE):
        print("[!] No model found. Run 'python face_guard.py enroll' first.")
        sys.exit(1)
    
    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read(MODEL_FILE)
    
    cap = get_camera()
    face_cascade = get_face_cascade()
    
    print(f"[*] Face Guard running (headless)")
    print(f"[*] Target window: {WINDOW_TARGET}")
    print("[*] Press Ctrl+C to quit")
    
    last_minimize_time = 0
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("Failed to read from camera")
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.3,
                minNeighbors=8,
                minSize=(100, 100)
            )
            
            current_time = time.time()
            
            for (x, y, w, h) in faces:
                face_roi = gray[y:y+h, x:x+w]
                label, confidence = recognizer.predict(face_roi)
                
                # Unknown face detected
                if confidence > CONFIDENCE_THRESHOLD:
                    if current_time - last_minimize_time > COOLDOWN_SECONDS:
                        hwnd = find_window(WINDOW_TARGET)
                        if hwnd:
                            minimize_window(hwnd)
                            last_minimize_time = current_time
                        break
            
            time.sleep(0.1)  # Reduce CPU usage
    
    except KeyboardInterrupt:
        print("\n[*] Stopped")
    finally:
        cap.release()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "enroll":
        enroll()
    else:
        guard()


if __name__ == "__main__":
    main()