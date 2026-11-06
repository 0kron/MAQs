import os
import select
import sys
import time

import cv2
import torch
from ultralytics import YOLO


# The nano model runs faster than yolov8s for live webcam inference.
use_cuda = torch.cuda.is_available()
device = 0 if use_cuda else "cpu"
if use_cuda:
    torch.backends.cudnn.benchmark = True
    print(f"Using CUDA GPU: {torch.cuda.get_device_name(0)}")
else:
    print("CUDA unavailable to PyTorch; using CPU. Install CUDA-enabled PyTorch for GPU inference.")

model = YOLO("yolov8n.pt")

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not access the webcam.")

qr_detector = cv2.QRCodeDetector()

# AprilTag detection is provided by the opencv-contrib aruco module.
aruco = getattr(cv2, "aruco", None)
april_detector = None
if aruco is None:
    print("AprilTag detection unavailable: install opencv-contrib-python.")
else:
    tag_dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
    if hasattr(aruco, "ArucoDetector"):
        april_detector = aruco.ArucoDetector(tag_dictionary, aruco.DetectorParameters())
    else:
        april_parameters = aruco.DetectorParameters_create()


def console_quit_pressed():
    """Allow q/Esc from the terminal as well as from the OpenCV window."""
    if os.name == "nt":
        import msvcrt

        if msvcrt.kbhit():
            return msvcrt.getwch().lower() in ("q", "\x1b")
        return False

    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip().lower() in ("q", "esc")
    return False


def read_qr_codes(image):
    """Try multi-code decoding first, then single-code decoding for compatibility."""
    try:
        found, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(image)
        if found and points is not None:
            codes = [(text, corners) for text, corners in zip(decoded_info, points) if text]
            if codes:
                return codes
    except (AttributeError, cv2.error):
        pass

    try:
        decoded, points, _ = qr_detector.detectAndDecode(image)
        if decoded and points is not None:
            return [(decoded, points[0])]
    except cv2.error:
        pass
    return []


window_name = "Webcam"
cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
print("Press 'q' or Esc in the webcam window, or type q in this console, to quit.")

previous_codes = set()
frame_count = 0
fps = 0.0
fps_started = time.perf_counter()
qr_status = "QR: searching"

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to grab a frame.")
            break

        visible_codes = set()

        # A smaller inference size reduces work while keeping the webcam feed full size.
        results = model(frame, conf=0.5, imgsz=416, device=device, half=use_cuda, verbose=False)
        annotated_frame = results[0].plot()

        qr_codes = read_qr_codes(frame)
        if qr_codes:
            qr_status = "QR: " + " | ".join(text[:45] for text, _ in qr_codes)
            for text, corners in qr_codes:
                visible_codes.add(("QR", text))
                corners = corners.astype(int).reshape((-1, 1, 2))
                cv2.polylines(annotated_frame, [corners], True, (0, 255, 0), 2)
        else:
            qr_status = "QR: searching (hold code steady and fill more of the view)"

        # Detect AprilTags from the 36h11 family.
        if april_detector is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if hasattr(aruco, "ArucoDetector"):
                corners, tag_ids, _ = april_detector.detectMarkers(gray)
            else:
                corners, tag_ids, _ = aruco.detectMarkers(
                    gray, tag_dictionary, parameters=april_parameters
                )

            if tag_ids is not None:
                for tag_id, tag_corners in zip(tag_ids.flatten(), corners):
                    visible_codes.add(("AprilTag", str(tag_id)))
                    cv2.polylines(annotated_frame, [tag_corners.astype(int)], True,
                                  (255, 0, 255), 2)
                    x, y = tag_corners.reshape((-1, 2))[0].astype(int)
                    cv2.putText(annotated_frame, f"AprilTag: {tag_id}", (x, max(25, y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        for kind, value in visible_codes - previous_codes:
            print(f"{kind}: {value}")
        previous_codes = visible_codes

        frame_count += 1
        elapsed = time.perf_counter() - fps_started
        if elapsed >= 1.0:
            fps = frame_count / elapsed
            print(f"FPS: {fps:.1f}")
            frame_count = 0
            fps_started = time.perf_counter()

        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(annotated_frame, qr_status[:90], (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.imshow(window_name, annotated_frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27) or console_quit_pressed():
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
