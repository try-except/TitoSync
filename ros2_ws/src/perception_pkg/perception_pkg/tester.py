import cv2

# try index 0 first; change to 1 if you really have a second camera
idx = 0
cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)  # CAP_V4L2 is appropriate on Linux
print("Backend:", cap.getBackendName() if hasattr(cap, "getBackendName") else "n/a")
if not cap.isOpened():
    print(f"ERROR: camera index {idx} not opened. Try another index (0,1,2).")
    cap.release()
    exit(1)

while True:
    ret, frame = cap.read()
    print("ret:", ret, "frame shape:", None if frame is None else frame.shape, end="\r")
    if not ret or frame is None:
        # avoid busy spinning with no frames
        cv2.waitKey(100)
        continue

    cv2.imshow("raw", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
