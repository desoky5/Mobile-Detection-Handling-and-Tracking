import torch
import cv2

WEIGHTS = "best.pt"
CONF = 0.20
CENTER_TOL = 25
FAR = 50
STOP = 140

model = torch.hub.load(
    'ultralytics/yolov5',
    'custom',
    path=WEIGHTS
)
model.conf = CONF

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    cx_img = w // 2

    results = model(frame, size=320)
    dets = results.xyxy[0]

    command = "SEARCH"

    if len(dets):
        best = max(dets, key=lambda x: x[4])
        x1, y1, x2, y2, conf, _ = best
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

        box_h = y2 - y1
        cx_box = (x1 + x2) // 2
        err = cx_box - cx_img

        if abs(err) > CENTER_TOL:
            command = "RIGHT" if err > 0 else "LEFT"
        else:
            if box_h < FAR:
                command = "FORWARD_FAST"
            elif box_h < STOP:
                command = "FORWARD_SLOW"
            else:
                command = "STOP"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
        cv2.putText(
            frame,
            f"{command} H={box_h}",
            (10,20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255,255,255),
            2
        )

    cv2.imshow("Pi Bottle Tracker", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
