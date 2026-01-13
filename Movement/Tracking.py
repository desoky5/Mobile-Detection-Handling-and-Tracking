import time
import serial
import cv2
import torch

# =============================
# ⚙️ CONFIGURATION
# =============================

# Serial (Motor Controller)
PORT = "/dev/ttyAMA0"
BAUD = 115200

# YOLOv5 Model
MODEL_PATH = "best.pt"   # put best.pt inside yolov5 directory
CONF_THRESH = 0.4
IOU_THRESH = 0.5

# Camera
CAM_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Centering zone
LEFT_LIMIT = 220
RIGHT_LIMIT = 420

# Speeds
SPEED_SEARCH = 40.0
SPEED_APPROACH = 50.0
SPEED_TURN = 30.0

# Bottle close enough to grab
GRAB_HEIGHT_THRESHOLD = 350  # pixels

# Fail-safe timeout (seconds)
TARGET_TIMEOUT = 1.0

# =============================
# 🔌 INITIALIZATION
# =============================

print("🔌 Connecting to motors...")
ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.flush()
print("✅ Serial connected")

print("🧠 Loading YOLOv5 model...")
model = torch.hub.load(
    'ultralytics/yolov5',
    'custom',
    path=MODEL_PATH,
    device='cpu'
)
model.conf = CONF_THRESH
model.iou = IOU_THRESH
print("✅ Model loaded")

print("📷 Opening camera...")
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

if not cap.isOpened():
    raise RuntimeError("❌ Camera failed to open")

print("🤖 ROBOT ACTIVE — searching for bottles")

# =============================
# 🛠️ HELPERS
# =============================

def send_command(left, right):
    cmd = f"{left:.1f},{right:.1f}\n"
    ser.write(cmd.encode())

last_seen = time.time()

# =============================
# 🔁 MAIN LOOP
# =============================

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Camera frame lost")
            break

        results = model(frame)
        target = None

        # ---- Detection ----
        for *xyxy, conf, cls in results.xyxy[0]:
            cls = int(cls)
            if model.names[cls] == "bottle":
                x1, y1, x2, y2 = map(int, xyxy)
                height = y2 - y1
                x_center = (x1 + x2) // 2

                target = {
                    "x": x_center,
                    "h": height,
                    "box": (x1, y1, x2, y2)
                }

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                break

        # ---- Decision Logic ----
        status = ""

        if target:
            last_seen = time.time()

            # Close enough → STOP & GRAB
            if target["h"] > GRAB_HEIGHT_THRESHOLD:
                status = "GRAB SEQUENCE"
                send_command(0.0, 0.0)
                print("🛑 Bottle close — trigger arm")
                time.sleep(2)  # placeholder for arm code

            # Turn Left
            elif target["x"] < LEFT_LIMIT:
                status = "TURN LEFT"
                send_command(-SPEED_TURN, SPEED_TURN)

            # Turn Right
            elif target["x"] > RIGHT_LIMIT:
                status = "TURN RIGHT"
                send_command(SPEED_TURN, -SPEED_TURN)

            # Centered → Forward
            else:
                status = "APPROACH"
                send_command(SPEED_APPROACH, SPEED_APPROACH)

        else:
            # Fail-safe: stop if target lost
            if time.time() - last_seen > TARGET_TIMEOUT:
                status = "TARGET LOST — STOP"
                send_command(0.0, 0.0)
            else:
                status = "SEARCHING"
                send_command(SPEED_SEARCH, SPEED_SEARCH)

        # ---- GUI ----
        cv2.line(frame, (LEFT_LIMIT, 0), (LEFT_LIMIT, FRAME_HEIGHT), (255,0,0), 2)
        cv2.line(frame, (RIGHT_LIMIT, 0), (RIGHT_LIMIT, FRAME_HEIGHT), (255,0,0), 2)
        cv2.putText(frame, status, (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        cv2.imshow("Robot Vision", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("🛑 Interrupted")

finally:
    send_command(0.0, 0.0)
    ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("✅ Robot stopped safely")
