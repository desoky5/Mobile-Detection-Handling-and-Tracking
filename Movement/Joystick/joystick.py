import pygame
import socket
import time
import struct

# -----------------------
# CONFIG
# -----------------------
UDP_IP = "172.30.247.154"   # Raspberry Pi Wi-Fi IP
UDP_PORT = 5005

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No joystick detected")

js = pygame.joystick.Joystick(0)
js.init()
print(f"Joystick connected: {js.get_name()}")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# -----------------------
# LOOP
# -----------------------
try:
    while True:
        pygame.event.pump()

        # Left stick X = turning
        turn = js.get_axis(0)

        # L2/R2 = speed
        l2 = (js.get_axis(2) + 1) / 2  # normalize 0 → 1
        r2 = (js.get_axis(5) + 1) / 2  # normalize 0 → 1

        # Calculate speed
        # Forward: R2 pressed, Backward: L2 pressed
        speed = (r2 - l2) * 100   # -100 → 100

        # Differential drive: turn mixes with speed
        left = speed + turn * 50
        right = speed - turn * 50

        # Clamp
        left = max(min(left, 100), -100)
        right = max(min(right, 100), -100)

        # Pack and send
        data = struct.pack('ii', int(left), int(right))
        sock.sendto(data, (UDP_IP, UDP_PORT))

        time.sleep(0.05)  # 20 Hz
except KeyboardInterrupt:
    sock.close()
    print("Stopped")
