import serial
import socket
import struct

PORT = "/dev/ttyAMA0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.flush()

UDP_IP = "172.30.247.195"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Motor server running...")

def send_command(left, right):
    cmd = f"{left:.1f},{right:.1f}\n"
    ser.write(cmd.encode())

try:
    while True:
        data, addr = sock.recvfrom(1024)
        left, right = struct.unpack('ii', data)
        send_command(left, right)

except KeyboardInterrupt:
    send_command(0, 0)
    ser.close()
    sock.close()
    print("Stopped safely")
