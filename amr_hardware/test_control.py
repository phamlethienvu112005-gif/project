import serial
import time

# Kết nối tới ESP32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
time.sleep(1)

# Gửi thử lệnh tìm HOME cho Motor 3
print("Gửi lệnh tìm HOME...")
ser.write(b"home\n")

# Đọc phản hồi liên tục
try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print("[ESP32]:", line)
except KeyboardInterrupt:
    # Dừng động cơ an toàn khi bấm Ctrl+C
    ser.write(b"M0\n")
    ser.write(b"L0\n")
    ser.write(b"R0\n")
    ser.close()
    print("\nĐã ngắt kết nối!")
