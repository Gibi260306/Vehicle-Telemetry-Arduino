import serial

ser = serial.Serial('COM7', 9600, timeout=1)

# Continuous reading
while True:
    line = ser.readline()
    if line:
        text = line.decode('utf-8', errors='ignore').strip()
        if text:
            print(text)