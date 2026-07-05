import serial
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------- Config ----------
PORT = 'COM7'        # change if your Arduino is on a different port
BAUD = 9600          # must match Serial.begin() in the Arduino sketch
WINDOW = 200         # how many recent samples to keep on screen

# ---------- Serial connection ----------
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except serial.SerialException as e:
    print(f"Could not open {PORT}: {e}")
    print("Check the port name, and make sure the Arduino Serial Monitor is closed.")
    raise SystemExit

# ---------- Rolling buffers (auto-drop oldest) ----------
samples    = deque(maxlen=WINDOW)   # x-axis (sample number)
throttle_b = deque(maxlen=WINDOW)   # throttle value
brake_b    = deque(maxlen=WINDOW)   # brake value
angle_b    = deque(maxlen=WINDOW)   # steering angle (deg)
sample_count = 0

# ---------- Figure: three stacked plots ----------
fig, (ax_throttle, ax_brake, ax_steer) = plt.subplots(3, 1, figsize=(10, 9))
fig.suptitle('Car Simulation Telemetry')

# Throttle plot
line_throttle, = ax_throttle.plot([], [], '-', color='tab:blue', label='Throttle')
ax_throttle.set_ylabel('Throttle')
ax_throttle.set_ylim(-50, 1073)
ax_throttle.legend(loc='upper left')
ax_throttle.grid(True)

# Brake plot
line_brake, = ax_brake.plot([], [], '-', color='tab:red', label='Brake')
ax_brake.set_ylabel('Brake')
ax_brake.set_ylim(-50, 1073)
ax_brake.legend(loc='upper left')
ax_brake.grid(True)

# Steering plot: angle with a centre line
line_angle, = ax_steer.plot([], [], '-', color='tab:green', label='Wheel angle')
ax_steer.axhline(0, color='gray', linewidth=0.8)   # straight-ahead reference
ax_steer.set_ylabel('Steering (deg)')
ax_steer.set_xlabel('Sample')
ax_steer.set_ylim(-40, 40)
ax_steer.legend(loc='upper left')
ax_steer.grid(True)

# ---------- Called repeatedly to refresh the plots ----------
def update(_frame):
    global sample_count

    # Drain whatever serial data has arrived since last frame
    # (cap the loop so a flood of data can't stall the animation)
    for _ in range(50):
        if not ser.in_waiting:
            break
        raw = ser.readline()
        if not raw:
            break
        text = raw.decode('utf-8', errors='ignore').strip()
        if not text:
            continue

        # Expected line: "Throttle , Brake , Wheel_angle"
        parts = text.split(',')
        if len(parts) != 3:
            continue   # skip malformed / partial lines

        try:
            throttle = float(parts[0])
            brake    = float(parts[1])
            angle    = float(parts[2])
        except ValueError:
            continue

        sample_count += 1
        samples.append(sample_count)
        throttle_b.append(throttle)
        brake_b.append(brake)
        angle_b.append(angle)

    # Push new data into the plot lines
    line_throttle.set_data(samples, throttle_b)
    line_brake.set_data(samples, brake_b)
    line_angle.set_data(samples, angle_b)

    # Scroll the x-axis to follow the latest samples
    if samples:
        xmax = samples[-1]
        xmin = max(0, xmax - WINDOW)
        ax_throttle.set_xlim(xmin, xmax + 1)
        ax_brake.set_xlim(xmin, xmax + 1)
        ax_steer.set_xlim(xmin, xmax + 1)

        # Live readout in the titles
        ax_throttle.set_title(f"Throttle: {throttle_b[-1]:.0f}")
        ax_brake.set_title(f"Brake: {brake_b[-1]:.0f}")
        ax_steer.set_title(f"Steering: {angle_b[-1]:.0f} deg")

    return line_throttle, line_brake, line_angle

# interval=50 -> refresh ~20x/sec. blit=False because the axes limits change.
ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()

ser.close()