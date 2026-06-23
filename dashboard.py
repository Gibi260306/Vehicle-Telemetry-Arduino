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
samples   = deque(maxlen=WINDOW)   # x-axis (sample number)
target_b  = deque(maxlen=WINDOW)   # target speed
speed_b   = deque(maxlen=WINDOW)   # actual car speed
angle_b   = deque(maxlen=WINDOW)   # steering angle (deg)
sample_count = 0

# ---------- Convert raw joystick (Vry) to wheel angle ----------
# Matches the Arduino: deadzone 485-535 -> 0, else map(0..1023 -> -35..35)
def vry_to_angle(vry):
    if 485 < vry < 535:
        return 0.0
    return vry * 70.0 / 1023.0 - 35.0

# ---------- Figure: two stacked plots ----------
fig, (ax_speed, ax_steer) = plt.subplots(2, 1, figsize=(10, 7))
fig.suptitle('Car Simulation Telemetry')

# Speed plot: target (dashed) vs actual
line_target, = ax_speed.plot([], [], '--', color='tab:orange', label='Target speed')
line_speed,  = ax_speed.plot([], [], '-',  color='tab:blue',   label='Actual speed')
ax_speed.set_ylabel('Speed (km/h)')
ax_speed.set_ylim(-5, 105)
ax_speed.legend(loc='upper left')
ax_speed.grid(True)

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

        # Expected line: "Pot , target , speed , Vrx , Vry"
        parts = text.split(',')
        if len(parts) != 6:
            continue   # skip malformed / partial lines

        try:
            target = float(parts[1])
            speed  = float(parts[2])
            vry    = float(parts[4])
            flag   = int(parts[5])   # the new 6th field
        except ValueError:
            continue

        sample_count += 1
        samples.append(sample_count)
        target_b.append(target)
        speed_b.append(speed)
        angle_b.append(vry_to_angle(vry))

    # Push new data into the plot lines
    line_target.set_data(samples, target_b)
    line_speed.set_data(samples, speed_b)
    line_angle.set_data(samples, angle_b)

    # Scroll the x-axis to follow the latest samples
    if samples:
        xmax = samples[-1]
        xmin = max(0, xmax - WINDOW)
        ax_speed.set_xlim(xmin, xmax + 1)
        ax_steer.set_xlim(xmin, xmax + 1)

        # Live readout in the titles
        ax_speed.set_title(f"Speed: {speed_b[-1]:.1f} km/h   (target {target_b[-1]:.1f})")
        ax_steer.set_title(f"Steering: {angle_b[-1]:.0f} deg")

    return line_target, line_speed, line_angle

# interval=50 -> refresh ~20x/sec. blit=False because the axes limits change.
ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.show()

ser.close()