import serial
from collections import deque
from datetime import datetime
import csv
import math
import os
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------- Config ----------
PORT = 'COM7'              # Change this if the Arduino uses a different port
BAUD = 115200              # Must match Serial.begin() in the Arduino sketch
WINDOW_SECONDS = 20        # Amount of recent data shown on the graphs
EXPECTED_RATE = 20         # Arduino sends telemetry at 20 Hz
WINDOW = WINDOW_SECONDS * EXPECTED_RATE
LOG_DATA = True

# Leave blank for live serial data.
# Enter a CSV path to replay an old telemetry log instead.
REPLAY_FILE = 'logs/telemetry_20260731_205551.csv'
REPLAY_SPEED = 1.0

VALID_STATES = ['OFF', 'RUNNING', 'WARNING', 'CRASHED', 'SENSOR_FAULT']
UINT32_SIZE = 4294967296

# ---------- Runtime variables ----------
ser = None
log_file = None
log_writer = None
log_name = 'Logging disabled'
connection_status = 'Starting'

# Packet statistics
received_packets = 0
lost_packets = 0
duplicate_packets = 0
out_of_order_packets = 0
malformed_packets = 0
last_sequence = None

# Arduino timestamp rollover tracking
last_raw_time = None
time_wrap_offset = 0
first_device_time = None

# Latest packet values
latest_state = 'NO DATA'
latest_distance = -1
latest_faults = 0

# Packet rate tracking
arrival_times = deque()

# Rolling graph buffers
time_b = deque(maxlen=WINDOW)
throttle_b = deque(maxlen=WINDOW)
brake_b = deque(maxlen=WINDOW)
angle_b = deque(maxlen=WINDOW)
distance_b = deque(maxlen=WINDOW)

# Replay variables
replay_packets = []
replay_index = 0
replay_start_time = 0.0

# Figure variables
fig = None
ax_throttle = None
ax_brake = None
ax_steer = None
ax_distance = None
line_throttle = None
line_brake = None
line_angle = None
line_distance = None
status_text = None
ani = None


def parseTelemetry(text):
    # Expected line:
    # VTP1,time,sequence,throttle,brake,steering,distance,state,faults
    parts = text.strip().split(',')

    if len(parts) != 9:
        return None

    if parts[0].strip() != 'VTP1':
        return None

    try:
        packet = {
            'timestamp': int(parts[1]),
            'sequence': int(parts[2]),
            'throttle': float(parts[3]),
            'brake': float(parts[4]),
            'angle': int(parts[5]),
            'distance': int(parts[6]),
            'state': parts[7].strip(),
            'faults': int(parts[8], 0)
        }
    except ValueError:
        return None

    # Reject impossible values instead of plotting bad packets
    if packet['timestamp'] < 0 or packet['timestamp'] >= UINT32_SIZE:
        return None

    if packet['sequence'] < 0 or packet['sequence'] >= UINT32_SIZE:
        return None

    if packet['throttle'] < 0 or packet['throttle'] > 100:
        return None

    if packet['brake'] < 0 or packet['brake'] > 100:
        return None

    if packet['angle'] < -35 or packet['angle'] > 35:
        return None

    if packet['distance'] < -1 or packet['distance'] > 400:
        return None

    if packet['state'] not in VALID_STATES:
        return None

    if packet['faults'] < 0 or packet['faults'] > 255:
        return None

    return packet


def packetFromCSV(row):
    try:
        packet = {
            'timestamp': int(row['timestamp_ms']),
            'sequence': int(row['sequence']),
            'throttle': float(row['throttle_pct']),
            'brake': float(row['brake_pct']),
            'angle': int(row['steering_deg']),
            'distance': int(row['distance_cm']),
            'state': row['state'],
            'faults': int(row['fault_flags'], 0)
        }
    except (KeyError, ValueError):
        return None

    text = (
        f"VTP1,{packet['timestamp']},{packet['sequence']},"
        f"{packet['throttle']},{packet['brake']},{packet['angle']},"
        f"{packet['distance']},{packet['state']},{packet['faults']}"
    )

    return parseTelemetry(text)


def getElapsedTime(timestamp):
    global last_raw_time
    global time_wrap_offset
    global first_device_time

    # millis() rolls over after reaching the end of a 32-bit unsigned number
    if last_raw_time is not None:
        if timestamp < last_raw_time:
            if last_raw_time - timestamp > UINT32_SIZE // 2:
                time_wrap_offset += UINT32_SIZE

    last_raw_time = timestamp
    unwrapped_time = time_wrap_offset + timestamp

    if first_device_time is None:
        first_device_time = unwrapped_time

    return (unwrapped_time - first_device_time) / 1000.0


def updateSequenceStats(sequence):
    global received_packets
    global lost_packets
    global duplicate_packets
    global out_of_order_packets
    global last_sequence

    received_packets += 1

    if last_sequence is None:
        last_sequence = sequence
        return

    difference = (sequence - last_sequence) % UINT32_SIZE

    if difference == 0:
        duplicate_packets += 1
    elif difference < UINT32_SIZE // 2:
        if difference > 1:
            lost_packets += difference - 1

        last_sequence = sequence
    else:
        out_of_order_packets += 1


def getFaultText(faults):
    names = []

    if faults & 0x01:
        names.append('ULTRASONIC')

    unknown = faults & ~0x01

    if unknown:
        names.append(f'UNKNOWN 0x{unknown:02X}')

    if not names:
        return 'NONE'

    return ' | '.join(names)


def openSerial():
    global ser
    global connection_status

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        ser.reset_input_buffer()
        connection_status = f'Connected to {PORT} at {BAUD}'
        print(connection_status)
    except serial.SerialException as e:
        ser = None
        connection_status = f'Could not open {PORT}: {e}'
        print(connection_status)
        print('Check the COM port and close the Arduino Serial Monitor.')


def openLogFile():
    global log_file
    global log_writer
    global log_name

    if not LOG_DATA or REPLAY_FILE:
        return

    os.makedirs('logs', exist_ok=True)
    file_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    file_path = os.path.join('logs', f'telemetry_{file_time}.csv')

    log_file = open(file_path, 'w', newline='', encoding='utf-8')
    log_writer = csv.writer(log_file)

    log_writer.writerow([
        'host_time_iso',
        'timestamp_ms',
        'sequence',
        'throttle_pct',
        'brake_pct',
        'steering_deg',
        'distance_cm',
        'state',
        'fault_flags'
    ])

    log_name = file_path
    print(f'Logging to {file_path}')


def savePacket(packet):
    if log_writer is None:
        return

    log_writer.writerow([
        datetime.now().astimezone().isoformat(),
        packet['timestamp'],
        packet['sequence'],
        f"{packet['throttle']:.1f}",
        f"{packet['brake']:.1f}",
        packet['angle'],
        packet['distance'],
        packet['state'],
        packet['faults']
    ])

    # Flush regularly so a closed window does not lose the whole recording
    if received_packets % 20 == 0:
        log_file.flush()


def addPacket(packet):
    global latest_state
    global latest_distance
    global latest_faults

    elapsed = getElapsedTime(packet['timestamp'])
    updateSequenceStats(packet['sequence'])

    time_b.append(elapsed)
    throttle_b.append(packet['throttle'])
    brake_b.append(packet['brake'])
    angle_b.append(packet['angle'])

    if packet['distance'] < 0:
        distance_b.append(math.nan)
    else:
        distance_b.append(packet['distance'])

    latest_state = packet['state']
    latest_distance = packet['distance']
    latest_faults = packet['faults']

    current_time = time.monotonic()
    arrival_times.append(current_time)

    while arrival_times and current_time - arrival_times[0] > 1.0:
        arrival_times.popleft()

    savePacket(packet)


def readSerialData():
    global malformed_packets
    global ser
    global connection_status

    if ser is None:
        return

    try:
        # Drain several packets each update so the dashboard does not fall behind
        for _ in range(100):
            if not ser.in_waiting:
                break

            raw = ser.readline()

            if not raw:
                break

            text = raw.decode('utf-8', errors='ignore').strip()

            if not text:
                continue

            packet = parseTelemetry(text)

            if packet is None:
                malformed_packets += 1
                continue

            addPacket(packet)

    except serial.SerialException as e:
        connection_status = f'Disconnected: {e}'
        print(connection_status)

        try:
            ser.close()
        except Exception:
            pass

        ser = None


def loadReplayFile():
    global replay_packets
    global replay_start_time
    global connection_status

    if not REPLAY_FILE:
        return

    try:
        with open(REPLAY_FILE, 'r', newline='', encoding='utf-8') as file:
            reader = csv.DictReader(file)

            replay_last_raw = None
            replay_wrap = 0
            replay_first_time = None

            for row in reader:
                packet = packetFromCSV(row)

                if packet is None:
                    continue

                timestamp = packet['timestamp']

                if replay_last_raw is not None:
                    if timestamp < replay_last_raw:
                        if replay_last_raw - timestamp > UINT32_SIZE // 2:
                            replay_wrap += UINT32_SIZE

                replay_last_raw = timestamp
                unwrapped = replay_wrap + timestamp

                if replay_first_time is None:
                    replay_first_time = unwrapped

                replay_elapsed = (unwrapped - replay_first_time) / 1000.0
                replay_packets.append((replay_elapsed, packet))

        replay_start_time = time.monotonic()
        connection_status = f'Replaying {REPLAY_FILE}'
        print(f'Loaded {len(replay_packets)} replay packets')

    except OSError as e:
        connection_status = f'Replay error: {e}'
        print(connection_status)


def readReplayData():
    global replay_index
    global connection_status

    if not replay_packets:
        return

    replay_elapsed = (time.monotonic() - replay_start_time) * REPLAY_SPEED

    while replay_index < len(replay_packets):
        packet_time, packet = replay_packets[replay_index]

        if packet_time > replay_elapsed:
            break

        addPacket(packet)
        replay_index += 1

    if replay_index >= len(replay_packets):
        connection_status = 'Replay complete'


def setupGraph():
    global fig
    global ax_throttle
    global ax_brake
    global ax_steer
    global ax_distance
    global line_throttle
    global line_brake
    global line_angle
    global line_distance
    global status_text

    fig, (ax_throttle, ax_brake, ax_steer, ax_distance) = plt.subplots(4, 1, figsize=(11, 10))
    fig.suptitle('Vehicle Telemetry Dashboard')

    # Throttle plot
    line_throttle, = ax_throttle.plot([], [], '-', label='Throttle')
    ax_throttle.set_ylabel('Throttle (%)')
    ax_throttle.set_ylim(-5, 105)
    ax_throttle.legend(loc='upper left')
    ax_throttle.grid(True)

    # Brake plot
    line_brake, = ax_brake.plot([], [], '-', label='Brake')
    ax_brake.set_ylabel('Brake (%)')
    ax_brake.set_ylim(-5, 105)
    ax_brake.legend(loc='upper left')
    ax_brake.grid(True)

    # Steering plot
    line_angle, = ax_steer.plot([], [], '-', label='Wheel angle')
    ax_steer.axhline(0, linewidth=0.8)
    ax_steer.set_ylabel('Steering (deg)')
    ax_steer.set_ylim(-40, 40)
    ax_steer.legend(loc='upper left')
    ax_steer.grid(True)

    # Distance plot
    line_distance, = ax_distance.plot([], [], '-', label='Distance')
    ax_distance.axhline(20, linewidth=0.8, linestyle='--', label='Warning distance')
    ax_distance.axhline(10, linewidth=0.8, linestyle=':', label='Crash distance')
    ax_distance.set_ylabel('Distance (cm)')
    ax_distance.set_xlabel('Elapsed time (s)')
    ax_distance.set_ylim(-5, 405)
    ax_distance.legend(loc='upper left')
    ax_distance.grid(True)

    status_text = fig.text(0.01, 0.01, 'Starting...', fontsize=9)
    fig.canvas.mpl_connect('close_event', closeDashboard)


def update(_frame):
    if REPLAY_FILE:
        readReplayData()
    else:
        readSerialData()

    line_throttle.set_data(time_b, throttle_b)
    line_brake.set_data(time_b, brake_b)
    line_angle.set_data(time_b, angle_b)
    line_distance.set_data(time_b, distance_b)

    if time_b:
        xmax = time_b[-1]
        xmin = max(0, xmax - WINDOW_SECONDS)

        ax_throttle.set_xlim(xmin, max(WINDOW_SECONDS, xmax + 0.1))
        ax_brake.set_xlim(xmin, max(WINDOW_SECONDS, xmax + 0.1))
        ax_steer.set_xlim(xmin, max(WINDOW_SECONDS, xmax + 0.1))
        ax_distance.set_xlim(xmin, max(WINDOW_SECONDS, xmax + 0.1))

        ax_throttle.set_title(f'Throttle: {throttle_b[-1]:.1f}%')
        ax_brake.set_title(f'Brake: {brake_b[-1]:.1f}%')
        ax_steer.set_title(f'Steering: {angle_b[-1]:.0f} deg')

        if math.isnan(distance_b[-1]):
            ax_distance.set_title('Distance: ERROR')
        else:
            ax_distance.set_title(f'Distance: {distance_b[-1]:.0f} cm')

    packet_rate = len(arrival_times)
    fault_text = getFaultText(latest_faults)

    status_text.set_text(
        f'{connection_status} | State={latest_state} | Distance={latest_distance} cm | '
        f'Faults={fault_text} | Rate={packet_rate}/s | '
        f'Received={received_packets} Lost={lost_packets} Duplicate={duplicate_packets} '
        f'Out of order={out_of_order_packets} Malformed={malformed_packets} | {log_name}'
    )

    return line_throttle, line_brake, line_angle, line_distance, status_text


def closeDashboard(_event=None):
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass

    if log_file is not None:
        try:
            log_file.flush()
            log_file.close()
        except Exception:
            pass


def main():
    global ani

    if REPLAY_FILE:
        loadReplayFile()
    else:
        openSerial()
        openLogFile()

    setupGraph()

    ani = FuncAnimation(
        fig,
        update,
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    plt.tight_layout(rect=[0, 0.05, 1, 0.96])
    plt.show()
    closeDashboard()


if __name__ == '__main__':
    main()
