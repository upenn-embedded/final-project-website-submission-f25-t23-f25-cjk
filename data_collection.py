import serial
import time
import os
import glob
import sys

# -----------------------------------------------------
# CONFIGURE YOUR SERIAL PORT HERE
# -----------------------------------------------------
SERIAL_PORT = "/dev/cu.usbmodem1102"
BAUD = 9600

BASE_DIR = "activities_data"
os.makedirs(BASE_DIR, exist_ok=True)

def get_activity_folder(activity_type):
    """Return the folder path for this activity and create if needed."""
    folder = os.path.join(BASE_DIR, activity_type)
    os.makedirs(folder, exist_ok=True)
    return folder

def get_next_index(activity_type):
    """Return next index based on existing CSVs in the activity folder."""
    folder = get_activity_folder(activity_type)
    pattern = os.path.join(folder, f"{activity_type}_*.csv")
    existing = glob.glob(pattern)
    return len(existing)

def record_activity(activity_type, duration_seconds=10):
    """Record IMU UART output for 10 seconds and save to CSV."""
    folder = get_activity_folder(activity_type)
    index = get_next_index(activity_type)
    out_path = os.path.join(folder, f"{activity_type}_{index}.csv")

    print(f"Opening serial port: {SERIAL_PORT} @ {BAUD} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)

    time.sleep(2)

    print(f"Recording {duration_seconds} seconds of data...")
    print(f"Saving to {out_path}")

    start_time = time.time()
    lines = []

    # CSV header
    header = "t_ms,ax,ay,az,gx,gy,gz"
    lines.append(header)

    while time.time() - start_time < duration_seconds:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if raw:
                # Remove prefixes like "t=", "ax=", etc.
                raw = (
                    raw.replace("t=", "")
                       .replace("ax=", "")
                       .replace("ay=", "")
                       .replace("az=", "")
                       .replace("gx=", "")
                       .replace("gy=", "")
                       .replace("gz=", "")
                       .replace(" ", "")
                )
                lines.append(raw)
                print(raw)

        except KeyboardInterrupt:
            break

    ser.close()

    # Write CSV file
    with open(out_path, "w") as f:
        f.write("\n".join(lines))

    print("Saved:", out_path)
    print("Done.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        activity = sys.argv[1].strip()
    else:
        activity = input("Enter activity type (e.g., sitting, running, jumpingjacks): ").strip()

    if not activity:
        print("Error: Activity type cannot be empty")
        sys.exit(1)

    record_activity(activity)
