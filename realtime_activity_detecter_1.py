import serial
import time
import numpy as np
import joblib

# ------------------ CONFIG ------------------
SERIAL_PORT = "/dev/cu.usbmodem1102"  # change if your port name differs
BAUD = 9600
WINDOW_SIZE = 100   # number of samples per window (~1 second at 100 Hz)
MODEL_PATH = "activity_tree.pkl"
# --------------------------------------------


def extract_features_from_window(ax, ay, az, gx, gy, gz):
    """
    ax, ay, ... gz are numpy arrays of shape (N,)
    Returns the same 12-D feature vector as in training:
      [mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz,
       std_ax,  std_ay,  std_az,  std_gx,  std_gy,  std_gz]
    """
    means = np.array([
        ax.mean(), ay.mean(), az.mean(),
        gx.mean(), gy.mean(), gz.mean()
    ])

    stds = np.array([
        ax.std(ddof=0), ay.std(ddof=0), az.std(ddof=0),
        gx.std(ddof=0), gy.std(ddof=0), gz.std(ddof=0)
    ])

    return np.concatenate([means, stds])


def parse_line(line: str):
    """
    Parse a line like:
      17880,3584,-365,16175,53,25,-8
    into (t_ms, ax, ay, az, gx, gy, gz) or None on failure.
    """
    line = line.strip()
    if not line:
        return None

    parts = line.split(",")
    if len(parts) != 7:
        return None

    try:
        vals = list(map(int, parts))
    except ValueError:
        return None

    return vals  # [t_ms, ax, ay, az, gx, gy, gz]


def main():
    # Load trained model
    saved = joblib.load(MODEL_PATH)
    clf = saved["model"]

    print(f"[INFO] Loaded model from {MODEL_PATH}")

    print(f"[INFO] Opening serial port {SERIAL_PORT} @ {BAUD} baud...")
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    time.sleep(2)
    print("[INFO] Listening for IMU data...")

    # Buffers for window
    ax_buf, ay_buf, az_buf = [], [], []
    gx_buf, gy_buf, gz_buf = [], [], []

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue

            try:
                line = raw.decode("utf-8", errors="ignore").strip()
            except UnicodeDecodeError:
                continue

            parsed = parse_line(line)
            if parsed is None:
                continue

            t_ms, ax, ay, az, gx, gy, gz = parsed

            # append to buffers
            ax_buf.append(ax)
            ay_buf.append(ay)
            az_buf.append(az)
            gx_buf.append(gx)
            gy_buf.append(gy)
            gz_buf.append(gz)

            # when we have enough samples, classify
            if len(ax_buf) >= WINDOW_SIZE:
                ax_arr = np.array(ax_buf[-WINDOW_SIZE:])
                ay_arr = np.array(ay_buf[-WINDOW_SIZE:])
                az_arr = np.array(az_buf[-WINDOW_SIZE:])
                gx_arr = np.array(gx_buf[-WINDOW_SIZE:])
                gy_arr = np.array(gy_buf[-WINDOW_SIZE:])
                gz_arr = np.array(gz_buf[-WINDOW_SIZE:])

                feat = extract_features_from_window(
                    ax_arr, ay_arr, az_arr, gx_arr, gy_arr, gz_arr
                ).reshape(1, -1)

                pred = clf.predict(feat)[0]
                print(f"[PREDICTION] Activity: {pred}")

                # === send prediction back to MCU ===
                if pred == "sitting":
                    ser.write(b"S")   # sitting
                elif pred == "running":
                    ser.write(b"R")   # running
                else:
                    ser.write(b"U")   # unknown

                # Option: keep overlapping windows.
                # Here we just keep using the last WINDOW_SIZE samples, so
                # buffer continues to grow but we always slice the tail.
                # If you want non-overlapping windows, uncomment below:
                # ax_buf.clear(); ay_buf.clear(); az_buf.clear()
                # gx_buf.clear(); gy_buf.clear(); gz_buf.clear()

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
