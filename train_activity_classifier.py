import os
import glob
import numpy as np
import pandas as pd
from sklearn.tree import DecisionTreeClassifier
from sklearn.utils import shuffle
from sklearn.metrics import accuracy_score, classification_report
import joblib

BASE_DIR = "activities_data"
LABELS = ["sitting", "running", "jumpingjacks"]


# ------------------------------------------------------------
# Feature Extraction
# ------------------------------------------------------------
def compute_magnitude(x, y, z):
    return np.sqrt(x*x + y*y + z*z)


def extract_features_from_df(df: pd.DataFrame) -> np.ndarray:
    """
    Extract a rich set of 36 IMU features.
    """

    ax, ay, az = df["ax"], df["ay"], df["az"]
    gx, gy, gz = df["gx"], df["gy"], df["gz"]

    # 1) Basic mean/std (6 + 6 = 12)
    mean_vals = df[["ax","ay","az","gx","gy","gz"]].mean().values
    std_vals  = df[["ax","ay","az","gx","gy","gz"]].std(ddof=0).values

    # 2) Magnitudes
    acc_mag = compute_magnitude(ax, ay, az)
    gyr_mag = compute_magnitude(gx, gy, gz)

    mag_features = [
        acc_mag.mean(),
        acc_mag.std(ddof=0),
        gyr_mag.mean(),
        gyr_mag.std(ddof=0),
    ]

    # 3) Peak-to-peak amplitude
    ptp_features = [
        ax.max()-ax.min(), ay.max()-ay.min(), az.max()-az.min(),
        gx.max()-gx.min(), gy.max()-gy.min(), gz.max()-gz.min()
    ]

    # 4) Signal magnitude area (SMA)
    sma_acc = (abs(ax)+abs(ay)+abs(az)).mean()
    sma_gyr = (abs(gx)+abs(gy)+abs(gz)).mean()

    sma_features = [sma_acc, sma_gyr]

    # 5) Jerk (derivative magnitude)
    dax = np.diff(ax, prepend=ax.iloc[0])
    day = np.diff(ay, prepend=ay.iloc[0])
    daz = np.diff(az, prepend=az.iloc[0])
    jerk_mag = compute_magnitude(dax, day, daz)

    jerk_features = [jerk_mag.mean(), jerk_mag.std(ddof=0)]

    # 6) Axis correlations (acc + gyro)
    def safe_corr(a, b):
        if np.std(a)==0 or np.std(b)==0:
            return 0
        return np.corrcoef(a, b)[0,1]

    corr_features = [
        safe_corr(ax, ay), safe_corr(ay, az), safe_corr(ax, az),
        safe_corr(gx, gy), safe_corr(gy, gz), safe_corr(gx, gz)
    ]

    # Combine all into 1 feature vector
    features = np.concatenate([
        mean_vals,      # 6
        std_vals,       # 6
        mag_features,   # 4
        ptp_features,   # 6
        sma_features,   # 2
        jerk_features,  # 2
        corr_features   # 6
    ])

    return features


# ------------------------------------------------------------
# Dataset Loading
# ------------------------------------------------------------
def load_dataset():
    X = []
    y = []

    for label in LABELS:
        folder = os.path.join(BASE_DIR, label)
        pattern = os.path.join(folder, f"{label}_*.csv")
        files = sorted(glob.glob(pattern))

        if not files:
            print(f"[WARN] No files for '{label}'")
            continue

        print(f"[INFO] Found {len(files)} for '{label}'")

        for path in files:
            df = pd.read_csv(path).dropna()
            if len(df) < 5:
                continue

            X.append(extract_features_from_df(df))
            y.append(label)

    return np.array(X), np.array(y)


def split_dataset(X, y):
    X, y = shuffle(X, y, random_state=42)

    n = len(X)
    n_train = int(0.7*n)
    n_val = int(0.2*n)

    X_train = X[:n_train]
    y_train = y[:n_train]

    X_val = X[n_train:n_train+n_val]
    y_val = y[n_train:n_train+n_val]

    X_test = X[n_train+n_val:]
    y_test = y[n_train+n_val:]

    print(f"[INFO] Split sizes: train={len(y_train)}, val={len(y_val)}, test={len(y_test)}")
    return X_train, y_train, X_val, y_val, X_test, y_test


# ------------------------------------------------------------
# Training Script
# ------------------------------------------------------------
def main():
    X, y = load_dataset()
    if len(y) < 3:
        print("[ERROR] Not enough samples.")
        return

    X_train, y_train, X_val, y_val, X_test, y_test = split_dataset(X, y)

    clf = DecisionTreeClassifier(
        max_depth=5,     # increase depth for more complex activities
        min_samples_leaf=1,
        random_state=42
    )

    clf.fit(X_train, y_train)

    for name, Xs, ys in [("Train", X_train, y_train),
                         ("Val", X_val, y_val),
                         ("Test", X_test, y_test)]:
        if len(ys)==0:
            continue
        y_pred = clf.predict(Xs)
        print(f"\n[{name}] accuracy: {accuracy_score(ys, y_pred):.3f}")
        print(classification_report(ys, y_pred, zero_division=0))

    joblib.dump({"model": clf, "labels": LABELS}, "activity_tree.pkl")
    print("Tree depth:", clf.get_depth())
    print("Number of leaves:", clf.get_n_leaves())
    print("[INFO] Saved model to activity_tree.pkl")


if __name__ == "__main__":
    main()
