import joblib
import numpy as np

MODEL_PATH = "activity_tree.pkl"


def export_tree_to_c():
    saved = joblib.load(MODEL_PATH)
    clf = saved["model"]
    labels = list(saved["labels"])  # canonical class order for C indices

    tree = clf.tree_
    feature = tree.feature
    threshold = tree.threshold
    children_left = tree.children_left
    children_right = tree.children_right
    value = tree.value  # shape (n_nodes, 1, n_classes)

    # How many features did the model train with?
    n_features = clf.n_features_in_
    print("[INFO] Model n_features_in_:", n_features)

    # Try to infer feature names based on known configs
    feature_names_12 = [
        "mean_ax", "mean_ay", "mean_az",
        "mean_gx", "mean_gy", "mean_gz",
        "std_ax",  "std_ay",  "std_az",
        "std_gx",  "std_gy",  "std_gz",
    ]

    feature_names_32 = [
        # 6 means
        "mean_ax", "mean_ay", "mean_az",
        "mean_gx", "mean_gy", "mean_gz",
        # 6 stds
        "std_ax",  "std_ay",  "std_az",
        "std_gx",  "std_gy",  "std_gz",
        # 4 magnitudes
        "acc_mag_mean", "acc_mag_std",
        "gyr_mag_mean", "gyr_mag_std",
        # 6 peak-to-peak
        "ptp_ax", "ptp_ay", "ptp_az",
        "ptp_gx", "ptp_gy", "ptp_gz",
        # 2 SMA
        "sma_acc", "sma_gyr",
        # 2 jerk stats
        "jerk_mean", "jerk_std",
        # 6 correlations
        "corr_ax_ay", "corr_ay_az", "corr_ax_az",
        "corr_gx_gy", "corr_gy_gz", "corr_gx_gz",
    ]

    if n_features == 12:
        feature_names = feature_names_12
    elif n_features == 32:
        feature_names = feature_names_32
    else:
        # Fallback: generic names f0..fN-1
        feature_names = [f"f{i}" for i in range(n_features)]
    assert len(feature_names) == n_features

    print("[INFO] Tree node_count:", tree.node_count)
    print("[INFO] Sklearn classes_:", clf.classes_)
    print("[INFO] Canonical labels:", labels)

    # ------------------------------------------------------------------
    # Build mapping from sklearn's internal class index -> our C index
    # ------------------------------------------------------------------
    sk_classes = list(clf.classes_)
    sk_to_c = {}

    for sk_idx, sk_name in enumerate(sk_classes):
        if sk_name in labels:
            c_idx = labels.index(sk_name)
        else:
            labels.append(sk_name)
            c_idx = labels.index(sk_name)
        sk_to_c[sk_idx] = c_idx

    print("[INFO] sklearn->C class index map:", sk_to_c)

    lines = []

    def gen_node(node_id, indent=2):
        ind = " " * indent
        left = children_left[node_id]
        right = children_right[node_id]

        # Leaf node
        if left == -1 and right == -1:
            class_counts = value[node_id][0]
            sk_idx = int(np.argmax(class_counts))
            c_idx = sk_to_c[sk_idx]
            c_name = labels[c_idx]
            lines.append(f"{ind}return {c_idx};  // {c_name}")
            return

        feat_idx = feature[node_id]
        thr = threshold[node_id]

        # Safety check: feature index should be valid
        if feat_idx < 0 or feat_idx >= n_features:
            raise ValueError(f"Invalid feature index {feat_idx} for n_features={n_features}")

        feat_name = feature_names[feat_idx]

        lines.append(
            f"{ind}if (features[{feat_idx}] <= {thr:.6f}f) {{  // {feat_name}"
        )
        gen_node(left, indent + 2)
        lines.append(f"{ind}}} else {{  // {feat_name}")
        gen_node(right, indent + 2)
        lines.append(f"{ind}}}")

    # Special case: 1-node tree (single leaf)
    if tree.node_count == 1:
        class_counts = value[0][0]
        sk_idx = int(np.argmax(class_counts))
        c_idx = sk_to_c[sk_idx]
        c_name = labels[c_idx]
        print(f"[WARN] Tree has only one node, always predicts: {c_name} (C index {c_idx})")

        with open("activity_model.h", "w") as f:
            f.write("/* Auto-generated decision tree (single leaf) */\n")
            f.write("#ifndef ACTIVITY_MODEL_H\n#define ACTIVITY_MODEL_H\n\n")
            for idx, name in enumerate(labels):
                macro = "ACTIVITY_" + name.upper().replace(" ", "_")
                f.write(f"#define {macro} {idx}\n")
            f.write(f"\nint predict_activity(const float features[{n_features}]);\n\n")
            f.write("#endif // ACTIVITY_MODEL_H\n")

        with open("activity_model.c", "w") as f:
            f.write("/* Auto-generated decision tree (single leaf) */\n")
            f.write("#include \"activity_model.h\"\n\n")
            f.write("int predict_activity(const float features[%d]) {\n" % n_features)
            f.write(f"    return {c_idx};  // {c_name}\n")
            f.write("}\n")

        print("[INFO] Generated activity_model.c/.h for leaf-only tree")
        return

    # General multi-node case
    gen_node(0)

    if not lines:
        print("[WARN] No lines generated; falling back to majority-class stub.")
        class_counts = value[0][0]
        sk_idx = int(np.argmax(class_counts))
        c_idx = sk_to_c[sk_idx]
        c_name = labels[c_idx]

        with open("activity_model.h", "w") as f:
            f.write("/* Fallback stub: majority-class predictor */\n")
            f.write("#ifndef ACTIVITY_MODEL_H\n#define ACTIVITY_MODEL_H\n\n")
            for idx, name in enumerate(labels):
                macro = "ACTIVITY_" + name.upper().replace(" ", "_")
                f.write(f"#define {macro} {idx}\n")
            f.write(f"\nint predict_activity(const float features[{n_features}]);\n\n")
            f.write("#endif // ACTIVITY_MODEL_H\n")

        with open("activity_model.c", "w") as f:
            f.write("/* Fallback stub: majority-class predictor */\n")
            f.write("#include \"activity_model.h\"\n\n")
            f.write("int predict_activity(const float features[%d]) {\n" % n_features)
            f.write(f"    return {c_idx};  // {c_name}\n")
            f.write("}\n")
        return

    # ------------------------------------------------------------------
    # Write header
    # ------------------------------------------------------------------
    with open("activity_model.h", "w") as f:
        f.write("/* Auto-generated decision tree from activity_tree.pkl */\n")
        f.write("#ifndef ACTIVITY_MODEL_H\n#define ACTIVITY_MODEL_H\n\n")
        f.write("// features[i] correspond to the training feature order.\n")
        f.write(f"// Number of features: {n_features}\n\n")
        for idx, name in enumerate(labels):
            macro = "ACTIVITY_" + name.upper().replace(" ", "_")
            f.write(f"#define {macro} {idx}\n")
        f.write(f"\nint predict_activity(const float features[{n_features}]);\n\n")
        f.write("#endif // ACTIVITY_MODEL_H\n")

    # ------------------------------------------------------------------
    # Write source
    # ------------------------------------------------------------------
    with open("activity_model.c", "w") as f:
        f.write("/* Auto-generated decision tree from activity_tree.pkl */\n")
        f.write("#include \"activity_model.h\"\n\n")
        f.write("int predict_activity(const float features[%d]) {\n" % n_features)
        for line in lines:
            f.write(line + "\n")
        f.write("}\n")

    print("[INFO] Generated activity_model.c and activity_model.h")


if __name__ == "__main__":
    export_tree_to_c()
