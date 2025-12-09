/* Auto-generated decision tree from activity_tree.pkl */
#ifndef ACTIVITY_MODEL_H
#define ACTIVITY_MODEL_H

// features[i] correspond to the training feature order.
// Number of features: 32

#define ACTIVITY_SITTING 0
#define ACTIVITY_RUNNING 1
#define ACTIVITY_JUMPINGJACKS 2

int predict_activity(const float features[32]);

#endif // ACTIVITY_MODEL_H
