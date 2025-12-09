/* Auto-generated decision tree from activity_tree.pkl */
#include "activity_model.h"

int predict_activity(const float features[32]) {
  if (features[13] <= 4548.258301f) {  // acc_mag_std
    return 0;  // sitting
  } else {  // acc_mag_std
    if (features[0] <= 12306.643066f) {  // mean_ax
      return 2;  // jumpingjacks
    } else {  // mean_ax
      return 1;  // running
    }
  }
}
