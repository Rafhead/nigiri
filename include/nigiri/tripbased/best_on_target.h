#pragma once

#include "nigiri/types.h"

namespace nigiri::tripbased {

struct best_on_target {
  size_t segment_idx_;
  // day on first stop
  day_idx_t day_;
  // stop from which target was reached
  stop_idx_t stop_idx_;
  unixtime_t start_time_;
  unixtime_t abs_time_on_target_;
  uint8_t n_transfers_;

  bool dominates(best_on_target const& o) const {
    if (start_time_ <= abs_time_on_target_) {
      return n_transfers_ <= o.n_transfers_ && start_time_ >= o.start_time_ &&
             abs_time_on_target_ <= o.abs_time_on_target_;
    } else {
      return n_transfers_ <= o.n_transfers_ && start_time_ <= o.start_time_ &&
             abs_time_on_target_ >= o.abs_time_on_target_;
    }
  }
};

}  // namespace nigiri::tripbased