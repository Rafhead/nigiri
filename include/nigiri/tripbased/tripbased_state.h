#pragma once

#include "nigiri/types.h"

namespace nigiri::tripbased {

struct tripbased_state {
public:
  void add(trip_segment const& new_segment) {
    trip_segments_.push_back(new_segment);
  }

  size_t segments_size() { return trip_segments_.size(); }

  void reset() { trip_segments_.clear(); }

  void add_best(best_on_target const& b) {
    for (auto best : best_) {
      if (b.dominates(best)) {
        best.day_ = day_idx_t::invalid();
      }
    }
  }

  std::vector<best_on_target> best_;
  std::vector<trip_segment> trip_segments_;
};

}  // namespace nigiri::tripbased