#pragma once

#include "nigiri/types.h"

namespace nigiri::tripbased {

struct trip_segment {
  trip_segment(transport_idx_t t_idx,
               location_idx_t from_idx,
               location_idx_t to_idx)
      : t_idx_(t_idx), from_idx_(from_idx), to_idx_(to_idx) {}

  transport_idx_t t_idx() const { return t_idx_; };
  location_idx_t from() const { return from_idx_; };
  location_idx_t to() const { return to_idx_; }

private:
  transport_idx_t t_idx_;
  location_idx_t from_idx_;
  location_idx_t to_idx_;
};
}  // namespace nigiri::tripbased
