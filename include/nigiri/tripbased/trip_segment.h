#pragma once

#include "nigiri/types.h"

namespace nigiri::tripbased {

struct trip_segment {
  trip_segment() = default;

  trip_segment(transport_idx_t t_idx,
               size_t from_idx,
               size_t to_idx,
               size_t n_transfers)
      : t_idx_(t_idx),
        from_idx_(from_idx),
        to_idx_(to_idx),
        n_transfers_(n_transfers) {
    prev_idx_ = -1;
  }

  trip_segment(transport_idx_t t_idx,
               size_t from_idx,
               size_t to_idx,
               size_t n_transfers,
               size_t prev_idx)
      : t_idx_(t_idx),
        from_idx_(from_idx),
        to_idx_(to_idx),
        prev_idx_(prev_idx),
        n_transfers_(n_transfers) {}

  transport_idx_t t_idx() const { return t_idx_; };
  size_t from() const { return from_idx_; };
  size_t to() const { return to_idx_; }
  size_t prev_idx() const { return prev_idx_; }

private:
  transport_idx_t t_idx_;
  size_t from_idx_;
  size_t to_idx_;
  size_t prev_idx_;
  size_t n_transfers_;
};
}  // namespace nigiri::tripbased
