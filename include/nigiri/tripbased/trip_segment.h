#pragma once

#include "nigiri/types.h"

namespace nigiri::tripbased {

struct trip_segment {
  trip_segment() = default;

  trip_segment(transport_idx_t t_idx,
               size_t from_idx,
               size_t to_idx,
               uint8_t n_transfers,
               bool on_query_day)
      : t_idx_(t_idx),
        from_idx_(from_idx),
        to_idx_(to_idx),
        n_transfers_(n_transfers),
        on_query_day_(on_query_day) {
    // for the first stations the previous index is -1
    prev_idx_ = -1;
  }

  trip_segment(transport_idx_t t_idx,
               size_t from_idx,
               size_t to_idx,
               uint8_t n_transfers,
               size_t prev_idx,
               bool on_query_day)
      : t_idx_(t_idx),
        from_idx_(from_idx),
        to_idx_(to_idx),
        prev_idx_(prev_idx),
        n_transfers_(n_transfers),
        on_query_day_(on_query_day) {}

  transport_idx_t t_idx() const { return t_idx_; };
  size_t from() const { return from_idx_; };
  size_t to() const { return to_idx_; }
  size_t prev_idx() const { return prev_idx_; }
  uint8_t n_transfers() const { return n_transfers_; }
  bool on_query_day() const { return on_query_day_; }

private:
  transport_idx_t t_idx_;
  size_t from_idx_;
  size_t to_idx_;
  size_t prev_idx_;
  uint8_t n_transfers_;
  // If trip segment is on query day on station from_idx
  bool on_query_day_;
};
}  // namespace nigiri::tripbased
