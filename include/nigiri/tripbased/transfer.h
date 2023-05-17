#pragma once

#include "nigiri/types.h"

namespace nigiri::tripbased {

struct transfer {
  transfer(transport_idx_t const to,
           unsigned const stop_idx,
           bitfield_idx_t const traffic_days_idx,
           bool const day_change)
      : to_transport_idx_{to_idx(to)},
        to_transport_stop_idx_{stop_idx},
        traffic_days_idx_{to_idx(traffic_days_idx)},
        day_change_{day_change ? 1U : 0U} {
    assert(to_idx(to) < (std::numeric_limits<std::uint32_t>::max() >> 4));
    assert(to_idx(traffic_days_idx) <
           (std::numeric_limits<std::uint32_t>::max() >> 8));
    assert(stop_idx < 2048);
  }

  transport_idx_t to() const { return transport_idx_t{to_transport_idx_}; }
  unsigned stop_idx() const { return to_transport_stop_idx_; }
  bool day_change() const { return day_change_ != 0U; }
  bitfield_idx_t traffic_days() const {
    return bitfield_idx_t{traffic_days_idx_};
  }

private:
  std::uint64_t to_transport_idx_ : 28;  // max 278M
  std::uint64_t to_transport_stop_idx_ : 11;  // 2048
  std::uint64_t traffic_days_idx_ : 24;  // 16M
  std::uint64_t day_change_ : 1;
};

}  // namespace nigiri::tripbased
