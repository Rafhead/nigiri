#pragma once

#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/fws_multimap.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/types.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::tripbased {

struct tripbased {
  tripbased(timetable& tt, routing::query q);

  static constexpr auto const bits_per_block = 64U;

private:
  void init_arrival_times();
  void extract_transfers();
  bool update_time();

  timetable& tt_;
  routing::query q_;
  // To store indexes to bitsets in timetable for times on stations
  vecvec<location_idx_t, minutes_after_midnight_t> arrival_times_ =
      vecvec<location_idx_t, minutes_after_midnight_t>();
  vecvec<location_idx_t, minutes_after_midnight_t> ea_change_times_ =
      vecvec<location_idx_t, minutes_after_midnight_t>();
  // Transfers
  nvec<std::uint32_t, transfer, 2> transfers_ =
      nvec<std::uint32_t, transfer, 2>();
  fws_multimap<std::uint32_t> transfer_idxs_ = fws_multimap<std::uint32_t>();
};

}  // namespace nigiri::tripbased
