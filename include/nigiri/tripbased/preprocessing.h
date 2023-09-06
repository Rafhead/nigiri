#pragma once

#include "nigiri/common/it_range.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/types.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

namespace nigiri::tripbased {

extern hash_map<bitfield, bitfield_idx_t> bitfields_;

nvec<std::uint32_t, transfer, 2> compute_transfers(timetable& tt);

// Help methods
bitfield_idx_t update_time(
    mutable_fws_multimap<location_idx_t,
                         std::pair<minutes_after_midnight_t, bitfield>>& times,
    location_idx_t l_idx,
    minutes_after_midnight_t new_time_on_l,
    const bitfield bf,
    bool day_change,
    timetable& tt);

// Get bitfield's index or create a bitfield and return the index to new bf
bitfield_idx_t get_bitfield_idx(bitfield const& b, timetable& tt) {
  bitfield_idx_t idx;
  return utl::get_or_create(bitfields_, b, [&]() {
    idx = bitfield_idx_t{tt.bitfields_.size()};
    tt.bitfields_.emplace_back(b);
    return idx;
  });
}

}  // namespace nigiri::tripbased
