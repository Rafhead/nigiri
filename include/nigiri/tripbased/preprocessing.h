#pragma once

#include "nigiri/common/it_range.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/types.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

namespace nigiri::tripbased {

nvec<std::uint32_t, transfer, 2> compute_transfers(timetable& tt);

// Help methods
bitfield_idx_t update_time(
    mutable_fws_multimap<location_idx_t,
                         std::pair<minutes_after_midnight_t, bitfield>>& times,
    location_idx_t l_idx,
    minutes_after_midnight_t new_time_on_l,
    const bitfield bf,
    bool day_change);

}  // namespace nigiri::tripbased
