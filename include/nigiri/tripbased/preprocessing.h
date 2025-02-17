#pragma once

#include "nigiri/common/it_range.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/types.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

namespace nigiri::tripbased {
static constexpr uint16_t const bitsetSize = 512U;
static constexpr uint32_t const max_transport_idx =
    transfer::max_transport_idx - 1;

static hash_map<bitfield, bitfield_idx_t> bitfields_;

static constexpr bool debug = false;

nvec<std::uint32_t, transfer, 2> compute_transfers(timetable& tt);

// Help methods
bitfield_idx_t update_time(
    std::vector<std::vector<std::pair<minutes_after_midnight_t, bitfield>>>&
        times,
    location_idx_t l_idx,
    minutes_after_midnight_t new_time_on_l,
    const bitfield bf,
    bool day_change,
    timetable& tt);

// Get bitfield's index or create a bitfield and return the index to new bf
const bitfield_idx_t get_bitfield_idx(bitfield const& b, timetable& tt);

}  // namespace nigiri::tripbased
