#pragma once

#include "nigiri/types.h"
#include "tripbased.h"

namespace nigiri {
struct timetable;
}  // namespace nigiri

namespace nigiri::tripbased {

struct tripbased_state;
using query = nigiri::routing::query;
using journey = nigiri::routing::journey;

void reconstruct_journey(
    timetable const& tt,
    query const& q,
    day_idx_t q_day,
    tripbased_state const& state,
    std::vector<std::pair<location_idx_t, duration_t>> const& is_dest,
    journey& j);

}  // namespace nigiri::tripbased