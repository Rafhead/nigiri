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

void reconstruct_journey(timetable const&,
                         query const&,
                         tripbased_state const&,
                         journey&){};

}  // namespace nigiri::tripbased