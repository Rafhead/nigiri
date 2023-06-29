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
bool update_time();

}  // namespace nigiri::tripbased
