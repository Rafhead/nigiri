#pragma once

#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/fws_multimap.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/types.h"
#include <queue>

namespace nigiri {
struct timetable;
}

namespace nigiri::tripbased {

struct tripbased {
  tripbased(timetable& tt,
            routing::query q,
            nvec<std::uint32_t, transfer, 2>& transfers);

private:
  const timetable& tt_;
  const routing::query q_;
  const nvec<std::uint32_t, transfer, 2>& transfers_;
  std::queue<std::uint16_t> trip_segments_;
};

}  // namespace nigiri::tripbased
