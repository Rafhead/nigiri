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
  tripbased(timetable& tt,
            routing::query q,
            nvec<std::uint32_t, transfer, 2>& transfers);

private:
  timetable& tt_;
  routing::query q_;
  nvec<std::uint32_t, transfer, 2>& transfers_;
};

}  // namespace nigiri::tripbased
