#include "nigiri/tripbased/tripbased.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

namespace nigiri::tripbased {

tripbased::tripbased(timetable& tt,
                     routing::query q,
                     nvec<uint32_t, transfer, 2>& transfers)
    : tt_{tt}, q_{std::move(q)}, transfers_(transfers) {}

}  // namespace nigiri::tripbased
