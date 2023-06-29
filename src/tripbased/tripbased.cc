#include "nigiri/tripbased/tripbased.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

namespace nigiri::tripbased {

tripbased::tripbased(timetable& tt, routing::query q)
    : tt_{tt}, q_{std::move(q)} {}

}  // namespace nigiri::tripbased
