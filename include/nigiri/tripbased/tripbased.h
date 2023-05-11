#pragma once

#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/fws_multimap.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/types.h"

#ifndef NIGIRI_TRIPBASED_H
#define NIGIRI_TRIPBASED_H

namespace nigiri {
struct timetable;
}

namespace nigiri::tripbased {

struct tripbased {
  tripbased(timetable& tt, routing::query q);

  static constexpr auto const bits_per_block = 64U;

private:
  void extract_transfers();

  timetable& tt_;
  routing::query q_;
  nvec<std::uint32_t, transfer, 3> transfers_ =
      nvec<std::uint32_t, transfer, 3>();
  fws_multimap<std::uint32_t> transfer_indexes_ = fws_multimap<std::uint32_t>();
};

}  // namespace nigiri::tripbased

#endif  // NIGIRI_TRIPBASED_H
