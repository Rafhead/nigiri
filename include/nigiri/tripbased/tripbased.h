#pragma once

#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/fws_multimap.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/tripbased/trip_segment.h"
#include "nigiri/types.h"
#include <queue>

namespace nigiri {
struct timetable;
}

namespace nigiri::tripbased {

struct tripbased_stats {
  std::uint64_t n_trip_segments_visited_{0ULL};
  std::uint64_t n_footpaths_visited_{0ULL};
  std::uint64_t n_routes_visited_{0ULL};
};

struct tripbased_state {
  std::vector<trip_segment> trip_segments_;

  trip_segment& get_next_segment() { return trip_segments_[last_]; }

  trip_segment& get_segment(size_t i) {
    if (i < trip_segments_.size()) {
      return trip_segments_[i];
    }
    std::cout << "Trying to get a segment outside of range: "
              << trip_segments_.size() << i << std::endl;
    return trip_segments_[last_];
  }

  void add(trip_segment& new_segment) { trip_segments_.push_back(new_segment); }

  void reset() { trip_segments_.resize(0); }

private:
  size_t last_ = 0U;
};

struct tripbased {
  using algo_stats_t = tripbased_stats;
  using algo_state_t = tripbased_state;
  tripbased(timetable const& tt,
            routing::query const& q,
            nvec<std::uint32_t, transfer, 2>& transfers)
      : tt_{tt}, q_{q}, transfers_{transfers} {};

  // Reset completely R(t)
  void reset_arrivals() {}

  // Used when iterating through start times
  // trip_segments_ must be emptied
  void next_start_time() {}

  // Add start stations - init for algorithm
  void add_start() {}

  // EA query itself - main part of algorithm
  void execute() {}

  // Provide journeys in right format
  void reconstruct() {}

private:
  const timetable& tt_;
  const routing::query& q_;
  const nvec<std::uint32_t, transfer, 2>& transfers_;
};

}  // namespace nigiri::tripbased
