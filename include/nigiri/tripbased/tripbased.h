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

  void add(trip_segment& new_segment) { trip_segments_.push_back(new_segment); }

  void reset() { trip_segments_.resize(0); }
};

struct tripbased {
  using algo_stats_t = tripbased_stats;
  using algo_state_t = tripbased_state;
  tripbased(timetable const& tt,
            tripbased_state& state,
            std::vector<bool>& is_dest,
            nvec<std::uint32_t, transfer, 2>& transfers,
            day_idx_t const base)
      : tt_{tt}, state_{state}, is_dest_{is_dest}, transfers_{transfers} {
    first_locs_.resize(2);
    first_locs_[0].resize(tt_.transport_to_trip_section_.size());
    first_locs_[1].resize(tt_.transport_to_trip_section_.size());
  };

  trip_segment& get_next_segment() {
    return state_.trip_segments_[last_removed_];
  }

  trip_segment& get_segment(size_t i) {
    if (i < state_.trip_segments_.size()) {
      return state_.trip_segments_[i];
    }
    std::cout << "Trying to get a segment outside of range: "
              << state_.trip_segments_.size() << i << std::endl;
    return state_.trip_segments_[last_removed_];
  }

  // Reset completely R(t)
  void reset_arrivals() {
    first_locs_.resize(0);
    first_locs_[0].resize(0);
    first_locs_[1].resize(0);
    first_locs_.resize(2);
    first_locs_[0].resize(tt_.transport_to_trip_section_.size());
    first_locs_[1].resize(tt_.transport_to_trip_section_.size());
  }

  // Used when iterating through start times
  // trip_segments_ must be emptied
  void next_start_time() { state_.reset(); }

  // Add start stations - init for algorithm
  void add_start(location_idx_t const l, unixtime_t const t) {
    auto const time = tt_.day_idx_mam(t);
    abs_q_mam_ = minutes_after_midnight_t{time.first.v_ * 1440} + time.second;
    std::cout << "Time on first station " << l << " is " << abs_q_mam_
              << std::endl;
    auto const src_loc = tt_.locations_.get(l);
    // TODO: find out the earliest trip for a line based on time

    auto const src_loc_footpaths = src_loc.footpaths_out_;
    for (auto footpath_it : src_loc_footpaths) {
    }
  }

  // EA query itself - main part of algorithm
  void execute() {}

  // Provide journeys in right format
  void reconstruct() {}

private:
  void enqueue(transport_idx_t t_idx,
               size_t stop_index,
               size_t n_transfers_,
               size_t day_idx) {
    // TODO: check u is on the same day
    if (stop_index < first_locs_[day_idx][t_idx.v_].v_) {
      auto const new_trip_segment = trip_segment();
    }
  }
  timetable const& tt_;
  tripbased_state& state_;
  const std::vector<bool>& is_dest_;
  const nvec<std::uint32_t, transfer, 2>& transfers_;
  std::vector<location_idx_t> first_locs_d_;
  std::vector<location_idx_t> first_locs_d_next;
  std::vector<std::vector<location_idx_t>> first_locs_;
  size_t last_added_ = 0U;
  size_t last_removed_ = 0U;
  minutes_after_midnight_t abs_q_mam_;
};

}  // namespace nigiri::tripbased
