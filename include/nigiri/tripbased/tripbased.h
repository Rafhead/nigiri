#pragma once

#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/fws_multimap.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/tripbased/trip_segment.h"
#include "nigiri/types.h"
#include <queue>
#include "utl/enumerate.h"

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
            nvec<std::uint32_t, transfer, 2>& transfers)
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
    // day = absolute day index
    auto const [day, mam] = tt_.day_idx_mam(t);
    abs_q_mam_ = minutes_after_midnight_t{day.v_ * 1440U} + mam;
    std::cout << "Time on first station " << l << " is " << abs_q_mam_
              << std::endl;

    // for the first location
    enqueue_start_trips(l, day, mam, duration_t{0U});

    // for all that are reachable by footpath
    auto const src_loc = tt_.locations_.get(l);
    auto const src_loc_footpaths = src_loc.footpaths_out_;
    for (auto footpath : src_loc_footpaths) {
      enqueue_start_trips(footpath.target(), day, mam, footpath.duration());
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

  void enqueue_start_trips(location_idx_t l,
                           day_idx_t day,
                           duration_t mam,
                           duration_t footpath) {
    // Footpath can make day change
    auto mam_to_target = mam + footpath;
    auto day_change = false;
    if (mam_to_target.count() / 1440U > mam.count() / 1440U) {
      day_change = true;
      mam_to_target = duration_t{mam_to_target.count() - 1440U};
    }
    // Iterate through routes visiting station
    for (auto const r_idx : tt_.location_routes_[location_idx_t{l}]) {
      // Iterate through stops of route
      auto const loc_seq = tt_.route_location_seq_.at(r_idx);
      for (auto const [i, s] : utl::enumerate(loc_seq)) {
        auto const r_stop = stop{s};

        // Skip if this is another stop
        if (r_stop.location_idx() != l) {
          continue;
        }

        // Check if this stop is not the last in the sequence, and it is
        // possible to get on board to it
        if ((i == loc_seq.size() - 1) || !r_stop.in_allowed()) {
          continue;
        }

        // Found a station that is the start station
        // Iterate through trip of this route
        auto const& transport_range = tt_.route_transport_ranges_[r_idx];
        auto ed_transport_idx = 8192U;
        auto ed_delta = delta{1024U, 1441U};
        for (auto const transport : transport_range) {
          auto const& transport_bitset =
              tt_.bitfields_[tt_.transport_traffic_days_[transport]];
          auto const [day_at_stop, mam_at_stop] =
              tt_.event_mam(transport, i, event_type::kDep);
          // If trip is active:
          // 1. On the query day  &&   mam of trip later than query mam
          // 2. On the next day   &&   mam of trip earlier that query mam &&
          //                           footpath doesn't make day change
          auto const transport_delta = delta{day_at_stop, mam_at_stop};
          if (transport_bitset.test(to_idx(day - day_at_stop)) &&
              mam_at_stop % 1440U >= mam_to_target.count() % 1440U &&
              transport_delta < ed_delta) {
            ed_transport_idx = transport.v_;
            ed_delta = transport_delta;
            continue;
          }
          if (!day_change &&
              transport_bitset.test(to_idx(day + 1 - day_at_stop)) &&
              mam_at_stop % 1440U < mam_to_target.count() % 1440U &&
              transport_delta < ed_delta) {
            ed_transport_idx = transport.v_;
            ed_delta = transport_delta;
          }
        }

        // If trip for this route was found
        if (ed_transport_idx != 8192U) {
          enqueue(transport_idx_t{ed_transport_idx}, i, 0, ed_delta.days_);
        }
      }
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
