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

  void add(trip_segment const& new_segment) {
    trip_segments_.push_back(new_segment);
  }

  void reset() { trip_segments_.resize(0); }
};

struct tripbased {
  using algo_stats_t = tripbased_stats;
  using algo_state_t = tripbased_state;
  tripbased(timetable const& tt,
            tripbased_state& state,
            std::vector<bool>& is_dest,
            nvec<std::uint32_t, transfer, 2>& transfers)
      : tt_{tt}, state_{state}, transfers_{transfers} {
    first_locs_.resize(2);
    first_locs_[0].resize(tt_.transport_to_trip_section_.size());
    utl::fill(first_locs_[0], 256U);

    first_locs_[1].resize(tt_.transport_to_trip_section_.size());
    utl::fill(first_locs_[1], 256U);

    n_transfers_ = 0U;

    is_dest_.resize(tt_.n_locations());
    is_dest_line_.resize(tt_.n_routes());
    utl::fill(is_dest_, std::make_pair(false, duration_t{0U}));
    utl::fill(is_dest_line_, std::make_pair(0U, duration_t{0}));
    find_target_lines(is_dest);
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
    utl::fill(first_locs_[0], 256U);
    first_locs_[1].resize(tt_.transport_to_trip_section_.size());
    utl::fill(first_locs_[1], 256U);
  }

  // Used when iterating through start times
  // trip_segments_ must be emptied
  void next_start_time() {
    state_.reset();
    n_transfers_ = 0U;
  }

  // Add start stations - init for algorithm
  void add_start(location_idx_t const l, unixtime_t const t) {
    // day = absolute day index
    auto const [day, mam] = tt_.day_idx_mam(t);
    q_day_ = day;
    q_mam_ = mam;
    abs_q_mam_ = minutes_after_midnight_t{day.v_ * 1440U} + mam;
    std::cout << "Time on first station " << l << " is " << abs_q_mam_
              << std::endl;

    // for the first location
    enqueue_start_trips(l, duration_t{0U});

    // for all that are reachable by footpath
    auto const src_loc = tt_.locations_.get(l);
    auto const src_loc_footpaths = src_loc.footpaths_out_;
    for (auto footpath : src_loc_footpaths) {
      enqueue_start_trips(footpath.target(), footpath.duration());
    }
  }

  // EA query itself - main part of algorithm
  void execute() {}

  // Provide journeys in right format
  void reconstruct() {}

private:
  void enqueue(route_idx_t r_idx,
               transport_idx_t t_idx,
               size_t stop_index,
               size_t n_transfers,
               size_t day_idx) {
    if (stop_index < first_locs_[day_idx][t_idx.v_]) {
      auto const latest_loc = first_locs_[day_idx][t_idx.v_];
      auto new_trip_segment =
          trip_segment(t_idx, stop_index, latest_loc, n_transfers_);
      state_.add(new_trip_segment);

      auto const& transport_bitset =
          tt_.bitfields_[tt_.transport_traffic_days_[t_idx]];
      auto const [day_at_stop, mam_at_stop] =
          tt_.event_mam(t_idx, stop_index, event_type::kDep);
      auto const transport_range = tt_.route_transport_ranges_[r_idx];
      for (auto const transport : transport_range) {
        if (transport == t_idx) {
          continue;
        }
        // Check day if it is the same and time is later for other trips of the
        // same line
        auto const& next_transport_bitset =
            tt_.bitfields_[tt_.transport_traffic_days_[transport]];
        // TODO: Potentially count multiple same stations in route plan
        auto const [next_transport_day_at_stop, next_transport_mam_at_stop] =
            tt_.event_mam(transport, stop_index, event_type::kDep);
        if (next_transport_bitset.test(to_idx(
                q_day_ - next_transport_day_at_stop + day_idx_t{day_idx})) &&
            next_transport_mam_at_stop >= mam_at_stop) {
          first_locs_[day_idx][transport.v_] =
              std::min(stop_index, first_locs_[day_idx][transport.v_]);
        }
      }
    }
  }

  void enqueue_start_trips(location_idx_t l, duration_t footpath) {
    // Footpath can make day change
    auto mam_to_target = q_mam_ + footpath;
    auto day_change = false;
    if (mam_to_target.count() / 1440U > q_mam_.count() / 1440U) {
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
        // Loop continues for possible same stations in the route plan
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
        auto trip_on_the_next_day = 0U;
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
          if (transport_bitset.test(to_idx(q_day_ - day_at_stop)) &&
              mam_at_stop % 1440U >= mam_to_target.count() % 1440U &&
              transport_delta < ed_delta) {
            ed_transport_idx = transport.v_;
            ed_delta = transport_delta;
            trip_on_the_next_day = 0U;
            continue;
          }
          if (!day_change &&
              transport_bitset.test(to_idx(q_day_ + 1U - day_at_stop)) &&
              mam_at_stop % 1440U < mam_to_target.count() % 1440U &&
              transport_delta < ed_delta) {
            ed_transport_idx = transport.v_;
            ed_delta = transport_delta;
            trip_on_the_next_day = 1U;
          }
        }

        // If trip for this route was found
        if (ed_transport_idx != 8192U) {
          enqueue(r_idx, transport_idx_t{ed_transport_idx}, i, 0,
                  trip_on_the_next_day);
        }
      }
    }
  }

  // TODO: same line visits multiple destination stops
  // structure dest_lines with stop offset and duration
  void find_target_lines(std::vector<bool>& is_dest) {
    for (auto i = 0U; i < is_dest.size(); i++) {
      if (is_dest[i]) {
        // For initial destination itself
        auto routes_from_loc = tt_.location_routes_[location_idx_t{i}];
        for (auto route_from_loc : routes_from_loc) {
          auto stop_idx = 0U;
          auto const route_stops = tt_.route_location_seq_[route_from_loc];
          // Find index of the location relative to route
          // TODO: multiple same stations - circled routes?
          for (auto route_loc : route_stops) {
            if (route_loc == i) {
              stop_idx = route_loc;
              break;
            }
          }
          is_dest_line_[route_from_loc.v_] =
              std::make_pair(stop_idx, duration_t{0U});
        }
        // TODO: footpaths_in are footpath from other stations to this?
        auto const footpaths_in =
            tt_.locations_.get(location_idx_t{i}).footpaths_in_;
        auto duration = duration_t{0U};
        auto from_loc_idx = location_idx_t{0U};
        // TODO: meta stations? start_times.cc
        for (auto footpath : footpaths_in) {
          duration = footpath.duration();
          from_loc_idx = footpath.target();
          is_dest_[from_loc_idx.v_] = std::make_pair(true, duration);
          // Iterate through routes of this location
          routes_from_loc = tt_.location_routes_[from_loc_idx];
          for (auto route_from_loc : routes_from_loc) {
            auto stop_idx = 0U;
            auto const route_stops = tt_.route_location_seq_[route_from_loc];
            // Find index of the location relative to route
            // TODO: multiple same stations - circled routes?
            for (auto route_loc : route_stops) {
              if (route_loc == from_loc_idx.v_) {
                stop_idx = route_loc;
                break;
              }
            }
            is_dest_line_[route_from_loc.v_] =
                std::make_pair(stop_idx, duration);
          }
        }
      }
    }
  }

  timetable const& tt_;
  tripbased_state& state_;
  std::vector<std::pair<bool, duration_t>> is_dest_;
  std::vector<std::pair<uint32_t, duration_t>> is_dest_line_;
  const nvec<std::uint32_t, transfer, 2>& transfers_;
  std::vector<std::vector<size_t>> first_locs_;
  size_t last_added_ = 0U;
  size_t last_removed_ = 0U;
  minutes_after_midnight_t abs_q_mam_;
  day_idx_t q_day_;
  minutes_after_midnight_t q_mam_;
  size_t n_transfers_;
};

}  // namespace nigiri::tripbased
