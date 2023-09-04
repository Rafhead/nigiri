#pragma once

#include "nigiri/common/delta_t.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/fws_multimap.h"
#include "nigiri/tripbased/reconstruct.h"
#include "nigiri/tripbased/transfer.h"
#include "nigiri/tripbased/trip_segment.h"
#include "nigiri/types.h"
#include <queue>
#include "utl/enumerate.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::tripbased {

using journey = routing::journey;
using query = routing::query;

struct best_on_target {
  size_t segment_idx_;
  // day on first stop
  day_idx_t day_;
  // stop from which target was reached
  stop_idx_t stop_idx_;
  unixtime_t start_time_;
  unixtime_t abs_time_on_target_;
  uint8_t n_transfers_;

  bool dominates(best_on_target const& o) const {
    if (start_time_ <= abs_time_on_target_) {
      return n_transfers_ <= o.n_transfers_ && start_time_ >= o.start_time_ &&
             abs_time_on_target_ <= o.abs_time_on_target_;
    } else {
      return n_transfers_ <= o.n_transfers_ && start_time_ <= o.start_time_ &&
             abs_time_on_target_ >= o.abs_time_on_target_;
    }
  }
};

struct tripbased_stats {
  std::uint64_t n_trip_segments_visited_{0ULL};
  std::uint64_t n_footpaths_visited_{0ULL};
  std::uint64_t n_routes_visited_{0ULL};
};

struct tripbased_state {
public:
  void add(trip_segment const& new_segment) {
    trip_segments_.push_back(new_segment);
  }

  size_t segments_size() { return trip_segments_.size(); }

  void reset() { trip_segments_.resize(0); }

  void add_best(best_on_target const& b) {
    for (auto best : best_) {
      if (b.dominates(best)) {
        best.day_ = day_idx_t::invalid();
      }
    }
  }

  std::vector<best_on_target> best_;
  std::vector<trip_segment> trip_segments_;
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
    for (auto dest : is_dest_) {
      dest.resize(0);
      // utl::fill(dest,
      //           std::make_pair(location_idx_t::invalid(), duration_t{64U}));
    }
    for (auto dest_line : is_dest_line_) {
      dest_line.resize(0);
      // utl::fill(dest_line, std::make_pair(0U, location_idx_t ::invalid()));
    }
    find_target_lines(is_dest);
  };

  trip_segment& get_segment(size_t i) {
    if (i < state_.trip_segments_.size()) {
      return state_.trip_segments_[i];
    }
    std::cout << "Trying to get a segment outside of range: "
              << state_.trip_segments_.size() << ", but searched for " << i
              << std::endl;
    return state_.trip_segments_[state_.segments_size() - 1];
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
    // TODO: check if 16 bit value types can fit the whole absolute time
    // abs_q_mam_ = minutes_after_midnight_t{day.v_ * 1440U} + mam;
    abs_q_mam_ = tt_.to_unixtime(day, mam);
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
  void execute(unixtime_t const start_time,
               std::uint8_t const max_transfers,
               unixtime_t const worst_time_at_dest,
               pareto_set<journey>& results) {
    auto abs_max_time = tt_.to_unixtime(q_day_ + day_idx_t{1U}, q_mam_);
    auto abs_min_time = tt_.to_unixtime(q_day_ + day_idx_t{1U}, q_mam_);
    // Iterate through segments
    for (auto seg_idx = 0U; seg_idx < state_.segments_size(); seg_idx++) {
      auto const curr_segment = get_segment(seg_idx);
      auto const seg_route_idx = tt_.transport_route_[curr_segment.t_idx()];
      auto const seg_stops = tt_.route_location_seq_[seg_route_idx];
      // Day difference between segment's first stop and first stop of route
      // Needed due to on_query_day property is stored relative to first
      // segment's stop, not relative to the first route's station
      auto const [day_on_r_start, mam_at_r_start] =
          tt_.event_mam(curr_segment.t_idx(), 0U, event_type::kDep);
      auto const delta_on_seg_start = tt_.event_mam(
          curr_segment.t_idx(), curr_segment.from(), event_type::kArr);
      auto const day_on_seg_start = delta_on_seg_start.days();
      auto const mam_on_seg_start = delta_on_seg_start.mam();
      auto const day_diff = day_on_seg_start - day_on_r_start;
      // Absolute time on segment start
      auto const abs_time_on_seg_start =
          tt_.to_unixtime(q_day_ + curr_segment.on_query_day() - day_diff,
                          minutes_after_midnight_t{mam_on_seg_start});

      // Iterate through destination lines
      for (auto dest_line_idx = 0U; dest_line_idx < is_dest_line_.size();
           dest_line_idx++) {
        // Skip if this is another line than currently segment's line
        if (seg_route_idx.v_ != dest_line_idx) {
          continue;
        }
        // Skip if this line doesn't visit any target
        if (is_dest_line_[dest_line_idx].empty()) {
          continue;
        }

        // Iterate through dest_line stops and check every stop's arrival times
        // Case when line can visit multiple targets considered
        auto [dest_stop_idx, dest_l_idx] = is_dest_line_[dest_line_idx][0];
        auto dest_footpath = duration_t{64U};
        auto target_stop_idx = stop_idx_t{0U};
        for (auto [d_stop_idx, d_l_idx] : is_dest_line_[dest_line_idx]) {
          target_stop_idx =
              contains_target(curr_segment.from() + 1U, seg_stops.size() - 1U,
                              d_stop_idx, route_idx_t{dest_line_idx});
          // Skip if segment's line not visiting target
          // Or if it's stop range doesn't contain target
          if (target_stop_idx == 512U) {
            continue;
          }
          // Calculate footpath from segment's stop to target dest_l_idx
          auto r_stop_to_target = stop{seg_stops[d_stop_idx]};
          auto footpaths_from =
              tt_.locations_.get(r_stop_to_target.location_idx())
                  .footpaths_out_;
          for (auto footpath : footpaths_from) {
            if (footpath.target() == dest_l_idx) {
              dest_footpath = footpath.duration();
            }
          }
          // Case when no footpath exists ==> route visits target directly
          if (dest_footpath == duration_t{64U}) {
            dest_footpath = duration_t{0U};
          }

          // Check if it improves arrival time
          auto const delta_on_target = tt_.event_mam(
              curr_segment.t_idx(), target_stop_idx, event_type::kArr);
          // Calculating absolute arrival time
          auto const abs_time_target = abs_time_on_seg_start +
                                       (delta_on_target.as_duration() -
                                        delta_on_seg_start.as_duration()) +
                                       dest_footpath;
          // Check if arrival time better than currently known
          if (abs_time_target >= abs_min_time) {
            continue;
          }
          // Check if arrival time later than 24 hours after query start
          if (abs_time_target > abs_max_time) {
            continue;
          }
          // Update min known time at destination
          abs_min_time = abs_time_target;
          // Add journey
          // Note: if there is a footpath from a station with target_stop_idx to
          // actual target station then it must be considered in reconstruction
          auto const [optimal, it, dominated_by] = results.add(journey{
              .legs_ = {},
              .start_time_ = start_time,
              .dest_time_ = abs_time_target,
              .dest_ = location_idx_t{seg_stops[target_stop_idx]},
              .transfers_ =
                  static_cast<std::uint8_t>(curr_segment.n_transfers())});
          if (optimal) {
            auto new_best = best_on_target{
                .segment_idx_ = seg_idx,
                .day_ = q_day_ + curr_segment.on_query_day() - day_diff,
                .stop_idx_ = target_stop_idx,
                .start_time_ = start_time,
                .abs_time_on_target_ = abs_time_target,
                .n_transfers_ =
                    static_cast<std::uint8_t>(curr_segment.n_transfers())};
            state_.add_best(new_best);
          }
        }
      }

      // Transfers section
      // Absolute time of the segment's next station from start
      auto const delta_on_seg_stop_next = tt_.event_mam(
          curr_segment.t_idx(), curr_segment.from() + 1U, event_type::kArr);
      auto const abs_time_seq_stop_next =
          abs_time_on_seg_start + (delta_on_seg_stop_next.as_duration() -
                                   delta_on_seg_start.as_duration());
      // Check if arrival is better than known on target
      // And 24 Hours check
      if (abs_time_on_seg_start >= abs_min_time ||
          abs_time_on_seg_start > abs_max_time) {
        continue;
      }
      // Iterating through segment's stops
      for (auto seg_stop_idx = curr_segment.from() + 1U;
           seg_stop_idx <= curr_segment.to(); seg_stop_idx++) {
        // Absolute time on the segment's stop
        auto const delta_on_seg_stop =
            tt_.event_mam(curr_segment.t_idx(), seg_stop_idx, event_type::kArr);
        auto const abs_time_seq_stop =
            abs_time_on_seg_start + (delta_on_seg_stop.as_duration() -
                                     delta_on_seg_start.as_duration());
        // Difference between first and current segment's stop in days
        auto const seg_day_diff_on_stop =
            delta_on_seg_stop.days() / 1440U - day_on_seg_start / 1440U;
        if (seg_day_diff_on_stop >= 1 && !curr_segment.on_query_day()) {
          std::cout << "Segment takes longer than one day to his next stop "
                       "and segment starts on next query day"
                    << std::endl;
          break;
        }
        // Iterate through transfers on this stop
        // Note: transfers are not possible from first stop therefore skip it
        for (auto transfer :
             transfers_.at(curr_segment.t_idx().v_, seg_stop_idx)) {
          // Take n_transfers from current segment to be able to count it
          auto const n_transfers = curr_segment.n_transfers() + 1U;
          if (n_transfers > max_transfers) {
            std::cout << "Max n_transfers reached" << std::endl;
            break;
          }
          auto const to_transport_idx = transfer.to();
          auto const to_stop_idx = transfer.stop_idx();
          // Check 24 Hours rule
          auto const delta_on_transfer_stop =
              tt_.event_mam(to_transport_idx, to_stop_idx, event_type::kDep);
          auto const abs_transfer_day =
              (q_day_ + curr_segment.on_query_day() + transfer.day_change());
          auto const abs_time_on_transfer_stop = tt_.to_unixtime(
              (abs_transfer_day - delta_on_transfer_stop.days()),
              delta_on_transfer_stop.as_duration());
          if (abs_time_on_transfer_stop > abs_max_time) {
            std::cout << "Transfer overflows 24 hours rule" << std::endl;
            continue;
          }

          enqueue(tt_.transport_route_[to_transport_idx], to_transport_idx,
                  to_stop_idx, seg_idx, seg_stop_idx, n_transfers,
                  !(abs_transfer_day == q_day_));
        }
      }
    }
    // TODO: possibly break or return something after run
  }

  // Provide journeys in right format
  void reconstruct(query const& q, journey& j) {
    reconstruct_journey(tt_, q, q_day_, state_, is_dest_, j);
  }

private:
  // stop_index - stop in route sequence
  // day_idx = {0, 1} for query day and day + 1
  void enqueue(route_idx_t r_idx,
               transport_idx_t t_idx,
               stop_idx_t stop_index,
               size_t prev_idx,
               stop_idx_t prev_stop_idx,
               uint8_t n_transfers,
               size_t day_idx) {
    if (stop_index < first_locs_[day_idx][t_idx.v_]) {
      auto const latest_loc = first_locs_[day_idx][t_idx.v_];
      auto new_trip_segment =
          trip_segment(t_idx, stop_index, latest_loc, prev_idx, prev_stop_idx,
                       n_transfers, !day_idx);
      state_.add(new_trip_segment);

      auto const [day_at_stop, mam_at_stop] =
          tt_.event_mam(t_idx, stop_index, event_type::kDep);
      auto const transport_range = tt_.route_transport_ranges_[r_idx];
      // Iterate through all trips of the same line
      for (auto const transport : transport_range) {
        // Skip the trip the function was called with
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
        // TODO: check correctness
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

        // Found a station that is the start station or start reachable with
        // footpath.
        // Iterate through trip of this route and find earliest
        auto const& transport_range = tt_.route_transport_ranges_[r_idx];
        auto ed_transport_idx = transport_idx_t::invalid();
        // Absolute max time as query time + 24 hours
        // Will not allow trips that start later than 24 hours after query start
        auto abs_ed_time = tt_.to_unixtime(q_day_ + day_idx_t{1U}, q_mam_);
        auto trip_on_the_next_day = 0U;
        for (auto const transport : transport_range) {
          auto const& transport_bitset =
              tt_.bitfields_[tt_.transport_traffic_days_[transport]];
          auto const [day_at_stop, mam_at_stop] =
              tt_.event_mam(transport, i, event_type::kDep);
          // Check trips on the current day and day + 1
          // If footpath makes day change then only look at day + 1
          for (auto day_offset = day_idx_t{day_change}; day_offset < 2;
               day_offset++) {
            auto const is_active = transport_bitset.test(
                to_idx(q_day_ + day_offset - day_at_stop));
            auto const abs_dep_time = tt_.to_unixtime(
                q_day_ + day_offset, minutes_after_midnight_t{mam_at_stop});
            if (is_active && abs_dep_time < abs_ed_time) {
              ed_transport_idx = transport;
              abs_ed_time = abs_dep_time;
              if (day_offset > 0) {
                trip_on_the_next_day = 1U;
              }
            }
          }
        }

        // If trip for this route was found
        if (ed_transport_idx != transport_idx_t::invalid()) {
          enqueue(r_idx, transport_idx_t{ed_transport_idx}, i, 0U, 0U, 0,
                  trip_on_the_next_day);
        }
      }
    }
  }

  void find_target_lines(std::vector<bool>& is_dest) {
    for (auto i = 0U; i < is_dest.size(); i++) {
      if (is_dest[i]) {
        // Destination is reachable from itself with footpath 0
        is_dest_[i].emplace_back(std::make_pair(location_idx_t{i}, 0U));
        // For initial destination itself
        auto routes_from_loc = tt_.location_routes_[location_idx_t{i}];
        for (auto route_from_loc : routes_from_loc) {
          auto stop_idx = 0U;
          auto const route_stops = tt_.route_location_seq_[route_from_loc];
          // Find index of the location relative to route
          // Multiple same stations counted - circled routes
          for (auto route_loc : route_stops) {
            if (route_loc == i) {
              stop_idx = route_loc;
              // Destination line visits target from is_dest directly
              is_dest_line_[route_from_loc.v_].emplace_back(
                  std::make_pair(stop_idx, location_idx_t{i}));
            }
          }
        }

        // Footpaths_in are footpath from other stations to this
        // For stations that reach target station using footpath
        auto const footpaths_in =
            tt_.locations_.get(location_idx_t{i}).footpaths_in_;
        auto duration = duration_t{0U};
        auto from_loc_idx = location_idx_t{0U};
        // TODO: meta stations? start_times.cc
        for (auto footpath : footpaths_in) {
          duration = footpath.duration();
          from_loc_idx = footpath.target();
          is_dest_[from_loc_idx.v_].emplace_back(
              std::make_pair(location_idx_t{i}, duration));
          // Iterate through routes of location that reach tgt with footpath
          routes_from_loc = tt_.location_routes_[from_loc_idx];
          for (auto route_from_loc : routes_from_loc) {
            auto stop_idx = 0U;
            auto const route_stops = tt_.route_location_seq_[route_from_loc];
            // Find index of the location relative to route
            // Multiple same stations counted - circled routes
            for (auto route_loc : route_stops) {
              if (route_loc == from_loc_idx.v_) {
                stop_idx = route_loc;
                is_dest_line_[route_from_loc.v_].emplace_back(
                    std::make_pair(stop_idx, location_idx_t{i}));
              }
            }
          }
        }
      }
    }
  }

  // Checks if a range of route stops contains target station
  // Multiple stations in one route considered
  stop_idx_t contains_target(stop_idx_t from,
                             stop_idx_t to,
                             stop_idx_t target,
                             route_idx_t route_idx) {
    auto const route_stops = tt_.route_location_seq_[route_idx];
    for (auto stop_idx = from; stop_idx <= to; stop_idx++) {
      if (route_stops[stop_idx] == route_stops[target]) {
        return stop_idx;
      }
    }
    return 512U;
  }

  timetable const& tt_;
  tripbased_state& state_;
  // location_idx - list of location_idx_t of targets reachable from it and
  // footpath. Duration = 0 for the target itself
  std::vector<std::vector<std::pair<location_idx_t, duration_t>>> is_dest_;
  // route_idx - list of his stop indexes and locations. Each pair says which
  // target location_idx is reachable using stop idx of route.
  std::vector<std::vector<std::pair<stop_idx_t, location_idx_t>>> is_dest_line_;
  const nvec<std::uint32_t, transfer, 2>& transfers_;
  // R(t) - first known index of the trip's earliest found station
  std::vector<std::vector<stop_idx_t>> first_locs_;
  unixtime_t abs_q_mam_;
  day_idx_t q_day_;
  minutes_after_midnight_t q_mam_;
  uint8_t n_transfers_;
};

}  // namespace nigiri::tripbased
