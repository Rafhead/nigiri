#pragma once

#include "nigiri/routing/journey.h"
#include "nigiri/routing/query.h"
#include "nigiri/timetable.h"
#include "nigiri/tripbased/tripbased_state.h"
#include "nigiri/types.h"

namespace nigiri {
struct timetable;
}  // namespace nigiri

namespace nigiri::tripbased {

using query = nigiri::routing::query;
using journey = nigiri::routing::journey;
struct tripbased_state;

void reconstruct_journey(
    timetable const& tt,
    query const&,
    day_idx_t q_day,
    tripbased_state const& state,
    std::vector<std::vector<std::pair<location_idx_t, duration_t>>> const&
        is_dest,
    journey& j) {
  // Iterate through best and find valid
  for (auto best : state.best_) {
    if (best.day_ == day_idx_t::invalid() ||
        best.abs_time_on_target_ != j.dest_time_ ||
        best.n_transfers_ != j.transfers_ ||
        best.start_time_ != j.start_time_) {
      std::cout << "Best candidate not found for a journey j" << std::endl;
      continue;
    }
    auto seg = state.trip_segments_[best.segment_idx_];
    auto seg_route_idx = tt.transport_route_[seg.t_idx()];
    auto seg_stops = tt.route_location_seq_[seg_route_idx];
    auto seg_l_idx = location_idx_t{seg_stops[best.stop_idx_]};
    auto seg_from_l_idx = location_idx_t{seg_stops[seg.from()]};
    // transport day as: day + on query day - days in trip since from stop
    auto seg_transport = nigiri::transport{
        .t_idx_ = seg.t_idx(),
        .day_ = q_day + seg.on_query_day() -
                tt.event_mam(seg.t_idx(), seg.from(), event_type::kDep).days()};
    auto seg_dep_time_from =
        tt.event_time(seg_transport, seg.from(), event_type::kDep);

    // Ensure footpath to target isn't needed
    // Check all targets that are reachable from the station where segments
    // reaches one of the target stations and find minimal possible distance to
    // target to minimize the arrival time.
    auto [target_l_idx, footpath_dur] = is_dest[seg_l_idx.v_][0];
    for (auto [dest_l_idx, footpath_to_dest] : is_dest[seg_l_idx.v_]) {
      if (footpath_to_dest < footpath_dur &&
          dest_l_idx != location_idx_t::invalid()) {
        target_l_idx = dest_l_idx;
        footpath_dur = footpath_to_dest;
      }
    }

    // Segment doesn't visit target directly
    if (footpath_dur != duration_t{0U}) {
      auto const footpath_to_target = footpath(target_l_idx, footpath_dur);
      // Add firstly footpath to target...
      j.add(journey::leg{direction::kForward, seg_l_idx, target_l_idx,
                         best.abs_time_on_target_ - footpath_dur,
                         best.abs_time_on_target_, footpath_to_target});
      // ... then the segment itself
      j.add(journey::leg{
          direction::kForward, seg_from_l_idx, seg_l_idx, seg_dep_time_from,
          best.abs_time_on_target_ - footpath_dur,
          journey::run_enter_exit(
              {.t_ = seg_transport,
               .stop_range_ = interval<stop_idx_t>{seg.from(), seg.to()}},
              seg.from(), seg.to())});
    } else {
      // Else segment visits one of the targets directly --> add segment
      j.add(journey::leg{
          direction::kForward, seg_from_l_idx, seg_l_idx, seg_dep_time_from,
          best.abs_time_on_target_,
          journey::run_enter_exit(
              {.t_ = seg_transport,
               .stop_range_ = interval<stop_idx_t>{seg.from(), seg.to()}},
              seg.from(), seg.to())});
    }

    // Recursively construct legs by looking on prev segment_idx
    // tt + transport struct gives unix time
    // Each iteration adds footpath as:
    // - duration to wait for next trip in journey at the same station
    // - or duration to change the stations
    while (seg.prev_stop_idx() != 0U) {
      auto prev_seg = state.trip_segments_[seg.prev_idx()];
      auto prev_seg_route_idx = tt.transport_route_[prev_seg.t_idx()];
      auto prev_seg_stops = tt.route_location_seq_[prev_seg_route_idx];
      auto prev_to_l_idx = location_idx_t{prev_seg_stops[seg.prev_stop_idx()]};
      auto prev_from_l_idx = location_idx_t{prev_seg_stops[seg.from()]};
      auto prev_seg_transport = nigiri::transport{
          .t_idx_ = seg.t_idx(),
          .day_ =
              q_day + prev_seg.on_query_day() -
              tt.event_mam(prev_seg.t_idx(), prev_seg.from(), event_type::kDep)
                  .days()};
      // Arrival with index of stop where next segment was reached
      auto prev_seg_arr_time = tt.event_time(
          prev_seg_transport, seg.prev_stop_idx(), event_type::kArr);
      /*auto prev_seg_dep_time =
          tt.event_time(prev_seg_transport, prev_seg.from(),
         event_type::kDep);*/

      // Adding footpath
      auto const transfer_footpath = footpath(
          seg_from_l_idx, duration_t{prev_seg_arr_time - seg_dep_time_from});
      j.add(journey::leg{direction::kForward, prev_to_l_idx, seg_l_idx,
                         prev_seg_arr_time, seg_dep_time_from,
                         transfer_footpath});
      j.add(journey::leg{
          direction::kForward, prev_from_l_idx, prev_to_l_idx,
          prev_seg_arr_time, seg_dep_time_from,
          journey::run_enter_exit(
              {.t_ = prev_seg_transport,
               .stop_range_ =
                   interval<stop_idx_t>{prev_seg.from(), prev_seg.to()}},
              prev_seg.from(), prev_seg.to())});
      seg = prev_seg;
      seg_route_idx = tt.transport_route_[seg.t_idx()];
      seg_stops = tt.route_location_seq_[seg_route_idx];
      seg_l_idx = location_idx_t{seg_stops[seg.from()]};
      seg_transport = prev_seg_transport;
      seg_dep_time_from =
          tt.event_time(seg_transport, seg.from(), event_type::kDep);
    }

    // TODO: Ensure start footpath added properly if needed

    std::reverse(begin(j.legs_), end(j.legs_));
  }
}
}  // namespace nigiri::tripbased