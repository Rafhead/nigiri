#include "nigiri/tripbased/reconstruct.h"

namespace nigiri::tripbased {

void reconstruct_journey(
    timetable const& tt,
    query const& q,
    tripbased_state const& state,
    std::vector<std::pair<location_idx_t, duration_t>> is_dest,
    journey& j) {
  // Iterate through best and find valid
  for (auto best : state.best_) {
    if (best.abs_time_on_target_ != j.dest_time_) {
      continue;
    }
    auto const seg = state.trip_segments_[best.segment_idx_];
    auto const seg_route_idx = tt.transport_route_[seg.t_idx()];
    auto const seg_stops = tt.route_location_seq_[seg_route_idx];
    auto const seg_loc_idx = location_idx_t{seg_stops[best.stop_idx_]};
    // Ensure footpath to target isn't needed
    auto const [target_l_idx, footpath_dur] = is_dest[seg_loc_idx.v_];
    // Segment doesn't visit target directly
    if (footpath_dur != duration_t{0U}) {
      auto const footpath_to_target = footpath(target_l_idx, footpath_dur);
      j.add(journey::leg{direction::kForward, seg_loc_idx, target_l_idx,
                         best.abs_time_on_target_ - footpath_dur,
                         best.abs_time_on_target_, footpath_to_target});
    }  // Else segment visits one of the targets directly

    // Recursively construct legs by looking on prev segment_idx
    // tt + transport struct gives unix time

    // Ensure start footpath added properly if needed

    std::reverse(begin(j.legs_), end(j.legs_));
  }
};

}  // namespace nigiri::tripbased