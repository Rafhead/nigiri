#include "nigiri/tripbased/preprocessing.h"
#include "nigiri/common/it_range.h"
namespace nigiri::tripbased {

nvec<std::uint32_t, transfer, 2> compute_transfers(timetable& tt) {
  // Transfers
  nvec<std::uint32_t, transfer, 2> transfers;
  bitfield transport_from_bf;
  bitfield_idx_t transport_from_bf_idx;
  bitfield_idx_t transport_to_bf_idx;
  bool day_change;
  location_idx_t location_to_idx;
  duration_t footpath_duration;
  minutes_after_midnight_t transport_from_mam;
  minutes_after_midnight_t transport_to_mam;

  auto bitfields = hash_map<bitfield, bitfield_idx_t>{};
  for (auto const [i, bf] : utl::enumerate(tt.bitfields_)) {
    bitfields.emplace(bf, bitfield_idx_t{i});
  }

  // Get bitfield's index or create a bitfield and return the index to new bf
  auto const get_bitfield_idx = [&](bitfield const b) {
    return utl::get_or_create(bitfields, b, [&]() {
      auto const idx = bitfield_idx_t{tt.bitfields_.size()};
      tt.bitfields_.emplace_back(b);
      return idx;
    });
  };

  // Iterate through all routes
  for (auto route_from_idx_ = 0U; route_from_idx_ < tt.n_routes();
       ++route_from_idx_) {
    // Iterate through all trips for this route
    for (auto transport_idx :
         tt.route_transport_ranges_[route_idx_t{route_from_idx_}]) {
      // To store to bitsets in timetable for times on stations
      vecvec<location_idx_t,
             std::pair<minutes_after_midnight_t, bitfield_idx_t>>
          arrival_times_;
      vecvec<location_idx_t,
             std::pair<minutes_after_midnight_t, bitfield_idx_t>>
          ea_change_times_;

      // Iterate through all stops of this route in opposite direction
      auto const location_seq =
          tt.route_location_seq_[route_idx_t{route_from_idx_}];
      for (auto stop_idx = location_seq.size() - 2U; stop_idx != 0U;
           --stop_idx) {
        auto const location_idx = location_seq[stop_idx];
        update_time();
        // Iterate through all stops that are reachable by footpath
        // +1 for the station itself
        // Station itself is not included in footpaths from them
        // TODO: Possibly check meta stations too
        auto const location_footpaths_from =
            tt.locations_.footpaths_out_[location_idx_t{location_idx}];
        auto const footpath_from_idx = 0U;
        // Iterate through all footpaths from
        while (footpath_from_idx != location_footpaths_from.size() + 1) {
          // Pick up the station the footpath goes to
          if (footpath_from_idx == location_footpaths_from.size()) {
            location_to_idx = location_idx_t{location_idx};
            footpath_duration = duration_t{0U};
          } else {
            location_to_idx =
                location_footpaths_from.at(footpath_from_idx).target_;
            footpath_duration =
                location_footpaths_from.at(footpath_from_idx).duration_;
          }
          day_change = false;
          // Get mam for the trip from and add footpath
          transport_from_mam =
              tt.event_mam(route_idx_t{route_from_idx_}, transport_idx,
                           location_idx, event_type::kArr);
          auto const mam_at_stop_from = transport_from_mam + footpath_duration;
          // Check if footpath makes day change
          if ((transport_from_mam.count() / 1440) !=
              ((mam_at_stop_from).count()) / 1440) {
            day_change = true;
          }
          update_time();
          update_time();
          // Iterate through all routes on current stop reachable by
          // footpath
          for (auto route_to_idx : tt.location_routes_[location_to_idx]) {
            // Skip route if it is his last station
            if (tt.route_location_seq_
                    [route_idx_t{route_to_idx}]
                    [tt.route_location_seq_[route_idx_t{route_to_idx}].size() -
                     1] == location_to_idx.v_) {
              continue;
            }
            // Set the current trips bitfields to its actual value and shift
            // TODO: bitfields must be set by this way?
            transport_from_bf.set(
                tt.bitfields_[tt.transport_traffic_days_[transport_idx]]
                    .to_string());
            transport_from_bf.operator>>=(
                std::floor((transport_from_mam.count() / 1440)));
            day_change = false;
            // Find the earliest trip for line L
            // Get a look on event times for current route
            auto const event_times = tt.event_times_at_stop(
                route_to_idx, location_idx, event_type::kDep);
            // Set iterator on that to iterate through times
            auto const ev_time_range = it_range{
                std::lower_bound(event_times.begin(), event_times.end(),
                                 mam_at_stop_from,
                                 [&](auto&& a, auto&& b) { return a <= b; }),
                event_times.end()};
            // TODO: iterator belongs to the earliest time on
            // station based on the arrival time?
            auto ea_time_it = begin(ev_time_range);
            // ea transport offset
            auto ea_transport_offset =
                static_cast<std::size_t>(&*ea_time_it - event_times.data());
            // ea transport itself
            auto const transport =
                tt.route_transport_ranges_[route_to_idx][ea_transport_offset];
            // Event on the station itself
            auto const ev_on_stop_to = *ea_time_it;
            // Check if footpath makes day change
            if (mam_at_stop_from.count() / 1440 !=
                transport_from_mam.count() / 1440) {
              day_change = true;
            }
            // Check if ea transport found
            if (ea_time_it == end(ev_time_range)) {
              ea_time_it = begin(ev_time_range);
              day_change = true;
            }
            auto ea_time = ea_time_it;
            // while...
          }
        }
      }
    }
  }

  return transfers;
}

}  // namespace nigiri::tripbased
