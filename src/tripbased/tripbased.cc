#include "nigiri/tripbased/tripbased.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

namespace nigiri::tripbased {

tripbased::tripbased(timetable& tt, routing::query q)
    : tt_{tt}, q_{std::move(q)} {}

void init_arrival_times() {}

bool tripbased::update_time() { return true; }

void tripbased::extract_transfers() {
  auto bitfields = hash_map<bitfield, bitfield_idx_t>{};
  for (auto const [i, bf] : utl::enumerate(tt_.bitfields_)) {
    bitfields.emplace(bf, bitfield_idx_t{i});
  }

  // Get bitfield's index or create a bitfield and return the index to new bf
  auto const get_bitfield_idx = [&](bitfield const b) {
    return utl::get_or_create(bitfields, b, [&]() {
      auto const idx = bitfield_idx_t{tt_.bitfields_.size()};
      tt_.bitfields_.emplace_back(b);
      return idx;
    });
  };

  bitfield transport_from_bf;
  bitfield_idx_t transport_from_bf_idx;
  bitfield_idx_t transport_to_bf_idx;
  bool day_change;
  location_idx_t location_to_idx;
  duration_t footpath_duration_t;
  minutes_after_midnight_t transport_from_mam_t;
  minutes_after_midnight_t transport_to_mam_t;
  auto const bf_idx = get_bitfield_idx(transport_from_bf);

  // Iterate through all routes
  for (auto route_from_idx_ = 0U; route_from_idx_ < tt_.n_routes();
       ++route_from_idx_) {
    // Iterate through all trips for this route
    for (auto transport_idx :
         tt_.route_transport_ranges_[route_idx_t{route_from_idx_}]) {

      // Iterate through all stops of this route in opposite direction
      auto const location_seq =
          tt_.route_location_seq_[route_idx_t{route_from_idx_}];
      for (auto stop_idx = location_seq.size() - 2U; stop_idx != 0U;
           --stop_idx) {
        auto const location_idx = location_seq[stop_idx];
        // TODO: Call update time
        update_time();
        // Iterate through all stops that are reachable by footpath
        // +1 for the station itself
        // TODO: Ask if the station itself is included in footpaths from them
        // Answer: NO
        // TODO: Possibly check meta stations too
        auto const location_footpaths_from =
            tt_.locations_.footpaths_out_[location_idx_t{location_idx}];
        auto const footpath_from_idx = 0U;
        // Iterate through all footpaths from
        while (footpath_from_idx != location_footpaths_from.size() + 1) {
          // Pick up the station the footpath goes to
          if (footpath_from_idx == location_footpaths_from.size()) {
            location_to_idx = location_idx_t{location_idx};
            footpath_duration_t = duration_t{0U};
          } else {
            location_to_idx =
                location_footpaths_from.at(footpath_from_idx).target_;
            footpath_duration_t =
                location_footpaths_from.at(footpath_from_idx).duration_;
          }
          day_change = false;
          // Get mam for the trip from
          transport_from_mam_t =
              tt_.event_mam(route_idx_t{route_from_idx_}, transport_idx,
                            location_idx, event_type::kArr);
          // Check if footpath makes day change
          if (std::floor((transport_from_mam_t.count() / 1440)) !=
              std::floor(
                  ((transport_from_mam_t + footpath_duration_t).count()) /
                  1440)) {
            day_change = true;
          }
          // TODO: Call UPD_TIME
          update_time();
          update_time();
          // Iterate through all routes on current stop reachable by
          // footpath
          for (auto route_to_idx : tt_.location_routes_[location_to_idx]) {
            // Skip route if it is his last station
            if (tt_.route_location_seq_
                    [route_idx_t{route_to_idx}]
                    [tt_.route_location_seq_[route_idx_t{route_to_idx}].size() -
                     1] == location_to_idx.v_) {
              continue;
            }
            // Set the current trips bitfields to its actual value and shift
            transport_from_bf.set(
                tt_.bitfields_[tt_.transport_traffic_days_[transport_idx]]
                    .to_string());
            transport_from_bf.operator>>=(
                std::floor((transport_from_mam_t.count() / 1440)));
            day_change = false;
            // Find the earliest trip for line L
            auto const route_stop_time_interval =
                tt_.route_stop_time_ranges_[route_idx_t{route_to_idx}];
            // ...
          }
        }
      }
    }
  }
}

}  // namespace nigiri::tripbased
