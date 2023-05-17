#include "nigiri/tripbased/tripbased.h"
#include "utl/enumerate.h"
#include "utl/get_or_create.h"

namespace nigiri::tripbased {

tripbased::tripbased(timetable& tt, routing::query q)
    : tt_{tt}, q_{std::move(q)} {}

void tripbased::extract_transfers() {
  auto bitfields = hash_map<bitfield, bitfield_idx_t>{};
  for (auto const [i, bf] : utl::enumerate(tt_.bitfields_)) {
    bitfields.emplace(bf, bitfield_idx_t{i});
  }
  auto const get_bitfield_idx = [&](bitfield const b) {
    return utl::get_or_create(bitfields, b, [&]() {
      auto const idx = bitfield_idx_t{tt_.bitfields_.size()};
      tt_.bitfields_.emplace_back(b);
      return idx;
    });
  };

  bitfield bf;
  auto const bf_idx = get_bitfield_idx(bf);
  
  bitfield b;

  // Iterate through all routes
  for (auto route_idx = 0U; route_idx < tt_.n_routes(); ++route_idx) {
    // Iterate through all trips for this route
    for (auto transport_idx :
         tt_.route_transport_ranges_[route_idx_t{route_idx}]) {
      // Get the bitfield index of current trip
      auto const transport_bitfield_idx =
          tt_.transport_traffic_days_[transport_idx];
      // Iterate through all stops of this route (and therefore trips)
      auto const location_seq = tt_.route_location_seq_[route_idx_t{route_idx}];
      for (auto j = 0U; j != location_seq.size() - 1; ++j) {
        auto const stop_idx = location_seq.size() - j - 1;
        auto const location_idx = location_seq[stop_idx];

        // Iterate through all stops that are reachable by footpath
        // TODO: Ask if the station itself is included in footpaths from them
        // Answer: NO
        // TODO: Possibly check meta stations too
        for (auto location_footpath :
             tt_.locations_.footpaths_out_[location_idx_t{location_idx}]) {
          // Iterate through all routes on current stop reachable by footpath
          for (auto route_id :
               tt_.location_routes_[location_footpath.target_]) {
            // Skip route if it is his last station
            if (tt_.route_location_seq_[route_idx_t{
                    route_idx}][tt_.route_location_seq_[route_idx_t{route_idx}]
                                    .size() -
                                1]) {
              continue;
            }
            // Set the current trips bitfields to his actual value
            b.set(tt_.bitfields_[transport_bitfield_idx].to_string());
          }
        }
      }
    }
  }
}

}  // namespace nigiri::tripbased
