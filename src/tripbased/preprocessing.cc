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
  // minutes_after_midnight_t transport_to_mam;

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
    for (auto transport_from_idx :
         tt.route_transport_ranges_[route_idx_t{route_from_idx_}]) {
      // To store to bitsets in timetable for times on stations
      vecvec<location_idx_t, std::pair<minutes_after_midnight_t, bitfield>>
          arrival_times;
      vecvec<location_idx_t, std::pair<minutes_after_midnight_t, bitfield>>
          ea_change_times;
      vector<vector<transfer>> trip_transfers;
      // Iterate through all stops of this route in opposite direction
      // skipping the first station
      auto const location_from_seq =
          tt.route_location_seq_[route_idx_t{route_from_idx_}];
      for (auto stop_from_idx = location_from_seq.size() - 1U;
           stop_from_idx != 0U; --stop_from_idx) {
        // To store transfers for a trip and for his unique stop
        vector<transfer> transfers_from = vector<transfer>();
        auto const location_from_idx = location_from_seq[stop_from_idx];
        transport_from_mam =
            tt.event_mam(route_idx_t{route_from_idx_}, transport_from_idx,
                         location_from_idx, event_type::kArr);
        transport_from_bf_idx = tt.transport_traffic_days_[transport_from_idx];
        update_time(arrival_times, location_idx_t{location_from_idx},
                    transport_from_mam % 1440,
                    tt.bitfields_[transport_from_bf_idx], false);
        // Iterate through all stops that are reachable by footpath
        // +1 for the station itself
        // Station itself is not included in footpaths from them
        // TODO: Possibly check meta stations too
        auto const location_footpaths_from =
            tt.locations_.footpaths_out_[location_idx_t{location_from_idx}];
        auto const footpath_from_idx = 0U;
        // Iterate through all footpaths from
        while (footpath_from_idx != location_footpaths_from.size() + 1) {
          // Pick up the station the footpath goes to
          if (footpath_from_idx == location_footpaths_from.size()) {
            location_to_idx = location_idx_t{location_from_idx};
            footpath_duration = duration_t{0U};
          } else {
            location_to_idx =
                location_footpaths_from.at(footpath_from_idx).target_;
            footpath_duration =
                location_footpaths_from.at(footpath_from_idx).duration_;
          }
          day_change = false;
          // Get mam for the trip from and add footpath
          auto const mam_at_stop_from = transport_from_mam + footpath_duration;
          // Check if footpath makes day change
          if ((transport_from_mam.count() / 1440) !=
              ((mam_at_stop_from).count()) / 1440) {
            day_change = true;
          }
          update_time(arrival_times, location_idx_t{location_to_idx},
                      mam_at_stop_from % 1440,
                      tt.bitfields_[transport_from_bf_idx], day_change);
          update_time(ea_change_times, location_idx_t{location_to_idx},
                      mam_at_stop_from % 1440,
                      tt.bitfields_[transport_from_bf_idx], day_change);
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
            // Get position of stop to in route sequence
            auto const route_to_stop_seq =
                tt.route_location_seq_[route_idx_t{route_to_idx}];
            auto const location_to_pos_it =
                std::find(route_to_stop_seq.begin(), route_to_stop_seq.end(),
                          location_to_idx.v_);
            // Set the current trips bitfields to its actual value and shift
            // TODO: bitfields must be set by this way?
            // b[t]' <-- ...
            transport_from_bf.set(
                tt.bitfields_[transport_from_bf_idx].to_string());
            auto transport_from_bf_cpy = bitset<512>(transport_from_bf);
            transport_from_bf_cpy >>= (transport_from_mam.count() / 1440);
            day_change = false;
            // Find the earliest trip for line L
            // Get a look on event times for current route
            auto const stop_to_idx =
                location_to_pos_it - route_to_stop_seq.begin();
            auto const location_to_event_times = tt.event_times_at_stop(
                route_to_idx, stop_to_idx, event_type::kDep);
            // Set iterator on that to iterate through times
            auto const ev_time_range =
                it_range{std::lower_bound(
                             location_to_event_times.begin(),
                             location_to_event_times.end(), mam_at_stop_from,
                             [&](auto&& a, auto&& b) { return a <= b; }),
                         location_to_event_times.end()};
            // TODO: iterator belongs to the earliest time on
            // station based on the arrival time?
            auto ea_time_it = begin(ev_time_range);
            // Check if footpath makes day change
            if (mam_at_stop_from.count() / 1440 !=
                transport_from_mam.count() / 1440) {
              day_change = true;
            }
            // Check if ea transport found
            if (ea_time_it == end(ev_time_range) && !day_change) {
              ea_time_it = begin(ev_time_range);
              day_change = true;
            }
            // get copy of earliest time to iterate later on it and keep
            // original for comparison
            auto ea_time = ea_time_it;
            //  while...
            // TODO check ea_time != ...
            while (transport_from_bf_cpy.any() && ea_time != ea_time_it - 1) {
              // transport to offset
              auto transport_to_offset = static_cast<std::size_t>(
                  &*ea_time - location_to_event_times.data());
              // transport to itself
              auto const transport_to_idx =
                  tt.route_transport_ranges_[route_to_idx][transport_to_offset];
              // bitset index of transport to
              transport_to_bf_idx =
                  tt.transport_traffic_days_[transport_to_idx];
              // bitset of transport to
              auto const transport_to_bf = tt.bitfields_[transport_to_bf_idx];
              // event on stop to
              auto const event_on_stop_to = *ea_time;
              // [b_u]'
              auto transport_to_bf_cpy =
                  bitset<512>(transport_to_bf.to_string());
              // [b_u] >>> ...
              transport_to_bf_cpy >>= event_on_stop_to.count() / 1440;
              transport_to_bf_cpy <<= day_change;
              // [b_tr] <-- ...
              auto transfer_bf = bitset<512>(transport_from_bf_cpy.to_string());
              transfer_bf &= transport_to_bf_cpy;
              auto keep = bitset<512>();
              // Checking for U-turn as well
              if (location_from_idx == location_to_idx.v_ &&
                  stop_from_idx != 0 &&
                  location_to_pos_it != route_to_stop_seq.end() - 2) {
                auto const transport_from_prev_mam = tt.event_mam(
                    route_idx_t{route_from_idx_}, transport_from_idx,
                    stop_from_idx - 1, event_type::kArr);
                auto const day_diff = (transport_from_mam / 1440) -
                                      (transport_from_prev_mam / 1440);
                auto const transport_from_bf_cc_m_idx =
                    get_bitfield_idx(transport_from_bf >>
                                     (transport_from_prev_mam / 1440).count());
                auto next_c = false;
                auto const change_time =
                    tt.locations_
                        .transfer_time_[location_idx_t{location_from_idx}];
                if ((transport_from_prev_mam + change_time).count() / 1440 !=
                    transport_from_prev_mam.count() / 1440) {
                  next_c = true;
                }
                // auto const route_to_n_transports = static_cast<unsigned>(
                //     tt.route_transport_ranges_[route_to_idx].size());
                auto const location_to_next_mam =
                    tt.event_mam(route_to_idx, transport_to_idx,
                                 stop_to_idx + 1, event_type::kDep);
                //(ea_time - 2 * route_to_n_transports)->count();
                if ((transport_from_prev_mam + change_time).count() % 1440 >
                    location_to_next_mam.count() % 1440) {
                  next_c = true;
                }
                //[b_u]'' <-- ...
                auto transport_to_bf_cc = transport_to_bf;
                transport_to_bf_cc >>= location_to_next_mam.count() / 1440;
                transport_to_bf_cc <<= next_c;
                //[b_tr]'
                auto transfer_bf_cpy =
                    tt.bitfields_[transport_from_bf_cc_m_idx];
                transfer_bf_cpy &= transport_to_bf_cc;
                //[b_tr] <--
                transfer_bf_cpy >>= day_diff.count();
                transfer_bf_cpy = ~transfer_bf_cpy;
                transfer_bf &= transfer_bf_cpy;
              }
              if (!transfer_bf.any()) {
                continue;
              }
              // for each stop p_k^u ...
              for (auto next_locs = location_to_pos_it;
                   next_locs < route_to_stop_seq.end(); ++next_locs) {
                // TODO: adapt day_change for the stations after transfer
                // 24 hrs max journey duration check
                if (day_change &&
                    ((ea_time + (next_locs - location_to_pos_it))->count() %
                     1440) - (transport_from_mam.count() % 1440) >
                        0) {
                  break;
                }
                // TODO: complete method call
                keep |= tt.bitfields_[update_time(
                    arrival_times, location_idx_t{*next_locs},
                    *(ea_time + (next_locs - location_to_pos_it)) % 1440,
                    transfer_bf, day_change)];
                // Iter through footpaths
                auto const next_locs_foot =
                    tt.locations_.footpaths_out_[location_idx_t{*next_locs}];
                for (size_t next_locs_to_idx = 0;
                     next_locs_to_idx != next_locs_foot.size();
                     ++next_locs_to_idx) {
                  auto const arr_to_time =
                      (*ea_time +
                       next_locs_foot.at(next_locs_to_idx).duration_) %
                      1440;
                  // TODO: complete method call
                  keep |= tt.bitfields_[update_time(
                      arrival_times,
                      next_locs_foot.at(next_locs_to_idx).target_, arr_to_time,
                      transfer_bf, day_change)];
                  keep |= tt.bitfields_[update_time(
                      ea_change_times,
                      next_locs_foot.at(next_locs_to_idx).target_, arr_to_time,
                      transfer_bf, day_change)];
                }
              }
              if (keep.any()) {
                transport_from_bf_cpy &= ~keep;
                auto const new_transfer =
                    transfer(transport_to_idx, stop_from_idx,
                             get_bitfield_idx(keep), day_change);
                transfers_from.push_back(new_transfer);
              }
              // TODO: check correct assignment ea_time
              if (ea_time == end(ev_time_range) && !day_change) {
                day_change = true;
                ea_time = location_to_event_times.begin();
                continue;
              }
              ea_time++;
            }
          }
        }
        trip_transfers.push_back(transfers_from);
      }
      transfers.emplace_back(trip_transfers);
    }
  }

  return transfers;
}

}  // namespace nigiri::tripbased
