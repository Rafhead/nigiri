#include "nigiri/tripbased/preprocessing.h"
#include "nigiri/common/it_range.h"
#include "nigiri/common/linear_lower_bound.h"

namespace nigiri::tripbased {

nvec<std::uint32_t, transfer, 2> compute_transfers(timetable& tt) {
  // Transfers
  nvec<std::uint32_t, transfer, 2> transfers;
  bitfield transport_from_bf;
  bitfield_idx_t transport_from_bf_idx;
  bitfield_idx_t transport_to_bf_idx;
  bool day_change;
  location_idx_t loc_to_idx;
  duration_t footpath_duration;
  // minutes_after_midnight_t transport_from_mam;

  // minutes_after_midnight_t transport_to_mam;
  // Whether after transfer to u and traversing next station a footpath can make
  // day change
  // bool day_change_footpath;

  // auto bitfields = hash_map<bitfield, bitfield_idx_t>{};
  for (auto const [i, bf] : utl::enumerate(tt.bitfields_)) {
    bitfields_.emplace(bf, bitfield_idx_t{i});
  }

  std::cout << "n routes " << tt.n_routes() << std::endl;
  std::cout << "n locations " << tt.n_locations() << std::endl;
  std::cout << "n transports " << tt.transport_traffic_days_.size()
            << std::endl;

  // transfers for a single trip
  vector<vector<transfer>> trip_transfers;
  // transfers for a trip and for his unique stop
  vector<transfer> transfers_from;

  // To store to bitsets in timetable for times on stations
  std::vector<std::vector<std::pair<minutes_after_midnight_t, bitfield>>>
      arrival_times;
  std::vector<std::vector<std::pair<minutes_after_midnight_t, bitfield>>>
      ea_change_times;

  // Iterate through all trips from index 0 to last
  for (auto transport_from_idx = transport_idx_t{0U};
       transport_from_idx < tt.transport_route_.size(); transport_from_idx++) {
    // reset trip transfers for a new trip
    trip_transfers.resize(0U);

    arrival_times.clear();
    arrival_times.resize(tt.n_locations());
    for (auto arrival_time : arrival_times) {
      arrival_time.resize(0);
    }
    ea_change_times.clear();
    ea_change_times.resize(tt.n_locations());

    auto const route_from_idx = tt.transport_route_[transport_from_idx];

    // Iterate through all stops of this trip's route in opposite direction
    // skipping the first station
    auto const loc_from_seq =
        tt.route_location_seq_[route_idx_t{route_from_idx}];
    for (unsigned stop_from_idx = loc_from_seq.size() - 1U; stop_from_idx != 0U;
         stop_from_idx--) {
      // Reset transfers for a unique stop for the next stop
      transfers_from.resize(0U);

      // Skip if it is not possible to transfer from this stop
      auto const stop_from = stop{stop_from_idx};
      /*std::cout << "Out from stop " << stop_from.location_ << " is "
                << stop_from.out_allowed();
      if (!stop_from.out_allowed()) {
        trip_transfers.emplace_back(transfers_from);
        continue;
      }*/
      auto const loc_from_idx = stop_from.location_idx();
      // auto const loc_from_idx = loc_from_seq[stop_from_idx];
      auto const [transport_from_days, transport_from_mam] =
          tt.event_mam(route_idx_t{route_from_idx}, transport_from_idx,
                       stop_from_idx, event_type::kArr);
      transport_from_bf_idx = tt.transport_traffic_days_[transport_from_idx];
      update_time(arrival_times, location_idx_t{loc_from_idx},
                  minutes_after_midnight_t{transport_from_mam},
                  tt.bitfields_[transport_from_bf_idx], false, tt);

      // Iterate through all stops that are reachable by footpath
      // +1 for the station itself
      // Station itself is not included in footpaths from them
      // TODO: Possibly check meta stations too
      auto const loc_footpaths_from =
          tt.locations_.footpaths_out_[location_idx_t{loc_from_idx}];
      auto footpath_from_idx = 0U;
      // Iterate through all footpaths from
      while (footpath_from_idx != loc_footpaths_from.size() + 1) {
        // Pick up the station the footpath goes to
        if (footpath_from_idx == loc_footpaths_from.size() ||
            loc_footpaths_from.empty()) {
          loc_to_idx = location_idx_t{loc_from_idx};
          footpath_duration = duration_t{0U};
        } else {
          loc_to_idx =
              location_idx_t{loc_footpaths_from.at(footpath_from_idx).target_};
          footpath_duration =
              duration_t{loc_footpaths_from.at(footpath_from_idx).duration_};
        }

        // Skip if in is not allowed
        /*auto const stop_to = stop{loc_to_idx.v_};
        std::cout << "In to stop " << stop_to.location_ << " is "
                  << stop_to.in_allowed() << std::endl;
        if (!stop_to.in_allowed()) {
          continue;
        }*/

        day_change = false;
        // Get mam for the trip from and add footpath
        auto const mam_at_stop_from =
            transport_from_mam + footpath_duration.count();
        // Check if footpath makes day change
        if (transport_from_mam / 1440U != mam_at_stop_from / 1440U) {
          day_change = true;
        }
        update_time(arrival_times, loc_to_idx,
                    minutes_after_midnight_t{mam_at_stop_from} % 1440U,
                    tt.bitfields_[transport_from_bf_idx], day_change, tt);
        update_time(ea_change_times, loc_to_idx,
                    minutes_after_midnight_t{mam_at_stop_from} % 1440U,
                    tt.bitfields_[transport_from_bf_idx], day_change, tt);

        // Iterate through all routes on current stop or on stops reachable by
        // footpath
        for (route_idx_t route_to_idx : tt.location_routes_[loc_to_idx]) {
          // Skip route if it is his last station
          if (tt.route_location_seq_
                  [route_to_idx][tt.route_location_seq_[route_to_idx].size() -
                                 1] == loc_to_idx.v_) {
            continue;
          }
          // Get position of stop to in route sequence
          auto const route_to_stop_seq =
              tt.route_location_seq_[route_idx_t{route_to_idx}];
          auto loc_to_pos_idx = 0U;
          for (auto idx = 0U; idx < route_to_stop_seq.size(); idx++) {
            auto const stop_to = stop{route_to_stop_seq[idx]};
            if (stop_to.location_idx() == loc_to_idx) {
              loc_to_pos_idx = idx;
            }
          }
          // Set the current trips bitfields to its actual value and shift
          // b[t]' <-- ...
          transport_from_bf.set(
              tt.bitfields_[transport_from_bf_idx].to_string());
          auto transport_from_bf_cpy = bitset<512>(transport_from_bf);
          transport_from_bf_cpy >>= (transport_from_days);
          day_change = false;

          // Find the earliest trip for line L
          // Get a look on event times for current route
          auto const stop_to_idx = loc_to_pos_idx;
          auto const loc_to_ev_times = tt.event_times_at_stop(
              route_to_idx, stop_to_idx, event_type::kDep);
          // Set iterator on that to iterate through times
          auto const ev_time_range = it_range{
              linear_lb(loc_to_ev_times.begin(), loc_to_ev_times.end(),
                        minutes_after_midnight_t{mam_at_stop_from},
                        [&](delta const a, minutes_after_midnight_t b) {
                          return a.mam() < b.count();
                        }),
              loc_to_ev_times.end()};
          // TODO: iterator belongs to the earliest time on
          // station based on the arrival time?
          auto ea_time_it = begin(ev_time_range);
          // Check if footpath makes day change
          if (mam_at_stop_from / 1440U != transport_from_mam / 1440U) {
            day_change = true;
          }
          // Check if ea transport found
          if ((ev_time_range.empty() || ea_time_it == end(ev_time_range)) &&
              !day_change) {
            ea_time_it = begin(loc_to_ev_times);
            day_change = true;
          }
          // get copy of earliest time to iterate later on it and keep
          // original for comparison
          auto ea_time = ea_time_it;

          //  while...
          // TODO check ea_time != ...
          while (transport_from_bf_cpy.any() && ea_time != ea_time_it - 1) {
            // transport to offset
            auto transport_to_offset =
                static_cast<std::size_t>(&*ea_time - loc_to_ev_times.data());
            // transport to itself
            auto const transport_to_idx =
                tt.route_transport_ranges_[route_to_idx][transport_to_offset];
            // bitset index of transport to
            transport_to_bf_idx = tt.transport_traffic_days_[transport_to_idx];
            // bitset of transport to
            auto const transport_to_bf = tt.bitfields_[transport_to_bf_idx];
            // event on stop to
            auto const event_on_stop_to = *ea_time;
            // [b_u]'
            auto transport_to_bf_cpy = bitset<512>(transport_to_bf.to_string());
            // [b_u] >>> ...
            transport_to_bf_cpy >>= event_on_stop_to.days();
            transport_to_bf_cpy <<= day_change;
            // [b_tr] <-- ...
            auto transfer_bf = bitset<512>(transport_from_bf_cpy.to_string());
            transfer_bf &= transport_to_bf_cpy;
            auto keep = bitset<512>();

            // Check for U-turn
            // Check if u turn possible
            auto u_turn_valid = true;
            auto loc_from_prev_idx = location_idx_t{0U};
            if (stop_from_idx < 1U) {
              u_turn_valid = false;
            }
            auto loc_to_next_idx = location_idx_t{0U};
            if (loc_to_pos_idx > route_to_stop_seq.size() - 2U) {
              u_turn_valid = false;
            }
            if (u_turn_valid) {
              loc_from_prev_idx =
                  location_idx_t{loc_from_seq[stop_from_idx - 1U]};
              loc_to_next_idx = location_idx_t{loc_to_pos_idx + 1U};
            }
            if (u_turn_valid && loc_from_prev_idx == loc_to_next_idx &&
                stop_from_idx != 1U &&
                loc_to_pos_idx != route_to_stop_seq.size() - 2U) {
              std::cout << "U turn started\n";
              auto const [transport_from_prev_days, transport_from_prev_mam] =
                  tt.event_mam(route_idx_t{route_from_idx}, transport_from_idx,
                               stop_from_idx - 1, event_type::kArr);
              auto const day_diff =
                  transport_from_days - transport_from_prev_days;
              auto const transport_from_bf_cc_m_idx = get_bitfield_idx(
                  transport_from_bf >> transport_from_prev_days, tt);
              auto next_c = false;
              auto const change_time =
                  tt.locations_.transfer_time_[location_idx_t{loc_from_idx}];
              if (transport_from_prev_mam + change_time.count() !=
                  transport_from_prev_mam) {
                next_c = true;
              }
              // auto const route_to_n_transports = static_cast<unsigned>(
              //     tt.route_transport_ranges_[route_to_idx].size());
              auto const [loc_to_next_days, loc_to_next_mam] =
                  tt.event_mam(route_to_idx, transport_to_idx, stop_to_idx + 1,
                               event_type::kDep);
              //(ea_time - 2 * route_to_n_transports)->count();
              if (transport_from_prev_mam + change_time.count() % 1440U >
                  loc_to_next_mam % 1440U) {
                next_c = true;
              }
              //[b_u]'' <-- ...
              auto transport_to_bf_cc = transport_to_bf;
              transport_to_bf_cc >>= loc_to_next_days;
              transport_to_bf_cc <<= next_c;
              //[b_tr]'
              auto transfer_bf_cpy = tt.bitfields_[transport_from_bf_cc_m_idx];
              transfer_bf_cpy &= transport_to_bf_cc;
              //[b_tr] <--
              transfer_bf_cpy >>= day_diff;
              transfer_bf_cpy = ~transfer_bf_cpy;
              transfer_bf &= transfer_bf_cpy;
              std::cout << "U turn ended\n";
            }  // End check U-turn

            // for each stop p_k^u ...
            // Check for improvements
            /*auto const loc_first_arrivals = tt.event_times_at_stop(
                route_to_idx, stop_to_idx, event_type::kArr);
            auto const loc_first_arr = loc_first_arrivals[transport_to_offset];
            for (auto next_locs = loc_to_pos_it;
                 next_locs < route_to_stop_seq.end(); ++next_locs) {
              // TODO: check use of ea_time on the next stations
              auto const loc_after_arrivals = tt.event_times_at_stop(
                  route_to_idx, *next_locs, event_type::kArr);
              auto const loc_after_arr =
                  loc_after_arrivals[transport_to_offset];
              // 24 hours check
              if (footpath_duration.count() % 1440 + loc_after_arr.count() -
                      loc_first_arr.count() >
                  1440) {
                break;
              }
              // check if day change in the next station
              if (!day_change &&
                  loc_after_arr.count() % 1440 <= mam_at_stop_from) {
                day_change = 1;
              }
              keep |= tt.bitfields_[update_time(
                  arrival_times, location_idx_t{*next_locs},
                  minutes_after_midnight_t{
                      (*(ea_time + (next_locs - loc_to_pos_it))).mam()} %
                      1440,
                  transfer_bf, day_change, tt)];
              // Iter through footpaths
              auto const next_locs_foot =
                  tt.locations_.footpaths_out_[location_idx_t{*next_locs}];
              for (size_t next_locs_to_idx = 0;
                   next_locs_to_idx != next_locs_foot.size();
                   ++next_locs_to_idx) {
                day_change_footpath = day_change;
                auto const arr_to_time =
                    (minutes_after_midnight_t{(*ea_time).mam()} +
                     next_locs_foot.at(next_locs_to_idx).duration()) %
                    1440;
                // 24 hours check
                if (footpath_duration.count() % 1440 + loc_after_arr.count() +
                        next_locs_foot.at(next_locs_to_idx).duration().count() -
                        loc_first_arr.count() >
                    1440) {
                  continue;
                }
                day_change_footpath = day_change;
                // check if footpath makes day change
                if (!day_change_footpath &&
                    (loc_after_arr.count() +
                     next_locs_foot.at(next_locs_to_idx).duration().count()) %
                            1440 <=
                        mam_at_stop_from) {
                  day_change_footpath = 1;
                }
                keep |= tt.bitfields_[update_time(
                    arrival_times, next_locs_foot.at(next_locs_to_idx).target(),
                    arr_to_time, transfer_bf, day_change_footpath, tt)];
                keep |= tt.bitfields_[update_time(
                    ea_change_times,
                    next_locs_foot.at(next_locs_to_idx).target(), arr_to_time,
                    transfer_bf, day_change_footpath, tt)];
              }
            }  // End check improvements

            if (keep.any()) {
              transport_from_bf_cpy &= ~keep;*/
            if (transfer_bf.any()) {
              auto const new_transfer =
                  transfer(transport_to_idx, stop_to_idx,
                           get_bitfield_idx(keep, tt), day_change);
              /*std::cout << "Added transfer from trip " << transport_from_idx
                        << " to " << transport_to_idx << " with bitfield "
                        << transfer_bf.to_string() << std::endl;*/
              transfers_from.emplace_back(new_transfer);
              // std::cout << "Transfer pushed\n";
            }

            if (ea_time == end(ev_time_range) && !day_change) {
              day_change = true;
              ea_time = loc_to_ev_times.begin();
              continue;
            }
            ea_time++;

            // special case when both event times point to the whole array
            if (ea_time_it == loc_to_ev_times.begin() &&
                ea_time == loc_to_ev_times.end()) {
              break;
            }
          }
        }
        footpath_from_idx++;
      }
      trip_transfers.emplace_back(transfers_from);
    }
    // Reverse transfers because we iterated in opposite direction of stops
    std::reverse(trip_transfers.begin(), trip_transfers.end());
    transfers.emplace_back(trip_transfers);
  }

  return transfers;
}

bitfield_idx_t update_time(
    std::vector<std::vector<std::pair<minutes_after_midnight_t, bitfield>>>&
        times,
    location_idx_t l_idx,
    minutes_after_midnight_t new_time_on_l,
    const bitfield bf,
    bool day_change,
    timetable& tt) {
  auto bf_cpy = bf >> day_change;
  // check if empty
  if (times[l_idx.v_].empty()) {
    // insert
    times[l_idx.v_].emplace_back(std::make_pair(new_time_on_l, bf_cpy));
    return get_bitfield_idx(bf_cpy, tt);
  }
  auto equal = false;
  auto improve = cista::bitset<512>();
  auto temp_bf = cista::bitset<512>();
  cista::bitset<512> bf_on_l_cpy;
  auto times_on_l = times[l_idx.v_];
  for (auto [time_on_l, bf_on_l] : times_on_l) {
    bf_on_l_cpy = bf_on_l;
    // if new time is earlier
    if (new_time_on_l.count() < time_on_l.count()) {
      temp_bf = bf_on_l_cpy & ~(bf_cpy & bf_on_l_cpy);
      improve = improve | (bf_on_l_cpy & ~temp_bf);
      // keep time although the bitset is 0
      bf_on_l = temp_bf;
    } else if (new_time_on_l.count() > time_on_l.count()) {
      bf_cpy = bf_cpy & ~(bf_cpy & bf_on_l);
    } else {
      bf_cpy = bf_cpy & ~(bf_cpy & bf_on_l);
      improve = improve | (bf_cpy & ~bf_on_l);
      bf_on_l = bf_cpy | bf_on_l;
      equal = true;
    }
    if (!bf_cpy.any()) {
      break;
    }
  }
  if (!improve.any() && !equal) {
    times[l_idx.v_].emplace_back(std::make_pair(new_time_on_l, bf_cpy));
  }
  return get_bitfield_idx(improve, tt);
}

const bitfield_idx_t get_bitfield_idx(bitfield const& b, timetable& tt) {
  bitfield_idx_t idx;
  return utl::get_or_create(bitfields_, b, [&]() {
    idx = bitfield_idx_t{tt.bitfields_.size()};
    tt.bitfields_.emplace_back(b);
    return idx;
  });
}

}  // namespace nigiri::tripbased
