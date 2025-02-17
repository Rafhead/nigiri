#include "gtest/gtest.h"

#include "nigiri/loader/hrd/load_timetable.h"
#include "nigiri/loader/init_finish.h"

#include "../loader/hrd/hrd_timetable.h"

#include "nigiri/routing/search.h"
#include "nigiri/tripbased/preprocessing.h"
#include "nigiri/tripbased/tripbased.h"

#include "../raptor_search.h"

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::test_data::hrd_timetable;
using nigiri::test::raptor_search;

constexpr auto const fwd_journeys = R"(
[2020-03-30 05:00, 2020-03-30 07:15]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 05:00]
       TO: (C, 0000003) [2020-03-30 07:15]
leg 0: (A, 0000001) [2020-03-30 05:00] -> (B, 0000002) [2020-03-30 06:00]
   0: 0000001 A...............................................                               d: 30.03 05:00 [30.03 07:00]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/300/0000002/360/, src=0}]
   1: 0000002 B............................................... a: 30.03 06:00 [30.03 08:00]
leg 1: (B, 0000002) [2020-03-30 06:00] -> (B, 0000002) [2020-03-30 06:02]
  FOOTPATH (duration=2)
leg 2: (B, 0000002) [2020-03-30 06:15] -> (C, 0000003) [2020-03-30 07:15]
   0: 0000002 B...............................................                               d: 30.03 06:15 [30.03 08:15]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/375/0000003/435/, src=0}]
   1: 0000003 C............................................... a: 30.03 07:15 [30.03 09:15]
leg 3: (C, 0000003) [2020-03-30 07:15] -> (C, 0000003) [2020-03-30 07:15]
  FOOTPATH (duration=0)


[2020-03-30 05:30, 2020-03-30 07:45]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 05:30]
       TO: (C, 0000003) [2020-03-30 07:45]
leg 0: (A, 0000001) [2020-03-30 05:30] -> (B, 0000002) [2020-03-30 06:30]
   0: 0000001 A...............................................                               d: 30.03 05:30 [30.03 07:30]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/330/0000002/390/, src=0}]
   1: 0000002 B............................................... a: 30.03 06:30 [30.03 08:30]
leg 1: (B, 0000002) [2020-03-30 06:30] -> (B, 0000002) [2020-03-30 06:32]
  FOOTPATH (duration=2)
leg 2: (B, 0000002) [2020-03-30 06:45] -> (C, 0000003) [2020-03-30 07:45]
   0: 0000002 B...............................................                               d: 30.03 06:45 [30.03 08:45]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/405/0000003/465/, src=0}]
   1: 0000003 C............................................... a: 30.03 07:45 [30.03 09:45]
leg 3: (C, 0000003) [2020-03-30 07:45] -> (C, 0000003) [2020-03-30 07:45]
  FOOTPATH (duration=0)

)";

unixtime_t parse_time(std::string_view s, char const* format) {
  std::stringstream in;
  in << s;

  date::local_seconds ls;
  std::string tz;
  in >> date::parse(format, ls, tz);

  return std::chrono::time_point_cast<unixtime_t::duration>(
      date::make_zoned(tz, ls).get_sys_time());
}

TEST(routing, tripbased) {
  constexpr auto const src = source_idx_t{0U};

  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);
  tt.transfers_ = nigiri::tripbased::compute_transfers(tt);

  using algo_state_t = nigiri::tripbased::tripbased_state;
  static auto search_state = routing::search_state{};
  static auto algo_state = algo_state_t{};

  auto q = routing::query{
      .start_time_ =
          interval{unixtime_t{sys_days{2020_y / March / 30}} + 5_hours,
                   unixtime_t{sys_days{2020_y / March / 30}} + 6_hours},
      .start_ = {{tt.locations_.location_id_to_idx_.at({"0000001", src}),
                  0_minutes, 0U}},
      .destination_ = {{tt.locations_.location_id_to_idx_.at({"0000003", src}),
                        0_minutes, 0U}}};

  using algo_t = nigiri::tripbased::tripbased;
  auto const results = *(routing::search<direction::kForward, algo_t>{
      tt, nullptr, search_state, algo_state, std::move(q)}
                             .execute()
                             .journeys_);

  /*auto const result = raptor_search(
      tt, nullptr, "0000001", "0000003",
      interval{unixtime_t{sys_days{2020_y / March / 30}} + 5_hours,
               unixtime_t{sys_days{2020_y / March / 30}} + 6_hours});*/

  std::stringstream ss;
  ss << "\n";
  for (auto const& x : results) {
    x.print(ss, tt);
    ss << "\n\n";
  }
  EXPECT_EQ(std::string_view{fwd_journeys}, ss.str());
}

TEST(routing, raptor_forward) {
  constexpr auto const src = source_idx_t{0U};

  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const results = raptor_search(
      tt, nullptr, "0000001", "0000003",
      interval{unixtime_t{sys_days{2020_y / March / 30}} + 5_hours,
               unixtime_t{sys_days{2020_y / March / 30}} + 6_hours});

  std::stringstream ss;
  ss << "\n";
  for (auto const& x : results) {
    x.print(ss, tt);
    ss << "\n\n";
  }
  EXPECT_EQ(std::string_view{fwd_journeys}, ss.str());
}

constexpr auto const bwd_journeys = R"(
[2020-03-30 05:15, 2020-03-30 03:00]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 03:00]
       TO: (C, 0000003) [2020-03-30 05:15]
leg 0: (A, 0000001) [2020-03-30 03:00] -> (A, 0000001) [2020-03-30 03:00]
  FOOTPATH (duration=0)
leg 1: (A, 0000001) [2020-03-30 03:00] -> (B, 0000002) [2020-03-30 04:00]
   0: 0000001 A...............................................                               d: 30.03 03:00 [30.03 05:00]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/180/0000002/240/, src=0}]
   1: 0000002 B............................................... a: 30.03 04:00 [30.03 06:00]
leg 2: (B, 0000002) [2020-03-30 04:13] -> (B, 0000002) [2020-03-30 04:15]
  FOOTPATH (duration=2)
leg 3: (B, 0000002) [2020-03-30 04:15] -> (C, 0000003) [2020-03-30 05:15]
   0: 0000002 B...............................................                               d: 30.03 04:15 [30.03 06:15]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/255/0000003/315/, src=0}]
   1: 0000003 C............................................... a: 30.03 05:15 [30.03 07:15]


[2020-03-30 05:45, 2020-03-30 03:30]
TRANSFERS: 1
     FROM: (A, 0000001) [2020-03-30 03:30]
       TO: (C, 0000003) [2020-03-30 05:45]
leg 0: (A, 0000001) [2020-03-30 03:30] -> (A, 0000001) [2020-03-30 03:30]
  FOOTPATH (duration=0)
leg 1: (A, 0000001) [2020-03-30 03:30] -> (B, 0000002) [2020-03-30 04:30]
   0: 0000001 A...............................................                               d: 30.03 03:30 [30.03 05:30]  [{name=RE 1337, day=2020-03-30, id=1337/0000001/210/0000002/270/, src=0}]
   1: 0000002 B............................................... a: 30.03 04:30 [30.03 06:30]
leg 2: (B, 0000002) [2020-03-30 04:43] -> (B, 0000002) [2020-03-30 04:45]
  FOOTPATH (duration=2)
leg 3: (B, 0000002) [2020-03-30 04:45] -> (C, 0000003) [2020-03-30 05:45]
   0: 0000002 B...............................................                               d: 30.03 04:45 [30.03 06:45]  [{name=RE 7331, day=2020-03-30, id=7331/0000002/285/0000003/345/, src=0}]
   1: 0000003 C............................................... a: 30.03 05:45 [30.03 07:45]


)";

TEST(routing, raptor_backward) {
  using namespace date;
  constexpr auto const src = source_idx_t{0U};

  timetable tt;
  tt.date_range_ = full_period();
  load_timetable(src, loader::hrd::hrd_5_20_26, files_abc(), tt);
  finalize(tt);

  auto const results = raptor_search(
      tt, nullptr, "0000003", "0000001",
      interval{unixtime_t{sys_days{2020_y / March / 30}} + 5_hours,
               unixtime_t{sys_days{2020_y / March / 30}} + 6_hours},
      direction::kBackward);

  ASSERT_EQ(2U, results.size());

  std::stringstream ss;
  ss << "\n";
  for (auto const& x : results) {
    x.print(ss, tt);
    ss << "\n\n";
  }
  EXPECT_EQ(std::string_view{bwd_journeys}, ss.str());
}
