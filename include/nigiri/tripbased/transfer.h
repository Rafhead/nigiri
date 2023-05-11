#pragma once

#include "nigiri/timetable.h"
#include "nigiri/types.h"

#ifndef NIGIRI_TRANSFER_H
#define NIGIRI_TRANSFER_H

#endif  // NIGIRI_TRANSFER_H

namespace nigiri::tripbased {

struct transfer {
  transfer(uint32_t const to,
           bool const next,
           bitfield_idx_t const traffic_days)
      : to_{to}, next_{next ? 1U : 0U}, traffic_days_{traffic_days} {}

  transport_idx_t to() const { return transport_idx_t{to_}; }
  bool next() const { return next_ == 0U ? false : true; }
  bitfield_idx_t traffic_days() const { return traffic_days_; }

private:
  transport_idx_t::value_t to_ : 31;
  transport_idx_t::value_t next_ : 1;
  bitfield_idx_t traffic_days_;
};

}  // namespace nigiri::tripbased