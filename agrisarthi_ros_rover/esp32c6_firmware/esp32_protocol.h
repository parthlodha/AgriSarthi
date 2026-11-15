#pragma once

#include <stdint.h>

// RFC-style half-range comparison handles uint32_t wrap-around.
inline bool agrisarthiSequenceIsNewer(uint32_t candidate, uint32_t previous) {
  const uint32_t delta = candidate - previous;
  return delta != 0U && delta < 0x80000000U;
}
