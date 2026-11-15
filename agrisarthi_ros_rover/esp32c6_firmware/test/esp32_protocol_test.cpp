#include "../esp32_protocol.h"
#include <cassert>
#include <cstdint>

int main() {
  assert(agrisarthiSequenceIsNewer(2U, 1U));
  assert(!agrisarthiSequenceIsNewer(1U, 1U));
  assert(!agrisarthiSequenceIsNewer(1U, 2U));
  assert(agrisarthiSequenceIsNewer(0U, UINT32_MAX));
  assert(!agrisarthiSequenceIsNewer(UINT32_MAX, 0U));
  return 0;
}
