#pragma once
#include <stdint.h>

// Shared by the scheduler and the packet builder; control/status frames bypass it.
constexpr bool routineTelemetryDue(bool enabled, uint32_t now, uint32_t last,
                                   uint32_t period) {
  return enabled && uint32_t(now-last) >= period;
}
