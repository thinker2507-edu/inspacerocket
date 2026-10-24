#pragma once
#include <stdint.h>

// Hardware boundary only. No PSMS model, wiring, units or frame specification
// exists in the supplied project. In particular, Serial5 is NOT initialized.
class PsmsSensor {
public:
  enum Status : uint8_t { UNCONFIGURED=0, INITIALIZING=1, VALID=2, STALE=3, FAULT=4 };
  void begin() {}
  void service(uint32_t) {} // nonblocking, no fabricated readings
  constexpr Status status() const { return UNCONFIGURED; }
};
