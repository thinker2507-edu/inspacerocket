#pragma once

// Rocket launch-detector configuration. Inherited starting values, not flight-
// validated limits. Tune from this vehicle's simulation and measured sensor logs.
// Heights are relative to the calibrated pad; velocity is fused vertical m/s.
// LAUNCH_ACCEL_G retains the original total-g-equivalent convention: the loop
// compares gravity-compensated vertical acceleration with (this value-1)*g.
constexpr float LAUNCH_ACCEL_G = 2.5f;       // equivalent net threshold: 1.5 g
constexpr int LAUNCH_SAMPLES = 5;           // consecutive accepted baro samples
constexpr float LAUNCH_MIN_ALT_M = 3.0f;    // powered path: must also be rising
constexpr float LAUNCH_MIN_VEL_MPS = 0.0f;
constexpr float LAUNCH_FALLBACK_ALT_M = 15.0f;
constexpr float LAUNCH_FALLBACK_VEL_MPS = 10.0f;

static_assert(LAUNCH_ACCEL_G>1.0f && LAUNCH_ACCEL_G<15.0f,
              "Launch acceleration must fit the existing trusted IMU range");
static_assert(LAUNCH_SAMPLES>=2 && LAUNCH_SAMPLES<=255,
              "Launch requires sustained samples within the uint8 counter range");
static_assert(LAUNCH_MIN_ALT_M>0 && LAUNCH_MIN_VEL_MPS>=0 &&
              LAUNCH_FALLBACK_ALT_M>=LAUNCH_MIN_ALT_M &&
              LAUNCH_FALLBACK_VEL_MPS>LAUNCH_MIN_VEL_MPS,
              "Launch altitude/velocity limits must be positive and coherent");
