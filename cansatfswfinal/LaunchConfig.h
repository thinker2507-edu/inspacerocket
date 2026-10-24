#pragma once

// Independent CanSat launch configuration. These inherited rocket-baseline
// values are NOT qualified for the actual CanSat release/launch profile.
// Tune this local file independently from finalfsw/LaunchConfig.h using measured
// data. Heights are relative to the calibrated pad; fused vertical speed is m/s.
// The net acceleration threshold is (LAUNCH_ACCEL_G-1)*g, preserving old logic.
constexpr float LAUNCH_ACCEL_G = 2.5f;
constexpr int LAUNCH_SAMPLES = 5;
constexpr float LAUNCH_MIN_ALT_M = 3.0f;
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
