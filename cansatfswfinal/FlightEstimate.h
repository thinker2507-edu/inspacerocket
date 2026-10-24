#pragma once
#include <stdint.h>
#include <math.h>

// Acceleration and gravity share BNO08x device coordinates (m/s^2).
// Project linear acceleration onto local up; no assumed rocket mounting axis.
constexpr bool verticalAcceleration(float ax, float ay, float az,
                                 float gx, float gy, float gz, float& up) {
  float g = sqrtf(gx*gx + gy*gy + gz*gz);
  float a = sqrtf(ax*ax + ay*ay + az*az);
  if (!isfinite(g) || !isfinite(a) || g < 8.5f || g > 10.8f || a >= 150.0f) return false;
  up = ((ax-gx)*gx + (ay-gy)*gy + (az-gz)*gz) / g;
  return isfinite(up) && fabsf(up) < 140.0f;
}

struct VerticalEstimate {
  float height = 0, speed = 0, baroHeight = 0, baroSpeed = 0, bias = 0;
  uint32_t baroMs = 0;
  bool ready = false;
  uint8_t mode = 0; // 0 unavailable, 1 barometric correction only, 2 inertial + barometric

  constexpr void reset(uint32_t now, float padBias = 0) {
    height = speed = baroHeight = baroSpeed = 0;
    bias = padBias; baroMs = now; ready = true; mode = 1;
  }
  constexpr bool fresh(uint32_t now) const { return ready && uint32_t(now-baroMs) <= 100; }
  constexpr bool update(uint32_t now, float dt, bool baro, float measured, bool imu, float acceleration) {
    if (!ready) return false;
    if (!(dt > 0 && dt <= 0.1f)) { speed = 0; mode = 0; return false; }
    if (baro) {
      float elapsed = uint32_t(now-baroMs) * 0.001f;
      if (!isfinite(measured) || fabsf(measured) > 20000 ||
          fabsf(measured-baroHeight) > 5.0f + 300.0f*elapsed) baro = false;
    }
    if (!baro && uint32_t(now-baroMs) > 250) { speed = 0; mode = 0; return false; }
    float a = imu ? acceleration-bias : 0;
    height += speed*dt + 0.5f*a*dt*dt;
    speed += a*dt;
    if (baro) {
      float elapsed = uint32_t(now-baroMs)*0.001f;
      if (elapsed > 0 && elapsed <= 0.25f)
        baroSpeed += (elapsed/(0.25f+elapsed))*((measured-baroHeight)/elapsed-baroSpeed);
      else baroSpeed = 0;
      height += (dt/(0.35f+dt))*(measured-height);
      speed += (dt/(0.8f+dt))*(baroSpeed-speed);
      baroHeight = measured; baroMs = now;
    }
    if (!isfinite(height) || !isfinite(speed) || fabsf(height)>20000 || fabsf(speed)>400) {
      ready = false; mode = 0; height = baroHeight; speed = 0; return false;
    }
    mode = imu ? 2 : 1;
    return baro;
  }
};

