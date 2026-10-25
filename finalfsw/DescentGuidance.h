#pragma once
#include <stdint.h>
#include <math.h>

// Guidance calculations from desent/cruciform_descent_guidance.ino.
// Deliberately excludes its standalone setup/loop, placeholder sensors and servos.
// Candidate aerodynamic parameters are reference defaults, not flight calibration.
namespace RocketDescent {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float TURN_SIGN = 1.0f, YAW_RATE_SIGN = 1.0f;
constexpr float GYRO_TO_RADS = 0.0174533f;
constexpr uint32_t CANOPY_WAIT_MS = 6000, FIX_MAX_AGE_MS = 2000;
// ---- vehicle (MEASURE THESE IN A CALM-AIR DROP TEST)
constexpr float V_AIR       = 3.0f;     // [m/s] horizontal airspeed while gliding  (biggest unknown!)
constexpr float R_TURN_MIN  = 5.0f;     // [m] turn radius with one side fully pinched, other released
constexpr float DESCENT_NOM = 8.0f;     // [m/s] nominal sink rate
constexpr float BASE_BRAKE  = 0.3f;     // [0..1] trim pinch on both arms
constexpr float KT = V_AIR / R_TURN_MIN;// [rad/s per unit differential]  (differential = dR - dL, max 1)

// ---- guidance
constexpr float R_ORBIT_START = 60.0f;  // [m] loiter radius (around PIP) at guidance start
constexpr float R_ORBIT_END   = 10.0f;  // [m] loiter radius near touchdown (keep >= 2*R_TURN_MIN)
constexpr float K_ORBIT       = 0.03f;  // [1/m] vector-field smoothness (same as sim)
constexpr float TAU_HDG       = 2.0f;   // [s] desired heading-loop time constant -> Kp = 1/(KT*TAU)
constexpr float KD_RATE       = 0.25f;  // [unit per rad/s] gyro rate damping
constexpr float U_MAX         = 1.0f;   // max differential command
constexpr float ACQ_BIAS      = 0.5f;   // differential used to start gliding until heading is known

// ---- phases / timing
constexpr float DRIFT_SETTLE_S = 4.0f;  // ignore GNSS this long after canopy opens (deploy shock)
constexpr float DRIFT_AVG_S    = 8.0f;  // average ground velocity for the initial wind estimate
constexpr float HOLD_ALT_M     = 30.0f; // below this AGL: arms symmetric, no more steering
constexpr float HOLD_TGO_S     = 4.0f;

// ---- estimators
constexpr float AB_ALPHA   = 0.5f;      // GNSS position alpha-beta filter (tune to fix rate)
constexpr float AB_BETA    = 0.12f;
constexpr float GNSS_VELOCITY_ALPHA  = 0.4f;      // used when the GNSS module supplies Doppler velocity
constexpr float TAU_WIND   = 15.0f;     // [s] wind refinement time constant
constexpr float VA_VALID   = 0.8f;      // [m/s] min air-relative speed to trust GNSS heading
constexpr float K_HDG      = 0.15f;     // heading correction gain per GNSS fix
constexpr float HDG_LEAD_S = 1.0f;      // compensates lag of filtered velocity (0.2 with Doppler vel)
const int   MIN_SATS   = 5;
constexpr float FIT_WIN_S  = 40.0f;     // wind circle-fit window
const int   FIT_MIN_N  = 10;        // min velocity samples (0.5 s apart) for a fit
constexpr float FIT_MIN_ECC= 0.25f;     // min lambda_min/lambda_max = enough heading diversity (~>=120 deg turned)
constexpr float ACQ_TIMEOUT_S = 40.0f;  // if no circle fit by then, trust the drift-average wind

// ============================= TYPES =================================
struct Sensors {
  float   alt_m;     // baro altitude [m]  (telemetry 'alt')
  bool    fix;       // true ONLY on the loop where a NEW GNSS fix arrived
  double  lat, lon;  // [deg]
  uint8_t sats;
  bool    hasVel;    // true if GNSS module gives ground velocity (strongly recommended)
  float   vE, vN;    // [m/s] GNSS velocity East/North
  float   gz_dps;    // gyro rate about the vertical (yaw) axis [deg/s]
};

constexpr float M_PER_DEG = 111319.5f;
// Same wrapped angle, bounded work even after a large accumulated heading.
static constexpr float wrapPi(float a) {
  if(a>kPi || a<-kPi)a=fmodf(a,kTwoPi);
  if(a>kPi)a-=kTwoPi;
  if(a<-kPi)a+=kTwoPi;
  return a;
}
static constexpr float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// =========================== GUIDANCE CORE ===========================
class DescentGuidance {
public:
  enum Mode : uint8_t { IDLE, DRIFT, GUIDED, HOLD };
  Mode  mode = IDLE;
  float cmdL = BASE_BRAKE, cmdR = BASE_BRAKE;           // output deflections 0..1

  // exposed state (for logging)
  float px = 0, py = 0, vx = 0, vy = 0;                 // filtered position / ground velocity [m, m/s]
  float Wx = 0, Wy = 0;                                 // wind estimate (direction wind blows TO)
  float vaEst = V_AIR;                                  // airspeed estimate from circle fit
  bool  windFitted = false;
  float psi = 0;                                        // heading estimate [rad]
  bool  hdgValid = false;
  float altAgl = 0, vsF = DESCENT_NOM, tgo = 0;
  float dbgPsiCmd = 0, dbgR = 0, dbgRt = 0, dbgReach = 0;

  constexpr void setPad(double lat, double lon, float alt) {
    padLat = lat; padLon = lon; padAlt = alt;
    mPerDegLon = M_PER_DEG * cosf(lat * 0.0174533f);
  }
  constexpr void setGyroBias(float dps) { gyroBias = dps; }
  constexpr void start(float t) { mode = DRIFT; tStart = t; nV = 0; sumVx = sumVy = 0; }

  constexpr void update(const Sensors &s, float t, float dt) {
    yawRate = YAW_RATE_SIGN * (s.gz_dps - gyroBias) * GYRO_TO_RADS;
    psi = wrapPi(psi + yawRate * dt);
    altAgl = s.alt_m - padAlt;
    updateVertical(altAgl, t);
    if (s.fix && s.sats >= MIN_SATS) onFix(s, t);

    cmdL = cmdR = BASE_BRAKE;
    if (mode == IDLE || mode == DRIFT || !havePos) {
      if (mode == DRIFT && (t - tStart) > (DRIFT_SETTLE_S + DRIFT_AVG_S) && nV >= 4) {
        Wx = sumVx / nV; Wy = sumVy / nV;             // airspeed ~0 while arms are symmetric
        mode = GUIDED; tGuide = t; sN = 0; sI = 0; lastS = -10;
        tgo0 = fmaxf(altAgl / fmaxf(vsF, 3.0f), 30.0f);
      }
      return;
    }

    float vsUse = fmaxf(vsF, 3.0f);
    tgo = fmaxf(altAgl, 0.0f) / vsUse;
    if (mode == GUIDED && (tgo < HOLD_TGO_S || altAgl < HOLD_ALT_M)) mode = HOLD;
    if (mode == HOLD) return;                         // symmetric, stable landing

    // ---- predicted impact point relative to the pad (target = origin)
    float dtF = fminf(t - tFix, 2.0f);
    float ix = px + vx * dtF + Wx * tgo;
    float iy = py + vy * dtF + Wy * tgo;
    float r = hypotf(ix, iy);
    float theta = atan2f(iy, ix);
    float frac = clampf(tgo / tgo0, 0.0f, 1.0f);
    float Rt = R_ORBIT_END + (R_ORBIT_START - R_ORBIT_END) * frac;   // shrinking spiral
    float corr = atanf(K_ORBIT * (r - Rt));

    float u = 0;
    if (!hdgValid) {
      u = ACQ_BIAS;                                    // get some airspeed so heading is observable
    } else {
      if (dir == 0) {                                  // pick the orbit direction needing less turn
        float eCCW = fabsf(wrapPi(theta + (kPi / 2 + corr) - psi));
        float eCW  = fabsf(wrapPi(theta - (kPi / 2 + corr) - psi));
        dir = (eCCW <= eCW) ? +1 : -1;
      }
      float psiCmd = theta + dir * (kPi / 2 + corr);
      float err = wrapPi(psiCmd - psi);
      float Kp = 1.0f / (KT * TAU_HDG);
      u = clampf(Kp * err - KD_RATE * yawRate, -U_MAX, U_MAX);
      dbgPsiCmd = psiCmd;
    }
    dbgR = r; dbgRt = Rt; dbgReach = vaEst * tgo;      // dbgReach < dbgR - dbgRt => out of reach

    // ---- differential -> two deflections (preserve the difference when clipping)
    float diff = TURN_SIGN * u;
    float dL = BASE_BRAKE - diff * 0.5f, dR = BASE_BRAKE + diff * 0.5f;
    if (dL < 0) { dR -= dL; dL = 0; }
    if (dR < 0) { dL -= dR; dR = 0; }
    if (dL > 1) { dR -= (dL - 1); dL = 1; }
    if (dR > 1) { dL -= (dR - 1); dR = 1; }
    cmdL = clampf(dL, 0, 1); cmdR = clampf(dR, 0, 1);
  }

private:
  double padLat = 0, padLon = 0;
  float padAlt = 0, mPerDegLon = M_PER_DEG;
  float gyroBias = 0, yawRate = 0;
  bool  havePos = false, vsInit = false;
  float tFix = 0, tStart = 0, tgo0 = 120, altOld = 0, tOld = 0;
  float sumVx = 0, sumVy = 0; int nV = 0; int dir = 0;
  float tGuide = 0, lastS = -10;
  static const int NS = 48;
  float sT[NS]={}, sX[NS]={}, sY[NS]={}; int sN = 0, sI = 0;

  constexpr void pushSample(float t) {
    if (t - lastS < 0.5f) return;
    lastS = t; sT[sI] = t; sX[sI] = vx; sY[sI] = vy;
    sI = (sI + 1) % NS; if (sN < NS) sN++;
  }

  // Ground velocity = Wind + V_AIR*(cos psi, sin psi): samples lie on a circle whose CENTRE is the
  // wind and whose RADIUS is the airspeed. Kasa least-squares fit, needs heading diversity.
  constexpr void fitWind(float t) {
    int n = 0; float mx = 0, my = 0;
    for (int i = 0; i < sN; i++) if (t - sT[i] < FIT_WIN_S) { mx += sX[i]; my += sY[i]; n++; }
    if (n < FIT_MIN_N) return;
    mx /= n; my /= n;
    float Suu = 0, Suv = 0, Svv = 0, Sz = 0, Szu = 0, Szv = 0;
    for (int i = 0; i < sN; i++) if (t - sT[i] < FIT_WIN_S) {
      float u = sX[i] - mx, v = sY[i] - my, z = u * u + v * v;
      Suu += u * u; Suv += u * v; Svv += v * v; Sz += z; Szu += z * u; Szv += z * v;
    }
    float tr = Suu + Svv, det = Suu * Svv - Suv * Suv;
    float disc = sqrtf(fmaxf(tr * tr * 0.25f - det, 0.0f));
    float lmax = tr * 0.5f + disc, lmin = tr * 0.5f - disc;
    if (lmax < 1e-6f || lmin / lmax < FIT_MIN_ECC || fabsf(det) < 1e-9f) return;   // not enough turning
    float D = -(Szu * Svv - Szv * Suv) / det;
    float E = -(Szv * Suu - Szu * Suv) / det;
    float F = -Sz / n;
    float R2 = (D * D + E * E) * 0.25f - F;
    if (R2 <= 0) return;
    float Rf = sqrtf(R2);
    if (Rf < 0.4f * V_AIR || Rf > 2.0f * V_AIR) return;                           // implausible fit
    float Wfx = mx - D * 0.5f, Wfy = my - E * 0.5f;
    if (!windFitted) { Wx = Wfx; Wy = Wfy; vaEst = Rf; windFitted = true; }
    else { Wx += 0.5f * (Wfx - Wx); Wy += 0.5f * (Wfy - Wy); vaEst += 0.3f * (Rf - vaEst); }
  }

  constexpr void updateVertical(float alt, float t) {
    if (!vsInit) { altOld = alt; tOld = t; vsInit = true; return; }
    if (t - tOld >= 1.0f) {
      float raw = clampf((altOld - alt) / (t - tOld), 0.0f, 25.0f);
      vsF += 0.15f * (raw - vsF);
      altOld = alt; tOld = t;
    }
  }

  constexpr void onFix(const Sensors &s, float t) {
    float zx = (s.lon - padLon) * mPerDegLon, zy = (s.lat - padLat) * M_PER_DEG;
    if (!havePos) {
      px = zx; py = zy; vx = vy = 0; tFix = t; havePos = true;
      if (s.hasVel) { vx = s.vE; vy = s.vN; }
      return;
    }
    float d = t - tFix;
    if (d < 0.001f) return;
    float ex = px + vx * d, ey = py + vy * d;          // predicted
    float rx = zx - ex, ry = zy - ey;
    if (hypotf(rx, ry) > 60.0f) { tFix = t; return; }  // reject GNSS glitch
    px = ex + AB_ALPHA * rx; py = ey + AB_ALPHA * ry;
    if (s.hasVel) { vx += GNSS_VELOCITY_ALPHA * (s.vE - vx); vy += GNSS_VELOCITY_ALPHA * (s.vN - vy); }
    else          { vx += AB_BETA * rx / d;        vy += AB_BETA * ry / d; }
    tFix = t;

    if (mode == DRIFT) {
      if ((t - tStart) > DRIFT_SETTLE_S) { sumVx += vx; sumVy += vy; nV++; }
      return;
    }
    // ---- GUIDED: wind + airspeed from circle fit, light constraint refinement in between
    pushSample(t);
    fitWind(t);
    float ax = vx - Wx, ay = vy - Wy, m = hypotf(ax, ay);
    if (windFitted && m > 0.05f) {
      float res = m - vaEst;
      if (fabsf(res) < 2.5f * vaEst) {
        float k = clampf(d / TAU_WIND, 0.0f, 0.3f);
        Wx += k * res * ax / m; Wy += k * res * ay / m;
        ax = vx - Wx; ay = vy - Wy; m = hypotf(ax, ay);
      }
    }
    // ---- heading from GNSS air-relative velocity (lead-compensated for filter lag)
    bool wOk = windFitted || (t - tGuide) > ACQ_TIMEOUT_S;
    if (wOk && m > VA_VALID) {
      float psiGps = atan2f(ay, ax) + yawRate * HDG_LEAD_S;
      if (!hdgValid) { psi = wrapPi(psiGps); hdgValid = true; }
      else psi = wrapPi(psi + K_HDG * wrapPi(psiGps - psi));
    }
  }
};


} // namespace RocketDescent
