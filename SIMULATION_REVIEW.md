# Available simulation review

Source available in this workspace: `INSPC248pmAODATA.csv` (891 numeric rows, 58 columns). No file named `value.CSV` was found. This review uses the available file; no launch thresholds or flight logic were changed from it during the manual-radio work.

| Quantity | Simulation value |
| --- | --- |
| IGNITION / LAUNCH event | 0 s |
| LIFTOFF event | 0.583 s |
| Launch-rod departure event | 0.968 s |
| Motor burnout | 3.830 s |
| Apogee event | 15.883 s |
| Maximum exported altitude | 1083.505 m above pad |
| Maximum exported vertical velocity | 144.223 m/s |
| Maximum exported vertical acceleration | 98.784 m/s2 |
| Ground-hit/end event | 170.283 s |

The file has variable time steps. Time is simulation elapsed seconds, not UTC/RTC or Teensy millis. Pressure is mbar (multiply by100 for firmware Pa). Vertical acceleration is a dynamics value; it must not be passed straight into the raw accelerometer telemetry axes as if it were a body-frame specific-force report. Correct conversion would require sensor mounting/orientation conventions. This is not the GCS's native telemetry CSV layout.

Current launch configuration requires >14.71 m/s2 gravity-compensated vertical acceleration for five consecutive accepted baro observations, then height >3 m and positive vertical velocity. The first exported row meeting those numerical height/velocity/acceleration inequalities is at **0.938 s**, with height3.035 m, vertical velocity20.034 m/s, vertical acceleration80.330 m/s2. The first exported row meeting fallback height>15 m and velocity>10 m/s is at **1.328 s**. These are direct column comparisons, **not** execution of the full firmware estimator/state machine or measured onboard detection times; actual observation times and filtering matter.

The profile supports the possibility of launch detection using the existing thresholds; it does not justify weakening the sustained-sample/arming/validity checks or selecting new thresholds from a single perfect trajectory alone. No physical measurements are required to construct an initial simulated test, but sensor noise, timing, dropouts and expected flight variants still need explicit simulation assumptions.

The CSV ends at ground impact with vertical velocity **-6.868 m/s** and no stationary post-landing samples. It cannot establish the existing three-second stationary landing confirmation or the subsequent sixty-second camera-stop timer end to end. Post-deployment angular-rate fields are NaN, and there are no raw BNO gravity/accuracy/timestamp reports, so IMU-quality and orientation handling cannot be qualified from this CSV alone. Simulated recovery deployment at15.884 s is an input simulation event, not proof the current firmware/mechanism deploys at that time. Existing automatic servo deployment remains disabled pending mechanical confirmation.
