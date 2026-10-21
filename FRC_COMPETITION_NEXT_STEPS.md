# FRC 2026 REBUILT — current configuration & next checks

Branch: `intake-subsystem`. This is a current workflow checklist, not a
claim that robot code has been compiled or field-tested.

## Shooter + hood: clean fixed-RPM / minimum–maximum distance workflow

Only ONE calibration path remains in `RobotContainer`:
a single fixed motor-shaft RPM and two measured distance/hood endpoints.
The old per-distance RPM interpolation and its `ShotCalculator` source/tests
have been removed. The measured-distance `HoodDistanceCalculator` is retained.

### Driver controls (unchanged)
- B held: spin shooter. By default this uses the `Drive` tab's
  `Shooter Output (%)` (50% default, 75% output limit).
- A held: shooter-side feeder (CAN 11 leader, 14 inverted follower),
  interlocked by shooter readiness. Releasing A stops feeding.
- Right stick Y: manually adjust hood. Release: existing position hold.
- LB held: intake rollers (CAN IDs still need assignment).
- D-pad up/down: rack-and-pinion intake arm (CAN 15 currently configured;
  verify physical direction, end limits and motor hardware).
- X: AprilTag auto alignment toggle. Y: field-relative mode toggle.
- RB: swerve X-lock. START: zero navX heading.

### Small Shuffleboard layout
- `Drive`: drive speed, manual shooter output %, actual shooter RPM,
  feeder speed %, feeder active, drive toggles and PathPlanner auto chooser.
- `Shooter`: target/actual motor-shaft RPM, control mode, follower RPMs,
  flywheel readiness. `Feeder` widgets come from `FeederSubsystem`.
- `Hood`: raw duty-cycle encoder, automatic/hold target, position ready,
  safe current/limit signals.
- `Shot Tuning`:
  - `Fixed RPM Enabled` (OFF default)
  - `Fixed Shooter RPM` (0 = invalid; must be measured)
  - `Fixed RPM Valid`
  - `Distance Hood Enabled` (OFF default)
  - `Min Shoot Distance (m)`, `Hood at Min Distance`
  - `Max Shoot Distance (m)`, `Hood at Max Distance`
  - `Hood Calibration Valid`, `Calculated Hood Target`,
    `Hood At Distance Target`, `Shot Setup Status`
  - `Autonomous Feeder Enabled` (OFF default),
    `Autonomous Feed Duration (s)` (0.75 s initial),
    autonomous shot status and last result.
- `Vision`: PhotonVision raw pose and freshness, HUB center distance
  and alliance, optional vision fusion/reset.
- `Debug`: detailed field/vision/drive telemetry.
- `Intake Arm`: arm motor output/current/position; independent of shooter.
- Old `Shot Tuning` widgets may remain in a previously saved Shuffleboard
  dashboard layout. Remove old widgets/rebuild that tab in the client after
  deploying. Removed topics are not recreated by the new code.

### Calibration sequence — not performed yet
1. Keep `Fixed RPM Enabled = false` and `Distance Hood Enabled = false`
   for normal manual percent-output shooting.
2. In a supervised test, pick ONE stable leader motor-shaft RPM. Enable
   `Fixed RPM Enabled` and enter `Fixed Shooter RPM`. Confirm B-hold
   controls a stable measured shaft RPM, within current/output limits.
   Keep `Distance Hood Enabled = false` during these manual hood tests.
3. At that SAME fixed RPM, measure a repeatably successful **minimum**
   shooting distance and raw hood encoder position, then a repeatably
   successful **maximum** shooting distance and raw hood encoder position.
   Input those four measured values; no defaults represent valid shots.
4. Verify PhotonVision uses the 2026 Welded AprilTag layout, camera
   transform and selected alliance; confirm HUB-center distances versus
   physical measurement. HUB X/Y and robot-to-launcher offsets have not
   been physically checked.
5. Only after the previous checks, enable `Distance Hood Enabled`.
   Within the measured min/max range, B-hold / autonomous spin-up tracks
   the calculated hood target. Fixed RPM stays constant; never extrapolate.
6. Interlocks require measured shooter RPM stability and actual hood
   arrival before distance-mode feeding. Failed/stale camera poses, out-
   of-range requests and invalid calibration prevent distance-mode feeding.
   If the sample two-point straight-line relation misses shots in the
   middle, gather more real calibration points and replace the model.

### Unfinished / unverified
- [ ] Run `.\gradlew.bat test` and `.\gradlew.bat build` on the WPILib
      2026 project, repair errors before deploying.
- [ ] Verify CAN IDs, direction, real follower RPM, shooter current,
      leader RPM control gains and hood travel/limit behavior.
- [ ] Verify full robot hardware readiness, battery sag and shot
      consistency at fixed RPM after repeated fuel impacts.
- [ ] Verify fixed RPM mode B hold stops on B release and Disable;
      percent mode remains unchanged with both toggles OFF.
- [ ] Verify measured HUB distance freshness and out-of-range blocking.
- [ ] Verify manual hood override with right stick and target hold after
      losing PhotonVision estimates.
- [ ] Verify `Auto Shoot` is SEQUENTIAL between paths (NOT a driving event
      marker), waits for stopped chassis (<=0.10 m/s, <=0.15 rad/s held 0.25s),
      and stops feeder on loss of readiness or interruption.
- [ ] Verify `TestAutoWithShooter` fits match auto duration and that
      both shooting paths finish at Goal End Velocity = 0 m/s.
- [ ] No fuel exit sensor yet; a completed feeder timer does not guarantee
      every FUEL was launched.
- [ ] Intake-to-shooter transfer feeder unfinished; intake rollers require
      real CAN IDs. Intake arm CAN 15 must be verified with end-stop safety.
- [ ] Continue HUB center auto aiming, field pose/vision fusion validation,
      trench clearance, and full autonomous match testing.

Note: `Fixed Shooter RPM` represents shooter leader MOTOR SHAFT RPM,
not flywheel surface speed if belts/gearing are used. Current nominal
hood encoder bounds remain 0.196..0.410; do not move beyond safe real limits.
