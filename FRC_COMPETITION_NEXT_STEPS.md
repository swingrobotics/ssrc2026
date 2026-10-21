# FRC 2026 REBUILT — active software state

Branch: `intake-subsystem`. This document describes code status only,
not a successful robot build, drive test or calibrated shooting system.

## Current priority: multi-AprilTag HUB CENTER aiming

- `HubCenterAlign.java`: computes rotation toward the alliance HUB
  CENTER field coordinates rather than the yaw of a single AprilTag.
- Requires a valid PhotonVision coprocessor multi-tag field-pose result
  with >=2 distinct tags actually USED in the pose solution from the
  correct alliance's HUB. A single tag NEVER enables center turning.
- Red HUB tag IDs: 2,3,4,5,8,9,10,11.
  Blue HUB tag IDs: 18,19,20,21,24,25,26,27.
- Uses the vetted camera field pose and gyro heading delta to reduce
  latency; translation is updated on each new valid camera pose.
- Automatically commands ZERO rotation after stale frames (>0.30s),
  unknown/wrong alliance or invalid gyro/speed telemetry.
- Initial turn P=1.8, D=0.18, max normalized turn=0.25,
  heading tolerance=1.5 deg and settle time=0.20s:
  all preliminary tunings requiring safe real-robot tests.
- X toggles HUB CENTER align. During alignment, the left stick continues
  to control translation. Triggers control manual rotation when X is OFF.
- `Vision` Shuffleboard tab displays fresh HUB tag count, heading
  error, alignment status, alliance and raw vision pose.
- Verify 2026 Welded field version, HUB center constants, camera optical
  mounting pose, navX direction and actual shooter launch axis (+X
  currently assumed) before testing powered automatic rotation.
- `Fusion Enabled` remains OFF by default and independent of aiming.

## Shooter / feeder / hood after cleanup

- All shooter motor commands (B teleop and PathPlanner auto) are
  DIRECT OUTPUT PERCENT, not target RPM or variable RPM calibration.
- Drive: `Shooter Output (%)` from 0 to 75, default 50;
  `Feeder Speed (%)` separate and unchanged.
- Shooter subsystem CAN 9 leader, 10 same-direction follower,
  12 opposite-direction follower.
- Internal shooter encoder measurement is used ONLY to decide
  if actual rotation is reasonably settled before feeding;
  it is not a velocity PID controller or RPM setpoint.
- Shooter feeder CAN 11 leader and CAN 14 inverted follower.
  Teleop A requests feeding, and feeder stops if shooter
  readiness is lost or robot is disabled.
- Hood CAN 13 and DIO 0 retains right-stick manual movement and
  current-position hold on release. No distance-based target
  is commanded in the CURRENT RobotContainer.
- Old RPM and uncalibrated hood-distance controls were removed from
  RobotContainer/Shot Tuning. Do not enable abandoned dashboard widgets.
- NamedCommands preserved: `Auto Shooter Start`, `Auto Shoot`,
  `Auto Shooter Stop`. The auto shot waits for stationary chassis
  and shooter readiness, with explicit feeder arm OFF by default.
- PathPlanner `TestAutoWithShooter.auto` and existing paths retained;
  shots are sequential between paths and not while-driving event markers.

## Before use

- [ ] Run Windows WPILib 2026 `.\gradlew.bat test` and `.\gradlew.bat build`.
- [ ] Validate navX yaw sign and shooter/front orientation while lifted,
      then test small align rotations with team supervision.
- [ ] Confirm PhotonVision uses the exact same 2026 REBUILT Welded
      tag layout, and calibrate camera roll/pitch/yaw and gyro alignment.
- [ ] Confirm visible HUB tag IDs and `measurement.targetsUsed` reflect
      at least two valid HUB targets from a single pose estimate.
- [ ] At known field locations, compare reported robot pose and HUB
      center direction to measured truth, for both alliances.
- [ ] Test lost tag frames, unknown alliance, disabled robot, and
      movement with left stick; reject unstable or opposite-direction turns.
- [ ] Check shooter CAN IDs, original negative output direction,
      follower directions, current draw, feeder permissive behavior,
      hood limit and hold torque with adult/team supervision.
- [ ] Keep `Autonomous Feeder Enabled` OFF until all motor and
      mechanical checks pass. Fuel-exit sensor is not present.
- [ ] After center aim works, separately measure a fixed output percent
      and near/far successful shooting distances and hood positions.
- [ ] Intake roller CAN IDs are not assigned, and intake arm
      CAN 15/end-stop mechanism requires verification.

Important: An `ALIGNED` signal confirms a vision-derived heading
within its tolerance, not shot accuracy or a calibrated hood trajectory.
