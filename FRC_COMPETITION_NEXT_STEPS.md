# FRC 2026 REBUILT — next software/mechanism steps

Working reference: `intake-subsystem` branch. This file is the team's
persistent checklist, not a claim of match-readiness.

## Active — Shooter RPM closed-loop implementation
- [x] Replace shooter open-loop duty-cycle command with Spark MAX velocity RPM control.
- [x] Preserve shooter CAN IDs 9 / 10 / 12 and follower direction setup.
- [x] Preserve hold-B to run and release-B to stop.
- [x] Add configurable target RPM, safe maximum RPM, PID, and feedforward constants.
- [x] Show requested/actual leader RPM, follower RPMs, error, running and ready statuses.
- [ ] Build on team's WPILib 2026 development machine and resolve any errors.
- [ ] Confirm actual motor/encoder directions and safe mechanical operation.
- [ ] Tune kP and kV from recorded RPM/current/voltage data.
- [ ] Verify motor 10/12 measured RPM matches intended relative directions;
      current followers follow OUTPUT rather than controlling RPM independently.
- [ ] Establish actual match-ready RPM; current default 1500 RPM is a test value.
- [ ] Verify recovery time after a shot and adequate current/voltage under load.

## Shooter / feeder RPM interlock (code added; hardware validation pending)
- [x] Feed only when shooter leader reports stable target RPM.
- [x] Hold A to request feed, hold B to spin up, preserve both button mappings.
- [x] Stop feed automatically if RPM falls, B is released, or robot is disabled.
- [x] Resume feed once RPM recovers ONLY while the operator continues holding A.
- [x] Publish feeder permission and commanded state in Drive and Shooter tabs.
- [ ] Build the latest branch successfully and deploy.
- [ ] Confirm A alone cannot feed and B alone cannot feed.
- [ ] Confirm A+B waits for RPM then feeds; verify RPM drop suspends feeding.
- [ ] Confirm release A/B and Disable each stop feeder promptly.
- [ ] Tune readiness tolerance/stability window to measured firing recovery.
- [ ] Verify follower RPM and any jam conditions in a safely supervised test.
- [ ] Add hood-at-target interlock after hood target setpoints are implemented.

## Manual-distance shot tuning — stage 1 (code implemented, untested)
- [x] Add validated two-point distance lookup with linear interpolation (no extrapolation).
- [x] Keep manual RPM and manual hood operation as the default (Auto Shot Enabled = false).
- [x] Add Shot Tuning tab: manual distance, two measured calibration points,
      calculated motor RPM/hood encoder position, readiness and status.
- [x] Use B to command calculated RPM and bounded hood target while in auto mode.
- [x] Preserve right-stick priority; using the stick during auto latches hood
      manual override until B is released and pressed again.
- [x] Cancel old hood goal and hold present encoder position when auto ends.
- [x] Reject missing, invalid or out-of-range calibration; block shooter spin-up
      when the hood encoder is invalid / calibration mode is enabled.
- [x] Add pure JUnit tests for interpolation and invalid inputs.
- [ ] Run `./gradlew test` and Build Robot Code on the WPILib 2026 computer.
- [ ] Validate hood encoder target motion / min-max and operator override
      in a supervised, safe mechanism setup.
- [ ] Obtain TWO real shot measurements (distance, MOTOR RPM, raw hood encoder).
      Defaults (0 RPM, -1 hood) deliberately prohibit automatic motion.
- [ ] Enter calibration in Shuffleboard; check output on disabled robot first.
- [ ] Verify readiness, B release, loss of calibration, and disabled behavior.
- [ ] Store calibrated shot values persistently in source after validation.
- [ ] Stage 2: derive distance from field pose and verified HUB center,
      then make distance-source selection manual / vision; NOT implemented yet.

## Remaining planned tasks (in priority order)
1. **Intake-to-shooter FUEL flow:** Verify roller direction, feed continuity,
   avoid jams and overfeeding, and test with multiple game pieces.
2. **Shooter + feeder + hood integration:** Shooter-to-feeder RPM gating is
   implemented. Hood target/angle confirmation is NOT yet connected;
   add abort/stop behavior for fully automated firing.
3. **Distance-based shooting calibration:** Record hub distance, actual
   flywheel RPM, hood encoder positions/angles and accuracy; implement
   data-backed setpoint interpolation.
4. **Hub-centered aiming:** Target the actual alliance HUB rather than an
   arbitrary AprilTag; verify camera mount transform and fuse vision safely.
5. **Match autonomous:** Make paths fit within the 20-second autonomous period,
   attach intake/shoot events and validate pose resets and real-world obstacles.
6. **Final match/system checks:** CAN ID conflicts, drivetrain limits,
   navX/PhotonVision reliability, current/battery behavior, physical
   robot envelope, TRENCH/BUMP clearance, and legal firing conditions.

Notes:
- Shooter RPM telemetry reports motor-shaft RPM, not wheel RPM if geared.
- The feeder interlock checks the leader RPM only. Followers track output
  without independent closed-loop control. Physical testing is still needed.
- These changes have NOT been verified by a successful build or robot test.
