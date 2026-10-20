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
