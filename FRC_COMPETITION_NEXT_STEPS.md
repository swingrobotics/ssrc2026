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

## Remaining planned tasks (in priority order)
1. **Intake-to-shooter FUEL flow:** Verify roller direction, feed continuity,
   avoid jams and overfeeding, and test with multiple game pieces.
2. **Shooter + feeder + hood integration:** Only feed when shooter is at the
   validated target speed and the hood is in its desired safe position;
   add clear abort/stop behavior.
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
- The encoder-based readiness flag currently represents the LEADER only;
  feeder interlock must be implemented and tested separately.
- These changes have NOT been verified by a successful build or robot test.
