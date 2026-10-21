# FRC 2026 REBUILT — next software/mechanism steps

Working reference: `intake-subsystem` branch. This file is the team's
persistent checklist, not a claim of match-readiness.

## Shooter-side feeder available — autonomous feeding added, not field-tested
- [x] Correction from team: shooter-side feeder WORKS; only the
      INTAKE-SIDE feeder / intake transport is unfinished.
- [x] Existing `FeederSubsystem` is assumed to represent shooter feeder
      CAN 11 leader and CAN 14 inverted follower. **Verify wiring before enabling.**
- [x] Permit teleop A+B feeding in manual % shooter mode once motor RPM
      has been measured and reasonably stable for the safety dwell.
      RPM-calibrated mode still uses target-RPM stability.
- [x] PathPlanner `Auto Shoot` now checks shooter/hood readiness,
      then commands shooter-side feeder for a bounded burst (0.1–2.0 s).
- [x] Require explicit `Shot Tuning → Autonomous Feeder Enabled`
      (defaults OFF), with `Autonomous Feed Duration (s)` (default 0.75).
- [x] Stop feeder when shot command ends, readiness is lost, or auto
      stops/is interrupted; do not command unfinished intake feeder.
- [ ] Confirm CAN 11 and 14 are installed SHOOTER-feeder motors.
- [ ] Build successfully and confirm teleop A+B and autonomous feed
      on the safely supervised physical robot. Code is untested.
- [ ] Tune actual feeding duration, output %, percent-spin stabilization,
      and confirm that a ball actually exits; there is no exit sensor.

## Current shooter controls — percent driver output and optional RPM calibration
- [x] Restore original hold-B manual shooter control as genuine motor OUTPUT %.
- [x] Drive tab uses `Shooter Output (%)`, default 50%, limited to 75%.
      Original motor direction is preserved (50% -> -0.50 on leader 9).
- [x] Preserve follower configuration: 10 same output, 12 inverted.
- [x] Keep RPM encoder telemetry visible in Drive and Shooter tabs.
- [x] Reserve closed-loop RPM setpoint control for `Auto Shot Enabled`
      (teleop) or `Autonomous Use Calibration` (PathPlanner).
- [x] In PathPlanner without calibration, use the driver % slider for
      `Auto Shooter Start`; `Auto Shoot` checks >1 second spin-up and
      >100 measured motor RPM only. It does NOT claim shot accuracy.
- [x] In PathPlanner with calibration, check closed-loop RPM and hood target.
- [x] In percent mode, teleop A feeder is interlocked by measured stable
      flywheel speed; neither A alone nor B alone should command feeding.
- [ ] Build and verify code/real robot; validate percent directions safely.
- [ ] Validate shooter-feeder interlocks and autonomous feed timing on robot.

## Previous RPM closed-loop implementation (retained for automatic calibration)
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
- [x] Feed only when shooter leader reports stable RPM target (RPM mode)
      or sufficiently settled measured rotation (% mode).
- [x] Hold A to request feed, hold B to spin up, preserve both button mappings.
- [x] Stop feed automatically if RPM falls, B is released, or robot is disabled.
- [x] Resume feed once RPM recovers ONLY while the operator continues holding A.
- [x] Publish feeder permission and commanded state in Drive and Shooter tabs.
- [ ] Build the latest branch successfully and deploy.
- [ ] Confirm A alone cannot feed and B alone cannot feed.
- [ ] Confirm A+B waits for stable measured RPM in percent mode
      or calibrated RPM in RPM mode, then feeds.
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
- [ ] Verify readiness, B release, loss of calibration, disabled behavior,
      and switching between open-loop % and closed-loop RPM.
- [ ] Store calibrated shot values persistently in source after validation.
- [ ] Stage 2: derive distance from field pose and verified HUB center,
      then make distance-source selection manual / vision; NOT implemented yet.

## PathPlanner autonomous shooter sequence (added; not yet tested)
- [x] Register `Auto Shooter Start`, `Auto Shoot`, and `Auto Shooter Stop`
      as PathPlanner Named Commands BEFORE building the auto chooser.
- [x] Add `TestAutoWithShooter.auto` as a separate routine, preserving
      the existing `TestAuto.auto` and the original path files.
- [x] Spin up while following `shoot 1` and `shoot 2`; at the end of
      each path, wait at most 2s to check RPM and hood readiness.
- [x] Default to the Drive `Shooter Output (%)` slider; preserve current
      hood angle if shot calibration is incomplete.
- [x] Offer optional `Autonomous Use Calibration` on `Shot Tuning`.
      When enabled, reject invalid shot calibrations and command the
      snapshotted calibrated RPM plus hood setpoint.
- [x] Stop motor on named Stop and finallyDo when the auto completes or
      is interrupted; report status on the `Shot Tuning` tab.
- [x] Auto Shoot now optionally runs the completed shooter feeder
      (CAN 11/14) after readiness and explicit operator arming.
      An actual ball launch has not yet been verified.
- [ ] Verify Java build and inspect PathPlanner GUI named commands / auto chooser.
- [ ] Confirm motor and hood behavior in a supervised mechanism check.
- [ ] Re-time and validate paths: current total route may exceed 20-second auto.
- [x] Code includes gated, time-bounded shooter-feeder actuation
      with stop on RPM/hood readiness loss; still requires hardware tests.
- [ ] After shot calibration: verify calibrated autonomous position control,
      safe shot distances and repeatable accuracy.

## Remaining planned tasks (in priority order)
1. **Intake-to-shooter FUEL flow:** INTAKE-SIDE feeder is not completed.
   When assembled, verify roller direction, transfer continuity, prevent jams
   and overfeeding, and test with multiple game pieces.
2. **Shooter + shooter-feeder + hood integration:** Auto feeding checks
   calibrated hood target when enabled. In manual % mode, hood readiness
   means the encoder is healthy, not that shot trajectory is calibrated.
   Confirm wiring, timer, abort/stop, and actual firing in hardware tests.
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
