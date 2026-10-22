# SSRC 2026 — FRC REBUILT robot code

Active development branch: `hub-pose-fusion-2026`. Java/WPILib 2026 command-based robot with MAXSwerve, navX2, PhotonVision, PathPlanner, shooter, feeder, hood and intake. **Build/test success is not a physical robot calibration or field test.**

## Controls (Xbox controller port 0)

| Input | Teleop behavior |
| --- | --- |
| Left stick | Robot-relative swerve translation by default |
| Left/right triggers | Manual chassis rotation when HUB align is off |
| X (press) | Toggle V3 automatic HUB-center rotation directly during Teleop; press again to stop |
| Y (press) | Toggle robot-relative / field-relative translation |
| A (hold) | Feeder only, forward at Shooter > Feeder Speed (%); release stops feeder |
| B (hold) | Shooter only, SPARK MAX Velocity PID at Shooter > Shooter RPM Setpoint; release stops shooter |
| A + B (hold) | Independent feeder and shooter run together; either button can be released without stopping the other |
| Left bumper (hold) | Intake rollers |
| D-pad up/down (hold) | Intake arm forward/backward, limited runtime |
| Right bumper (hold) | Swerve X-lock |
| Right stick Y | Manual hood adjustment |
| Start | Zero navX heading |

Check the actual direction, wiring and mechanical limits before enabling motion. Intake CAN 16/17, arm CAN 15 (unverified), shooter CAN 9/10/12, feeder CAN 11/14, hood CAN 13, drive/steer CAN 1–8. Hardware configuration is in `src/main/java/frc/robot/Constants.java`.

## Shuffleboard

| Tab | Purpose |
| --- | --- |
| Drive | Driving controls, field pose and autonomous chooser |
| Shooter | Shooter motor RPM setpoint (starts at 0 = disarmed), PID/RPM feedback, feeder speed and 2-second feeder test |
| Shot Tuning | Gated autonomous firing setup and status |
| Hood | Hood command/hold overview |
| Intake / Intake Arm | Roller and arm diagnostics |
| Vision | HUB aim status, safety interlocks, calibration enable flags |
| Vision Diagnostics | Camera measurements, 3-D setup, vision fusion details |
| Drive Diagnostics | navX2, robot velocity and per-module telemetry |
| PathPlanner Diagnostics | Active path and trajectory-following data |
| Hood Diagnostics | Encoder and motion-limit diagnostics |

**Feeder troubleshooting:** `Feeder Speed (%)` is only the requested duty-cycle percentage; moving the slider does **not** turn on the motor. While TELEOP is enabled, hold A or tap `Shooter > Feeder Test (2 s)`, which automatically clears and stops within 2 seconds unless A is held. Verify `Mechanism Diagnostics > Controller Port 0 Connected`, `Raw Button 1 (A)`, `Feeder Speed Setpoint (%)`, `Feeder Command Active`, and `Feeder CAN 11/14 Applied (%)`. This helps distinguish wrong controller mode, slider=0, scheduler state, and CAN/motor wiring. The test does not override Disabled or Autonomous, and A/B remain independent. Keep the feeder clear of game pieces until its direction and actual hardware are verified.

**Dashboard layouts can retain old widgets from previous code versions.** If a removed widget remains visible, delete it from the saved Shuffleboard layout; code changes alone do not necessarily clear the saved client layout.

## HUB automatic heading alignment

- Teleop X toggles tracking of the **center of the alliance HUB**, not the yaw of an individual AprilTag.
- Active V3 uses `HubAimTrackerV3` to estimate one HUB point in **local odometry coordinates** from calibrated AprilTag 3-D measurements. Chassis movement updates its bearing; the target is not moved with the robot. No global field-pose initialization or V2 fallback is needed for V3.
- HUB V3 automatic turning does not require the older `Hub Direct Geometry Verified` switch or a global field pose.
- PhotonVision device: `PC_Camera`; image orientation is expected to be 180° and physical upward pitch 25°. The code assumes an effective camera transform with roll 0° and pitch -25°. Verify with real data before rotation.
- No moving-shot lead, automatic distance-to-hood calibration, or launcher-offset calibration is claimed.

**Detailed camera and safe validation steps:** [HUB_POSE_FUSION_2026.md](HUB_POSE_FUSION_2026.md).

## Shooter velocity control

- B (hold) runs **only** the shooter. CAN 9 controls its built-in encoder to the **motor-shaft RPM** setpoint; CAN 10 mirrors CAN 9 and CAN 12 mirrors it with inverted output. A remains feeder-only; A+B can run simultaneously.
- Set `Shooter > Shooter RPM Setpoint` above 0 before B will spin. It starts at **0 RPM (disarmed)** after deploying the new code. Invalid/nonfinite targets and targets above `MAX_TARGET_RPM = 4500` are rejected. Setpoint is RPM, **not output percent**; the old `Shooter Output (%)` slider is no longer read by code.
- REVLib 2026 SPARK velocity loop: `kP=0.0001, kI=0, kD=0`, `kV = 12 / 5676` **volts per motor RPM** (provisional, must be tuned from data), `voltageCompensation(12.0)`, with independent **20 A per motor** current limits unchanged. The voltage compensation cannot create voltage above the available battery voltage.
- `Shooter > Shooter Actual RPM`, `Shooter Command RPM`, `Shooter RPM Error`, `Shooter Applied (%)` and `Mechanism Diagnostics` individual shooter currents and CAN 9 voltage provide diagnostics for tuning. During physical commissioning use low RPM first with the mechanism cleared, and validate motor direction, actual speed and current with adult/mentor supervision.
- PathPlanner `Auto Shooter Start` uses the same closed-loop RPM target and stores a snapshot of the target for safety. `Auto Shoot` still requires stable robot pose, valid hood, RPM readiness and explicit autonomous feeder arming; an RPM target of zero blocks shooting.

## PathPlanner autos

- Active `src/main/deploy/pathplanner/autos/TestAuto.auto` references existing path files. It includes `Auto Shooter Start` but **no `Auto Shoot`**; do not confuse it with a complete firing sequence.
- `TestAutoWithShooter.auto` is **archived** at [docs/pathplanner-drafts/TestAutoWithShooter.auto](docs/pathplanner-drafts/TestAutoWithShooter.auto), because it references missing `shoot 1` and `shoot 2` paths. Restore these paths and verify trajectories before moving the draft back under `src/main/deploy/pathplanner/autos/`.
- The three registered named commands are `Auto Shooter Start`, `Auto Shoot`, and `Auto Shooter Stop`. Auto Shoot is a stationary sequence and should be placed **between** path commands, not as a moving-path marker.
- Other `.path` files have been preserved; test their geometry before enabling autonomous.

## Build / update locally

From the WPILib 2026 project root:

```powershell
git fetch origin
git switch hub-pose-fusion-2026
git pull --ff-only origin hub-pose-fusion-2026
.\gradlew.bat clean test build
```

These commands do **not** deploy to the robot. Deployment and physical tests are separate operations. Keep the robot disabled while inspecting calibration telemetry.

## Project layout

- `src/main/java/frc/robot/RobotContainer.java`: driver bindings, autonomous named commands, vision updates, dashboard entries.
- `src/main/java/frc/robot/HubRelativeTarget.java`: direct per-tag HUB positioning with outlier rejection.
- `src/main/java/frc/robot/HubCenterAlign.java`: constrained yaw controller and status.
- `src/main/java/frc/robot/subsystems/`: drivetrain and mechanisms.
- `src/test/java/`: geometry / validation regression tests.
- `src/main/deploy/pathplanner/`: active trajectories and autonomous routines.
- `vendordeps/`: robot dependencies.

This project was originally based on REV's MAXSwerve Java template. The original REV/WPILib license files and upstream changelog are retained for attribution and historical reference.

## HUB automatic aiming V3 — experimental (2026-10-23)

- New active `HubAimTrackerV3` uses the known tag mounting geometry to reconstruct the HUB center from one or more visible AprilTags, then stores it in the chassis local odometry frame at image capture time. Every drive/preview tick recomputes the robot-to-HUB bearing from updated odometry. When a tag reappears, the fixed point is corrected.
- A missing tag can be bridged for **at most 0.80 s**; after that the target expires. Disagreeing tag fixes, target teleports, and odometry resets invalidate the target immediately.
- The separate `HUB V3` Shuffleboard tab exposes target lock, tag freshness, local HUB X/Y, robot X/Y, bearing, distance, heading error and validation state. Heading previews are read-only even with `Hub Direct Geometry Verified` OFF.
- V3 automatic turning is directly controlled by X in Teleop. Valid AprilTag HUB geometry supplies a fixed local-odometry center; the heading PID continuously turns toward it as the robot moves. The existing controller limits turning to 45 deg/s and 90 deg/s². If navX is unavailable or the V3 target expires (0.80 s after the last accepted camera fix), automatic rotation commands zero. Manual left-stick translation remains available and is not artificially slowed.
- Old V2 and global pose code are retained for comparison and rollback; delete/consolidate them **only after V3 is successfully verified on the actual robot**.

## HUB automatic aiming V2 (2026-10-23)

The earlier classes were `HubAimTargetV2` and `HubAimHeadingV2`. V2 target is now a reference/diagnostic only, and the generic `HubAimHeadingV2` heading PID is reused by the V3 target.
They reconstruct the HUB center using individual AprilTag 3D SolvePnP estimates, verify the data, reject conflicting targets, and calculate wrapped heading with a profiled controller. Legacy classes remain source-only to avoid losing work.

**V3 operation:** Deploy the code, select the correct Driver Station alliance, enable Teleop, and press X to toggle automatic HUB-center pointing. No separate arming switch or geometry-verification flag is needed. Press X again or Disable to stop. Short tag occlusions use the existing bounded 0.80 s odometry hold; longer loss gives zero automatic turn. The physical camera geometry residual remains a possible source of aim error, not a software activation gate.

Before considering enabling rotation: fix and validate all four modules and the navX; calibrate/check PhotonVision intrinsics and camera-to-robot 3D pose; check official HUB tag layout, target heading and observed distance with the robot Disabled; inspect X-button and telemetry status. Unit tests are not a physical drivetrain test.
