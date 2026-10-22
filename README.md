# SSRC 2026 — FRC REBUILT robot code

Active development branch: `hub-pose-fusion-2026`. Java/WPILib 2026 command-based robot with MAXSwerve, navX2, PhotonVision, PathPlanner, shooter, feeder, hood and intake. **Build/test success is not a physical robot calibration or field test.**

## Controls (Xbox controller port 0)

| Input | Teleop behavior |
| --- | --- |
| Left stick | Robot-relative swerve translation by default |
| Left/right triggers | Manual chassis rotation when HUB align is off |
| X (press) | Request HUB aiming; automatic chassis rotation stays locked while a swerve wheel is faulty |
| Y (press) | Toggle robot-relative / field-relative translation |
| A (hold) | Request feeder; flywheel readiness interlock applies |
| B (hold) | Shooter motor output |
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
| Shooter | Shooter and feeder output settings / readiness |
| Shot Tuning | Gated autonomous firing setup and status |
| Hood | Hood command/hold overview |
| Intake / Intake Arm | Roller and arm diagnostics |
| Vision | HUB aim status, safety interlocks, calibration enable flags |
| Vision Diagnostics | Camera measurements, 3-D setup, vision fusion details |
| Drive Diagnostics | navX2, robot velocity and per-module telemetry |
| PathPlanner Diagnostics | Active path and trajectory-following data |
| Hood Diagnostics | Encoder and motion-limit diagnostics |

**Dashboard layouts can retain old widgets from previous code versions.** If a removed widget remains visible, delete it from the saved Shuffleboard layout; code changes alone do not necessarily clear the saved client layout.

## HUB automatic heading alignment

- Teleop X toggles tracking of the **center of the alliance HUB**, not the yaw of an individual AprilTag.
- The preferred `HubRelativeTarget` route uses one or more calibrated 3-D tag measurements plus timestamped odometry. The field-pose solution is only an optional verified fallback.
- The `Hub Direct Geometry Verified` switch starts **false**; only enable after verifying camera mounting and tag geometry on the actual setup.
- PhotonVision device: `PC_Camera`; image orientation is expected to be 180° and physical upward pitch 25°. The code assumes an effective camera transform with roll 0° and pitch -25°. Verify with real data before rotation.
- No moving-shot lead, automatic distance-to-hood calibration, or launcher-offset calibration is claimed.

**Detailed camera and safe validation steps:** [HUB_POSE_FUSION_2026.md](HUB_POSE_FUSION_2026.md).

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

## HUB automatic aiming V2 (2026-10-23)

The new active classes are `HubAimTargetV2` and `HubAimHeadingV2`.
They reconstruct the HUB center using individual AprilTag 3D SolvePnP estimates, verify the data, reject conflicting targets, and calculate wrapped heading with a profiled controller. Legacy classes remain source-only to avoid losing work.

**Current safety interlock:** `Constants.HubAimSafety.ENABLE_AUTO_ROTATION = false`.
Pressing X may request aiming but cannot produce automatic rotation while a swerve module is faulty. Diagnostic heading previews are available in the Vision tab after camera geometry is independently verified. The switch is deliberately not editable in Shuffleboard.

Before considering enabling rotation: fix and validate all four modules and the navX; calibrate/check PhotonVision intrinsics and camera-to-robot 3D pose; check official HUB tag layout, target heading and observed distance with the robot Disabled; inspect X-button and telemetry status. Unit tests are not a physical drivetrain test.
