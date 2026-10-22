# Active V3 HUB aiming (2026-10-23)

The active HUB auto-aim has one driver control: **X in Teleop** toggles
automatic chassis rotation to the stored HUB center. No separate dashboard
arm, calibration approval, global field localization, or V2 fallback is
required to operate it.

The existing `HubAimTrackerV3` computes the HUB center from visible AprilTags
with their known placement on the 2026 HUB, transforms it into the rover's
local odometry frame at camera capture time, and corrects it when tags are
observed. When the chassis translates or rotates, it keeps this single
target in odometry and recomputes heading. The existing
`HubAimHeadingV2` PID is still limited to 45 deg/s and 90 deg/s².

A lost HUB estimate expires 0.80 seconds after its last valid tag fix.
When navX is not ready or the HUB lock has expired, commanded automatic yaw
is zero. Disable returns the driver to the non-aiming mode. Driver translation
remains controlled by the left stick at the existing Drive Speed (%) scale,
while shooter, feeder, hood, intake and PathPlanner are unchanged.

The `HUB V3` tab is informational, not an arming interface. The previous
V2 diagnostic / global pose code remains preserved for later cleanup.
Robot-mounted camera geometry may still require calibration to improve
physical shooting accuracy.

---
# REBUILT HUB center alignment — experimental branch

Branch: `hub-pose-fusion-2026` based on `intake-subsystem`
(commit `0aa15352b6b6ea7f2c99314270354fdf974298ca`).
The parent branch is unchanged. Development does NOT mean this is
calibrated or proven accurate on a real robot.

## Camera mounting correction / validation — 2026-10-22

**Reported physical setup:** the USB camera housing is upside-down (physical
roll 180 degrees), its optical axis points 25 degrees UP relative to robot
forward, and **PhotonVision Input > Orientation = 180 degrees** is already
configured. PhotonVision rotates input frames BEFORE vision processing.

**The existing constants were already consistent with that description under
the effective upright processed-image coordinate-frame assumption.**
`ROBOT_TO_CAMERA` has roll 0 degrees, pitch -25 degrees (negative WPILib
pitch means the camera optical axis points UP), yaw 0 degrees.
We have intentionally **not** invented a new pitch or applied an additional
180-degree roll, which could double-correct the orientation.

This assumption must still be checked with REAL 3-D camera observations:
PhotonVision Orientation UI rotation does not give us a live readback of the
pose-coordinate convention through PhotonLib. Existing camera translation
X=0.31732884884m, Y=0m, Z=0.40394608582m was reported earlier, not
physically re-measured in this update. Real-world camera or tag alignment
can be off even when geometric unit tests pass.

### Live mount check with real 3D AprilTag detection

1. Keep PhotonVision camera calibration at the AprilTag pipeline resolution.
   Input > Orientation should be **180 degrees** as already configured.
   Enable **Always Do Single-Target Estimation** when using MultiTag. Ensure
   the individual camera-to-tag transform is nonzero.
2. Robot stays DISABLED, on a LEVEL floor. Use an AprilTag mounted at the
   EXACT OFFICIAL 2026 field-layout height; **an arbitrary practice-tag
   height invalidates the comparison**, even if the tag ID matches.
3. View the `Vision` and `Vision Diagnostics` Shuffleboard tabs:
   - `Vision Diagnostics` — `Camera Expected PV Orientation (deg)`: expected UI setting (180),
     NOT an actual readback from PhotonVision.
   - `Vision Diagnostics` — `Camera Config Roll/Pitch/Yaw (deg)`: effective `ROBOT_TO_CAMERA`
     settings (0/-25/0).
   - `Vision Diagnostics` — `Camera Mount Tag ID`: which observed tag was selected.
   - `Vision Diagnostics` — `Camera Mount Tag Z (m)`: reconstructed target height above robot
     floor, assuming the robot is level and its origin is at ground Z=0.
   - `Vision` — `Camera Mount Tag Height Error (m)`: reconstructed tag Z minus the
     official WPILib tag height. Compare while slowly changing camera
     position across DISTANCES with the robot safely DISABLED.
   - `Vision` — `Camera Mount Check Status`: whether the height residual is small or
     large; it does not by itself validate lateral position, yaw, or roll.
4. Compare tag X/Y and range with physically measured positions. A
   nonzero error that grows with tag distance suggests a pitch/roll
   convention issue, bad camera intrinsics, incorrect camera height or
   wrong tag mounting height. Verify the entire 3-D coordinate convention
   before modifying numeric extrinsics.
5. Only after physical geometry has been validated enable
   `Hub Direct Geometry Verified` and test slow yaw movement on a
   secured robot. Do NOT enable this safety gate merely because the
   constants match the reported angles.

Reference:
- https://docs.photonvision.org/en/latest/docs/pipelines/input.html
- https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
- https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html

## Current HUB aiming — 2026-10-22 update

**Direct aiming is now the primary Teleop X-button target.** This follows
[FRC 4107's PhotonVisionSystem](https://github.com/Storm4107/2026-Rebuilt/blob/main/src/main/java/frc/robot/vision/PhotonVisionSystem.java)
instead of requiring global field coordinates.

The processing chain for **one or more** visible HUB tags is:

1. PhotonVision 3-D `bestCameraToTarget` for each visible official HUB ID.
   MultiTag solution is NOT required. Known alliance comes from Driver Station;
   when unavailable, choose the color of the strongest valid HUB observation.
2. WPILib's 2026 Welded AprilTag tag Pose3d and the per-alliance HUB center
   provide a unique 3-D **tag-to-HUB-center** Transform3d for every tag ID.
3. `HubRelativeTarget` chains robot-to-camera, camera-to-tag, tag-to-HUB.
   It calculates the horizontal center relative to the robot, not the tag yaw.
4. Odometry pose is sampled at the camera image timestamp via
   `TimeInterpolatableBuffer<Pose2d>`; HUB coordinates are expressed in
   the *local* odometry frame, including latency compensation. **No initial
   global field pose is needed to aim while the HUB is visible.**
5. Individual single-tag observations with bad ambiguity/range are rejected.
   When two or more tags are visible, a spatial-consensus filter rejects
   inconsistent centers before weighting and smoothing valid estimates.
6. Teleop X toggles direct HUB yaw control using the existing profiled PID;
   the left stick keeps driving. If direct target disappears, a globally
   initialized, recently fused pose may serve as a secondary fallback.
   When neither is trustworthy, the rotation command is zero.

PhotonVision `Output` requirements: calibrate the camera with correct
resolution, enable 3-D, and when `Do Multi-Target Estimation` is active also
enable **`Always Do Single-Target Estimation`**. Without the latter,
`getBestCameraToTarget()` can be zero even when two or three tags are visible.
The local direct path needs usable individual SolvePnP transforms (not merely
2-D target boxes or the camera's MultiTag pose).

### First robot test of DIRECT aiming (practice HUB allowed)

1. Check `ROBOT_TO_CAMERA` XYZ and pitch/yaw/roll against the actual install.
   Ensure official HUB tags are physically mounted in their documented
   locations/orientations. If the rig uses different tag placements, change
   the tag-to-HUB model instead of falsifying a field pose.
2. In PhotonVision, turn on **Always Do Single-Target Estimation** along
   with 3-D and camera calibration, then confirm `bestCameraToTarget` is
   nonzero; `Pose Solver=SingleTag` is now perfectly acceptable.
3. In Shuffleboard **Vision**, set `Hub Direct Geometry Verified` true
   *after checking the real geometry*. **Do not** need `Reset Pose (Disabled)`,
   `Pose Initialized=true` or `Field Camera Verified=true` for local direct.
4. Check `HUB Direct Status = DIRECT HUB TARGET VALID: 1 TAG(S)` (or more),
   `HUB Direct Fresh=true`, and recognized `HUB Direct Best Tag ID`.
5. Enable TELEOP, press X once with the drivetrain safely controlled and
   first test rotation direction at a conservative speed. When aiming,
   `HUB Aim Source = DIRECT TAG 3D`.
6. When no usable individual 3-D transform exists, the direct status gives
   `NO VALID HUB 3D TAG: ENABLE ALWAYS DO SINGLE-TARGET ESTIMATION`.
   Other reasons include stale timestamps and inconsistent hub measurements.
7. To use field-relative fallback, additionally perform the original global
   calibration, multi-tag pose initialization and `Fusion Enabled` steps.
   A global pose still requires matching the actual field JSON. A separate
   practice HUB with official tag geometry can work via direct path without
   an official full-field layout.

Safety notes: Rejecting uncalibrated geometry is intentional. The robot
remains untested with real motors. No projectile-flight lead, automatic hood
angle, automatic shooter percentage or moving-shot compensation was added.
All original driver A/B/Y/bumper, feeder/hood/intake, PathPlanner controls
remain unchanged.

## Architecture

1. PhotonVision `PC_Camera` reports calibrated 2026 **Welded** field poses.
2. `PhotonPoseEstimator.estimateCoprocMultiTagPose` is preferred.
   If unavailable, validated `estimateLowestAmbiguityPose` single-tag
   observations may improve an **already initialized** fused field pose.
   Any visible field tag may contribute: HUB tag count is NOT a requirement.
3. `VisionPoseValidator` gates camera timestamps, pose bounds, Z/roll/pitch,
   tag distance, single-tag ambiguity, gyro readiness, robot motion and
   disagreement from predicted chassis odometry.
4. Adaptive translation measurement uncertainty uses target distance and
   tag count; camera heading uncertainty is very large (1000 rad) so
   navX gyro dictates short-term rotation. This follows the vision
   covariance approach in FRC 5024/6328 and FRC 6940.
5. `DriveSubsystem.SwerveDrivePoseEstimator` updates wheel+navX each 20ms
   and fuses each vetted, timestamped camera field pose through
   `addVisionMeasurement`. Its `getPose()` is the single source of
   truth for both path-following and hub aiming.
6. `HubCenterAlign` gets the fused robot pose every driver period and
   computes `atan2(hubY - launcherY, hubX - launcherX)`, adjusted for
   the physical launcher direction and the operator trim.
7. A `ProfiledPIDController` with wrapped heading, angular velocity
   and acceleration limits commands ROBOT rotation. The left stick retains
   direct translation. The X button toggles aiming. Autonomous PathPlanner,
   shooter percent output, manual hood, intake and feeder are preserved.

## Required field setup BEFORE automatic rotation

These are essential interlocks, not optional tuning steps:

1. Confirm PhotonVision Output -> **Do Multi-Target Estimation** is ON.
2. Confirm camera calibration was performed at the actual stream resolution.
3. Confirm PhotonVision device JSON AND `Constants.VisionConstants.FIELD_LAYOUT`
   match your actual 2026 Welded vs AndyMark field. The current code uses
   **Welded**, so do not check `Field Camera Verified` on a differently
   arranged practice field. Off-field AprilTag props DO NOT define a global
   field pose unless their positions match the configured JSON exactly.
4. Physically measure camera X/Y/Z, optical yaw/pitch/roll.
   Current values: forward 0.31732884884m, left 0m, up 0.40394608582m,
   roll 0, pitch -25deg, yaw 0. These MUST match actual mounting.
5. Check alliance from DriverStation. HUB centers are currently provisional
   X/Y constants: blue (4.625, 4.033)m, red (11.920, 4.033)m.
   Measure the actual launcher offset relative to robot X/Y; currently
   set to (0,0). The launch axis yaw currently assumes robot +X.
6. With the robot DISABLED and gyro ready, ensure a VALID MULTITAG field
   pose is visible. On `Vision` Shuffleboard tab turn on
   **Field Camera Verified** ONLY after steps 1-5 are checked.
7. Toggle **Reset Pose (Disabled)** ON. It clears automatically.
   Check `Pose Init Status = INITIALIZED FROM VERIFIED MULTITAG POSE`
   and fused X/Y/heading against surveyed truth.
8. Keep **Fusion Enabled** ON (default). Camera pose acceptance then
   continuously corrects odometry. Verify accepted-frame counter increases
   and rejected-frame counter does not increase rapidly.
9. With the robot safely secured under adult/team supervision, initially
   verify **direction/sign of rotation** and then very low-speed field trials.
   X toggles auto-aim. Aiming requires verified pose, healthy gyro,
   known alliance and vision accepted in the previous 1.0 seconds.
   No recent vision -> zero commanded rotation, even if odometry continues.
10. `Hub Aim Trim (deg)` starts at 0 and clamps to +/-15deg.
    A trim is NOT a repair for wrong camera calibration/field geometry.
    Record validated trim and measure launcher yaw mechanically.

## Compact V2 HUB tag diagnostics (easy to read)

Open the separate `HUB Tag Debug` Shuffleboard tab. Its `Tag 25 Reason` and
`Tag 26 Reason` widgets show a short gate result **first**, with raw
`Ambiguity`, `Range (m)`, and individual camera-to-tag X/Y/Z (m) in
separate number widgets. `HUB Valid Candidates` counts accepted per-tag
measurements before multi-tag consistency checking. `HUB Last Rejection`
shows the last overall failed measurement reason. The two full raw diagnostic
strings were moved out of the crowded `Vision Diagnostics` tab and into this
new tab, under `Tag 25 Full Details` and `Tag 26 Full Details`. If a tag
is not detected, its compact gate is `NOT SEEN` and numeric fields are NaN.

The compact gate is a display-only summary of the same existing decisions;
no ambiguity/geometry filtering, velocity control, or automatic rotation
interlocks were changed.

## Live V2 direct HUB tag diagnostics

In `Vision Diagnostics`, `HUB Tag 25 Diagnostics` and `HUB Tag 26 Diagnostics`
show each tag's ID, inferred alliance, raw pose ambiguity, camera-to-tag
X/Y/Z (m), 3D range (m), and the actual acceptance gate result.
`HUB Valid Candidate Count` counts individually accepted tag poses **before**
cross-tag consensus. `HUB Direct Last Rejection` retains the latest rejected
frame reason even after a later successful frame. A tag not detected in the
latest frame displays `NOT_SEEN_THIS_FRAME`.

- `INVALID_AMBIGUITY`: PhotonLib reports an ambiguity outside 0..0.20,
  including -1 (not a valid single-target ambiguity). MultiTag solving can
  succeed even when the independent single-target gate rejects a pose.
- `INVALID_TRANSFORM`: the individual camera-to-tag 3D range is zero,
  nonfinite, missing, or out of the configured range.
- `INVALID_HUB_GEOMETRY`: converting the individual 3D tag pose to a
  robot-relative HUB center failed the existing geometric bounds.
- `ALLIANCE_MISMATCH`: Driver Station alliance does not match the tag's HUB.
- `ACCEPTED`: the tag is part of the current agreed HUB estimate.
  `CONSENSUS_REJECTED` means individual candidates disagree about the
  position of the physical HUB.
- `STALE_FRAME_OR_MISSING_ODOMETRY`: the capture timestamp or capture-time
  odometry was unavailable/invalid.

This change adds **telemetry only**. It does not relax ambiguity/geometry
filters, change the aim calculation, or unlock automatic chassis rotation.
Continue to verify camera mounting and actual field tag geometry separately.

## Failure/status inspection

`Vision Gate Status` shows why an observation was rejected:
- `STALE OR FUTURE FRAME`: check camera time sync/network.
- `VISION/ODOMETRY POSITION DISAGREE`: verify layout, camera mount,
  odometry/pinion/wheel size or manual seed.
- `VISION/NAVX HEADING DISAGREE`: verify navX polarity and 3D yaw.
- `ROBOT MOVING TOO FAST FOR VISION`: stabilize before calibrating.
- `WAITING: DISABLED POSE INITIALIZATION`: no trusted global origin yet.
- `WAITING: VERIFY FIELD JSON/CAMERA MOUNT`: operator confirmation off.
- `FUSION DISABLED`: dashboard toggle off.

`HUB Align Status` explains refusal to turn: NAVX not ready,
pose not initialized, camera vision stale, alliance unknown,
invalid position or HUB too close.

### If 2-3 AprilTags are visible but `Pose Solver` says SingleTag

PhotonVision documentation: `estimateCoprocMultiTagPose(result)` needs a
**coprocessor MultiTag result**, not merely multiple target detections.
The robot reports this distinction on the `Vision Diagnostics` tab:

1. `Raw Detected Tag Count`: number of targets in the latest camera frame.
2. `Raw Coproc MultiTag Present`: whether PhotonVision transmitted
   `result.getMultiTagResult()`; if false despite 2-3 targets, check
   3D calibration, Output -> Do Multi-Target Estimation, tag IDs, and
   exact same field JSON on the camera and robot (2026 Welded/AndyMark).
3. `PhotonPose MultiTag Solved`: whether PhotonLib
   `estimateCoprocMultiTagPose(result)` returned a robot pose. If a raw
   MultiTag result is present but this is false, inspect RobotToCamera,
   version compatibility and the pose estimator rather than retuning PID.
4. `Pose Solver`: reports MultiTag only when the robot estimator actually
   returned a multi-tag pose; otherwise falls back to SingleTag.
5. `Pose Avg Tag Range (m)`: now calculated using global positions of
   known fiducials and the field camera pose, not the individual target
   `bestCameraToTarget` transform, which PhotonVision can omit during
   MultiTag unless Always Do Single-Target Estimation is enabled.
6. `Vision Gate Status`: details whether a solved MultiTag pose passed
   the physical pose/range/gyro checks. A valid MultiTag result is not
   guaranteed to pass every verification gate.

Do NOT turn on `Field Camera Verified` unless the layout and physical
mount/extrinsics have been checked. Multiple randomly placed practice
AprilTags are insufficient for official global field localization.

### If X is pressed but chassis does not rotate

On the `Vision` tab inspect these in order (the button and telemetry details identified below are on `Vision Diagnostics`):
1. `HUB X Button Pressed` (Vision Diagnostics): true **while holding X** proves controller mapping.
2. `Hub Center Align` (Drive): true **after one press** proves the toggle fired.
3. `HUB Teleop Enabled` (Vision Diagnostics): must be true; Disabled and autonomous do not
   execute the manual drive/default HUB heading controller.
4. `HUB Turn Blocker` (Vision): displays the first unmet interlock. Do not disable
   interlocks just to make wheels move.
5. `HUB Turn Command (%)` (Vision): nonzero means aim computes a rotation command.
   If it is nonzero and chassis remains still, check drive speed percentage,
   manual trigger turn, swerve wheel steering/motor state and driver station
   output, with wheels safely elevated under team supervision.
6. `HUB Error (deg)` (Vision): may be near zero if already aligned, where 0% is
   expected. If camera localization is wrong, do not use trim as a fix.
7. `Vision Gate Status` (Vision): rejected PnP frames prevent fusion. For any
   stop status check field JSON, camera extrinsics, tag distances, navX.
8. `Pose Init Status` (Vision): first global pose requires a fresh multitag
   solution, camera verification, and one disabled-only Reset Pose toggle.
   Start-button navX zeroing is NOT field-pose initialization.

`HUB Turn Command (%)` measures the normalised HUB controller output
before the independent Drive Speed (%) scaling, not measured wheel rotation.

## Tests

Run from project root in the 2026 WPILib VS Code environment:

```powershell
git switch hub-pose-fusion-2026
git pull --ff-only origin hub-pose-fusion-2026
.\gradlew.bat clean test
.\gradlew.bat build
```

The JUnit tests verify field-to-HUB geometry, launcher offsets, trim,
left/right aim while moving, initial MultiTag requirement, single-tag
fallback, pose outliers, invalid tags and uncertainty scaling.

## Sources actually used

- PhotonVision 2026 MultiTag:
  https://docs.photonvision.org/en/v2026.3.3/docs/apriltag-pipelines/multitag.html
- PhotonLib pose estimator:
  https://docs.photonvision.org/en/v2026.3.2/docs/programming/photonlib/robot-pose-estimator.html
- WPILib 2026 PoseEstimator:
  https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/PoseEstimator.html
- FRC 5024 adaptive covariance:
  https://github.com/frc5024/Rebuilt/blob/main/src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java
- FRC 6328 timestamped odometry/vision fusion:
  https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/RobotState.java
- FRC 6940 field-relative HUB targeting:
  https://github.com/Team6940/2026Rebuilt/blob/game-dev/src/main/java/frc/robot/subsystems/Drive/Drive.java
- FRC 135 profiled heading controller:
  https://github.com/Team135BlackKnights/2026-Rebuilt/blob/drive-CORE/src/main/java/frc/robot/commands/drive/AimToPose.java

Important: This project DOES NOT apply projectile travel-time / lateral
lead for shooting while translating. It points the launch axis toward
the stationary HUB center. Shooting-on-the-move trajectory compensation
requires separate calibration and validation.

## Maintenance notes — 2026-10-22

- Shuffleboard operator views are now separated from detailed telemetry: `Drive`, `Shooter`, `Shot Tuning`, `Hood`, `Intake`, `Intake Arm`, `Vision` and the dedicated `Vision Diagnostics`, `Drive Diagnostics`, `PathPlanner Diagnostics`, `Hood Diagnostics` tabs. Historical dashboard layouts may still retain locally saved widgets after a code change.
- The former `TestAutoWithShooter.auto` was archived under `docs/pathplanner-drafts/` because its `shoot 1` and `shoot 2` path files are missing. The original draft is preserved; it is not selectable as a deployed autonomous routine until complete. `TestAuto.auto` remains and its paths are present.
- Intake roller CAN IDs 16 and 17 were restored from the former `shooter-integration` branch. The arm CAN 15 configuration and physical directions/limits still require robot-level verification.
- The current `HoodDistanceCalculator` and `HubDistanceCalculator` are isolated tested geometry utilities; neither is currently wired into automatic hood positioning. Do not treat their existence as an enabled distance-to-hood feature.

## HUB aim V2 rebuild (2026-10-23)

Active targeting: `HubAimTargetV2` (individual AprilTag 3D measurement to official HUB center, timestamped local odometry, consensus/outlier rejection, 0.25 s freshness), `HubAimHeadingV2` (wrapped HUB yaw, 45 deg/s turn limit, 90 deg/s² acceleration limit, stable alignment and dashboard geometry preview).
The older `HubRelativeTarget` and `HubCenterAlign` implementations are retained in source and not used by RobotContainer. Global field pose fusion, PathPlanner and shooter/feeder/hood/intake logic remain unchanged.

**CRITICAL:** One swerve wheel is reported faulty. `HubAimSafety.ENABLE_AUTO_ROTATION=false` blocks chassis auto rotation even if X is pressed or dashboard verification is enabled; no dashboard override. The Vision tab reports the lock and calculates geometry preview only. No motion validation has been performed.

Before review of this code lock: repair drive+turn module; verify gyro/pose and Swerve state in stationary and slow tests supervised by the team; verify PhotonVision camera physical position, orientation and single-target 3D SolvePnP; use the same Welded/AndyMark tag layout as the real field; compare target angle and distance against physical measurements while Disabled. The expected `Input Orientation=180 deg` does not guarantee that the 3D camera pose is correct.
Design references: Storm4107/2026-Rebuilt; Team6940/2026Rebuilt; WPILib ProfiledPIDController; PhotonVision individual camera-to-target and MultiTag documentation.

## V3 trial: fixed HUB point rather than full-field localization

The active aim target is now `HubAimTrackerV3`. It uses a valid individual
tag's 3D translation AND rotation, plus its known pose on the official HUB,
to compute the physical HUB center in robot-relative coordinates. The robot
odom pose sampled at camera capture time moves that target into a fixed local
odometry frame. This fixed point is retained while the chassis translates
and turns. Fresh tag sightings gently correct it. With a tag temporarily
hidden, the point stays valid for at most **0.80 s**; otherwise V3 fails closed.

Open `HUB V3` in Shuffleboard for `V3 Tracking State`, `V3 Target Locked`,
`V3 Camera Fresh`, `V3 HUB Odom X/Y`, `V3 Robot Odom X/Y`,
`V3 Bearing (deg)`, `V3 Distance (m)`, `V3 Heading Error (deg)`,
and `V3 Last Rejection`. Check target X/Y remains nearly fixed while robot
odometry changes as the chassis moves, and compare predicted bearing with
the physical HUB at more than one robot pose.

V3 read-only angle preview is available while `Hub Direct Geometry Verified`
is OFF, so this switch does NOT need to be falsely affirmed to inspect V3.
**Auto chassis rotation remains code-locked**. Before changing that, resolve
the previously observed ~0.225 m discrepancy in reconstructed tag height,
validate the measured bearing and distance on the real HUB, and plan a
supervised low-speed safety trial. Old V2 remains as telemetry/reference only.
Cleanup of old files is intentionally deferred until real V3 validation.

## Shooter percent-output restoration (2026-10-23)

From the former `intake-subsystem` and `vscode-local-20261022` branches,
restore **direct duty-cycle shooter control**: the `Shooter Output (%)`
slider defaults to 50% and allows 0–75%, B held runs shooter CAN9
with followers 10 and 12, releasing B stops it. No target RPM or
closed-loop shooter velocity control is used. The current branch's
40 A current limits are preserved. `Auto Shooter Start` retains
the selected percentage, and `Auto Shoot` uses encoder spin-up
stability for feeding safeguards, not RPM-based flywheel control.
Old locally cached RPM widgets are unused and may remain until the
operator deletes them from saved Shuffleboard layouts.
