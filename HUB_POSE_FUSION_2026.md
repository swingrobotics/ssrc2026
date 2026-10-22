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
3. View Shuffleboard `Vision` values:
   - `Camera Expected PV Orientation (deg)`: expected UI setting (180),
     NOT an actual readback from PhotonVision.
   - `Camera Config Roll/Pitch/Yaw (deg)`: effective `ROBOT_TO_CAMERA`
     settings (0/-25/0).
   - `Camera Mount Tag ID`: which observed tag was selected.
   - `Camera Mount Tag Z (m)`: reconstructed target height above robot
     floor, assuming the robot is level and its origin is at ground Z=0.
   - `Camera Mount Tag Height Error (m)`: reconstructed tag Z minus the
     official WPILib tag height. Compare while slowly changing camera
     position across DISTANCES with the robot safely DISABLED.
   - `Camera Mount Check Status`: whether the height residual is small or
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
The robot reports this distinction directly on the `Vision` dashboard:

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

On the `Vision` tab inspect these in order:
1. `HUB X Button Pressed`: true **while holding X** proves controller mapping.
2. `HUB Align Requested`: true **after one press** proves the toggle fired.
3. `HUB Teleop Enabled`: must be true; Disabled and autonomous do not
   execute the manual drive/default HUB heading controller.
4. `HUB Turn Blocker`: displays the first unmet interlock. Do not disable
   interlocks just to make wheels move.
5. `HUB Turn Command (%)`: nonzero means aim computes a rotation command.
   If it is nonzero and chassis remains still, check drive speed percentage,
   manual trigger turn, swerve wheel steering/motor state and driver station
   output, with wheels safely elevated under team supervision.
6. `HUB Error (deg)`: may be near zero if already aligned, where 0% is
   expected. If camera localization is wrong, do not use trim as a fix.
7. `Vision Gate Status`: rejected PnP frames prevent fusion. For any
   stop status check field JSON, camera extrinsics, tag distances, navX.
8. `Pose Init Status`: first global pose requires a fresh multitag
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
