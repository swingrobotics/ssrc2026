# Phase 1: Single AprilTag memory + automatic heading (TEST ONLY)

This development branch changes the active HUB LOCK test into remembering an
**arbitrary fixed AprilTag mounted above the floor**. The real 2026 HUB
center, alliance selection, field-layout lookup and multi-tag fusion are
**deferred to the later field implementation**. No Hood geometry, Hood
angles, Shooter RPM or automatic ball trajectory calculations are included.

## How it works
1. Start with the robot stationary, navX2 ready and a calibrated PhotonVision
   `PC_Camera` AprilTag pipeline. No official field starting pose is required.
2. From three consistent, low-ambiguity observations of one tag ID,
   calculate `robot -> camera -> AprilTag` in 3D, transform into the robot's
   **local X/Y odometry frame**, then remember both that tag ID and location.
3. Move around. Press **X** once to turn toward the stored X/Y location; the
   tag may be entirely off camera. X again disables tracking; LT/RT manually
   override rotation. Translation remains under the driver's left stick.
4. When the remembered tag reappears, calculate a fresh robot X/Y position
   relative to the unchanged saved tag position. Use latency-compensated
   `SwerveDrivePoseEstimator` measurement for small errors. Require three
   matching position corrections before large slip/BUMP recovery.
5. During a BUMP (navX2 Pitch/Roll threshold) and until reliable recovery,
   heading lock stops rather than turning toward an untrustworthy target.
6. **Other AprilTag IDs are ignored** after first successful capture, even if
   PhotonVision reports one of them as its best target.
7. SmartDashboard `Tag Clear Memory` = true drops the saved target, permitting
   a different tag to be learned. Pressing controller Start to zero navX2, or
   a PathPlanner pose-frame reset, also clears the remembered tag.

## REQUIRED calibration before testing
- Measurements recorded on 2026-10-17: camera height
  **403.94608582 mm above the floor** (`CAMERA_UP_M=0.40394608582`);
  camera distance from robot center **317.32884884 mm**
  (`CAMERA_CENTER_DISTANCE_MEASURED_M=0.31732884884`). This distance
  is now provisionally treated as the **forward (+X)** mounting offset
  (`CAMERA_FORWARD_M=0.31732884884`), assuming the Shooter-facing end
  is the programmed robot front (+X) and the distance was measured horizontally.
  A mounting photo and explicit user confirmation establish that the camera
  is left/right centered, so `CAMERA_LEFT_M=0`.
  The user CONFIRMED the optical axis is 25 degrees ABOVE HORIZONTAL;
  in WPILib's NWU convention `CAMERA_PITCH_DEG=-25`.
  PhotonVision already displays the camera image upright and its reported
  target Y increases when the tag moves left, decreasing when moved right.
- **Phase-1 test enabled**: `CAMERA_MOUNT_VERIFIED = true`, while keeping
  `CAMERA_ROLL_DEG=0` and `CAMERA_YAW_DEG=0` to avoid double-rotating the
  software-corrected image. These values are a working assumption, NOT an
  independent proof of the 3D camera mount transform. Before driving, check
  that a fixed stationary tag produces consistent saved X/Y as the robot
  moves and that the target heading sign is correct. The Shooter-facing side
  must be robot +X, and 317.33 mm must be a horizontal center offset.
  If either assumption is wrong, set `CAMERA_MOUNT_VERIFIED = false` until
  the corrected values have been entered.
- PhotonVision needs camera intrinsics calibration for the selected resolution
  and the **correct physical AprilTag size**; otherwise 3D distance can be
  wrong. Use a stable, stationary tag secured above the floor.
- The switch is currently **true** only for stationary pose verification
  and supervised low-speed tests. Do not treat it as full 3D calibration
  certification. Watch `Tag Remembered`, `Tag Memory X/Y`, and
  `Tag Heading Error Deg` before allowing any automatic turn.
- The observed screenshot showed `Ambiguity Ratio = 0.44`, while the
  single-tag filter accepts only values below 0.18. That frame will be
  REJECTED. Stabilize the tag, improve viewing angle/lighting and verify
  AprilTag pose ambiguity falls below 0.18; do not simply bypass the filter.
- Confirm `PC_Camera` network identity and PhotonLib 2026 API version.
- Robot starting point is *arbitrary*. Its starting local frame is defined
  by odometry and navX2, not the official field; relative position matters.
- Test while the robot is stationary first: `Tag Remembered` should become
  true, `Tag Remembered ID` should equal the printed tag ID, and `Tag
  Memory X/Y` should stay approximately stable as the robot moves.
- Use a clear supervised low-speed test space. Verify turning direction at
  a safe limit before allowing automatic heading. The lock output is
  clamped to 35% of the project's existing max angular speed.
- This code has been committed, but a WPILib build and physical validation
  on the team's installed hardware are still required.

## Telemetry (SmartDashboard)
- `Tag Memory Status`: WAITING, LEARNING_TAG, TAG_REMEMBERED,
  SINGLE_TAG_FUSED, VERIFYING_RECOVERY, TAG_RELOCALIZED, BUMP_CROSSING, etc.
- `Tag Remembered`, `Tag Remembered ID`, `Tag Memory X/Y`
- `Tag Distance M`, `Tag Target Heading Deg`, `Tag Heading Error Deg`
- `Tag Visible`, `Tag Visible Count`, `Tag Last Visible ID`
- `Tag Lock Enabled`, `Tag Lock Ready`, `Tag Last Correction Age S`
- `Bump Active`, `Bump Recovery Required`

## Limitations
- This test remembers a point in a *relative coordinate frame*; restarting,
  resetting the navX, or changing PathPlanner's pose origin requires a fresh
  tag observation. Saved target does not survive robot reboot.
- The local field heading is navX2-derived. Unobservable gyro heading drift
  cannot be fully corrected by this single-tag XY-only recovery. A distant
  observation or significant yaw offset may degrade aiming.
- Wheel slip during BUMP cannot be reconstructed perfectly. If the tag
  cannot be observed again, the lock **waits for recovery**, not a promise
  of perfect blind aiming.
- Shooting/Hood aiming is independent and unchanged. The tag height is only
  used to transform camera observations into a 2D X/Y target for yaw.
- The future 2026 HUB + MultiTag design will restore alliance HUB center
  as target while keeping these tested localization principles.

API references:
- https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html
- https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/PoseEstimator.html
