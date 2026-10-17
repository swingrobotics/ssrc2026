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
  is NOT automatically a valid +X translation. The user reported the camera
  on the Shooter firing side and 25 degrees up from its underside reference.
  Set `CAMERA_PITCH_DEG=-25` provisionally for a camera whose optical axis
  is actually 25 degrees above *horizontal* in the WPILib frame.
- **Still required**: confirm whether 317.33 mm is horizontal or direct
  distance, whether the camera is centered left-to-right, which direction
  the shooter faces relative to robot +X, whether the former 90-degree
  camera mounting rotation remains, and whether the angle reference is
  horizontal. Set the corresponding `CAMERA_FORWARD_M`,
  `CAMERA_LEFT_M`, `CAMERA_ROLL_DEG`, and `CAMERA_YAW_DEG` accurately.
  These are intentionally pending, and the safety switch remains false.
- PhotonVision needs camera intrinsics calibration for the selected resolution
  and the **correct physical AprilTag size**; otherwise 3D distance can be
  wrong. Use a stable, stationary tag secured above the floor.
- ONLY AFTER the above, set `CAMERA_MOUNT_VERIFIED = true`. The shipped
  value is intentionally **false** to avoid operating with false geometry.
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
