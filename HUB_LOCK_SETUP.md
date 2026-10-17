# HUB LOCK development (2026 REBUILT)

This branch is based on `shooter-integration` and preserves the user's
A=Feeder, B=Shooter, X=HUB LOCK toggle, Y=Field/Robot-relative toggle,
right-stick-Y=manual Hood, RB=X-lock, LT/RT=manual rotation controls.

## Implementation
- navX2 and swerve encoders feed a single `SwerveDrivePoseEstimator`;
  PathPlanner receives that same field pose and its existing AutoBuilder
  reset callback is preserved.
- PhotonVision `PC_Camera` supplies ONE stream of unread results per robot
  tick. Coprocessor MultiTag is preferred; if unavailable, lowest ambiguity
  SingleTag is accepted after quality checks.
- Target position is the **alliance's HUB center**, fixed in the BLUE-origin
  WPILib field coordinate frame. 2026 manual: 158.6 inches from the alliance
  wall; Y is half the field width. Tag switching doesn't change target.
- X enables HUB LOCK: PID turns to the target while the left stick still
  drives translation. LT/RT manual rotation overrides the controller.
- BUMP detection from navX2 Pitch/Roll pauses auto-heading and requires a
  consistent valid post-BUMP camera pose to resume. Large vision position
  jumps must agree across multiple frames before estimator reset.
- Telemetry: `Pose Initialized`, `Bump Active`, `Bump Recovery Required`,
  `Hub Lock Ready`, `Hub Localization Status`, `Hub Distance M`,
  `Hub Target Heading`, `Hub Heading Error`, `Vision Tag Count`,
  `Vision Pose Age Sec`. All existing dashboard topics stay available.
- Hub distance is **telemetry only** for eventual Auto Hood calibration;
  no untested automatic Hood encoder angles are commanded.

## REQUIRED before robot testing
1. Edit `Constants.VisionConstants`. Measure camera center displacement
   (forward/left/up in meters) and orientation (roll/pitch/yaw in degrees)
   **from robot center**. The existing camera was reportedly mounted
   rotated by 90 degrees; do not guess the proper roll/yaw axis.
2. Choose `k2026RebuiltWelded` or `k2026RebuiltAndymark` to match the field.
   PhotonVision Settings must use the **same** layout.
3. Calibrate the camera lens, enable AprilTag 3D pipeline and **Do Multi-Target
   Estimation** in PhotonVision, confirm the camera is named `PC_Camera`.
4. Set `CAMERA_MOUNT_VERIFIED = true` only after the above. It is intentionally
   `false` by default. With false, vision cannot establish field pose, but
   PathPlanner's configured auto start pose can initialize field localization.
5. Check Robot Alliance from Driver Station. If alliance is absent, HUB LOCK
   pauses rather than aiming at a guessed target.
6. Confirm physical Drive controllers: current branch still uses SPARK MAX
   on CAN 1-8; the supplied ZIP used SPARK Flex on Drive CAN 1-4.
7. Build in WPILib 2026 on the team laptop before any deployment. Conduct
   supervised low-speed testing with a clear area and ability to disable.

## Known limitations / recovery
- A robot with no initial field pose cannot target a fixed HUB from
  gyro data alone. Localize via AprilTag or correct PathPlanner starting pose.
- Large BUMP wheel slip cannot be reconstructed by navX2 alone. HUB LOCK
  pauses until reliable tag observations re-localize it; it does NOT claim
  to keep accurate aiming through completely unobservable motion.
- If the raw navX2 is reset during the match, a new field pose fix is needed.
- Offsets from the robot center to the actual Shooter muzzle and
  shooter exit latency are NOT calibrated here. This v1 aligns the chassis
  toward the HUB center, not a ballistic lead solution.
- Single-tag fallback depends on camera calibration and 3D geometry;
  test tag switching and deliberately bad measurements before real match use.

References:
- https://docs.photonvision.org/en/v2026.3.2/docs/programming/photonlib/robot-pose-estimator.html
- https://docs.photonvision.org/en/v2026.3.3/docs/apriltag-pipelines/multitag.html
- https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator.html
- https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
