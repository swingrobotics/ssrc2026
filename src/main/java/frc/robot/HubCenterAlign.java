package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HubConstants;
import java.util.Optional;

/**
 * Face the alliance HUB CENTER using a vetted PhotonVision multi-tag pose.
 * A frame must contain at least two distinct alliance-HUB tags. Tracking
 * expires quickly on lost frames; single-tag yaw never drives the robot.
 *
 * Robot forward (+X) is assumed to be the shooter/launcher direction.
 * An operator aim trim compensates a VERIFIED constant heading bias during
 * initial calibration; it cannot correct a wrong field pose or camera mount.
 */
public final class HubCenterAlign {
  // Physical launcher yaw relative to robot +X (CCW-positive). Measure if
  // the actual launcher does not point exactly forward; do NOT guess.
  private static final double LAUNCHER_YAW_OFFSET_RADIANS = 0.0;
  private static final double MAX_AIM_TRIM_DEGREES = 15.0;
  // Conservative initial tune; field-check the heading sign before enabling
  // rotation on the floor.
  private static final double KP = 1.8;
  private static final double KD = 0.18;
  private static final double MAX_NORMALIZED_ROTATION = 0.25;
  private static final double POSE_TIMEOUT_SECONDS = 0.30;
  private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(1.5);
  private static final double MAX_STABLE_ROTATION_RAD_PER_SEC = 0.12;
  private static final double ALIGN_STABLE_SECONDS = 0.20;

  // MultiTag PnP can jump when the set of visible tags changes.
  // Filter legitimate pose updates and reject discontinuities first.
  private static final double POSITION_FILTER_ALPHA = 0.25;
  private static final double GYRO_FIELD_OFFSET_ALPHA = 0.12;
  private static final double MAX_POSITION_STEP_BASE_METERS = 0.30;
  private static final double MAX_POSITION_STEP_METERS_PER_SECOND = 2.0;
  private static final double MAX_HEADING_INNOVATION_RADIANS = Math.toRadians(18.0);
  // Once aligned, a wider band prevents pose noise from restarting turns.
  private static final double ALIGNED_HYSTERESIS_RADIANS = Math.toRadians(3.0);
  // Slew-limit valid turn commands to avoid abrupt reversals.
  private static final double MAX_TURN_COMMAND_CHANGE_PER_SECOND = 1.5;

  private Pose2d lastVisionPose = null;
  private Alliance visionAlliance = null;
  private double visionTimestampSeconds = -1.0;
  // Gyro heading is the fast, stable heading reference. The vision pose
  // supplies a slowly corrected transform from gyro-yaw to field heading.
  private double gyroToFieldOffsetRadians = Double.NaN;
  private int hubTags = 0;
  private int rejectedPoseUpdates = 0;
  private double lastPoseJumpMeters = Double.NaN;
  private double lastHeadingInnovationDegrees = Double.NaN;
  private String visionFilterStatus = "WAITING FOR FIRST MULTITAG POSE";
  private double lastTurnCommand = 0.0;
  private double lastTurnCommandTime = -1.0;
  private double headingErrorDegrees = Double.NaN;
  private double untrimmedHeadingErrorDegrees = Double.NaN;
  private double estimatedHeadingDegrees = Double.NaN;
  private double desiredHeadingDegrees = Double.NaN;
  private double hubBearingDegrees = Double.NaN;
  private double hubDistanceMeters = Double.NaN;
  private double aimTrimDegrees = 0.0;
  private double stableSinceSeconds = -1.0;
  private String status = "WAITING FOR HUB TAGS";

  public static boolean isAllianceHubTag(int id, Alliance alliance) {
    if (alliance == Alliance.Blue) {
      return id == 18 || id == 19 || id == 20 || id == 21
          || id == 24 || id == 25 || id == 26 || id == 27;
    }
    if (alliance == Alliance.Red) {
      return id == 2 || id == 3 || id == 4 || id == 5
          || id == 8 || id == 9 || id == 10 || id == 11;
    }
    return false;
  }

  /**
   * Driver-tunable offset, positive = robot turns farther CCW toward the HUB,
   * negative = robot turns farther CW. Clamped to prevent large surprises.
   * Adjust only after confirming field pose and physical shooter direction.
   */
  public void setAimTrimDegrees(double requestedDegrees) {
    double next = Double.isFinite(requestedDegrees)
        ? MathUtil.clamp(requestedDegrees, -MAX_AIM_TRIM_DEGREES, MAX_AIM_TRIM_DEGREES)
        : 0.0;
    if (Math.abs(aimTrimDegrees - next) > 1e-6) {
      aimTrimDegrees = next;
      stableSinceSeconds = -1.0;
    }
  }

  /** Positive trim means a more CCW target direction than the untrimmed aim. */
  static double desiredHeadingRadians(
      double bearingRadians, double launcherYawOffsetRadians, double trimDegrees) {
    return MathUtil.angleModulus(
        bearingRadians - launcherYawOffsetRadians + Math.toRadians(trimDegrees));
  }

  /**
   * Accept a PhotonVision multi-tag field pose. Keep the navX heading as the
   * fast heading reference; use camera heading only to slowly establish its
   * field-frame offset. Reject sudden tag-set-switch pose discontinuities.
   *
   * gyroDegrees and angularRateRadPerSec are CURRENT robot readings; frame
   * gyro yaw is back-predicted by the known camera timestamp lag.
   */
  public void acceptVision(
      Pose2d fieldPose, double timestampSeconds, double gyroDegrees,
      double angularRateRadPerSec, Alliance alliance, int uniqueHubTags) {
    double now = Timer.getFPGATimestamp();
    if (fieldPose == null || alliance == null || uniqueHubTags < 2
        || !Double.isFinite(timestampSeconds)
        || timestampSeconds <= visionTimestampSeconds
        || timestampSeconds > now + 0.05
        || now - timestampSeconds > POSE_TIMEOUT_SECONDS
        || !Double.isFinite(gyroDegrees)
        || !Double.isFinite(angularRateRadPerSec)
        || !Double.isFinite(fieldPose.getX())
        || !Double.isFinite(fieldPose.getY())
        || !Double.isFinite(fieldPose.getRotation().getRadians())) {
      return;
    }

    double captureAge = Math.max(0.0, now - timestampSeconds);
    // Approximate gyro yaw when the camera took this image, not when it arrived.
    double gyroAtExposure = MathUtil.angleModulus(
        Math.toRadians(gyroDegrees) - angularRateRadPerSec * captureAge);
    double measuredFieldOffset = MathUtil.angleModulus(
        fieldPose.getRotation().getRadians() - gyroAtExposure);

    if (lastVisionPose == null || visionAlliance != alliance
        || !Double.isFinite(gyroToFieldOffsetRadians)) {
      lastVisionPose = fieldPose;
      gyroToFieldOffsetRadians = measuredFieldOffset;
      visionAlliance = alliance;
      stableSinceSeconds = -1.0;
      visionFilterStatus = "INITIALIZED MULTITAG FIELD POSE";
    } else {
      double elapsed = Math.max(0.0, timestampSeconds - visionTimestampSeconds);
      double positionStep = fieldPose.getTranslation().getDistance(
          lastVisionPose.getTranslation());
      double headingInnovation = MathUtil.angleModulus(
          measuredFieldOffset - gyroToFieldOffsetRadians);
      lastPoseJumpMeters = positionStep;
      lastHeadingInnovationDegrees = Math.toDegrees(headingInnovation);

      double maxReasonableStep = MAX_POSITION_STEP_BASE_METERS
          + MAX_POSITION_STEP_METERS_PER_SECOND * elapsed;
      if (positionStep > maxReasonableStep
          || Math.abs(headingInnovation) > MAX_HEADING_INNOVATION_RADIANS) {
        rejectedPoseUpdates++;
        visionFilterStatus = "REJECTED MULTITAG POSE JUMP";
        // Do not refresh the last valid pose timestamp on an outlier.
        // Rotation will stop after POSE_TIMEOUT_SECONDS without clean data.
        return;
      }

      Translation2d smoothed = lastVisionPose.getTranslation().plus(
          fieldPose.getTranslation().minus(lastVisionPose.getTranslation())
              .times(POSITION_FILTER_ALPHA));
      double fieldHeadingOffset = MathUtil.angleModulus(
          gyroToFieldOffsetRadians
              + GYRO_FIELD_OFFSET_ALPHA * headingInnovation);
      lastVisionPose = new Pose2d(smoothed, fieldPose.getRotation());
      gyroToFieldOffsetRadians = fieldHeadingOffset;
      visionFilterStatus = "FILTERED MULTITAG POSE";
    }

    visionTimestampSeconds = timestampSeconds;
    hubTags = uniqueHubTags;
  }

  private boolean hasFreshPose(Optional<Alliance> alliance) {
    double ageSeconds = Timer.getFPGATimestamp() - visionTimestampSeconds;
    return lastVisionPose != null && alliance.isPresent()
        && alliance.get() == visionAlliance && hubTags >= 2
        && ageSeconds >= 0.0 && ageSeconds <= POSE_TIMEOUT_SECONDS;
  }

  /** Normalized turn command (-1..1), positive for WPILib CCW turn. */
  public double calculateTurn(
      double gyroDegrees, double angularRateRadPerSec,
      Optional<Alliance> alliance) {
    if (!hasFreshPose(alliance)
        || !Double.isFinite(gyroDegrees)
        || !Double.isFinite(angularRateRadPerSec)
        || !Double.isFinite(gyroToFieldOffsetRadians)) {
      headingErrorDegrees = Double.NaN;
      untrimmedHeadingErrorDegrees = Double.NaN;
      estimatedHeadingDegrees = Double.NaN;
      desiredHeadingDegrees = Double.NaN;
      hubBearingDegrees = Double.NaN;
      hubDistanceMeters = Double.NaN;
      stableSinceSeconds = -1.0;
      status = "NO FRESH MULTI-HUB-TAG POSE";
      lastTurnCommand = 0.0;
      lastTurnCommandTime = -1.0;
      return 0.0;
    }

    // Fast continuous gyro yaw + low-noise field-frame offset.
    // Unlike raw camera yaw, this heading does not jump every time a third
    // visible tag changes the coprocessor's PnP solution.
    double estimatedHeading = MathUtil.angleModulus(
        Math.toRadians(gyroDegrees) + gyroToFieldOffsetRadians);

    Translation2d launcher = lastVisionPose.getTranslation().plus(
        HubConstants.SHOOTER_OFFSET_ROBOT_METERS.rotateBy(
            new Rotation2d(estimatedHeading)));
    Translation2d hub = alliance.get() == Alliance.Blue
        ? HubConstants.BLUE_CENTER_METERS
        : HubConstants.RED_CENTER_METERS;
    double dx = hub.getX() - launcher.getX();
    double dy = hub.getY() - launcher.getY();
    if (Math.hypot(dx, dy) < 0.20) {
      stableSinceSeconds = -1.0;
      status = "TOO CLOSE TO HUB CENTER";
      return 0.0;
    }

    double bearingRadians = Math.atan2(dy, dx);
    double untrimmedTarget = desiredHeadingRadians(
        bearingRadians, LAUNCHER_YAW_OFFSET_RADIANS, 0.0);
    double targetHeading = desiredHeadingRadians(
        bearingRadians, LAUNCHER_YAW_OFFSET_RADIANS, aimTrimDegrees);
    double errorRadians = MathUtil.angleModulus(targetHeading - estimatedHeading);

    hubBearingDegrees = Math.toDegrees(bearingRadians);
    hubDistanceMeters = Math.hypot(dx, dy);
    estimatedHeadingDegrees = Math.toDegrees(estimatedHeading);
    desiredHeadingDegrees = Math.toDegrees(targetHeading);
    untrimmedHeadingErrorDegrees = Math.toDegrees(
        MathUtil.angleModulus(untrimmedTarget - estimatedHeading));
    headingErrorDegrees = Math.toDegrees(errorRadians);

    // Hysteresis: after settling, ignore small target bearing fluctuations
    // instead of commanding clockwise/counterclockwise alternation.
    if ((stableSinceSeconds >= 0.0
            && Math.abs(errorRadians) <= ALIGNED_HYSTERESIS_RADIANS)
        || (Math.abs(errorRadians) <= HEADING_TOLERANCE_RADIANS
            && Math.abs(angularRateRadPerSec) <= MAX_STABLE_ROTATION_RAD_PER_SEC)) {
      if (stableSinceSeconds < 0.0) {
        stableSinceSeconds = Timer.getFPGATimestamp();
      }
      status = isAligned(alliance) ? "ALIGNED" : "SETTLING";
      lastTurnCommand = 0.0;
      lastTurnCommandTime = -1.0;
      return 0.0;
    }

    stableSinceSeconds = -1.0;
    status = "TURNING TO HUB CENTER";
    double angularCommand = KP * errorRadians - KD * angularRateRadPerSec;
    double normalized = MathUtil.clamp(
        angularCommand / DriveConstants.kMaxAngularSpeed,
        -MAX_NORMALIZED_ROTATION, MAX_NORMALIZED_ROTATION);
    double now = Timer.getFPGATimestamp();
    if (lastTurnCommandTime >= 0.0) {
      double dt = MathUtil.clamp(now - lastTurnCommandTime, 0.0, 0.1);
      double maxStep = MAX_TURN_COMMAND_CHANGE_PER_SECOND * dt;
      normalized = MathUtil.clamp(normalized,
          lastTurnCommand - maxStep, lastTurnCommand + maxStep);
    }
    lastTurnCommand = normalized;
    lastTurnCommandTime = now;
    return normalized;
  }

  public boolean isAligned(Optional<Alliance> alliance) {
    return hasFreshPose(alliance)
        && stableSinceSeconds >= 0.0
        && Timer.getFPGATimestamp() - stableSinceSeconds >= ALIGN_STABLE_SECONDS;
  }

  public int getVisibleHubTags(Optional<Alliance> alliance) {
    return hasFreshPose(alliance) ? hubTags : 0;
  }

  public double getHeadingErrorDegrees() {
    return Double.isFinite(headingErrorDegrees) ? headingErrorDegrees : -999.0;
  }

  public double getUntrimmedHeadingErrorDegrees() {
    return finiteOrSentinel(untrimmedHeadingErrorDegrees);
  }

  public double getEstimatedHeadingDegrees() {
    return finiteOrSentinel(estimatedHeadingDegrees);
  }

  public double getDesiredHeadingDegrees() {
    return finiteOrSentinel(desiredHeadingDegrees);
  }

  public double getHubBearingDegrees() {
    return finiteOrSentinel(hubBearingDegrees);
  }

  public double getHubDistanceMeters() {
    return finiteOrSentinel(hubDistanceMeters);
  }

  public double getAimTrimDegrees() {
    return aimTrimDegrees;
  }

  public int getRejectedPoseUpdates() {
    return rejectedPoseUpdates;
  }

  public double getLastPoseJumpMeters() {
    return finiteOrSentinel(lastPoseJumpMeters);
  }

  public double getLastHeadingInnovationDegrees() {
    return finiteOrSentinel(lastHeadingInnovationDegrees);
  }

  public String getVisionFilterStatus() {
    return visionFilterStatus;
  }

  private static double finiteOrSentinel(double value) {
    return Double.isFinite(value) ? value : -999.0;
  }

  public String getStatus() {
    return status;
  }

  public void reset() {
    lastVisionPose = null;
    visionAlliance = null;
    visionTimestampSeconds = -1.0;
    gyroToFieldOffsetRadians = Double.NaN;
    hubTags = 0;
    rejectedPoseUpdates = 0;
    lastPoseJumpMeters = Double.NaN;
    lastHeadingInnovationDegrees = Double.NaN;
    visionFilterStatus = "WAITING FOR FIRST MULTITAG POSE";
    lastTurnCommand = 0.0;
    lastTurnCommandTime = -1.0;
    stableSinceSeconds = -1.0;
    headingErrorDegrees = Double.NaN;
    untrimmedHeadingErrorDegrees = Double.NaN;
    estimatedHeadingDegrees = Double.NaN;
    desiredHeadingDegrees = Double.NaN;
    hubBearingDegrees = Double.NaN;
    hubDistanceMeters = Double.NaN;
    // Deliberately retain operator trim across X toggle.
    status = "WAITING FOR HUB TAGS";
  }
}
