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

  private Pose2d lastVisionPose = null;
  private Alliance visionAlliance = null;
  private double visionTimestampSeconds = -1.0;
  private double gyroAtVisionRadians = Double.NaN;
  private int hubTags = 0;
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

  /** Receive an already-vetted multi-tag pose from RobotContainer. */
  public void acceptVision(
      Pose2d fieldPose, double timestampSeconds, double gyroDegrees,
      Alliance alliance, int uniqueHubTags) {
    double now = Timer.getFPGATimestamp();
    if (fieldPose == null || alliance == null || uniqueHubTags < 2
        || !Double.isFinite(timestampSeconds)
        || timestampSeconds <= visionTimestampSeconds
        || timestampSeconds > now + 0.05
        || now - timestampSeconds > POSE_TIMEOUT_SECONDS
        || !Double.isFinite(gyroDegrees)
        || !Double.isFinite(fieldPose.getX())
        || !Double.isFinite(fieldPose.getY())
        || !Double.isFinite(fieldPose.getRotation().getRadians())) {
      return;
    }
    if (alliance != visionAlliance) {
      stableSinceSeconds = -1.0;
    }
    lastVisionPose = fieldPose;
    visionAlliance = alliance;
    visionTimestampSeconds = timestampSeconds;
    gyroAtVisionRadians = Math.toRadians(gyroDegrees);
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
        || !Double.isFinite(gyroAtVisionRadians)) {
      headingErrorDegrees = Double.NaN;
      untrimmedHeadingErrorDegrees = Double.NaN;
      estimatedHeadingDegrees = Double.NaN;
      desiredHeadingDegrees = Double.NaN;
      hubBearingDegrees = Double.NaN;
      hubDistanceMeters = Double.NaN;
      stableSinceSeconds = -1.0;
      status = "NO FRESH MULTI-HUB-TAG POSE";
      return 0.0;
    }

    // Compensate rotation since camera capture using the robot gyro.
    double rotationDelta = MathUtil.angleModulus(
        Math.toRadians(gyroDegrees) - gyroAtVisionRadians);
    double estimatedHeading = MathUtil.angleModulus(
        lastVisionPose.getRotation().getRadians() + rotationDelta);

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

    if (Math.abs(errorRadians) <= HEADING_TOLERANCE_RADIANS
        && Math.abs(angularRateRadPerSec) <= MAX_STABLE_ROTATION_RAD_PER_SEC) {
      if (stableSinceSeconds < 0.0) {
        stableSinceSeconds = Timer.getFPGATimestamp();
      }
      status = isAligned(alliance) ? "ALIGNED" : "SETTLING";
      return 0.0;
    }

    stableSinceSeconds = -1.0;
    status = "TURNING TO HUB CENTER";
    double angularCommand = KP * errorRadians - KD * angularRateRadPerSec;
    double normalized = angularCommand / DriveConstants.kMaxAngularSpeed;
    return MathUtil.clamp(normalized,
        -MAX_NORMALIZED_ROTATION, MAX_NORMALIZED_ROTATION);
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
    gyroAtVisionRadians = Double.NaN;
    hubTags = 0;
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
