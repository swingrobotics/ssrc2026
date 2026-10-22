package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HubConstants;
import java.util.Optional;

/**
 * HUB V2 heading controller. Input: verified HUB center + robot pose.
 * Output: bounded normalized chassis-turn command; never directly owns motors.
 */
public final class HubAimHeadingV2 {
  private static final double MAX_TRIM_DEG = 15.0;
  private static final double MAX_TURN_RAD_S = Math.toRadians(45.0);
  private static final double MAX_ACCEL_RAD_S2 = Math.toRadians(90.0);
  private static final double ERROR_TOLERANCE_RAD = Math.toRadians(1.5);
  private static final double RATE_TOLERANCE_RAD_S = Math.toRadians(5.0);
  private static final double STABLE_TIME_S = 0.20;

  private final ProfiledPIDController pid = new ProfiledPIDController(
      2.0, 0.0, 0.08,
      new TrapezoidProfile.Constraints(MAX_TURN_RAD_S, MAX_ACCEL_RAD_S2));

  private boolean initialized = false;
  private double trimDegrees = 0.0;
  private double stableSince = -1.0;
  private double headingErrorDegrees = Double.NaN;
  private double rawErrorDegrees = Double.NaN;
  private double currentHeadingDegrees = Double.NaN;
  private double desiredHeadingDegrees = Double.NaN;
  private double hubBearingDegrees = Double.NaN;
  private double hubDistanceMeters = Double.NaN;
  private String source = "NONE";
  private String status = "OFF";

  public HubAimHeadingV2() {
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.setTolerance(ERROR_TOLERANCE_RAD, RATE_TOLERANCE_RAD_S);
  }

  public static Rotation2d desiredHeading(
      Pose2d robotPose, Translation2d hubCenter,
      Translation2d launcherOffset, Rotation2d launcherYaw,
      double trimDeg) {
    Translation2d launchPoint = robotPose.getTranslation().plus(
        launcherOffset.rotateBy(robotPose.getRotation()));
    Translation2d vector = hubCenter.minus(launchPoint);
    return Rotation2d.fromRadians(MathUtil.angleModulus(
        Math.atan2(vector.getY(), vector.getX())
        - launcherYaw.getRadians() + Math.toRadians(trimDeg)));
  }

  public static Translation2d hubForAlliance(Alliance alliance) {
    return alliance == Alliance.Red
        ? HubConstants.RED_CENTER_METERS : HubConstants.BLUE_CENTER_METERS;
  }

  public void setAimTrimDegrees(double value) {
    double clipped = Double.isFinite(value)
        ? MathUtil.clamp(value, -MAX_TRIM_DEG, MAX_TRIM_DEG) : 0.0;
    if (Math.abs(trimDegrees - clipped) > 1e-6) {
      trimDegrees = clipped;
      initialized = false;
      stableSince = -1.0;
    }
  }
  public void start() {
    initialized = false;
    stableSince = -1.0;
    source = "NONE";
    status = "WAITING FOR HUB";
  }
  public void stop() {
    initialized = false;
    stableSince = -1.0;
    source = "NONE";
    status = "OFF";
  }

  public double calculateDirectTurn(
      Pose2d pose, ChassisSpeeds speeds,
      Translation2d localHub, boolean gyroReady) {
    return calculate(pose, speeds, localHub, gyroReady, "DIRECT TAG V2");
  }

  public double calculateTurn(Pose2d pose, ChassisSpeeds speeds,
      Optional<Alliance> alliance, boolean poseReady,
      boolean gyroReady, boolean visionFresh) {
    if (alliance == null || alliance.isEmpty() || !poseReady || !visionFresh) {
      invalidate("FIELD POSE UNVERIFIED OR STALE");
      return 0.0;
    }
    return calculate(pose, speeds, hubForAlliance(alliance.get()), gyroReady,
        "FIELD POSE V2");
  }

  private double calculate(
      Pose2d pose, ChassisSpeeds speeds, Translation2d hub,
      boolean gyroReady, String nextSource) {
    if (!gyroReady || !valid(pose, hub) || speeds == null
        || !Double.isFinite(speeds.omegaRadiansPerSecond)) {
      invalidate("NAVX, ROBOT POSE OR TARGET INVALID");
      return 0.0;
    }
    updateGeometry(pose, hub, nextSource);
    if (!Double.isFinite(hubDistanceMeters) || hubDistanceMeters <= 0.4) {
      invalidate("HUB TOO CLOSE");
      return 0.0;
    }
    double now = Timer.getFPGATimestamp();
    double error = Math.toRadians(headingErrorDegrees);
    if (Math.abs(error) <= ERROR_TOLERANCE_RAD
        && Math.abs(speeds.omegaRadiansPerSecond) <= RATE_TOLERANCE_RAD_S) {
      if (stableSince < 0.0) stableSince = now;
      status = now - stableSince >= STABLE_TIME_S ? "ALIGNED" : "SETTLING";
      return 0.0;
    }

    stableSince = -1.0;
    if (!initialized) {
      pid.reset(pose.getRotation().getRadians(),
          speeds.omegaRadiansPerSecond);
      initialized = true;
    }
    double omega = pid.calculate(
        pose.getRotation().getRadians(),
        Math.toRadians(desiredHeadingDegrees)) + pid.getSetpoint().velocity;
    status = "TURNING TO HUB CENTER";
    return MathUtil.clamp(
        MathUtil.clamp(omega, -MAX_TURN_RAD_S, MAX_TURN_RAD_S)
            / DriveConstants.kMaxAngularSpeed, -1.0, 1.0);
  }

  /** Refresh only the numerical diagnostics, never produces a motor command. */
  public void previewOnly(Pose2d pose, Translation2d hub, String label) {
    if (!valid(pose, hub)) {
      invalidate("PREVIEW TARGET INVALID");
      return;
    }
    updateGeometry(pose, hub, label);
    status = "PREVIEW ONLY - AUTO ROTATION LOCKED";
    initialized = false;
    stableSince = -1.0;
  }

  private void updateGeometry(Pose2d pose, Translation2d hub, String nextSource) {
    if (!source.equals(nextSource)) {
      initialized = false;
      stableSince = -1.0;
    }
    source = nextSource;
    Translation2d origin = pose.getTranslation().plus(
        HubConstants.SHOOTER_OFFSET_ROBOT_METERS.rotateBy(pose.getRotation()));
    Translation2d toward = hub.minus(origin);
    hubDistanceMeters = toward.getNorm();
    hubBearingDegrees = Math.toDegrees(Math.atan2(toward.getY(), toward.getX()));
    Rotation2d desired = desiredHeading(pose, hub,
        HubConstants.SHOOTER_OFFSET_ROBOT_METERS, Rotation2d.kZero, trimDegrees);
    Rotation2d raw = desiredHeading(pose, hub,
        HubConstants.SHOOTER_OFFSET_ROBOT_METERS, Rotation2d.kZero, 0.0);
    desiredHeadingDegrees = desired.getDegrees();
    currentHeadingDegrees = pose.getRotation().getDegrees();
    headingErrorDegrees = desired.minus(pose.getRotation()).getDegrees();
    rawErrorDegrees = raw.minus(pose.getRotation()).getDegrees();
  }

  private static boolean valid(Pose2d pose, Translation2d hub) {
    return pose != null && hub != null
        && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians())
        && Double.isFinite(hub.getX()) && Double.isFinite(hub.getY());
  }

  private void invalidate(String why) {
    initialized = false;
    stableSince = -1.0;
    source = "NONE";
    status = "STOP: " + why;
    headingErrorDegrees = Double.NaN;
    desiredHeadingDegrees = Double.NaN;
    hubDistanceMeters = Double.NaN;
  }

  public boolean isAligned() {
    return status.equals("ALIGNED") && stableSince >= 0.0
        && Timer.getFPGATimestamp() - stableSince >= STABLE_TIME_S;
  }
  public String getStatus() { return status; }
  public String getSource() { return source; }
  public double getAimTrimDegrees() { return trimDegrees; }
  public double getHeadingErrorDegrees() { return finite(headingErrorDegrees); }
  public double getUntrimmedHeadingErrorDegrees() { return finite(rawErrorDegrees); }
  public double getEstimatedHeadingDegrees() { return finite(currentHeadingDegrees); }
  public double getDesiredHeadingDegrees() { return finite(desiredHeadingDegrees); }
  public double getHubBearingDegrees() { return finite(hubBearingDegrees); }
  public double getHubDistanceMeters() { return finite(hubDistanceMeters); }
  private static double finite(double x) {
    return Double.isFinite(x) ? x : -999.0;
  }
}
