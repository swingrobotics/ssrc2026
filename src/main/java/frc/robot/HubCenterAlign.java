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
 * Robot-facing HUB aiming based ONLY on the drivetrain's fused field Pose2d.
 * PhotonVision is a measurement source for the drive pose estimator, not an
 * independent heading controller. Equivalent architecture: FRC 135 AimToPose
 * and FRC 6940 getRotationToAllianceHub.
 */
public final class HubCenterAlign {
  // The physical launcher direction relative to robot +X (CCW positive).
  // Keep 0 until measured on the robot; trim is separately adjustable.
  private static final double LAUNCHER_YAW_OFFSET_DEGREES = 0.0;
  private static final double MAX_TRIM_DEGREES = 15.0;
  private static final double MAX_TURN_RAD_PER_SEC = Math.toRadians(75.0);
  private static final double MAX_ACCEL_RAD_PER_SEC2 = Math.toRadians(140.0);
  private static final double HEADING_TOLERANCE_RAD = Math.toRadians(1.5);
  private static final double RATE_TOLERANCE_RAD_PER_SEC = Math.toRadians(6.0);
  private static final double STABLE_SECONDS = 0.20;
  private static final double MIN_HUB_DISTANCE_METERS = 0.4;

  private final ProfiledPIDController thetaController = new ProfiledPIDController(
      2.0, 0.0, 0.12,
      new TrapezoidProfile.Constraints(
          MAX_TURN_RAD_PER_SEC, MAX_ACCEL_RAD_PER_SEC2));

  private boolean controllerInitialized = false;
  private double trimDegrees = 0.0;
  private double stableSince = -1.0;
  private double headingErrorDegrees = Double.NaN;
  private double untrimmedErrorDegrees = Double.NaN;
  private double currentHeadingDegrees = Double.NaN;
  private double desiredHeadingDegrees = Double.NaN;
  private double hubBearingDegrees = Double.NaN;
  private double hubDistanceMeters = Double.NaN;
  private String status = "OFF";
  private String source = "NONE";

  public HubCenterAlign() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(HEADING_TOLERANCE_RAD, RATE_TOLERANCE_RAD_PER_SEC);
  }

  /** Pure field-space geometry. Rotation of the launcher is chassis + offset. */
  public static Rotation2d desiredHeading(
      Pose2d robotPose, Translation2d hubCenter,
      Translation2d launcherOffsetRobot, Rotation2d launcherYaw,
      double trimDegrees) {
    Translation2d launcherWorld = robotPose.getTranslation().plus(
        launcherOffsetRobot.rotateBy(robotPose.getRotation()));
    Translation2d toHub = hubCenter.minus(launcherWorld);
    return Rotation2d.fromRadians(MathUtil.angleModulus(
        Math.atan2(toHub.getY(), toHub.getX())
            - launcherYaw.getRadians()
            + Math.toRadians(trimDegrees)));
  }

  public static Translation2d hubForAlliance(Alliance color) {
    return color == Alliance.Red
        ? HubConstants.RED_CENTER_METERS
        : HubConstants.BLUE_CENTER_METERS;
  }

  /** Positive trim rotates target CCW; bound to a narrow calibration band. */
  public void setAimTrimDegrees(double requestedDegrees) {
    double target = Double.isFinite(requestedDegrees)
        ? MathUtil.clamp(requestedDegrees, -MAX_TRIM_DEGREES, MAX_TRIM_DEGREES)
        : 0.0;
    if (Math.abs(target - trimDegrees) > 1e-6) {
      trimDegrees = target;
      controllerInitialized = false;
      stableSince = -1.0;
    }
  }

  public void start() {
    controllerInitialized = false;
    stableSince = -1.0;
    status = "WAITING FOR HUB TAG OR VERIFIED FIELD POSE";
    source = "NONE";
  }

  public void stop() {
    controllerInitialized = false;
    stableSince = -1.0;
    status = "OFF";
    source = "NONE";
  }

  /**
   * Return the normalized turn axis expected by DriveSubsystem.drive().
   * All drive translation stays owned by the left-stick command.
   */
  public double calculateTurn(
      Pose2d fusedPose, ChassisSpeeds measuredChassisSpeeds,
      Optional<Alliance> alliance, boolean poseReady,
      boolean gyroReady, boolean visionRecentlyAccepted) {
    if (!poseReady || !gyroReady || !visionRecentlyAccepted
        || alliance.isEmpty() || fusedPose == null || measuredChassisSpeeds == null
        || !Double.isFinite(fusedPose.getX())
        || !Double.isFinite(fusedPose.getY())
        || !Double.isFinite(fusedPose.getRotation().getRadians())
        || !Double.isFinite(measuredChassisSpeeds.omegaRadiansPerSecond)) {
      controllerInitialized = false;
      stableSince = -1.0;
      headingErrorDegrees = Double.NaN;
      status = !gyroReady ? "STOP: NAVX NOT READY"
          : !poseReady ? "STOP: POSE NOT INITIALIZED"
          : !visionRecentlyAccepted ? "STOP: VISION POSE STALE"
          : alliance.isEmpty() ? "STOP: ALLIANCE UNKNOWN"
          : "STOP: INVALID POSE OR RATE";
      return 0.0;
    }

    return calculateFacingTarget(
        fusedPose, measuredChassisSpeeds, hubForAlliance(alliance.get()),
        "FIELD FUSED");
  }

  /**
   * FRC 4107 cameraToTag -> tagToHub targeting path.
   * The robot pose and HUB coordinates are in the SAME local odometry frame,
   * so no field pose initialization or external alliance is needed here.
   */
  public double calculateDirectTurn(
      Pose2d localOdometryPose, ChassisSpeeds measuredChassisSpeeds,
      Translation2d hubInLocalOdometry, boolean gyroReady) {
    if (!gyroReady || localOdometryPose == null
        || hubInLocalOdometry == null || measuredChassisSpeeds == null
        || !Double.isFinite(localOdometryPose.getX())
        || !Double.isFinite(localOdometryPose.getY())
        || !Double.isFinite(localOdometryPose.getRotation().getRadians())
        || !Double.isFinite(measuredChassisSpeeds.omegaRadiansPerSecond)) {
      controllerInitialized = false;
      stableSince = -1.0;
      status = "STOP: DIRECT HUB POSE INVALID OR NAVX NOT READY";
      source = "NONE";
      return 0.0;
    }
    return calculateFacingTarget(
        localOdometryPose, measuredChassisSpeeds, hubInLocalOdometry,
        "DIRECT TAG 3D");
  }

  private double calculateFacingTarget(
      Pose2d robotPose, ChassisSpeeds measuredChassisSpeeds,
      Translation2d hub, String nextSource) {
    if (!source.equals(nextSource)) {
      controllerInitialized = false;
      stableSince = -1.0;
      source = nextSource;
    }
    Translation2d launcherWorld = robotPose.getTranslation().plus(
        HubConstants.SHOOTER_OFFSET_ROBOT_METERS.rotateBy(robotPose.getRotation()));
    double dx = hub.getX() - launcherWorld.getX();
    double dy = hub.getY() - launcherWorld.getY();
    hubDistanceMeters = Math.hypot(dx, dy);
    if (!Double.isFinite(hubDistanceMeters)
        || hubDistanceMeters <= MIN_HUB_DISTANCE_METERS) {
      controllerInitialized = false;
      stableSince = -1.0;
      status = "STOP: HUB TOO CLOSE OR INVALID";
      return 0.0;
    }

    Rotation2d desired = desiredHeading(
        robotPose, hub, HubConstants.SHOOTER_OFFSET_ROBOT_METERS,
        Rotation2d.fromDegrees(LAUNCHER_YAW_OFFSET_DEGREES), trimDegrees);
    Rotation2d untrimmed = desiredHeading(
        robotPose, hub, HubConstants.SHOOTER_OFFSET_ROBOT_METERS,
        Rotation2d.fromDegrees(LAUNCHER_YAW_OFFSET_DEGREES), 0.0);
    double current = robotPose.getRotation().getRadians();
    double goal = desired.getRadians();
    double error = MathUtil.angleModulus(goal - current);
    headingErrorDegrees = Math.toDegrees(error);
    untrimmedErrorDegrees = untrimmed.minus(robotPose.getRotation()).getDegrees();
    currentHeadingDegrees = robotPose.getRotation().getDegrees();
    desiredHeadingDegrees = desired.getDegrees();
    hubBearingDegrees = Math.toDegrees(Math.atan2(dy, dx));

    if (!controllerInitialized) {
      thetaController.reset(current, measuredChassisSpeeds.omegaRadiansPerSecond);
      controllerInitialized = true;
    }

    boolean inTolerance = Math.abs(error) < HEADING_TOLERANCE_RAD
        && Math.abs(measuredChassisSpeeds.omegaRadiansPerSecond)
            < RATE_TOLERANCE_RAD_PER_SEC;

    if (inTolerance) {
      if (stableSince < 0.0) stableSince = Timer.getFPGATimestamp();
      status = Timer.getFPGATimestamp() - stableSince >= STABLE_SECONDS
          ? "ALIGNED" : "SETTLING";
      return 0.0;
    }

    stableSince = -1.0;
    status = "TURNING TO HUB CENTER";
    double omega = thetaController.calculate(current, goal)
        + thetaController.getSetpoint().velocity;
    omega = MathUtil.clamp(
        omega, -MAX_TURN_RAD_PER_SEC, MAX_TURN_RAD_PER_SEC);
    return MathUtil.clamp(
        omega / DriveConstants.kMaxAngularSpeed, -1.0, 1.0);
  }

  public boolean isAligned() {
    return stableSince >= 0.0
        && Timer.getFPGATimestamp() - stableSince >= STABLE_SECONDS
        && status.equals("ALIGNED");
  }

  public String getStatus() { return status; }
  public String getSource() { return source; }
  public double getAimTrimDegrees() { return trimDegrees; }
  public double getHeadingErrorDegrees() { return finiteOrInvalid(headingErrorDegrees); }
  public double getUntrimmedHeadingErrorDegrees() { return finiteOrInvalid(untrimmedErrorDegrees); }
  public double getEstimatedHeadingDegrees() { return finiteOrInvalid(currentHeadingDegrees); }
  public double getDesiredHeadingDegrees() { return finiteOrInvalid(desiredHeadingDegrees); }
  public double getHubBearingDegrees() { return finiteOrInvalid(hubBearingDegrees); }
  public double getHubDistanceMeters() { return finiteOrInvalid(hubDistanceMeters); }

  private static double finiteOrInvalid(double value) {
    return Double.isFinite(value) ? value : -999.0;
  }
}
