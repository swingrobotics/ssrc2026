// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.List;
import java.util.Locale;
import java.util.StringJoiner;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // navX2-MXP connected through the roboRIO MXP SPI interface.
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Field visualization for odometry testing.
  private final Field2d m_field = new Field2d();
  private boolean m_pathPlannerConfigured = false;
  private boolean m_pathPlannerTargetValid = false;
  private String m_pathPlannerActivePath = "";
  private int m_pathPlannerPathPoints = 0;
  private Pose2d m_pathPlannerCurrentPose = new Pose2d();
  private Pose2d m_pathPlannerTargetPose = new Pose2d();

  // A field pose is not trustworthy just because the robot starts at (0, 0).
  // Initialize using a verified multi-tag pose while Disabled, or via
  // PathPlanner's explicit starting pose reset.
  private boolean m_fieldPoseInitialized = false;
  private double m_lastAcceptedVisionTime = -1.0;
  private long m_visionMeasurementsApplied = 0;

  // Swerve/navX2 odometry with optional, latency-compensated vision fusion.
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Field", m_field);
    driveTab.addDouble("Pose X (m)", () -> getPose().getX());
    driveTab.addDouble("Pose Y (m)", () -> getPose().getY());
    driveTab.addDouble("Pose Heading (deg)", () -> getPose().getRotation().getDegrees());
    driveTab.addBoolean("NavX Connected", m_gyro::isConnected);
    driveTab.addBoolean("NavX Ready", this::isGyroReady);
    driveTab.addBoolean("Field Pose Initialized", this::isFieldPoseInitialized);
    driveTab.addDouble("Latest Fused Vision Age (s)", this::getLastAcceptedVisionAgeSeconds);
    driveTab.addDouble("Fused Vision Count", () -> m_visionMeasurementsApplied);
    driveTab.addBoolean("PathPlanner Ready", () -> m_pathPlannerConfigured);

    ShuffleboardTab debug = Shuffleboard.getTab("Debug");
    debug.addBoolean("NavX Calibrating", m_gyro::isCalibrating);
    debug.addDouble("NavX Heading", this::getHeading);
    debug.addDouble("NavX Turn Rate", this::getTurnRate);
    debug.addDouble("NavX Yaw", m_gyro::getYaw);
    debug.addDouble("NavX Pitch", m_gyro::getPitch);
    debug.addDouble("NavX Roll", m_gyro::getRoll);
    debug.addDouble("NavX Compass Heading", m_gyro::getCompassHeading);
    debug.addDouble("NavX Fused Heading", m_gyro::getFusedHeading);
    debug.addBoolean("NavX Moving", m_gyro::isMoving);
    debug.addBoolean("NavX Rotating", m_gyro::isRotating);
    debug.addDouble("Robot Velocity X", () -> getRobotRelativeSpeeds().vxMetersPerSecond);
    debug.addDouble("Robot Velocity Y", () -> getRobotRelativeSpeeds().vyMetersPerSecond);
    debug.addDouble("Robot Angular Velocity",
        () -> Math.toDegrees(getRobotRelativeSpeeds().omegaRadiansPerSecond));
    debug.addBoolean("Autonomous Enabled", DriverStation::isAutonomousEnabled);
    debug.addBoolean("PP Target Valid", () -> m_pathPlannerTargetValid);
    debug.addDouble("PP Target X", () -> m_pathPlannerTargetPose.getX());
    debug.addDouble("PP Target Y", () -> m_pathPlannerTargetPose.getY());
    debug.addDouble("PP Target Heading", () -> m_pathPlannerTargetPose.getRotation().getDegrees());
    debug.addDouble("PP Current X", () -> m_pathPlannerCurrentPose.getX());
    debug.addDouble("PP Current Y", () -> m_pathPlannerCurrentPose.getY());
    debug.addDouble("PP Current Heading", () -> m_pathPlannerCurrentPose.getRotation().getDegrees());
    debug.addDouble("PP Path Points", () -> m_pathPlannerPathPoints);
    debug.addString("PP Active Path", () -> m_pathPlannerActivePath);
    addModuleDiagnostics(debug, "FL", m_frontLeft);
    addModuleDiagnostics(debug, "FR", m_frontRight);
    addModuleDiagnostics(debug, "RL", m_rearLeft);
    addModuleDiagnostics(debug, "RR", m_rearRight);

    configurePathPlanner();
  }

  /** Configures PathPlanner AutoBuilder to control this swerve drivetrain. */
  private void configurePathPlanner() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveRobotRelative(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);

      PathPlannerLogging.setLogCurrentPoseCallback(
          pose -> {
            m_field.setRobotPose(pose);
            m_pathPlannerCurrentPose = pose;
          });

      PathPlannerLogging.setLogTargetPoseCallback(
          pose -> {
            m_field.getObject("PathPlanner Target").setPose(pose);
            m_pathPlannerTargetValid = true;
            m_pathPlannerTargetPose = pose;
          });

      PathPlannerLogging.setLogActivePathCallback(
          poses -> {
            m_field.getObject("PathPlanner Path").setPoses(poses);
            m_pathPlannerActivePath = serializePath(poses);
            m_pathPlannerPathPoints = poses.size();
            if (poses.isEmpty()) {
              m_pathPlannerTargetValid = false;
            }
          });

      m_pathPlannerConfigured = true;
    } catch (Exception e) {
      m_pathPlannerConfigured = false;
      DriverStation.reportError(
          "Failed to configure PathPlanner. Save Robot Config in the PathPlanner GUI first.",
          e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Update the field visualization. Other values are live Shuffleboard suppliers.
    m_field.setRobotPose(getPose());
  }

  /** Full per-module values are retained on Debug, away from match controls. */
  private static void addModuleDiagnostics(ShuffleboardTab debug, String name, MAXSwerveModule module) {
    debug.addDouble("Swerve " + name + " Actual Speed", () -> module.getState().speedMetersPerSecond);
    debug.addDouble("Swerve " + name + " Actual Angle",
        () -> module.getState().angle.getDegrees());
    debug.addDouble("Swerve " + name + " Target Speed",
        () -> module.getDesiredState().speedMetersPerSecond);
    debug.addDouble("Swerve " + name + " Target Angle",
        () -> module.getDesiredState().angle.getDegrees());
  }

  /** Serializes the active PathPlanner path for the custom dashboard. */
  private static String serializePath(List<Pose2d> poses) {
    StringJoiner joiner = new StringJoiner(";");
    for (Pose2d pose : poses) {
      joiner.add(String.format(
          Locale.US,
          "%.4f,%.4f,%.2f",
          pose.getX(),
          pose.getY(),
          pose.getRotation().getDegrees()));
    }
    return joiner.toString();
  }

  /**
   * Returns the gyro heading using WPILib's counterclockwise-positive Rotation2d convention.
   */
  private Rotation2d getGyroRotation() {
    return m_gyro.getRotation2d();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
    m_fieldPoseInitialized = true;
  }

  public boolean isFieldPoseInitialized() {
    return m_fieldPoseInitialized;
  }

  public boolean isGyroReady() {
    return m_gyro.isConnected() && !m_gyro.isCalibrating();
  }

  public double getLastAcceptedVisionAgeSeconds() {
    return m_lastAcceptedVisionTime < 0.0 ? -1.0
        : Timer.getFPGATimestamp() - m_lastAcceptedVisionTime;
  }

  public boolean hasRecentVision(double maxAgeSeconds) {
    double age = getLastAcceptedVisionAgeSeconds();
    return age >= 0.0 && age <= maxAgeSeconds;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                m_fieldPoseInitialized ? getPose().getRotation() : getGyroRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Returns the measured module states in FL, FR, RL, RR order. */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  /** Returns current robot-relative chassis speeds for PathPlanner. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /** Drives the robot from robot-relative chassis speeds supplied by PathPlanner. */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds));
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Plot accepted camera-only estimates separately from the fused field pose. */
  public void showVisionPose(Pose2d visionPose) {
    m_field.getObject("Vision Raw Pose").setPose(visionPose);
  }

  /**
   * Timestamped vision fusion; wheel+navX odometry remains authoritative
   * between camera frames. Covariance is supplied by the validated vision
   * observation based on tag range/count (FRC 5024/6328 pattern).
   *
   * Vision heading is deliberately assigned large uncertainty so that
   * navX controls short-term heading; camera corrects field translation.
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds,
      double xyStdDevMeters, double thetaStdDevRadians) {
    if (!m_fieldPoseInitialized
        || !Double.isFinite(xyStdDevMeters)
        || !Double.isFinite(thetaStdDevRadians)
        || xyStdDevMeters <= 0.0 || thetaStdDevRadians <= 0.0) {
      return;
    }
    m_poseEstimator.addVisionMeasurement(
        visionPose, timestampSeconds,
        VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, thetaStdDevRadians));
    m_lastAcceptedVisionTime = Timer.getFPGATimestamp();
    m_visionMeasurementsApplied++;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Reset the raw navX heading without destroying the localized field pose.
   * Once global coordinates are established, field orientation must not jump
   * just because the driver presses Start.
   */
  public void zeroHeading() {
    Pose2d previousPose = getPose();
    m_gyro.reset();
    boolean wasInitialized = m_fieldPoseInitialized;
    Pose2d retainedPose = wasInitialized
        ? previousPose
        : new Pose2d(previousPose.getTranslation(), Rotation2d.kZero);
    resetOdometry(retainedPose);
    m_fieldPoseInitialized = wasInitialized;
  }

  /**
   * Returns the robot heading in WPILib's counterclockwise-positive convention.
   *
   * @return heading in degrees, wrapped to -180 to 180
   */
  public double getHeading() {
    return getGyroRotation().getDegrees();
  }

  /**
   * Returns the turn rate in WPILib's counterclockwise-positive convention.
   *
   * @return turn rate in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
