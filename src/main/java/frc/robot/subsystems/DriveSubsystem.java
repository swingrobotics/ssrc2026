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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
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

  // Pose estimator continuously integrates navX2 + swerve encoders and accepts
  // delayed PhotonVision field measurements. PathPlanner uses the same pose.
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      getModulePositions(),
      new Pose2d());

  // (0,0) is an uninitialized placeholder, not a real field starting location.
  private boolean m_poseInitialized = false;
  // A reset from PathPlanner or navX changes the local coordinate reference.
  private long m_poseFrameRevision = 0;
  private boolean m_bumpActive = false;
  private boolean m_bumpRecoveryRequired = false;
  private double m_bumpSettledSince = -1.0;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putString("PathPlanner Active Path", "");
    SmartDashboard.putNumber("PathPlanner Path Points", 0);
    SmartDashboard.putBoolean("PathPlanner Target Valid", false);
    SmartDashboard.putNumber("PathPlanner Target X", 0.0);
    SmartDashboard.putNumber("PathPlanner Target Y", 0.0);
    SmartDashboard.putNumber("PathPlanner Target Heading", 0.0);

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
            SmartDashboard.putNumber("PathPlanner Current X", pose.getX());
            SmartDashboard.putNumber("PathPlanner Current Y", pose.getY());
            SmartDashboard.putNumber(
                "PathPlanner Current Heading", pose.getRotation().getDegrees());
          });

      PathPlannerLogging.setLogTargetPoseCallback(
          pose -> {
            m_field.getObject("PathPlanner Target").setPose(pose);
            SmartDashboard.putBoolean("PathPlanner Target Valid", true);
            SmartDashboard.putNumber("PathPlanner Target X", pose.getX());
            SmartDashboard.putNumber("PathPlanner Target Y", pose.getY());
            SmartDashboard.putNumber(
                "PathPlanner Target Heading", pose.getRotation().getDegrees());
          });

      PathPlannerLogging.setLogActivePathCallback(
          poses -> {
            m_field.getObject("PathPlanner Path").setPoses(poses);
            SmartDashboard.putString("PathPlanner Active Path", serializePath(poses));
            SmartDashboard.putNumber("PathPlanner Path Points", poses.size());
            if (poses.isEmpty()) {
              SmartDashboard.putBoolean("PathPlanner Target Valid", false);
            }
          });

      SmartDashboard.putBoolean("PathPlanner Configured", true);
    } catch (Exception e) {
      SmartDashboard.putBoolean("PathPlanner Configured", false);
      DriverStation.reportError(
          "Failed to configure PathPlanner. Save Robot Config in the PathPlanner GUI first.",
          e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    m_poseEstimator.update(getGyroRotation(), getModulePositions());
    updateBumpDetection();

    // navX2 status/orientation values.
    SmartDashboard.putBoolean("NavX Connected", m_gyro.isConnected());
    SmartDashboard.putBoolean("NavX Calibrating", m_gyro.isCalibrating());
    SmartDashboard.putNumber("NavX Heading", getHeading());
    SmartDashboard.putNumber("NavX Turn Rate", getTurnRate());
    SmartDashboard.putNumber("NavX Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("NavX Pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("NavX Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("NavX Compass Heading", m_gyro.getCompassHeading());
    SmartDashboard.putNumber("NavX Fused Heading", m_gyro.getFusedHeading());
    SmartDashboard.putBoolean("NavX Moving", m_gyro.isMoving());
    SmartDashboard.putBoolean("NavX Rotating", m_gyro.isRotating());

    // Pose/odometry values for comparison with the ADIS test branch.
    Pose2d pose = getPose();
    m_field.setRobotPose(pose);
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Heading", pose.getRotation().getDegrees());
    SmartDashboard.putBoolean("Pose Initialized", m_poseInitialized);
    SmartDashboard.putBoolean("Bump Active", m_bumpActive);
    SmartDashboard.putBoolean("Bump Recovery Required", m_bumpRecoveryRequired);

    ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
    SmartDashboard.putNumber("Robot Velocity X", robotSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot Velocity Y", robotSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "Robot Angular Velocity", Math.toDegrees(robotSpeeds.omegaRadiansPerSecond));
    SmartDashboard.putBoolean("Autonomous Enabled", DriverStation.isAutonomousEnabled());

    publishModuleTelemetry("FL", m_frontLeft);
    publishModuleTelemetry("FR", m_frontRight);
    publishModuleTelemetry("RL", m_rearLeft);
    publishModuleTelemetry("RR", m_rearRight);
  }

  /** Publishes measured and optimized target state for one swerve module. */
  private static void publishModuleTelemetry(String name, MAXSwerveModule module) {
    SwerveModuleState actual = module.getState();
    SwerveModuleState target = module.getDesiredState();

    SmartDashboard.putNumber(
        "Swerve " + name + " Actual Speed", actual.speedMetersPerSecond);
    SmartDashboard.putNumber(
        "Swerve " + name + " Actual Angle", actual.angle.getDegrees());
    SmartDashboard.putNumber(
        "Swerve " + name + " Target Speed", target.speedMetersPerSecond);
    SmartDashboard.putNumber(
        "Swerve " + name + " Target Angle", target.angle.getDegrees());
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

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /** Called from PathPlanner. A new field frame invalidates remembered test targets. */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    m_poseInitialized = true;
    m_bumpRecoveryRequired = false;
    m_poseFrameRevision++;
  }

  /** Local test frame is usable without a known official FRC field starting pose. */
  public void enableLocalPoseFrame() {
    m_poseInitialized = true;
  }

  public long getPoseFrameRevision() {
    return m_poseFrameRevision;
  }

  /** Pose at PhotonVision frame capture time, including estimator latency history. */
  public java.util.Optional<Pose2d> getPoseAt(double timestampSeconds) {
    return m_poseEstimator.sampleAt(timestampSeconds);
  }

  /**
   * Re-localize the SAME local coordinate frame from repeated matching-tag observations.
   * Does not increment the frame revision: remembered tag position remains valid.
   */
  public void relocalizeFromRememberedTag(Pose2d pose) {
    m_poseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
    m_poseInitialized = true;
    m_bumpRecoveryRequired = false;
  }

  public void addVisionPose(Pose2d pose, double timestampSeconds, boolean multiTag) {
    m_poseEstimator.addVisionMeasurement(
        pose,
        timestampSeconds,
        multiTag ? VecBuilder.fill(0.20, 0.20, 0.35)
            : VecBuilder.fill(0.65, 0.65, 0.80));
    m_poseInitialized = true;
    if (!m_bumpActive) {
      m_bumpRecoveryRequired = false;
    }
  }

  public boolean isPoseInitialized() {
    return m_poseInitialized;
  }

  public boolean isBumpActive() {
    return m_bumpActive;
  }

  public boolean needsBumpRecovery() {
    return m_bumpRecoveryRequired;
  }

  public boolean isGyroReady() {
    return m_gyro.isConnected() && !m_gyro.isCalibrating();
  }

  /** Mark bump crossing using navX2 tilt with hysteresis to avoid noisy toggling. */
  private void updateBumpDetection() {
    double pitch = m_gyro.getPitch();
    double roll = m_gyro.getRoll();
    if (!Double.isFinite(pitch) || !Double.isFinite(roll) || !isGyroReady()) {
      m_bumpRecoveryRequired = true;
      return;
    }
    double tilt = Math.max(Math.abs(pitch), Math.abs(roll));
    if (tilt >= VisionConstants.BUMP_ENTRY_TILT_DEG) {
      m_bumpActive = true;
      m_bumpRecoveryRequired = true;
      m_bumpSettledSince = -1.0;
      return;
    }
    if (!m_bumpActive) {
      return;
    }
    if (tilt > VisionConstants.BUMP_EXIT_TILT_DEG) {
      m_bumpSettledSince = -1.0;
      return;
    }
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (m_bumpSettledSince < 0.0) {
      m_bumpSettledSince = now;
    } else if (now - m_bumpSettledSince >= VisionConstants.BUMP_SETTLE_SECONDS) {
      m_bumpActive = false;
    }
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
                getPose().getRotation())
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

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Do not reset navX2's raw heading after localization; preserve the field frame. */
  public void zeroHeading() {
    m_gyro.reset();
    // WPILib's estimator keeps its current pose through a gyro offset update.
    // A verified field pose must still be re-established before HUB LOCK is trusted.
    m_poseInitialized = false;
    m_bumpRecoveryRequired = false;
    m_poseFrameRevision++;
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
