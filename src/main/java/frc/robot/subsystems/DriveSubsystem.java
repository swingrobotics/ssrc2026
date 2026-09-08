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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

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

  // Odometry class for tracking robot pose.
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getGyroRotation(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    SmartDashboard.putData("Field", m_field);

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

      PathPlannerLogging.setLogActivePathCallback(
          poses -> m_field.getObject("PathPlanner Path").setPoses(poses));

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
    m_odometry.update(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

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
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getGyroRotation(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
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
                getGyroRotation())
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

  /** Zeroes the navX2 yaw/heading. */
  public void zeroHeading() {
    m_gyro.reset();
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
