// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import org.photonvision.PhotonCamera;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // PhotonVision camera nickname. Must exactly match the nickname in the PhotonVision UI.
  private static final String kPhotonCameraName = "PC_Camera";
  private final PhotonCamera m_photonCamera = new PhotonCamera(kPhotonCameraName);

  // Smooth vision-heading controller.
  // PhotonVision supplies yaw; the ADIS16470 gyro then handles the fast, smooth heading control.
  private static final double kVisionHeadingKp = 0.015;
  private static final double kVisionMaxTurnRateDegPerSec = Math.toDegrees(DriveConstants.kMaxAngularSpeed);
  private static final double kVisionMaxTurnAccelDegPerSecSq = 540.0;
  private static final double kVisionYawToleranceDegrees = 1.0;
  private static final double kVisionTargetHoldSeconds = 0.20;

  private final ProfiledPIDController m_visionHeadingController = new ProfiledPIDController(
      kVisionHeadingKp,
      0.0,
      0.0,
      new TrapezoidProfile.Constraints(
          kVisionMaxTurnRateDegPerSec,
          kVisionMaxTurnAccelDegPerSecSq));

  // Vision auto-align state.
  private boolean m_visionAlignEnabled = false;
  private boolean m_visionTargetVisible = false;
  private double m_visionTargetYawDegrees = 0.0;
  private double m_visionGoalHeadingDegrees = 0.0;
  private double m_lastVisionTargetTimestamp = -1.0;
  private double m_visionRotationCommand = 0.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    // Heading is an angle, so make the controller use the shortest path across -180/180.
    m_visionHeadingController.enableContinuousInput(-180.0, 180.0);
    m_visionHeadingController.setTolerance(kVisionYawToleranceDegrees, 5.0);

    SmartDashboard.putBoolean("Vision Align Enabled", false);
    SmartDashboard.putBoolean("Vision Target Visible", false);
    SmartDashboard.putNumber("Vision Target Yaw", 0.0);
    SmartDashboard.putNumber("Vision Goal Heading", 0.0);
    SmartDashboard.putNumber("Vision Rotation Command", 0.0);

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                getDriveRotationCommand(),
                false),
            m_robotDrive));
  }

  /**
   * Returns the normal manual rotation command from the controller triggers.
   * Left trigger = counterclockwise, right trigger = clockwise.
   */
  private double getTriggerRotation() {
    double leftRotation = MathUtil.applyDeadband(
        m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband);
    double rightRotation = MathUtil.applyDeadband(
        m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband);
    return leftRotation - rightRotation;
  }

  /**
   * Reads the newest PhotonVision frame. A valid target updates an absolute gyro heading goal.
   * Brief camera-frame gaps keep the last goal so the drivetrain does not repeatedly stop/start.
   */
  private void updateVisionTarget() {
    var results = m_photonCamera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        var target = result.getBestTarget();
        if (target != null && target.getFiducialId() >= 0) {
          m_visionTargetYawDegrees = target.getYaw();
          m_lastVisionTargetTimestamp = Timer.getFPGATimestamp();

          if (m_visionAlignEnabled) {
            double currentHeading = m_robotDrive.getHeading();

            // PhotonVision yaw is positive-left, while the existing drivetrain's working
            // auto-aim direction is -yaw. Convert that relative yaw into an absolute gyro goal.
            m_visionGoalHeadingDegrees = MathUtil.inputModulus(
                currentHeading - m_visionTargetYawDegrees,
                -180.0,
                180.0);
            m_visionHeadingController.setGoal(m_visionGoalHeadingDegrees);
          }
        }
      }
    }

    m_visionTargetVisible = m_lastVisionTargetTimestamp >= 0.0
        && (Timer.getFPGATimestamp() - m_lastVisionTargetTimestamp) <= kVisionTargetHoldSeconds;

    if (!m_visionTargetVisible) {
      m_visionTargetYawDegrees = 0.0;
    }

    SmartDashboard.putBoolean("Vision Align Enabled", m_visionAlignEnabled);
    SmartDashboard.putBoolean("Vision Target Visible", m_visionTargetVisible);
    SmartDashboard.putNumber("Vision Target Yaw", m_visionTargetYawDegrees);
    SmartDashboard.putNumber("Vision Goal Heading", m_visionGoalHeadingDegrees);
    SmartDashboard.putNumber("Vision Rotation Command", m_visionRotationCommand);
  }

  /**
   * Manual trigger rotation when auto-align is off. When auto-align is on, PhotonVision updates the
   * desired heading and WPILib's ProfiledPIDController makes the rotation fast but acceleration-
   * limited and smooth.
   */
  private double getDriveRotationCommand() {
    updateVisionTarget();

    if (!m_visionAlignEnabled) {
      m_visionRotationCommand = getTriggerRotation();
      return m_visionRotationCommand;
    }

    double currentHeading = m_robotDrive.getHeading();

    if (!m_visionTargetVisible) {
      // The tag has been gone longer than the short dropout window. Stop auto-rotation safely.
      m_visionHeadingController.reset(currentHeading, m_robotDrive.getTurnRate());
      m_visionHeadingController.setGoal(currentHeading);
      m_visionGoalHeadingDegrees = currentHeading;
      m_visionRotationCommand = 0.0;
      return 0.0;
    }

    // ProfiledPIDController smoothly moves its internal heading setpoint toward the vision goal.
    // The profile velocity becomes the main angular-speed command; P corrects gyro tracking error.
    double headingCorrection = m_visionHeadingController.calculate(currentHeading);
    double profiledTurnCommand =
        m_visionHeadingController.getSetpoint().velocity / kVisionMaxTurnRateDegPerSec;

    m_visionRotationCommand = MathUtil.clamp(
        profiledTurnCommand + headingCorrection,
        -1.0,
        1.0);

    // Once centered and nearly stopped, remove tiny residual commands that can cause chatter.
    if (Math.abs(m_visionTargetYawDegrees) <= kVisionYawToleranceDegrees
        && Math.abs(m_robotDrive.getTurnRate()) <= 5.0
        && m_visionHeadingController.atGoal()) {
      m_visionRotationCommand = 0.0;
    }

    return m_visionRotationCommand;
  }

  /** Toggle PhotonVision AprilTag auto-alignment on/off. */
  private void toggleVisionAlign() {
    m_visionAlignEnabled = !m_visionAlignEnabled;

    double currentHeading = m_robotDrive.getHeading();
    m_visionHeadingController.reset(currentHeading, m_robotDrive.getTurnRate());
    m_visionHeadingController.setGoal(currentHeading);
    m_visionGoalHeadingDegrees = currentHeading;
    m_visionRotationCommand = 0.0;

    // Require a fresh target after enabling so an old frame cannot start a turn.
    if (m_visionAlignEnabled) {
      m_lastVisionTargetTimestamp = -1.0;
      m_visionTargetVisible = false;
      m_visionTargetYawDegrees = 0.0;
    }

    SmartDashboard.putBoolean("Vision Align Enabled", m_visionAlignEnabled);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // User-requested toggle behavior: press A once for ON, press A again for OFF.
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(this::toggleVisionAlign));

    // Original right-bumper X-lock behavior.
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
