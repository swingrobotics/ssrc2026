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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  // PhotonVision camera nickname. This must match the camera nickname in the PhotonVision UI.
  private static final String kPhotonCameraName = "photonvision";
  private final PhotonCamera m_photonCamera = new PhotonCamera(kPhotonCameraName);

  // Vision auto-aim settings.
  // PhotonVision yaw is positive when the target is to the left.
  private static final double kVisionTurnKp = 0.02;
  private static final double kVisionMaxRotationCommand = 0.60;
  private static final double kVisionYawToleranceDegrees = 1.0;
  private static final double kVisionTargetTimeoutSeconds = 0.25;

  // Vision auto-aim state.
  private boolean m_visionAlignEnabled = false;
  private boolean m_visionTargetVisible = false;
  private int m_lockedVisionTargetId = -1;
  private double m_visionTargetYawDegrees = 0.0;
  private double m_lastVisionTargetTimestamp = -1.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putBoolean("Vision Align Enabled", false);
    SmartDashboard.putBoolean("Vision Target Visible", false);
    SmartDashboard.putNumber("Vision Locked Tag ID", -1);
    SmartDashboard.putNumber("Vision Target Yaw", 0.0);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // Left stick controls translation relative to the robot.
        // Triggers control manual rotation unless vision auto-align is enabled.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                getDriveRotationCommand(),
                false),
            m_robotDrive));
  }

  /**
   * Returns the manual rotation command from the controller triggers.
   * Left trigger = counterclockwise, right trigger = clockwise.
   * Trigger pressure controls rotation speed.
   */
  private double getTriggerRotation() {
    double leftRotation = MathUtil.applyDeadband(
        m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband);
    double rightRotation = MathUtil.applyDeadband(
        m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband);
    return leftRotation - rightRotation;
  }

  /**
   * Returns either manual trigger rotation or PhotonVision auto-aim rotation.
   */
  private double getDriveRotationCommand() {
    if (!m_visionAlignEnabled) {
      return getTriggerRotation();
    }

    updateVisionTarget();

    if (!m_visionTargetVisible) {
      // Do not rotate using stale target information.
      return 0.0;
    }

    // Stop hunting once the tag is very close to the center of the camera image.
    if (Math.abs(m_visionTargetYawDegrees) <= kVisionYawToleranceDegrees) {
      return 0.0;
    }

    // Positive PhotonVision yaw means the target is to the left.
    // Positive swerve rotation is counterclockwise, so the signs match directly.
    return MathUtil.clamp(
        m_visionTargetYawDegrees * kVisionTurnKp,
        -kVisionMaxRotationCommand,
        kVisionMaxRotationCommand);
  }

  /**
   * Reads the newest PhotonVision frame and tracks one AprilTag continuously.
   * When auto-align is first enabled, the best visible AprilTag is locked by ID.
   */
  private void updateVisionTarget() {
    var results = m_photonCamera.getAllUnreadResults();

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      boolean foundLockedTarget = false;

      if (result.hasTargets()) {
        if (m_lockedVisionTargetId < 0) {
          // Lock onto the best tag visible when auto-align starts.
          var target = result.getBestTarget();
          if (target != null && target.getFiducialId() >= 0) {
            m_lockedVisionTargetId = target.getFiducialId();
            m_visionTargetYawDegrees = target.getYaw();
            foundLockedTarget = true;
          }
        } else {
          // Keep following the same tag so we do not jump between multiple visible tags.
          for (var target : result.getTargets()) {
            if (target.getFiducialId() == m_lockedVisionTargetId) {
              m_visionTargetYawDegrees = target.getYaw();
              foundLockedTarget = true;
              break;
            }
          }
        }
      }

      m_visionTargetVisible = foundLockedTarget;
      if (foundLockedTarget) {
        m_lastVisionTargetTimestamp = Timer.getFPGATimestamp();
      }
    }

    // If frames stop arriving, do not continue driving from old yaw data.
    if (m_visionTargetVisible
        && (Timer.getFPGATimestamp() - m_lastVisionTargetTimestamp) > kVisionTargetTimeoutSeconds) {
      m_visionTargetVisible = false;
    }

    SmartDashboard.putBoolean("Vision Align Enabled", m_visionAlignEnabled);
    SmartDashboard.putBoolean("Vision Target Visible", m_visionTargetVisible);
    SmartDashboard.putNumber("Vision Locked Tag ID", m_lockedVisionTargetId);
    SmartDashboard.putNumber("Vision Target Yaw", m_visionTargetYawDegrees);
  }

  /** Toggle PhotonVision AprilTag auto-alignment on/off. */
  private void toggleVisionAlign() {
    m_visionAlignEnabled = !m_visionAlignEnabled;

    // Start fresh every time auto-align is enabled or disabled.
    m_lockedVisionTargetId = -1;
    m_visionTargetVisible = false;
    m_visionTargetYawDegrees = 0.0;
    m_lastVisionTargetTimestamp = -1.0;

    SmartDashboard.putBoolean("Vision Align Enabled", m_visionAlignEnabled);
    SmartDashboard.putBoolean("Vision Target Visible", false);
    SmartDashboard.putNumber("Vision Locked Tag ID", -1);
    SmartDashboard.putNumber("Vision Target Yaw", 0.0);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}).
   */
  private void configureButtonBindings() {
    // A button toggles AprilTag auto-alignment.
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
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
