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

  // PhotonVision's official aiming example uses target yaw with a proportional gain.
  // DriveSubsystem.drive() accepts a normalized rotation command and scales it by max angular speed.
  private static final double kVisionTurnKp = 0.01;

  // Vision auto-align state.
  private boolean m_visionAlignEnabled = false;
  private boolean m_visionTargetVisible = false;
  private double m_visionTargetYawDegrees = 0.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    SmartDashboard.putBoolean("Vision Align Enabled", false);
    SmartDashboard.putBoolean("Vision Target Visible", false);
    SmartDashboard.putNumber("Vision Target Yaw", 0.0);

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
   * Reads the newest PhotonVision result using the same pattern as PhotonVision's official
   * "Aiming at a Target" example: getAllUnreadResults(), use the newest result, then read yaw.
   */
  private void updateVisionTarget() {
    m_visionTargetVisible = false;
    m_visionTargetYawDegrees = 0.0;

    var results = m_photonCamera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed at least one new frame since the last call. Use the newest frame.
      var result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        // For this first version, aim at PhotonVision's best visible AprilTag.
        var target = result.getBestTarget();
        if (target != null && target.getFiducialId() >= 0) {
          m_visionTargetYawDegrees = target.getYaw();
          m_visionTargetVisible = true;
        }
      }
    }

    SmartDashboard.putBoolean("Vision Align Enabled", m_visionAlignEnabled);
    SmartDashboard.putBoolean("Vision Target Visible", m_visionTargetVisible);
    SmartDashboard.putNumber("Vision Target Yaw", m_visionTargetYawDegrees);
  }

  /**
   * Returns manual trigger rotation when auto-align is off. When auto-align is on and an
   * AprilTag is visible, it overrides manual rotation with PhotonVision yaw-based aiming.
   */
  private double getDriveRotationCommand() {
    updateVisionTarget();

    if (!m_visionAlignEnabled) {
      return getTriggerRotation();
    }

    if (!m_visionTargetVisible) {
      // Auto-align is enabled but no current target is available, so do not rotate.
      return 0.0;
    }

    // PhotonVision official example:
    // turn = -1.0 * targetYaw * VISION_TURN_kP * maxAngularSpeed
    // Our DriveSubsystem applies maxAngularSpeed internally, so return the normalized part here.
    return MathUtil.clamp(
        -1.0 * m_visionTargetYawDegrees * kVisionTurnKp,
        -1.0,
        1.0);
  }

  /** Toggle PhotonVision AprilTag auto-alignment on/off. */
  private void toggleVisionAlign() {
    m_visionAlignEnabled = !m_visionAlignEnabled;
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
