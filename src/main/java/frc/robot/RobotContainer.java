// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double kDefaultDrivePercent = 100.0;
  private static final double kDefaultShooterPercent = Math.abs(ShooterConstants.TEST_SPEED) * 100.0;
  private static final double kDefaultFeederPercent = Math.abs(FeederConstants.TEST_SPEED) * 100.0;

  // The original swerve/vision/autonomous subsystem, plus the ZIP's mechanism subsystems.
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();

  private final GenericEntry m_driveSpeedEntry;
  private final GenericEntry m_shooterSpeedEntry;
  private final GenericEntry m_feederSpeedEntry;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // PathPlanner autonomous chooser.
  private final SendableChooser<Command> m_autoChooser;

  private final HubVision m_hubVision = new HubVision();

  // Driving mode. False keeps robot-relative translation by default.
  private boolean m_fieldRelativeEnabled = false;

  // Phase 1 test: X toggles pointing at the ONE remembered elevated AprilTag.
  private boolean m_visionAlignEnabled = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Preserve the mechanism ZIP's Test-tab output sliders, in percent.
    ShuffleboardTab testTab = Shuffleboard.getTab("Test");
    m_driveSpeedEntry = testTab.add("Drive Speed (%)", kDefaultDrivePercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0))
        .getEntry();
    m_shooterSpeedEntry = testTab.add("Shooter Speed (%)", kDefaultShooterPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0))
        .getEntry();
    m_feederSpeedEntry = testTab.add("Feeder Speed (%)", kDefaultFeederPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0))
        .getEntry();

    configureButtonBindings();

    SmartDashboard.putBoolean("Field Relative Enabled", m_fieldRelativeEnabled);
    SmartDashboard.putBoolean("Vision Align Enabled", false);
    SmartDashboard.putBoolean("Vision Target Visible", false);
    SmartDashboard.putNumber("Vision Target Yaw", 0.0);

    SendableChooser<Command> autoChooser;
    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("None", Commands.none());
    }
    m_autoChooser = autoChooser;
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putString("Selected Auto", "None");

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
              double scale = getOutputScale(m_driveSpeedEntry, kDefaultDrivePercent);
              m_robotDrive.drive(
                  -scale * MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                  -scale * MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                  scale * getDriveRotationCommand(),
                  m_fieldRelativeEnabled);
            },
            m_robotDrive));

    // Right stick Y manually moves the hood, using the ZIP's low output cap.
    m_hood.setDefaultCommand(
        new RunCommand(
            () -> {
              double hoodInput = MathUtil.applyDeadband(
                  -m_driverController.getRightY(), OIConstants.kHoodDeadband);
              m_hood.setManual(hoodInput * HoodConstants.TEST_MAX_SPEED);
            },
            m_hood));
  }

  /** Read a 0-100% Shuffleboard slider and safely convert to an output multiplier. */
  private static double getOutputScale(GenericEntry entry, double defaultPercent) {
    double percent = entry.getDouble(defaultPercent);
    return Double.isFinite(percent) ? MathUtil.clamp(percent, 0.0, 100.0) / 100.0 : 0.0;
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

  /** Manual trigger rotation takes priority; phase-1 tag memory calculates heading. */
  private double getDriveRotationCommand() {
    double manual = getTriggerRotation();
    if (!m_visionAlignEnabled) {
      return manual;
    }
    return m_hubVision.calculateRotation(m_robotDrive, manual);
  }

  /** Toggle between robot-relative and field-relative translation control. */
  private void toggleFieldRelative() {
    m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
    SmartDashboard.putBoolean("Field Relative Enabled", m_fieldRelativeEnabled);
  }

  /** Toggle heading toward the remembered target, including when it is out of view. */
  private void toggleVisionAlign() {
    m_visionAlignEnabled = !m_visionAlignEnabled;
    m_hubVision.resetHeadingController();
    SmartDashboard.putBoolean("Vision Align Enabled", m_visionAlignEnabled);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // ZIP mapping takes precedence: A holds Feeder; releasing A stops it.
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(Commands.runEnd(
            () -> m_feeder.runFeeder(getOutputScale(m_feederSpeedEntry, kDefaultFeederPercent)),
            m_feeder::stopFeeder,
            m_feeder));

    // ZIP mapping takes precedence: B holds Shooter; preserve negative output direction.
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .whileTrue(Commands.runEnd(
            () -> m_shooter.runShooter(-getOutputScale(m_shooterSpeedEntry, kDefaultShooterPercent)),
            m_shooter::stopShooter,
            m_shooter));

    // Move the original A/B toggle functions to unused X/Y buttons.
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(this::toggleVisionAlign));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(this::toggleFieldRelative));

    // Original right-bumper X-lock behavior.
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> {
              m_robotDrive.zeroHeading();
              m_hubVision.clearMemory();
              m_visionAlignEnabled = false;
            },
            m_robotDrive));
  }

  /** Publishes dashboard-only state that is not owned by a subsystem. */
  public void updateDashboard() {
    // Run even during autonomous and disabled: only one consumer of camera frames.
    m_hubVision.update(m_robotDrive);
    SmartDashboard.putBoolean("Vision Align Enabled", m_visionAlignEnabled);
    SmartDashboard.putBoolean("Tag Lock Enabled", m_visionAlignEnabled);
    Command selectedAuto = m_autoChooser.getSelected();
    SmartDashboard.putString(
        "Selected Auto",
        selectedAuto != null ? selectedAuto.getName() : "None");
  }

  /**
   * Returns the autonomous routine selected from the PathPlanner chooser.
   *
   * @return selected autonomous command
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
