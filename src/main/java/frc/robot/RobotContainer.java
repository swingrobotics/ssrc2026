// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import java.util.Optional;

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
  // Creates no motor controllers until both intake CAN IDs are assigned.
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final GenericEntry m_driveSpeedEntry;
  private final GenericEntry m_shooterSpeedEntry;
  private final GenericEntry m_feederSpeedEntry;
  private final GenericEntry m_visionFusionEntry;
  private final GenericEntry m_visionResetEntry;
  private double m_lastTagId = -1.0;
  private String m_poseSolver = "None";
  private String m_visionResetStatus = "Not requested";

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // PathPlanner autonomous chooser.
  private final SendableChooser<Command> m_autoChooser;

  // PhotonVision camera nickname. Must exactly match the nickname in the PhotonVision UI.
  private final PhotonCamera m_photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private final PhotonPoseEstimator m_photonPoseEstimator =
      new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_CAMERA);

  // PhotonVision's official aiming example uses target yaw with a proportional gain.
  // DriveSubsystem.drive() accepts a normalized rotation command and scales it by max angular speed.
  private static final double kVisionTurnKp = 0.01;

  // Keep the last valid target briefly between camera frames so the robot command does not
  // alternate between turning and zero when the robot loop runs faster than the camera FPS.
  private static final double kVisionTargetHoldSeconds = 0.20;

  // Driving mode. False keeps robot-relative translation by default.
  private boolean m_fieldRelativeEnabled = false;

  // Vision auto-align state.
  private boolean m_visionAlignEnabled = false;
  private boolean m_visionTargetVisible = false;
  private double m_visionTargetYawDegrees = 0.0;
  private double m_lastVisionTargetTimestamp = -1.0;
  private double m_lastVisionPoseReceivedAt = -1.0;
  private double m_lastVisionMeasurementTimestamp = -1.0;
  private Pose2d m_lastRawVisionPose = null;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Driver view: controls and live pose only.
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    m_driveSpeedEntry = driveTab.add("Drive Speed (%)", kDefaultDrivePercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
    m_shooterSpeedEntry = driveTab.add("Shooter Speed (%)", kDefaultShooterPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
    m_feederSpeedEntry = driveTab.add("Feeder Speed (%)", kDefaultFeederPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
    driveTab.addBoolean("Field Relative", () -> m_fieldRelativeEnabled);
    driveTab.addBoolean("Tag Auto Align", () -> m_visionAlignEnabled);

    // Vision view: minimal AprilTag pose readings and operator-controlled fusion.
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    visionTab.addBoolean("Tag Visible", () -> m_visionTargetVisible);
    visionTab.addDouble("Tag ID", () -> m_lastTagId);
    visionTab.addBoolean("Pose Fresh", this::isVisionPoseFresh);
    visionTab.addDouble("Raw Pose X (m)",
        () -> m_lastRawVisionPose == null ? -1.0 : m_lastRawVisionPose.getX());
    visionTab.addDouble("Raw Pose Y (m)",
        () -> m_lastRawVisionPose == null ? -1.0 : m_lastRawVisionPose.getY());
    visionTab.addDouble("Raw Heading (deg)",
        () -> m_lastRawVisionPose == null ? 0.0
            : m_lastRawVisionPose.getRotation().getDegrees());
    visionTab.addString("Pose Solver", () -> m_poseSolver);
    m_visionFusionEntry = visionTab.add("Fusion Enabled", false).getEntry();
    m_visionResetEntry = visionTab.add("Reset Pose (Disabled)", false).getEntry();
    visionTab.addString("Reset Status", () -> m_visionResetStatus);

    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    debugTab.addString("Vision Field Layout", () -> "2026 REBUILT Welded");
    debugTab.addString("Vision Camera", () -> VisionConstants.CAMERA_NAME);
    debugTab.addDouble("Vision Target Yaw", () -> m_visionTargetYawDegrees);
    debugTab.addDouble("Vision Pose Age (s)", this::getVisionPoseAgeSeconds);

    configureButtonBindings();

    SendableChooser<Command> autoChooser;
    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("None", Commands.none());
    }
    m_autoChooser = autoChooser;
    driveTab.add("Auto Chooser", m_autoChooser);
    driveTab.addString("Selected Auto", () -> {
      Command chosen = m_autoChooser.getSelected();
      return chosen != null ? chosen.getName() : "None";
    });

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

  /**
   * Read PhotonVision frames exactly once per robot loop. This supplies both
   * the existing visible-tag yaw aiming and the NEW official-field robot pose.
   * Runs in disabled, autonomous, and teleop via updateDashboard().
   */
  private void updateVisionTarget() {
    double now = Timer.getFPGATimestamp();

    for (var result : m_photonCamera.getAllUnreadResults()) {
      if (!result.hasTargets()) {
        continue;
      }

      var bestTarget = result.getBestTarget();
      if (bestTarget != null
          && VisionConstants.FIELD_LAYOUT.getTagPose(bestTarget.getFiducialId()).isPresent()) {
        m_visionTargetYawDegrees = bestTarget.getYaw();
        m_lastVisionTargetTimestamp = now;
        m_lastTagId = bestTarget.getFiducialId();
      }

      // The camera must use the SAME official Welded JSON in PhotonVision UI.
      // Coprocessor MultiTag is preferred; valid single-tag measurements
      // provide a fallback when only one AprilTag is visible.
      Optional<EstimatedRobotPose> estimate =
          m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
      boolean multiTag = estimate.isPresent();

      if (!multiTag
          && bestTarget != null
          && VisionConstants.FIELD_LAYOUT.getTagPose(bestTarget.getFiducialId()).isPresent()
          && bestTarget.getPoseAmbiguity() >= 0.0
          && bestTarget.getPoseAmbiguity() <= VisionConstants.MAX_SINGLE_TAG_AMBIGUITY
          && bestTarget.getBestCameraToTarget().getTranslation().getNorm()
              <= VisionConstants.MAX_SINGLE_TAG_DISTANCE_M) {
        estimate = m_photonPoseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (estimate.isEmpty()) {
        continue;
      }

      var measurement = estimate.get();
      Pose3d pose3d = measurement.estimatedPose;
      Pose2d measuredPose = pose3d.toPose2d();
      double timestamp = measurement.timestampSeconds;
      AprilTagFieldLayout layout = VisionConstants.FIELD_LAYOUT;
      double tolerance = VisionConstants.FIELD_BOUNDS_TOLERANCE_M;

      // Reject invalid/out-of-field/camera-tilt poses and out-of-order frames.
      if (!Double.isFinite(timestamp)
          || timestamp <= m_lastVisionMeasurementTimestamp
          || timestamp > Timer.getFPGATimestamp() + 0.1
          || timestamp < Timer.getFPGATimestamp() - 1.0
          || !Double.isFinite(pose3d.getX())
          || !Double.isFinite(pose3d.getY())
          || !Double.isFinite(pose3d.getZ())
          || !Double.isFinite(pose3d.getRotation().getX())
          || !Double.isFinite(pose3d.getRotation().getY())
          || !Double.isFinite(pose3d.getRotation().getZ())
          || Math.abs(pose3d.getZ()) > VisionConstants.MAX_ABS_ESTIMATED_HEIGHT_M
          || Math.abs(pose3d.getRotation().getX()) > Math.toRadians(25.0)
          || Math.abs(pose3d.getRotation().getY()) > Math.toRadians(25.0)
          || measuredPose.getX() < -tolerance
          || measuredPose.getX() > layout.getFieldLength() + tolerance
          || measuredPose.getY() < -tolerance
          || measuredPose.getY() > layout.getFieldWidth() + tolerance) {
        continue;
      }

      m_lastRawVisionPose = measuredPose;
      m_lastVisionMeasurementTimestamp = timestamp;
      m_lastVisionPoseReceivedAt = Timer.getFPGATimestamp();
      m_robotDrive.showVisionPose(measuredPose);

      m_poseSolver = multiTag ? "MultiTag" : "SingleTag";

      // Disabled by default: opt in only after checking camera transform and
      // raw position on the physical practice field.
      if (m_visionFusionEntry.getBoolean(false)) {
        m_robotDrive.addVisionMeasurement(measuredPose, timestamp, multiTag);
      }
    }

    m_visionTargetVisible = m_lastVisionTargetTimestamp >= 0.0
        && (now - m_lastVisionTargetTimestamp) <= kVisionTargetHoldSeconds;
    if (!m_visionTargetVisible) {
      m_visionTargetYawDegrees = 0.0;
    }

    double poseAgeSeconds = m_lastVisionPoseReceivedAt < 0.0
        ? -1.0 : now - m_lastVisionPoseReceivedAt;
    boolean poseFresh = poseAgeSeconds >= 0.0 && poseAgeSeconds <= 0.5;
    // Explicit one-shot pose initialization, only while Disabled and while a
    // fresh, vetted raw pose is available. Never auto-reset during a match.
    if (m_visionResetEntry.getBoolean(false)) {
      if (DriverStation.isDisabled() && poseFresh && m_lastRawVisionPose != null) {
        m_robotDrive.resetOdometry(m_lastRawVisionPose);
        m_visionResetStatus = "Applied while Disabled";
      } else {
        m_visionResetStatus = "Rejected: Disable and show a valid tag";
      }
      m_visionResetEntry.setBoolean(false);
    }
  }

  /**
   * Returns manual trigger rotation when auto-align is off. When auto-align is on and an
   * AprilTag is visible, it overrides manual rotation with PhotonVision yaw-based aiming.
   */
  private double getDriveRotationCommand() {
    if (!m_visionAlignEnabled) {
      return getTriggerRotation();
    }

    if (!m_visionTargetVisible) {
      // Auto-align is enabled but the target has been gone longer than the hold window.
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

  /** Toggle between robot-relative and field-relative translation control. */
  private void toggleFieldRelative() {
    m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
  }

  /** Toggle PhotonVision AprilTag auto-alignment on/off. */
  private void toggleVisionAlign() {
    m_visionAlignEnabled = !m_visionAlignEnabled;
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


                // Hold LB to run the intake; release to stop.
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(Commands.runEnd(
            m_intake::runIntake,
            m_intake::stopIntake,
            m_intake));

            
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
  }

  /** Camera frames are read once per loop; Shuffleboard suppliers publish live values. */
  public void updateDashboard() {
    updateVisionTarget();
  }

  private double getVisionPoseAgeSeconds() {
    return m_lastVisionPoseReceivedAt < 0.0 ? -1.0
        : Timer.getFPGATimestamp() - m_lastVisionPoseReceivedAt;
  }

  private boolean isVisionPoseFresh() {
    double age = getVisionPoseAgeSeconds();
    return age >= 0.0 && age <= 0.5;
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
