// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Map;
import java.util.List;
import java.util.OptionalDouble;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.HoodDistanceCalculator.Point;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.Optional;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double kDefaultDrivePercent = 100.0;

  // Measured chassis speed thresholds for a stationary autonomous shot.
  // These are provisional, conservative starting values to validate on robot.
  private static final double kAutoShootMaxLinearSpeedMps = 0.10;
  private static final double kAutoShootMaxAngularSpeedRadPerSec = 0.15;
  private static final double kAutoShootStopStableSeconds = 0.25;
  private static final double kAutoShootStopTimeoutSeconds = 1.5;
  private static final double kAutoShootReadyTimeoutSeconds = 2.0;
  // A short pause with the feeder stopped; NOT confirmation that fuel exited.
  private static final double kAutoShootPostFeedPauseSeconds = 0.15;
   private static final double kDefaultFeederPercent = Math.abs(FeederConstants.TEST_SPEED) * 100.0;

  // Keep drivetrain, shooter/feeder, hood and intake mechanisms independent.
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // Shooter-side feeder: signed duty cycle chooses forward or reverse.
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final HoodSubsystem m_hood = new HoodSubsystem();
  // Intake roller controllers use CAN 16/17; confirm physical IDs before operation.
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // Intake arm is currently configured as CAN 15; verify hardware and limits.
  private final IntakeArmSubsystem m_intakeArm = new IntakeArmSubsystem();

  private final GenericEntry m_driveSpeedEntry;
   private final GenericEntry m_feederSpeedEntry;
  private final GenericEntry m_feederTestEntry;
  private final GenericEntry m_shooterTargetRpmEntry;
  private final GenericEntry m_shooterRpmToleranceEntry;
  private boolean m_teleopShootCommandActive = false;
  private boolean m_manualFeederCommandActive = false;
  private double m_feederDashboardTestDeadline = -1.0;
  private String m_feederManualStatus = "IDLE";
  // Hood manual control remains independent until future measured calibration.

  // Autonomous commands set a request; subsystem default commands own the
  // motors continuously, so the request persists across paths and is always
  // cleared on end/cancellation. Shooter-feeder actions are separately armed.
  // Explicitly opt in after confirming CAN 11/14 are the installed
  // shooter feeder (not the unfinished intake feeder).
  private final GenericEntry m_autoFeederEnabledEntry;
  private final GenericEntry m_autoFeedDurationEntry;
  private double m_autoFeedDeadline = -1.0;
  private double m_autoStopStableSince = -1.0;
  private String m_autoLastShotResult = "NOT RUN";
  private boolean m_autoShooterRequested = false;
  private double m_autoShooterTargetRPM = 0.0;
  private double m_autoShooterStartTime = -1.0;
  private String m_autoShooterStatus = "Idle";
  private final GenericEntry m_visionFusionEntry;
  private final GenericEntry m_visionResetEntry;
  private final GenericEntry m_fieldVerifiedEntry;
  private final GenericEntry m_directHubGeometryEntry;
  private final GenericEntry m_hubAimTrimEntry;
  // Preview-only distance-to-hood calibration; these do not command motors.
  private final GenericEntry m_hoodNearDistanceEntry;
  private final GenericEntry m_hoodNearEncoderEntry;
  private final GenericEntry m_hoodFarDistanceEntry;
  private final GenericEntry m_hoodFarEncoderEntry;
  private double m_hoodPreviewDistanceMeters = Double.NaN;
  private double m_hoodPreviewEncoderTarget = Double.NaN;
  private String m_hoodPreviewSource = "NONE";
  private String m_hoodPreviewStatus = "NO VERIFIED HUB DISTANCE";
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
  private final HubAimHeadingV2 m_hubCenterAlign = new HubAimHeadingV2();
  // Independent V2 direct HUB tracking; no global field pose seed needed.
  private final HubAimTargetV2 m_hubRelativeTarget =
      new HubAimTargetV2(VisionConstants.FIELD_LAYOUT);

  // Driving mode. False keeps robot-relative translation by default.
  private boolean m_fieldRelativeEnabled = false;

  // Vision auto-align state.
  private boolean m_visionAlignEnabled = false;
  // Actual normalized rotation requested by HUB aim, BEFORE drive scaling.
  private double m_lastHubTurnOutput = 0.0;
  private double m_lastVisionPoseReceivedAt = -1.0;
  private double m_lastVisionMeasurementTimestamp = -1.0;
  private Pose2d m_lastRawVisionPose = null;
  private Pose2d m_latestGoodSeedPose = null;
  private double m_latestGoodSeedTimestamp = -1.0;
  private double m_lastVisionInnovationMeters = -1.0;
  private int m_lastVisionUsedTagCount = 0;
  private int m_lastDetectedTagCount = 0;
  private boolean m_lastCoprocMultitagResultPresent = false;
  private boolean m_lastPhotonPoseMultitagSolved = false;
  private double m_lastAverageTagRangeMeters = -1.0;
  private int m_acceptedVisionCount = 0;
  private int m_rejectedVisionCount = 0;
  private String m_visionGateStatus = "WAITING FOR CAMERA";

  // Independent of global field localization. This compares the camera's
  // 3-D target Z against the OFFICIAL tag mounting height on level ground.
  // It is not valid when an AprilTag prop is mounted at a custom height.
  private double m_cameraTagHeightErrorMeters = Double.NaN;
  private double m_cameraTagHeightRobotMeters = Double.NaN;
  private int m_cameraMountCheckTagId = -1;
  private String m_cameraMountCheckStatus = "WAITING FOR VALID SINGLE-TAG 3D";

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Driver view: controls and live pose only.
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    m_driveSpeedEntry = driveTab.add("Drive Speed (%)", kDefaultDrivePercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0))
        .withPosition(0, 0).withSize(2, 1).getEntry();
    driveTab.addBoolean("Field Relative", () -> m_fieldRelativeEnabled)
        .withPosition(2, 0);
    driveTab.addBoolean("Hub Center Align", () -> m_visionAlignEnabled)
        .withPosition(3, 0);

    // Closed-loop shooter target is motor-shaft RPM, never an output %.
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    m_shooterTargetRpmEntry = shooterTab.add(
        "Shooter RPM Setpoint", ShooterConstants.DEFAULT_TARGET_RPM)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", ShooterConstants.MAX_TARGET_RPM))
        .withPosition(0, 0).withSize(3, 1).getEntry();
    m_feederSpeedEntry = shooterTab.add("Feeder Speed (%)", kDefaultFeederPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0))
        .withPosition(3, 0).withSize(3, 1).getEntry();
    // This explicit momentary request is NOT the output slider. Any accepted
    // request expires after 2 seconds and is rejected outside TELEOP.
    m_feederTestEntry = shooterTab.add("Feeder Test (2 s)", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(3, 3).withSize(2, 1).getEntry();
    m_feederTestEntry.setBoolean(false); // never carry a prior latched request
    // Tolerance is used to confirm a closed-loop setpoint for autonomous shots.
    m_shooterRpmToleranceEntry = shooterTab.add("Shooter RPM Tolerance", 150.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 25.0, "max", 500.0))
        .withPosition(2, 1).withSize(2, 1).getEntry();
    shooterTab.addDouble("Shooter Actual RPM", m_shooter::getActualRPM)
        .withPosition(4, 1).withSize(2, 1);
    shooterTab.addBoolean("Shooter At Target RPM",
        () -> m_shooter.isAtTargetRPM(getShooterTargetRPM(), getShooterRpmTolerance()))
        .withPosition(6, 1).withSize(2, 1);
    shooterTab.addString("Feeder Direction", m_feeder::getDirection)
        .withPosition(6, 0).withSize(2, 1);
    shooterTab.addString("Shooter Feeder Status", this::getTeleopShooterFeederStatus)
        .withPosition(0, 3).withSize(3, 1);
    shooterTab.addString("Feeder Manual Status", () -> m_feederManualStatus)
        .withPosition(0, 4).withSize(7, 1);

    // Button/robot state troubleshooting is separated from operator controls.
    ShuffleboardTab mechanismDebug = Shuffleboard.getTab("Mechanism Diagnostics");
    mechanismDebug.addBoolean("Driver B Pressed", m_driverController::getBButton);
    mechanismDebug.addBoolean("Driver A Pressed", m_driverController::getAButton);
    mechanismDebug.addBoolean("Teleop Enabled", DriverStation::isTeleopEnabled);
    mechanismDebug.addBoolean("B Shoot Command Active", () -> m_teleopShootCommandActive);
    mechanismDebug.addBoolean("A Feeder-Only Active", () -> m_manualFeederCommandActive);
    mechanismDebug.addBoolean("Controller Port 0 Connected",
        () -> DriverStation.isJoystickConnected(OIConstants.kDriverControllerPort));
    mechanismDebug.addString("Controller Port 0 Name",
        () -> DriverStation.getJoystickName(OIConstants.kDriverControllerPort));
    mechanismDebug.addBoolean("Raw Button 1 (A)",
        () -> m_driverController.getRawButton(1));
    mechanismDebug.addBoolean("Feeder Dashboard Test Active", () ->
        TeleopFeederControl.isDashboardTestActive(
            Timer.getFPGATimestamp(), m_feederDashboardTestDeadline));
    mechanismDebug.addDouble("Feeder Speed Setpoint (%)",
        () -> getOutputScale(m_feederSpeedEntry, kDefaultFeederPercent) * 100.0);

    // Shooter RPM is controlled by the SPARK MAX closed loop on CAN 9.
    // A-button feeder control remains manual and independent.
    ShuffleboardTab shotTab = Shuffleboard.getTab("Shot Tuning");
    m_autoFeederEnabledEntry = shotTab.add("Autonomous Feeder Enabled", false).getEntry();
    m_autoFeedDurationEntry = shotTab.add("Autonomous Feed Duration (s)", 0.75)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.1, "max", 2.0)).getEntry();
    shotTab.addBoolean("Autonomous Shot Ready", this::isAutonomousShooterReady);
    shotTab.addString("Autonomous Shooter Status", () -> m_autoShooterStatus);
    shotTab.addString("Last Auto Shot Result", () -> m_autoLastShotResult);
    shotTab.addBoolean("Auto Robot Stopped", this::isAutonomousDriveStoppedNow);

    // Calibrate the raw DUTY CYCLE encoder at two measured distances.
    // Values start at zero to prevent a fabricated calibration. This stage
    // previews an interpolated target only; it never moves the hood.
    ShuffleboardTab hoodCal = Shuffleboard.getTab("Hood Calibration");
    m_hoodNearDistanceEntry = hoodCal.add("Near Distance (m)", 0.0)
        .withPosition(0, 0).withSize(2, 1).getEntry();
    m_hoodNearEncoderEntry = hoodCal.add("Near Hood Encoder", 0.0)
        .withPosition(2, 0).withSize(2, 1).getEntry();
    m_hoodFarDistanceEntry = hoodCal.add("Far Distance (m)", 0.0)
        .withPosition(0, 1).withSize(2, 1).getEntry();
    m_hoodFarEncoderEntry = hoodCal.add("Far Hood Encoder", 0.0)
        .withPosition(2, 1).withSize(2, 1).getEntry();
    hoodCal.addDouble("HUB Distance (m)", () ->
        Double.isFinite(m_hoodPreviewDistanceMeters) ? m_hoodPreviewDistanceMeters : -1.0)
        .withPosition(0, 2);
    hoodCal.addString("Distance Source", () -> m_hoodPreviewSource).withPosition(2, 2);
    hoodCal.addDouble("Preview Hood Encoder", () ->
        Double.isFinite(m_hoodPreviewEncoderTarget) ? m_hoodPreviewEncoderTarget : -1.0)
        .withPosition(0, 3);
    hoodCal.addString("Preview Status", () -> m_hoodPreviewStatus)
        .withPosition(2, 3).withSize(4, 1);

    // FRC 5024: validate distance/count and supply range-aware covariance.
    // FRC 6328: maintain one sensor-fused pose, with camera capture times.
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    ShuffleboardTab visionDebugTab = Shuffleboard.getTab("Vision Diagnostics");
    m_fieldVerifiedEntry = visionTab.add("Field Camera Verified", false).getEntry();
    // Independent from the global field-pose check: practice HUBs can be
    // aimed at without defining the robot's field X/Y origin.
    m_directHubGeometryEntry =
        visionTab.add("Hub Direct Geometry Verified", false).getEntry();
    m_visionFusionEntry = visionTab.add("Fusion Enabled", true).getEntry();
    m_visionResetEntry = visionTab.add("Reset Pose (Disabled)", false).getEntry();
    visionTab.addBoolean("Pose Initialized", m_robotDrive::isFieldPoseInitialized);
    visionTab.addBoolean("NavX Ready", m_robotDrive::isGyroReady);
    visionTab.addBoolean("Fused Vision Fresh",
        () -> m_robotDrive.hasRecentVision(1.0));
    visionTab.addString("Pose Init Status", () -> m_visionResetStatus);
    visionTab.addString("Vision Gate Status", () -> m_visionGateStatus);
    visionDebugTab.addDouble("Vision Accepted Frames", () -> m_acceptedVisionCount);
    visionDebugTab.addDouble("Vision Rejected Frames", () -> m_rejectedVisionCount);
    visionDebugTab.addDouble("Visible Tags Used", () -> m_lastVisionUsedTagCount);
    // Raw PhotonPipelineResult vs. the result of the robot-side solver.
    // Two detected tags do not guarantee that coprocessor PnP produced a pose.
    visionDebugTab.addDouble("Raw Detected Tag Count", () -> m_lastDetectedTagCount);
    visionDebugTab.addBoolean("Raw Coproc MultiTag Present",
        () -> m_lastCoprocMultitagResultPresent);
    visionDebugTab.addBoolean("PhotonPose MultiTag Solved",
        () -> m_lastPhotonPoseMultitagSolved);
    visionDebugTab.addDouble("Pose Avg Tag Range (m)",
        () -> m_lastAverageTagRangeMeters);

    // The PhotonVision 180-degree Input Orientation is a *manual UI setting*;
    // the number here is the expected configuration, not a live PV readback.
    visionDebugTab.addDouble("Camera Expected PV Orientation (deg)",
        () -> VisionConstants.EXPECTED_PHOTONVISION_IMAGE_ORIENTATION_DEG);
    visionDebugTab.addDouble("Camera Config Roll (deg)",
        () -> VisionConstants.CAMERA_ROLL_DEG);
    visionDebugTab.addDouble("Camera Config Pitch (deg)",
        () -> VisionConstants.CAMERA_PITCH_DEG);
    visionDebugTab.addDouble("Camera Config Yaw (deg)",
        () -> VisionConstants.CAMERA_YAW_DEG);
    visionDebugTab.addDouble("Camera Mount Tag ID",
        () -> m_cameraMountCheckTagId);
    visionDebugTab.addDouble("Camera Mount Tag Z (m)",
        () -> Double.isFinite(m_cameraTagHeightRobotMeters)
            ? m_cameraTagHeightRobotMeters : -999.0);
    visionDebugTab.addDouble("Camera Mount Tag Height Error (m)",
        () -> Double.isFinite(m_cameraTagHeightErrorMeters)
            ? m_cameraTagHeightErrorMeters : -999.0);
    visionDebugTab.addString("Camera Mount Check Status",
        () -> m_cameraMountCheckStatus);
    visionDebugTab.addDouble("Vision/Fused Delta XY (m)",
        () -> m_lastVisionInnovationMeters);
    visionDebugTab.addDouble("Tag ID", () -> m_lastTagId);
    visionDebugTab.addBoolean("Raw Pose Fresh", this::isVisionPoseFresh);
    visionDebugTab.addDouble("Raw Pose X (m)",
        () -> m_lastRawVisionPose == null ? -1.0 : m_lastRawVisionPose.getX());
    visionDebugTab.addDouble("Raw Pose Y (m)",
        () -> m_lastRawVisionPose == null ? -1.0 : m_lastRawVisionPose.getY());
    visionDebugTab.addDouble("Raw Heading (deg)",
        () -> m_lastRawVisionPose == null ? -999.0
            : m_lastRawVisionPose.getRotation().getDegrees());
    visionDebugTab.addString("Pose Solver", () -> m_poseSolver);
    visionTab.addString("Alliance",
        () -> DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN"));
    visionTab.addString("HUB Align Status", m_hubCenterAlign::getStatus);
    visionTab.addBoolean("HUB Auto Turn Armed (code)",
        () -> Constants.HubAimSafety.ENABLE_AUTO_ROTATION);
    visionTab.addString("HUB Safety Status", () ->
        Constants.HubAimSafety.ENABLE_AUTO_ROTATION
            ? "AUTO ROTATION ARMED: VERIFY SWERVE"
            : "PREVIEW ONLY - REPAIR SWERVE WHEEL FIRST");
    visionTab.addString("HUB Aim Source", m_hubCenterAlign::getSource);
    visionDebugTab.addString("HUB Direct Status", m_hubRelativeTarget::getStatus);
    // Per-tag gate results and raw 3D values. These are telemetry only;
    // they never override a rejected measurement or enable auto rotation.
    visionDebugTab.addString("HUB Tag 25 Diagnostics",
        m_hubRelativeTarget::getTag25Diagnostics);
    visionDebugTab.addString("HUB Tag 26 Diagnostics",
        m_hubRelativeTarget::getTag26Diagnostics);
    visionDebugTab.addString("HUB Direct Last Rejection",
        m_hubRelativeTarget::getLastRejectionReason);
    visionDebugTab.addDouble("HUB Valid Candidate Count",
        m_hubRelativeTarget::getValidCandidateCount);
    visionTab.addBoolean("HUB Direct Fresh",
        () -> m_hubRelativeTarget.isFresh(Timer.getFPGATimestamp()));
    visionDebugTab.addDouble("HUB Direct Tag Count",
        m_hubRelativeTarget::getLastTagCount);
    visionDebugTab.addDouble("HUB Direct Best Tag ID",
        m_hubRelativeTarget::getLastHubTagId);
    visionDebugTab.addString("HUB Direct Tag Alliance",
        m_hubRelativeTarget::getTargetAlliance);
    visionDebugTab.addDouble("HUB Direct Age (s)",
        () -> m_hubRelativeTarget.getLastAgeSeconds(Timer.getFPGATimestamp()));
    visionDebugTab.addDouble("HUB Direct Rejected Frames",
        m_hubRelativeTarget::getRejectedFrames);
    // Diagnostic chain: driver X -> enable state -> localization gates ->
    // actual computed turn output. Unlike camera yaw, these answer
    // "why does the robot not rotate?" without bypassing safety checks.
    visionDebugTab.addBoolean("HUB X Button Pressed", m_driverController::getXButton);
    visionDebugTab.addBoolean("HUB Teleop Enabled", DriverStation::isTeleopEnabled);
    visionTab.addString("HUB Turn Blocker", this::getHubRotationBlocker);
    visionTab.addDouble("HUB Turn Command (%)", () -> m_lastHubTurnOutput * 100.0);
    visionTab.addDouble("HUB Error (deg)",
        m_hubCenterAlign::getHeadingErrorDegrees);
    visionTab.addBoolean("HUB Aligned",
        () -> m_visionAlignEnabled && m_hubCenterAlign.isAligned());
    m_hubAimTrimEntry = visionTab.add("Hub Aim Trim (deg)", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -15.0, "max", 15.0)).getEntry();
    visionDebugTab.addDouble("Hub Trim Applied (deg)", m_hubCenterAlign::getAimTrimDegrees);
    visionDebugTab.addDouble("Hub Untrimmed Error (deg)",
        m_hubCenterAlign::getUntrimmedHeadingErrorDegrees);
    visionDebugTab.addDouble("Hub Estimated Heading (deg)",
        m_hubCenterAlign::getEstimatedHeadingDegrees);
    visionDebugTab.addDouble("Hub Desired Heading (deg)",
        m_hubCenterAlign::getDesiredHeadingDegrees);
    visionDebugTab.addDouble("Hub Bearing (deg)", m_hubCenterAlign::getHubBearingDegrees);
    visionDebugTab.addDouble("Hub Distance (m)", m_hubCenterAlign::getHubDistanceMeters);

    visionDebugTab.addString("Vision Field Layout", () -> "2026 REBUILT Welded");
    visionDebugTab.addString("Vision Camera", () -> VisionConstants.CAMERA_NAME);
    visionDebugTab.addDouble("Vision Pose Age (s)", this::getVisionPoseAgeSeconds);

    configureButtonBindings();
    // Must register before AutoBuilder.buildAutoChooser() parses .auto files.
    registerAutonomousShooterCommands();

    SendableChooser<Command> autoChooser;
    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (Exception e) {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("None", Commands.none());
    }
    m_autoChooser = autoChooser;
    // Autonomous controls belong with their existing feeder arm/timer widgets.
    shotTab.add("Auto Chooser", m_autoChooser)
        .withPosition(0, 0).withSize(3, 1);
    shotTab.addString("Selected Auto", () -> {
      Command chosen = m_autoChooser.getSelected();
      return chosen != null ? chosen.getName() : "None";
    }).withPosition(3, 0).withSize(2, 1);

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
              // A gap between autonomous path commands must not execute
              // joystick-driven motion. PathPlanner overrides this default.
              if (!DriverStation.isTeleopEnabled()) {
                m_lastHubTurnOutput = 0.0;
                m_robotDrive.driveRobotRelative(new ChassisSpeeds());
                return;
              }
              double scale = getOutputScale(m_driveSpeedEntry, kDefaultDrivePercent);
              m_robotDrive.drive(
                  -scale * MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                  -scale * MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                  scale * getDriveRotationCommand(),
                  m_fieldRelativeEnabled);
            },
            m_robotDrive));

    // Right stick remains the only hood motion request; release holds position.
    m_hood.setDefaultCommand(new RunCommand(() -> {
      double input = DriverStation.isTeleopEnabled()
          ? MathUtil.applyDeadband(-m_driverController.getRightY(),
              OIConstants.kHoodDeadband)
          : 0.0;
      m_hood.setManual(input * HoodConstants.TEST_MAX_SPEED);
    }, m_hood));

    // Keeps motor ownership in one command (and avoids path-marker commands
    // with drive subsystem requirements interfering with path following).
    m_shooter.setDefaultCommand(new RunCommand(() -> {
      if (DriverStation.isAutonomousEnabled() && m_autoShooterRequested) {
        m_shooter.setTargetRPM(m_autoShooterTargetRPM);
      } else if (m_shooter.isRunning()) {
        m_shooter.stopShooter();
      }
    }, m_shooter));

    // Own the feeder continuously in an independent command so A and the
    // short-lived dashboard test share exactly the same stop/speed logic.
    // Existing autonomous feeder commands take the feeder requirement and
    // preempt this default command while their own safety gates apply.
    m_feeder.setDefaultCommand(new RunCommand(this::updateManualFeeder, m_feeder));
  }

  /** Read a 0-100% Shuffleboard slider and safely convert to an output multiplier. */
  private static double getOutputScale(GenericEntry entry, double defaultPercent) {
    double percent = entry.getDouble(defaultPercent);
    return Double.isFinite(percent) ? MathUtil.clamp(percent, 0.0, 100.0) / 100.0 : 0.0;
  }

  /** Zero/off and invalid targets are rejected, not converted to duty cycle. */
  private double getShooterTargetRPM() {
    return ShooterVelocitySetpoint.sanitizeRPM(
        m_shooterTargetRpmEntry.getDouble(ShooterConstants.DEFAULT_TARGET_RPM));
  }

  private double getShooterRpmTolerance() {
    double tolerance = m_shooterRpmToleranceEntry.getDouble(150.0);
    return Double.isFinite(tolerance) && tolerance > 0.0
        ? MathUtil.clamp(tolerance, 1.0, 500.0) : 0.0;
  }

  /** Describes the independent A feeder and B shooter teleop controls. */
  private String getTeleopShooterFeederStatus() {
    if (!DriverStation.isTeleopEnabled()) return "BLOCKED: TELEOP NOT ENABLED";
    boolean a = m_driverController.getAButton();
    boolean b = m_driverController.getBButton();
    boolean dashboardTest = TeleopFeederControl.isDashboardTestActive(
        Timer.getFPGATimestamp(), m_feederDashboardTestDeadline);
    if (!a && !b && !dashboardTest)
      return "IDLE: A=FEEDER, B=SHOOTER, OR USE 2s FEEDER TEST";
    String shooter = b
        ? (m_teleopShootCommandActive && m_shooter.isRunning()
            ? "SHOOTER RUNNING" : "SHOOTER WAITING / ZERO OUTPUT")
        : "SHOOTER OFF";
    boolean dashboard = TeleopFeederControl.isDashboardTestActive(
        Timer.getFPGATimestamp(), m_feederDashboardTestDeadline);
    String feeder = (a || dashboard)
        ? (m_feeder.isFeeding() ? "FEEDER " + m_feeder.getDirection()
            : "FEEDER WAITING / ZERO OUTPUT") : "FEEDER OFF";
    return shooter + " | " + feeder;
  }

  /**
   * Shoots only after the chassis has actually settled. Auto Shoot holds
   * the drive subsystem for its entire duration, commanding zero robot-
   * relative velocity even when a feeder burst is in progress.
   *
   * IMPORTANT: place Auto Shoot SEQUENTIALLY BETWEEN PATHS in PathPlanner.
   * This command requires the drivetrain, so do not trigger it as an event
   * marker while a path is actively following.
   */
  private void registerAutonomousShooterCommands() {
    NamedCommands.registerCommand("Auto Shooter Start",
        Commands.runOnce(this::startAutonomousShooter));
    NamedCommands.registerCommand("Auto Shoot", makeStationaryAutoShootCommand());
    NamedCommands.registerCommand("Auto Shooter Stop",
        Commands.runOnce(this::stopAutonomousShooter));
  }

  /** Checks actual encoder-derived chassis velocity, not commanded velocity. */
  private boolean isAutonomousDriveStoppedNow() {
    ChassisSpeeds speeds = m_robotDrive.getRobotRelativeSpeeds();
    return Double.isFinite(speeds.vxMetersPerSecond)
        && Double.isFinite(speeds.vyMetersPerSecond)
        && Double.isFinite(speeds.omegaRadiansPerSecond)
        && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            <= kAutoShootMaxLinearSpeedMps
        && Math.abs(speeds.omegaRadiansPerSecond)
            <= kAutoShootMaxAngularSpeedRadPerSec;
  }

  /** Require measured motion to remain inside thresholds for 0.25s. */
  private boolean isAutonomousDriveStoppedStable() {
    if (!DriverStation.isAutonomousEnabled() || !isAutonomousDriveStoppedNow()) {
      m_autoStopStableSince = -1.0;
      return false;
    }
    double now = Timer.getFPGATimestamp();
    if (m_autoStopStableSince < 0.0) {
      m_autoStopStableSince = now;
    }
    return now - m_autoStopStableSince >= kAutoShootStopStableSeconds;
  }

  private boolean isAutonomousShotSafe() {
    return isAutonomousDriveStoppedStable() && isAutonomousShooterReady();
  }

  /** A shooting phase cannot overlap PathPlanner drive movement. */
  private Command makeStationaryAutoShootCommand() {
    Command steps = Commands.sequence(
        Commands.runOnce(() -> {
          m_autoStopStableSince = -1.0;
          m_autoLastShotResult = "WAITING: chassis stationary";
          m_autoShooterStatus = "WAITING: chassis stationary";
        }),
        Commands.waitUntil(this::isAutonomousDriveStoppedStable)
            .withTimeout(kAutoShootStopTimeoutSeconds),
        Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> m_autoShooterStatus =
                    "CHASSIS STOPPED: checking shooter and hood"),
                Commands.waitUntil(this::isAutonomousShotSafe)
                    .withTimeout(kAutoShootReadyTimeoutSeconds),
                Commands.either(
                    makeAutonomousFeederCommand(),
                    Commands.runOnce(() -> {
                      m_autoLastShotResult = isAutonomousShotSafe()
                          ? "DISARMED: Autonomous Feeder Enabled is OFF"
                          : "SKIPPED: shooter/hood or stop interlock not ready";
                      m_autoShooterStatus = m_autoLastShotResult;
                    }),
                    () -> isAutonomousShotSafe()
                        && m_autoFeederEnabledEntry.getBoolean(false))),
            Commands.runOnce(() -> {
              m_autoLastShotResult = "SKIPPED: robot did not settle in time";
              m_autoShooterStatus = m_autoLastShotResult;
            }),
            this::isAutonomousDriveStoppedStable));

    // The deadline owns the feed/wait sequence; the concurrent drive command
    // continuously requests zero speed and holds the drive requirement.
    return Commands.deadline(
        steps,
        Commands.run(() -> m_robotDrive.driveRobotRelative(new ChassisSpeeds()),
            m_robotDrive))
        .finallyDo(interrupted -> {
          m_feeder.stopFeeder();
          m_autoFeedDeadline = -1.0;
          m_autoStopStableSince = -1.0;
          m_robotDrive.driveRobotRelative(new ChassisSpeeds());
          if (interrupted) {
            m_autoLastShotResult = "INTERRUPTED: feeder stopped";
            m_autoShooterStatus = m_autoLastShotResult;
          }
        });
  }

  /** A bounded feed burst, stopped on timeout, readiness loss, or cancellation. */
  private Command makeAutonomousFeederCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          double requestedSeconds = m_autoFeedDurationEntry.getDouble(0.75);
          double seconds = Double.isFinite(requestedSeconds)
              ? MathUtil.clamp(requestedSeconds, 0.1, 2.0) : 0.0;
          m_autoFeedDeadline = Timer.getFPGATimestamp() + seconds;
          m_autoShooterStatus = "FEEDING: shooter feeder CAN 11/14";
        }),
        Commands.runEnd(
            () -> {
              if (DriverStation.isAutonomousEnabled()
                  && m_autoFeederEnabledEntry.getBoolean(false)
                  && isAutonomousShotSafe()) {
                m_feeder.runFeeder(
                    getOutputScale(m_feederSpeedEntry, kDefaultFeederPercent));
              } else {
                m_feeder.stopFeeder();
              }
            },
            m_feeder::stopFeeder,
            m_feeder)
            .until(() -> !DriverStation.isAutonomousEnabled()
                || !m_autoFeederEnabledEntry.getBoolean(false)
                || !isAutonomousShotSafe()
                || m_autoFeedDeadline < 0.0
                || Timer.getFPGATimestamp() >= m_autoFeedDeadline),
        Commands.runOnce(() -> {
          // No ball-exit sensor: completing the timer does not prove that
          // every game piece has exited. Distinguish a normal timed feed
          // from an interlock-aborted feed.
          m_feeder.stopFeeder();
          if (!DriverStation.isAutonomousEnabled()) {
            m_autoLastShotResult = "FEED ABORTED: autonomous disabled";
          } else if (!m_autoFeederEnabledEntry.getBoolean(false)) {
            m_autoLastShotResult = "FEED ABORTED: feeder disarmed";
          } else if (!isAutonomousShotSafe()) {
            m_autoLastShotResult = "FEED ABORTED: robot/shooter/hood not ready";
          } else if (Timer.getFPGATimestamp() >= m_autoFeedDeadline) {
            m_autoLastShotResult = "FEED TIMER COMPLETE: fuel exit unverified";
          } else {
            m_autoLastShotResult = "FEED STOPPED: check status";
          }
          m_autoFeedDeadline = -1.0;
          m_autoShooterStatus = m_autoLastShotResult;
        }),
        Commands.waitSeconds(kAutoShootPostFeedPauseSeconds));
  }

  private void startAutonomousShooter() {
    stopAutonomousShooter();
    if (!DriverStation.isAutonomousEnabled()) {
      m_autoShooterStatus = "BLOCKED: autonomous disabled";
      return;
    }
    if (HoodConstants.CALIBRATION_MODE || !m_hood.isHoodReady()) {
      m_autoShooterStatus = "BLOCKED: hood encoder not ready";
      return;
    }
    double targetRpm = getShooterTargetRPM();
    if (targetRpm <= 0.0) {
      m_autoShooterStatus = "BLOCKED: set a valid Shooter RPM Setpoint";
      return;
    }
    m_autoShooterTargetRPM = targetRpm;
    m_autoShooterStartTime = Timer.getFPGATimestamp();
    m_autoShooterRequested = true;
    m_autoShooterStatus = "SPINNING: closed-loop motor RPM";
  }

  private boolean isAutonomousShooterReady() {
    return DriverStation.isAutonomousEnabled()
        && m_autoShooterRequested && m_hood.isHoodReady()
        && m_shooter.isAtTargetRPM(m_autoShooterTargetRPM, getShooterRpmTolerance())
        && m_autoShooterStartTime >= 0.0
        && Timer.getFPGATimestamp() - m_autoShooterStartTime >= 1.0;
  }

  private void stopAutonomousShooter() {
    m_feeder.stopFeeder();
    m_autoFeedDeadline = -1.0;
    m_autoShooterRequested = false;
    m_autoShooterTargetRPM = 0.0;
    m_autoShooterStartTime = -1.0;
    m_shooter.stopShooter();
    m_autoShooterStatus = "STOPPED";
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
   * Camera extrinsics plausibility check for a robot standing level on the
   * official REBUILT playing field. A physical tag placed at a different
   * height makes this test invalid, NOT proof of bad calibration.
   *
   * Uses a real independent individual-tag SolvePnP estimate; requires
   * "Always Do Single-Target Estimation" in PhotonVision for MultiTag mode.
   */
  private void updateCameraMountCheck(List<PhotonTrackedTarget> targets) {
    PhotonTrackedTarget selected = null;
    double smallestAmbiguity = Double.POSITIVE_INFINITY;
    for (PhotonTrackedTarget target : targets) {
      if (VisionConstants.FIELD_LAYOUT.getTagPose(
          target.getFiducialId()).isEmpty()) continue;
      if (target.getBestCameraToTarget() == null) continue;
      double range = target.getBestCameraToTarget()
          .getTranslation().getNorm();
      double ambiguity = target.getPoseAmbiguity();
      if (!Double.isFinite(range) || range < 0.15 || range > 5.5
          || !Double.isFinite(ambiguity)
          || ambiguity < 0.0 || ambiguity > 0.30) continue;
      if (ambiguity < smallestAmbiguity) {
        smallestAmbiguity = ambiguity;
        selected = target;
      }
    }
    if (selected == null) {
      m_cameraMountCheckTagId = -1;
      m_cameraTagHeightErrorMeters = Double.NaN;
      m_cameraTagHeightRobotMeters = Double.NaN;
      m_cameraMountCheckStatus = "NO VALID PER-TAG 3D; CHECK PHOTONVISION 3D OUTPUT";
      return;
    }

    Pose3d observedTagInRobot = new Pose3d()
        .transformBy(VisionConstants.ROBOT_TO_CAMERA)
        .transformBy(selected.getBestCameraToTarget());
    double expectedTagHeightMeters = VisionConstants.FIELD_LAYOUT
        .getTagPose(selected.getFiducialId()).orElseThrow().getZ();
    m_cameraMountCheckTagId = selected.getFiducialId();
    m_cameraTagHeightRobotMeters = observedTagInRobot.getZ();
    m_cameraTagHeightErrorMeters =
        observedTagInRobot.getZ() - expectedTagHeightMeters;
    if (!Double.isFinite(m_cameraTagHeightErrorMeters)) {
      m_cameraMountCheckStatus = "INVALID 3D MOUNT MEASUREMENT";
    } else if (Math.abs(m_cameraTagHeightErrorMeters) <= 0.12) {
      m_cameraMountCheckStatus =
          "TAG HEIGHT CLOSE: VERIFY X/Y, YAW AND CAMERA ROLL TOO";
    } else {
      m_cameraMountCheckStatus =
          "TAG HEIGHT DIFFERS: CHECK PITCH/ROLL/HEIGHT OR PRACTICE TAG";
    }
  }

  /**
   * PhotonVision is a MEASUREMENT SOURCE, not the heading controller.
   * Every accepted pose enters WPILib's timestamped SwerveDrivePoseEstimator
   * (FRC 5024 / 6328 pattern). All visible field tags can contribute.
   * Initial field coordinates require a verified disabled multitag seed.
   */
  private void updateVisionTarget() {
    double now = Timer.getFPGATimestamp();
    for (var result : m_photonCamera.getAllUnreadResults()) {
      updateCameraMountCheck(result.getTargets());
      // Process all visible HUB tags for LOCAL relative aiming first,
      // regardless of success/failure in the separate global pose solver.
      // Rewind odometry to the image timestamp before transforming HUB XY.
      m_hubRelativeTarget.observe(
          result.getTargets(),
          DriverStation.getAlliance(),
          m_robotDrive.getPoseAtTime(result.getTimestampSeconds()).orElse(null),
          result.getTimestampSeconds(),
          now);
      m_lastDetectedTagCount = result.getTargets().size();
      m_lastCoprocMultitagResultPresent = result.getMultiTagResult().isPresent();
      m_lastPhotonPoseMultitagSolved = false;
      if (!result.hasTargets()) {
        m_poseSolver = "No Targets";
        continue;
      }

      var bestTarget = result.getBestTarget();
      if (bestTarget != null
          && VisionConstants.FIELD_LAYOUT.getTagPose(
              bestTarget.getFiducialId()).isPresent()) {
        m_lastTagId = bestTarget.getFiducialId();
      }

      Optional<EstimatedRobotPose> estimate =
          m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
      boolean multiTag = estimate.isPresent();
      m_lastPhotonPoseMultitagSolved = multiTag;

      if (estimate.isEmpty() && bestTarget != null
          && VisionConstants.FIELD_LAYOUT.getTagPose(
              bestTarget.getFiducialId()).isPresent()
          && bestTarget.getPoseAmbiguity() >= 0.0
          && bestTarget.getPoseAmbiguity()
              <= VisionConstants.MAX_SINGLE_TAG_AMBIGUITY
          && bestTarget.getBestCameraToTarget().getTranslation().getNorm()
              <= VisionConstants.MAX_SINGLE_TAG_DISTANCE_M) {
        estimate = m_photonPoseEstimator.estimateLowestAmbiguityPose(result);
      }
      if (estimate.isEmpty()) {
        m_poseSolver = "No Valid Pose";
        m_rejectedVisionCount++;
        m_visionGateStatus = m_lastCoprocMultitagResultPresent
            ? "MULTITAG RESULT EXISTS BUT ROBOT ESTIMATOR RETURNED EMPTY"
            : "NO COPROC MULTITAG RESULT; SINGLE TAG FALLBACK FAILED";
        continue;
      }

      var measurement = estimate.get();
      double stamp = measurement.timestampSeconds;
      if (stamp <= m_lastVisionMeasurementTimestamp) continue;

      int used = measurement.targetsUsed == null
          ? 0 : (int) measurement.targetsUsed.stream()
              .mapToInt(t -> t.getFiducialId()).distinct().count();
      Pose3d rawPose3d = measurement.estimatedPose;
      Pose2d rawPose = rawPose3d.toPose2d();

      // In coprocessor MultiTag mode PhotonVision may omit per-target
      // camera-to-tag SolvePnP transforms unless "Always Do Single-Target
      // Estimation" is enabled (PhotonVision MultiTag documentation).
      // Compute distance from the KNOWN global tag poses instead.
      Pose3d fieldCameraPose = rawPose3d.transformBy(
          VisionConstants.ROBOT_TO_CAMERA);
      double avgDistance = measurement.targetsUsed == null ? Double.NaN
          : measurement.targetsUsed.stream()
              .map(t -> VisionConstants.FIELD_LAYOUT.getTagPose(
                  t.getFiducialId()))
              .filter(Optional::isPresent)
              .mapToDouble(tagPose -> tagPose.get().getTranslation()
                  .getDistance(fieldCameraPose.getTranslation()))
              .filter(Double::isFinite)
              .average().orElse(Double.NaN);
      m_lastAverageTagRangeMeters = avgDistance;
      double ambiguity = bestTarget != null
          ? bestTarget.getPoseAmbiguity() : Double.NaN;

      m_lastRawVisionPose = rawPose;
      m_lastVisionPoseReceivedAt = now;
      m_lastVisionMeasurementTimestamp = stamp;
      m_lastVisionUsedTagCount = used;
      m_poseSolver = multiTag ? "MultiTag" : "SingleTag";
      m_robotDrive.showVisionPose(rawPose);

      ChassisSpeeds measured = m_robotDrive.getRobotRelativeSpeeds();
      double linearSpeed = Math.hypot(
          measured.vxMetersPerSecond, measured.vyMetersPerSecond);
      var decision = VisionPoseValidator.validate(
          rawPose3d, stamp, now,
          m_robotDrive.getPose(), m_robotDrive.isFieldPoseInitialized(),
          multiTag, used, avgDistance, ambiguity,
          linearSpeed, measured.omegaRadiansPerSecond,
          VisionConstants.FIELD_LAYOUT.getFieldLength(),
          VisionConstants.FIELD_LAYOUT.getFieldWidth(),
          m_robotDrive.isGyroReady());

      m_lastVisionInnovationMeters = rawPose.getTranslation()
          .getDistance(m_robotDrive.getPose().getTranslation());

      if (!decision.accepted()) {
        m_rejectedVisionCount++;
        m_visionGateStatus = decision.reason();
        continue;
      }

      if (decision.canInitialize()) {
        m_latestGoodSeedPose = rawPose;
        m_latestGoodSeedTimestamp = now;
      }

      if (!m_fieldVerifiedEntry.getBoolean(false)) {
        m_visionGateStatus = "WAITING: VERIFY FIELD JSON/CAMERA MOUNT";
        continue;
      }

      if (!m_robotDrive.isFieldPoseInitialized()) {
        m_visionGateStatus = "WAITING: DISABLED POSE INITIALIZATION";
        continue;
      }

      if (!m_visionFusionEntry.getBoolean(true)) {
        m_visionGateStatus = "FUSION DISABLED";
        continue;
      }

      m_robotDrive.addVisionMeasurement(
          rawPose, stamp,
          decision.xyStdDevMeters(), decision.thetaStdDevRadians());
      m_acceptedVisionCount++;
      m_visionGateStatus = decision.reason();
    }

    if (m_visionResetEntry.getBoolean(false)) {
      if (!DriverStation.isDisabled()) {
        m_visionResetStatus = "REJECTED: DISABLE ROBOT FIRST";
      } else if (!m_fieldVerifiedEntry.getBoolean(false)) {
        m_visionResetStatus = "REJECTED: VERIFY FIELD/CAMERA FIRST";
      } else if (!m_robotDrive.isGyroReady()) {
        m_visionResetStatus = "REJECTED: NAVX NOT READY";
      } else if (m_latestGoodSeedPose == null
          || now - m_latestGoodSeedTimestamp > 0.25) {
        m_visionResetStatus = "REJECTED: NEED FRESH MULTITAG POSE";
      } else {
        m_robotDrive.resetOdometry(m_latestGoodSeedPose);
        m_hubRelativeTarget.clear(); // global odometry frame changed
        m_visionResetStatus = "INITIALIZED FROM VERIFIED MULTITAG POSE";
        // Wait for the next validated frame before allowing auto-aim.
      }
      m_visionResetEntry.setBoolean(false);
    }
  }



  /**
   * First unmet condition in the actual drive-to-HUB command chain.
   * Never bypass these requirements just to make wheels rotate.
   */
  private String getHubRotationBlocker() {
    if (!DriverStation.isTeleopEnabled()) return "TELEOP NOT ENABLED";
    if (!m_visionAlignEnabled) return "X: HUB ALIGN IS OFF";
    if (!Constants.HubAimSafety.ENABLE_AUTO_ROTATION)
      return "HUB ROTATION LOCKED IN CODE: WHEEL REPAIR REQUIRED";
    if (!m_robotDrive.isGyroReady()) return "NAVX NOT CONNECTED OR CALIBRATING";

    boolean directFresh = m_hubRelativeTarget.isFresh(Timer.getFPGATimestamp());
    if (directFresh && m_directHubGeometryEntry.getBoolean(false)) {
      return m_hubCenterAlign.isAligned() ? "DIRECT HUB: ALIGNED"
          : "DIRECT HUB TAG 3D: " + m_hubCenterAlign.getStatus();
    }

    // The original field localization path is ONLY a secondary fallback.
    if (m_fieldVerifiedEntry.getBoolean(false)
        && m_robotDrive.isFieldPoseInitialized()
        && m_robotDrive.hasRecentVision(1.0)
        && DriverStation.getAlliance().isPresent()) {
      return m_hubCenterAlign.isAligned() ? "FIELD FUSED: ALIGNED"
          : "FIELD FUSED: " + m_hubCenterAlign.getStatus();
    }

    if (directFresh && !m_directHubGeometryEntry.getBoolean(false))
      return "ENABLE HUB DIRECT GEOMETRY VERIFIED AFTER CALIBRATION";
    if (!m_fieldVerifiedEntry.getBoolean(false))
      return "NO DIRECT TARGET; FIELD CAMERA VERIFIED IS OFF";
    if (!m_robotDrive.isFieldPoseInitialized())
      return "NO DIRECT TARGET; FIELD POSE NOT INITIALIZED";
    if (DriverStation.getAlliance().isEmpty())
      return "NO DIRECT TARGET; ALLIANCE UNKNOWN";
    if (!m_robotDrive.hasRecentVision(1.0))
      return "NO DIRECT TARGET; FUSED VISION STALE";
    return "NO VALID HUB TARGET: " + m_hubRelativeTarget.getStatus();
  }

  /**
   * Prefer the independently verified FRC 4107-style HUB-relative target.
   * This path requires no global X/Y initialization and supports one tag.
   * On missing/uncalibrated direct tag measurements, use the existing
   * validated field-based aim when available.
   */
  private double getDriveRotationCommand() {
    m_hubCenterAlign.setAimTrimDegrees(m_hubAimTrimEntry.getDouble(0.0));
    if (!m_visionAlignEnabled) {
      m_lastHubTurnOutput = 0.0;
      return getTriggerRotation();
    }
    // X can request/preview aiming but cannot rotate a chassis with a bad wheel.
    if (!Constants.HubAimSafety.ENABLE_AUTO_ROTATION) {
      m_lastHubTurnOutput = 0.0;
      return 0.0;
    }

    if (m_directHubGeometryEntry.getBoolean(false)
        && m_hubRelativeTarget.isFresh(Timer.getFPGATimestamp())) {
      m_lastHubTurnOutput = m_hubCenterAlign.calculateDirectTurn(
          m_robotDrive.getPose(),
          m_robotDrive.getRobotRelativeSpeeds(),
          m_hubRelativeTarget.getHubInOdometryFrame(
              Timer.getFPGATimestamp()).orElse(null),
          m_robotDrive.isGyroReady());
      return m_lastHubTurnOutput;
    }

    m_lastHubTurnOutput = m_hubCenterAlign.calculateTurn(
        m_robotDrive.getPose(),
        m_robotDrive.getRobotRelativeSpeeds(),
        DriverStation.getAlliance(),
        m_robotDrive.isFieldPoseInitialized()
            && m_fieldVerifiedEntry.getBoolean(false),
        m_robotDrive.isGyroReady(),
        m_robotDrive.hasRecentVision(1.0));
    return m_lastHubTurnOutput;
  }

  /** Toggle between robot-relative and field-relative translation control. */
  private void toggleFieldRelative() {
    m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
  }

  /** Toggle PhotonVision AprilTag auto-alignment on/off. */
  private void toggleVisionAlign() {
    m_visionAlignEnabled = !m_visionAlignEnabled;
    if (m_visionAlignEnabled) {
      m_hubCenterAlign.start();
    } else {
      m_hubCenterAlign.stop();
      m_lastHubTurnOutput = 0.0;
    }
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // B controls ONLY the shooter. Releasing B stops the shooter, not the feeder.
    // This command requires only m_shooter, so A and B can operate together.
    new Trigger(() -> DriverStation.isTeleopEnabled() && m_driverController.getBButton())
        .whileTrue(Commands.runEnd(
            () -> {
              m_teleopShootCommandActive = true;
              m_shooter.setTargetRPM(getShooterTargetRPM());
            },
            () -> {
              m_shooter.stopShooter();
              m_teleopShootCommandActive = false;
            },
            m_shooter));

    // Feeder A-button and timed dashboard test are handled by the feeder's
    // default command, independently of this shooter-only B trigger.

    // Move the original A/B toggle functions to unused X/Y buttons.
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(this::toggleVisionAlign));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(this::toggleFieldRelative));

    // Hold LB to run the intake; release or interruption stops both rollers.
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(Commands.runEnd(
            m_intake::runIntake,
            m_intake::stopIntake,
            m_intake));

    // Xbox controller left-side D-pad / POV.
    // UP (0 deg): move intake ARM forward; DOWN (180 deg): move it back.
    // Each command ends and stops when the D-pad is released.
    // Confirm the actual arm CAN ID, motion directions and physical limits.
    new Trigger(() -> DriverStation.isTeleopEnabled() && m_driverController.getPOV() == 0)
        .whileTrue(Commands.runEnd(
            m_intakeArm::moveForward,
            m_intakeArm::stopArm,
            m_intakeArm));

    new Trigger(() -> DriverStation.isTeleopEnabled() && m_driverController.getPOV() == 180)
        .whileTrue(Commands.runEnd(
            m_intakeArm::moveBackward,
            m_intakeArm::stopArm,
            m_intakeArm));

    // Original right-bumper X-lock behavior.
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> {
          m_robotDrive.zeroHeading();
          m_hubRelativeTarget.clear(); // odometry heading reference changed
        }, m_robotDrive));
  }

  /**
   * Read-only hood aim preview. Direct HUB-tag geometry takes priority if it
   * was physically verified and the observation remains fresh. A verified
   * field pose with recent accepted vision is a fallback. Stale/unverified
   * sources, invalid calibration points and distances outside the measured
   * range never produce a target, nor do they drive the hood.
   */
  private void updateHoodDistancePreview() {
    m_hoodPreviewDistanceMeters = Double.NaN;
    m_hoodPreviewEncoderTarget = Double.NaN;
    m_hoodPreviewSource = "NONE";
    m_hoodPreviewStatus = "NO VERIFIED FRESH HUB POSITION";

    if (!m_robotDrive.isGyroReady()) {
      m_hoodPreviewStatus = "NAVX NOT READY";
      return;
    }

    OptionalDouble distance = OptionalDouble.empty();
    double now = Timer.getFPGATimestamp();
    if (m_directHubGeometryEntry.getBoolean(false)
        && m_hubRelativeTarget.isFresh(now)) {
      Optional<Translation2d> hub = m_hubRelativeTarget.getHubInOdometryFrame(now);
      if (hub.isPresent()) {
        distance = HubDistanceCalculator.calculateFromLocalHub(
            m_robotDrive.getPose(), hub.get());
        if (distance.isPresent()) m_hoodPreviewSource = "FRESH DIRECT HUB TAG";
      }
    }
    if (distance.isEmpty()
        && m_fieldVerifiedEntry.getBoolean(false)
        && m_robotDrive.isFieldPoseInitialized()
        && m_robotDrive.hasRecentVision(1.0)) {
      distance = HubDistanceCalculator.calculate(
          m_robotDrive.getPose(), DriverStation.getAlliance());
      if (distance.isPresent()) m_hoodPreviewSource = "VERIFIED FIELD POSE";
    }
    if (distance.isEmpty()) return;
    m_hoodPreviewDistanceMeters = distance.getAsDouble();

    Point near = new Point(
        m_hoodNearDistanceEntry.getDouble(0.0), m_hoodNearEncoderEntry.getDouble(0.0));
    Point far = new Point(
        m_hoodFarDistanceEntry.getDouble(0.0), m_hoodFarEncoderEntry.getDouble(0.0));
    if (!HoodDistanceCalculator.areValidPair(near, far)) {
      m_hoodPreviewStatus = "ENTER TWO MEASURED DISTANCE/ENCODER POINTS";
      return;
    }
    OptionalDouble target = HoodDistanceCalculator.interpolate(
        m_hoodPreviewDistanceMeters, near, far);
    if (target.isEmpty()) {
      m_hoodPreviewStatus = "DISTANCE OUTSIDE CALIBRATED RANGE";
      return;
    }
    m_hoodPreviewEncoderTarget = target.getAsDouble();
    m_hoodPreviewStatus = m_hood.isHoodReady()
        ? "PREVIEW ONLY - NO AUTOMATIC HOOD MOTION"
        : "PREVIEW ONLY - HOOD SENSOR/LIMITS NOT READY";
  }

  /**
   * Geometry-only preview works while Disabled; no automatic motor outputs.
   * A valid recent tag and verified physical camera geometry are required.
   */
  private void updateHubAimPreview() {
    if (Constants.HubAimSafety.ENABLE_AUTO_ROTATION
        && DriverStation.isTeleopEnabled() && m_visionAlignEnabled) return;
    if (!m_robotDrive.isGyroReady()) {
      m_hubCenterAlign.stop();
      return;
    }
    double now = Timer.getFPGATimestamp();
    Pose2d pose = m_robotDrive.getPose();
    if (m_directHubGeometryEntry.getBoolean(false)
        && m_hubRelativeTarget.isFresh(now)) {
      m_hubRelativeTarget.getHubInOdometryFrame(now).ifPresent(hub ->
          m_hubCenterAlign.previewOnly(pose, hub, "V2 DIRECT TAG"));
    } else if (m_fieldVerifiedEntry.getBoolean(false)
        && m_robotDrive.isFieldPoseInitialized()
        && m_robotDrive.hasRecentVision(1.0)
        && DriverStation.getAlliance().isPresent()) {
      m_hubCenterAlign.previewOnly(pose, HubAimHeadingV2.hubForAlliance(
          DriverStation.getAlliance().orElseThrow()), "V2 FIELD POSE");
    } else {
      m_hubCenterAlign.stop();
    }
  }

  /**
   * Single owner for teleop manual feeding. The speed slider sets output only;
   * the A button or explicitly pressed 2s test button starts motion.
   * Neither source has an effect outside TELEOP. Autonomous owns its own
   * feeder command whenever a NamedCommand requires this subsystem.
   */
  private void updateManualFeeder() {
    if (!DriverStation.isTeleopEnabled()) {
      m_manualFeederCommandActive = false;
      m_feederDashboardTestDeadline = -1.0;
      m_feeder.stopFeeder();
      m_feederManualStatus = "BLOCKED: TELEOP NOT ENABLED";
      return;
    }
    boolean aPressed = m_driverController.getAButton();
    boolean timedTest = TeleopFeederControl.isDashboardTestActive(
        Timer.getFPGATimestamp(), m_feederDashboardTestDeadline);
    double output = TeleopFeederControl.selectOutput(
        true, aPressed, timedTest, m_feederSpeedEntry.getDouble(kDefaultFeederPercent));
    if (output > 0.0) m_feeder.runFeeder(output);
    else m_feeder.stopFeeder();
    m_manualFeederCommandActive = aPressed && m_feeder.isFeeding();
    if (output <= 0.0 && (aPressed || timedTest)) {
      m_feederManualStatus = "BLOCKED: FEEDER SPEED IS ZERO OR INVALID";
    } else if (aPressed) {
      m_feederManualStatus = "A BUTTON: FORWARD";
    } else if (timedTest) {
      m_feederManualStatus = "DASHBOARD TEST: FORWARD (AUTO STOP)";
    } else {
      m_feederManualStatus = "IDLE: HOLD A OR PRESS FEEDER TEST (2 s)";
    }
  }

  /** A rising dashboard request starts a bounded test; never latch motor on. */
  private void updateFeederDashboardTest() {
    if (!DriverStation.isTeleopEnabled()) {
      m_feederDashboardTestDeadline = -1.0;
    }
    if (!m_feederTestEntry.getBoolean(false)) return;
    m_feederTestEntry.setBoolean(false);
    if (DriverStation.isTeleopEnabled()) {
      m_feederDashboardTestDeadline =
          Timer.getFPGATimestamp() + TeleopFeederControl.DASHBOARD_TEST_SECONDS;
    } else {
      m_feederManualStatus = "TEST REJECTED: ENABLE TELEOP FIRST";
    }
  }

  /** Camera frames are read once per loop; Shuffleboard suppliers publish live values. */
  public void updateDashboard() {
    // Keep trim preview and its settling reset current even while disabled.
    m_hubCenterAlign.setAimTrimDegrees(m_hubAimTrimEntry.getDouble(0.0));
    updateFeederDashboardTest();
    updateVisionTarget();
    updateHubAimPreview();
    updateHoodDistancePreview();
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
    Command auto = m_autoChooser.getSelected();
    if (auto == null) {
      return Commands.none();
    }
    // Stop flywheels even when an auto is interrupted before its Stop marker.
    return auto.finallyDo(interrupted -> stopAutonomousShooter());
  }
}
