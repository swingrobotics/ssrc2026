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
  private static final double kAutoShootReadyTimeoutSeconds = 3.0;
  private static final double kAutoHoodDownTimeoutSeconds = 4.0;
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
  private final GenericEntry m_shooterSpeedEntry;
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
  // NaN means no autonomous target; otherwise this is the requested encoder position.
  private double m_autoHoodTargetPosition = Double.NaN;
  private String m_autoHoodDownStatus = "NOT RUN";
  private double m_autoShooterTargetPercent = 0.0;
  private double m_autoShooterStartTime = -1.0;
  private String m_autoShooterStatus = "Idle";
  // The intake request survives between PathPlanner paths; 1 = in, -1 = reverse.
  // The subsystem's default command owns the motors, not the path command.
  private int m_autoIntakeDirection = 0;
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
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  // PathPlanner autonomous chooser.
  private final SendableChooser<Command> m_autoChooser;

  // PhotonVision camera nickname. Must exactly match the nickname in the PhotonVision UI.
  private final PhotonCamera m_photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private final PhotonPoseEstimator m_photonPoseEstimator =
      new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_CAMERA);
  private final HubAimHeadingV2 m_hubCenterAlign = new HubAimHeadingV2();
  // V3 is the active aiming target: one fixed HUB point in LOCAL odometry.
  private final HubAimTrackerV3 m_hubTrackerV3 =
      new HubAimTrackerV3(VisionConstants.FIELD_LAYOUT);
  // Retain V2 as telemetry-only comparison until V3 is tested on the robot.
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

    // Shooter output is DIRECT duty-cycle percentage, as in the old branch.
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    m_shooterSpeedEntry = shooterTab.add(
        "Shooter Output (%)", ShooterConstants.DEFAULT_OUTPUT_PERCENT)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0,
            "max", ShooterConstants.MAX_MOTOR_OUTPUT * 100.0))
        .withPosition(0, 0).withSize(3, 1).getEntry();
    m_feederSpeedEntry = shooterTab.add("Feeder Speed (%)", kDefaultFeederPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0))
        .withPosition(3, 0).withSize(3, 1).getEntry();
    m_feederTestEntry = shooterTab.add("Feeder Test (2 s)", false)
        .withWidget(BuiltInWidgets.kToggleButton)
        .withPosition(3, 3).withSize(2, 1).getEntry();
    m_feederTestEntry.setBoolean(false);
    shooterTab.addDouble("Shooter Actual RPM (diagnostic only)", m_shooter::getActualRPM)
        .withPosition(4, 1).withSize(2, 1);
    shooterTab.addString("Feeder Direction", m_feeder::getDirection)
        .withPosition(6, 0).withSize(2, 1);
    shooterTab.addString("Shooter Feeder Status", this::getTeleopShooterFeederStatus)
        .withPosition(0, 3).withSize(3, 1);
    shooterTab.addString("Feeder Manual Status", () -> m_feederManualStatus)
        .withPosition(0, 4).withSize(7, 1);

    // Button/robot state troubleshooting is separated from operator controls.
    ShuffleboardTab mechanismDebug = Shuffleboard.getTab("Mechanism Diagnostics");
    mechanismDebug.addBoolean("Operator B Pressed", m_operatorController::getBButton);
    mechanismDebug.addBoolean("Operator A Pressed", m_operatorController::getAButton);
    mechanismDebug.addBoolean("Operator X Pressed", m_operatorController::getXButton);
    mechanismDebug.addBoolean("Operator Y Pressed", m_operatorController::getYButton);
    mechanismDebug.addDouble("Operator Arm Stick Y",
        () -> -m_operatorController.getLeftY());
    mechanismDebug.addBoolean("Teleop Enabled", DriverStation::isTeleopEnabled);
    mechanismDebug.addBoolean("B Shoot Command Active", () -> m_teleopShootCommandActive);
    mechanismDebug.addBoolean("A Feeder-Only Active", () -> m_manualFeederCommandActive);
    mechanismDebug.addBoolean("Controller Port 0 Connected",
        () -> DriverStation.isJoystickConnected(OIConstants.kDriverControllerPort));
    mechanismDebug.addString("Controller Port 0 Name",
        () -> DriverStation.getJoystickName(OIConstants.kDriverControllerPort));
    mechanismDebug.addBoolean("Operator Port 1 Connected",
        () -> DriverStation.isJoystickConnected(OIConstants.kOperatorControllerPort));
    mechanismDebug.addString("Operator Port 1 Name",
        () -> DriverStation.getJoystickName(OIConstants.kOperatorControllerPort));
    mechanismDebug.addBoolean("Raw Button 1 (Operator A)",
        () -> m_operatorController.getRawButton(1));
    mechanismDebug.addBoolean("Feeder Dashboard Test Active", () ->
        TeleopFeederControl.isDashboardTestActive(
            Timer.getFPGATimestamp(), m_feederDashboardTestDeadline));
    mechanismDebug.addDouble("Feeder Speed Setpoint (%)",
        () -> getOutputScale(m_feederSpeedEntry, kDefaultFeederPercent) * 100.0);

    // Shooter CAN 9 direct duty cycle, with 10/12 hardware followers.
    // A-button feeder control remains manual and independent.
    ShuffleboardTab shotTab = Shuffleboard.getTab("Shot Tuning");
    m_autoFeederEnabledEntry = shotTab.add("Autonomous Feeder Enabled", true).getEntry();
    m_autoFeedDurationEntry = shotTab.add("Autonomous Feed Duration (s)", 0.75)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.1, "max", 2.0)).getEntry();
    shotTab.addBoolean("Autonomous Shot Ready", this::isAutonomousShooterReady);
    shotTab.addString("Autonomous Shooter Status", () -> m_autoShooterStatus);
    shotTab.addString("Last Auto Shot Result", () -> m_autoLastShotResult);
    shotTab.addBoolean("Auto Robot Stopped", this::isAutonomousDriveStoppedNow);
    shotTab.addBoolean("Auto Hood Max Reached",
        () -> m_autoHoodTargetPosition == HoodConstants.AUTO_FULL_RAISED_POSITION
            && m_hood.isAtTargetPosition());
    shotTab.addString("Auto Hood Down Status", () -> m_autoHoodDownStatus);

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
    visionTab.addBoolean("HUB Auto Aim Active", () -> m_visionAlignEnabled);
    visionTab.addString("HUB Safety Status", this::getHubRotationBlocker);
    visionTab.addString("HUB Aim Source", m_hubCenterAlign::getSource);
    // Read-only V3 numbers have their OWN tab, separate from V2 diagnostics.
    ShuffleboardTab hubV3Tab = Shuffleboard.getTab("HUB V3");
    hubV3Tab.addString("V3 Auto Aim Status", this::getHubRotationBlocker)
        .withPosition(0, 5).withSize(8, 1);
    hubV3Tab.addString("V3 Tracking State",
        () -> m_hubTrackerV3.getStatus(Timer.getFPGATimestamp()))
        .withPosition(0, 0).withSize(4, 1);
    hubV3Tab.addBoolean("V3 Target Locked",
        () -> m_hubTrackerV3.hasLock(Timer.getFPGATimestamp()))
        .withPosition(4, 0).withSize(2, 1);
    hubV3Tab.addBoolean("V3 Camera Fresh",
        () -> m_hubTrackerV3.hasLiveTag(Timer.getFPGATimestamp()))
        .withPosition(6, 0).withSize(2, 1);
    hubV3Tab.addDouble("V3 HUB Odom X (m)",
        () -> m_hubTrackerV3.getHubX(Timer.getFPGATimestamp()))
        .withPosition(0, 1).withSize(2, 1);
    hubV3Tab.addDouble("V3 HUB Odom Y (m)",
        () -> m_hubTrackerV3.getHubY(Timer.getFPGATimestamp()))
        .withPosition(2, 1).withSize(2, 1);
    hubV3Tab.addDouble("V3 Tag Age (s)",
        () -> m_hubTrackerV3.getTargetAge(Timer.getFPGATimestamp()))
        .withPosition(4, 1).withSize(2, 1);
    hubV3Tab.addDouble("V3 Used Tag Count", m_hubTrackerV3::getTagCount)
        .withPosition(6, 1).withSize(2, 1);
    hubV3Tab.addDouble("V3 Robot Odom X", () -> m_robotDrive.getPose().getX())
        .withPosition(0, 2).withSize(2, 1);
    hubV3Tab.addDouble("V3 Robot Odom Y", () -> m_robotDrive.getPose().getY())
        .withPosition(2, 2).withSize(2, 1);
    hubV3Tab.addDouble("V3 Bearing (deg)",
        () -> m_hubTrackerV3.getBearingDegrees(
            m_robotDrive.getPose(), Timer.getFPGATimestamp()))
        .withPosition(4, 2).withSize(2, 1);
    hubV3Tab.addDouble("V3 Distance (m)",
        () -> m_hubTrackerV3.getDistanceMeters(
            m_robotDrive.getPose(), Timer.getFPGATimestamp()))
        .withPosition(6, 2).withSize(2, 1);
    hubV3Tab.addDouble("V3 Heading Error (deg)",
        m_hubCenterAlign::getHeadingErrorDegrees)
        .withPosition(0, 3).withSize(2, 1);
    hubV3Tab.addDouble("V3 Desired Heading (deg)",
        m_hubCenterAlign::getDesiredHeadingDegrees)
        .withPosition(2, 3).withSize(2, 1);
    hubV3Tab.addBoolean("V3 Geometry Verified",
        () -> m_directHubGeometryEntry.getBoolean(false))
        .withPosition(4, 3).withSize(2, 1);
    hubV3Tab.addBoolean("V3 Auto Aim Active",
        () -> m_visionAlignEnabled)
        .withPosition(6, 3).withSize(2, 1);
    hubV3Tab.addString("V3 Last Rejection", m_hubTrackerV3::getLastRejection)
        .withPosition(0, 4).withSize(6, 1);
    hubV3Tab.addString("V3 Alliance", m_hubTrackerV3::getAlliance)
        .withPosition(6, 4).withSize(2, 1);
    visionDebugTab.addString("HUB Direct Status", m_hubRelativeTarget::getStatus);
    // Keep the already-crowded Vision Diagnostics tab readable. Use a new
    // dedicated tab to show per-tag rejection gates in SHORT strings and
    // numeric 3D telemetry. No impact on HUB decisions or robot outputs.
    ShuffleboardTab hubTagDebugTab = Shuffleboard.getTab("HUB Tag Debug");
    hubTagDebugTab.addString("Tag 25 Reason",
        () -> m_hubRelativeTarget.getTagDebugGate(25))
        .withPosition(0, 0).withSize(4, 1);
    hubTagDebugTab.addDouble("Tag 25 Ambiguity",
        () -> m_hubRelativeTarget.getTagDebugAmbiguity(25))
        .withPosition(4, 0).withSize(2, 1);
    hubTagDebugTab.addDouble("Tag 25 Range (m)",
        () -> m_hubRelativeTarget.getTagDebugRangeMeters(25))
        .withPosition(6, 0).withSize(2, 1);

    hubTagDebugTab.addString("Tag 26 Reason",
        () -> m_hubRelativeTarget.getTagDebugGate(26))
        .withPosition(0, 1).withSize(4, 1);
    hubTagDebugTab.addDouble("Tag 26 Ambiguity",
        () -> m_hubRelativeTarget.getTagDebugAmbiguity(26))
        .withPosition(4, 1).withSize(2, 1);
    hubTagDebugTab.addDouble("Tag 26 Range (m)",
        () -> m_hubRelativeTarget.getTagDebugRangeMeters(26))
        .withPosition(6, 1).withSize(2, 1);

    hubTagDebugTab.addDouble("Tag 25 X (m)",
        () -> m_hubRelativeTarget.getTagDebugX(25))
        .withPosition(0, 2).withSize(2, 1);
    hubTagDebugTab.addDouble("Tag 25 Y (m)",
        () -> m_hubRelativeTarget.getTagDebugY(25))
        .withPosition(2, 2).withSize(2, 1);
    hubTagDebugTab.addDouble("Tag 25 Z (m)",
        () -> m_hubRelativeTarget.getTagDebugZ(25))
        .withPosition(4, 2).withSize(2, 1);
    hubTagDebugTab.addDouble("Tag 26 X (m)",
        () -> m_hubRelativeTarget.getTagDebugX(26))
        .withPosition(0, 3).withSize(2, 1);
    hubTagDebugTab.addDouble("Tag 26 Y (m)",
        () -> m_hubRelativeTarget.getTagDebugY(26))
        .withPosition(2, 3).withSize(2, 1);
    hubTagDebugTab.addDouble("Tag 26 Z (m)",
        () -> m_hubRelativeTarget.getTagDebugZ(26))
        .withPosition(4, 3).withSize(2, 1);
    hubTagDebugTab.addString("HUB Last Rejection",
        m_hubRelativeTarget::getLastRejectionReason)
        .withPosition(0, 4).withSize(6, 1);
    hubTagDebugTab.addDouble("HUB Valid Candidates",
        m_hubRelativeTarget::getValidCandidateCount)
        .withPosition(6, 4).withSize(2, 1);
    // Retain detailed raw strings in this uncrowded tab as secondary telemetry.
    hubTagDebugTab.addString("Tag 25 Full Details",
        m_hubRelativeTarget::getTag25Diagnostics)
        .withPosition(0, 5).withSize(8, 1);
    hubTagDebugTab.addString("Tag 26 Full Details",
        m_hubRelativeTarget::getTag26Diagnostics)
        .withPosition(0, 6).withSize(8, 1);
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
    registerAutonomousIntakeCommands();

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

    // Teleop right stick is unchanged. In autonomous, the hood follows the
    // requested up/down encoder target without interrupting path following.
    m_hood.setDefaultCommand(new RunCommand(() -> {
      if (DriverStation.isAutonomousEnabled()
          && Double.isFinite(m_autoHoodTargetPosition)) {
        m_hood.setTargetPosition(m_autoHoodTargetPosition);
      } else {
        double input = DriverStation.isTeleopEnabled()
            ? MathUtil.applyDeadband(-m_driverController.getRightY(),
                OIConstants.kHoodDeadband)
            : 0.0;
        m_hood.setManual(input * HoodConstants.TEST_MAX_SPEED);
      }
    }, m_hood));

    // Keeps motor ownership in one command (and avoids path-marker commands
    // with drive subsystem requirements interfering with path following).
    m_shooter.setDefaultCommand(new RunCommand(() -> {
      if (DriverStation.isAutonomousEnabled() && m_autoShooterRequested) {
        m_shooter.setOutputPercent(m_autoShooterTargetPercent);
      } else if (m_shooter.isRunning()) {
        m_shooter.stopShooter();
      }
    }, m_shooter));

    // Auto intake keeps running while PathPlanner follows a path. Operator X
    // temporarily overrides this default command without affecting shooter.
    m_intake.setDefaultCommand(new RunCommand(() -> {
      if (!DriverStation.isAutonomousEnabled()) {
        m_intake.stopIntake();
      } else if (m_autoIntakeDirection > 0) {
        m_intake.runIntake();
      } else if (m_autoIntakeDirection < 0) {
        m_intake.reverseIntake();
      } else {
        m_intake.stopIntake();
      }
    }, m_intake));

    // Operator left-stick Y gives proportional intake arm control in teleop.
    // PathPlanner arm pulses preempt this default command during autonomous.
    m_intakeArm.setDefaultCommand(new RunCommand(() -> {
      if (DriverStation.isTeleopEnabled()) {
        double stick = MathUtil.applyDeadband(
            -m_operatorController.getLeftY(), OIConstants.kIntakeArmDeadband);
        m_intakeArm.setManualSpeed(stick);
      } else {
        m_intakeArm.stopArm();
      }
    }, m_intakeArm));

    // Own the feeder continuously in an independent command so operator A and
    // the short-lived dashboard test share exactly the same stop/speed logic.
    // Existing autonomous feeder commands take the feeder requirement and
    // preempt this default command while their own safety gates apply.
    m_feeder.setDefaultCommand(new RunCommand(this::updateManualFeeder, m_feeder));
  }

  /** Read a 0-100% Shuffleboard slider and safely convert to an output multiplier. */
  private static double getOutputScale(GenericEntry entry, double defaultPercent) {
    double percent = entry.getDouble(defaultPercent);
    return Double.isFinite(percent) ? MathUtil.clamp(percent, 0.0, 100.0) / 100.0 : 0.0;
  }

  /** Read the user-selected direct shooter motor percentage. */
  private double getShooterOutputPercent() {
    return ShooterSubsystem.sanitizeOutputPercent(
        m_shooterSpeedEntry.getDouble(ShooterConstants.DEFAULT_OUTPUT_PERCENT));
  }

  /** Describes the operator controller's A feeder and B shooter controls. */
  private String getTeleopShooterFeederStatus() {
    if (!DriverStation.isTeleopEnabled()) return "BLOCKED: TELEOP NOT ENABLED";
    boolean a = m_operatorController.getAButton();
    boolean b = m_operatorController.getBButton();
    boolean dashboardTest = TeleopFeederControl.isDashboardTestActive(
        Timer.getFPGATimestamp(), m_feederDashboardTestDeadline);
    if (!a && !b && !dashboardTest)
      return "IDLE: OPERATOR A=FEEDER, B=SHOOTER, OR USE 2s TEST";
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
    NamedCommands.registerCommand("Auto Feeder Stop",
        Commands.runOnce(this::stopAutonomousFeeder));
    NamedCommands.registerCommand("Auto Hood Max",
        Commands.runOnce(this::requestAutonomousHoodMax));
    NamedCommands.registerCommand("Auto Hood Down", makeAutonomousHoodDownCommand());
    NamedCommands.registerCommand("Auto Hood Hold",
        Commands.runOnce(this::stopAutonomousHoodTarget));
  }

  private void requestAutonomousHoodMax() {
    requestAutonomousHoodTarget(HoodConstants.AUTO_FULL_RAISED_POSITION);
  }

  private void requestAutonomousHoodDown() {
    requestAutonomousHoodTarget(HoodConstants.AUTO_FULL_LOWERED_POSITION);
  }

  private void requestAutonomousHoodTarget(double targetPosition) {
    m_autoHoodTargetPosition =
        DriverStation.isAutonomousEnabled() && m_hood.isHoodReady()
            && Double.isFinite(targetPosition)
            && targetPosition >= HoodConstants.MIN_POSITION
            && targetPosition <= HoodConstants.MAX_POSITION
                ? targetPosition : Double.NaN;
  }

  private boolean isAutonomousHoodDownReached() {
    return DriverStation.isAutonomousEnabled()
        && m_autoHoodTargetPosition == HoodConstants.AUTO_FULL_LOWERED_POSITION
        && m_hood.getTargetPosition() == HoodConstants.AUTO_FULL_LOWERED_POSITION
        && m_hood.isAtTargetPosition();
  }

  /**
   * Waits for a full lowering stroke, rather than letting the auto finish
   * immediately and canceling the hood request before it moves.
   * A timeout/interruption captures the current angle for safety.
   */
  private Command makeAutonomousHoodDownCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          requestAutonomousHoodDown();
          m_autoHoodDownStatus = Double.isFinite(m_autoHoodTargetPosition)
              ? "MOVING TO LOWERED LIMIT" : "BLOCKED: HOOD NOT READY";
        }),
        Commands.waitUntil(() -> !Double.isFinite(m_autoHoodTargetPosition)
            || isAutonomousHoodDownReached())
            .withTimeout(kAutoHoodDownTimeoutSeconds))
        .finallyDo(interrupted -> {
          boolean requested =
              m_autoHoodTargetPosition == HoodConstants.AUTO_FULL_LOWERED_POSITION;
          boolean reached = isAutonomousHoodDownReached();
          m_autoHoodDownStatus = interrupted ? "INTERRUPTED"
              : !requested ? "BLOCKED: NO VALID TARGET"
              : reached ? "LOWERED POSITION REACHED"
              : "TIMEOUT: HOLDING CURRENT POSITION";
          stopAutonomousHoodTarget();
        });
  }

  private void stopAutonomousHoodTarget() {
    m_autoHoodTargetPosition = Double.NaN;
    m_hood.captureCurrentPosition();
  }

  /**
   * PathPlanner intake actions are independent of the drivetrain, so they can
   * be used as sequential auto steps or path event markers.
   *
   * Arm actions are short pulses only: there is no calibrated travel limit
   * or absolute arm encoder, so they are NOT automatic deploy-to-position.
   */
  private void registerAutonomousIntakeCommands() {
    NamedCommands.registerCommand("Auto Intake Start",
        Commands.runOnce(() -> setAutonomousIntakeDirection(1)));
    NamedCommands.registerCommand("Auto Intake Reverse",
        Commands.runOnce(() -> setAutonomousIntakeDirection(-1)));
    NamedCommands.registerCommand("Auto Intake Stop",
        Commands.runOnce(this::stopAutonomousIntake));
    NamedCommands.registerCommand("Auto Intake Arm Forward",
        Commands.runEnd(m_intakeArm::moveForward, m_intakeArm::stopArm, m_intakeArm)
            .withTimeout(0.5));
    NamedCommands.registerCommand("Auto Intake Arm Backward",
        Commands.runEnd(m_intakeArm::moveBackward, m_intakeArm::stopArm, m_intakeArm)
            .withTimeout(0.5));
  }

  private void setAutonomousIntakeDirection(int direction) {
    m_autoIntakeDirection = DriverStation.isAutonomousEnabled()
        ? Integer.signum(direction) : 0;
  }

  private void stopAutonomousIntake() {
    m_autoIntakeDirection = 0;
    m_intake.stopIntake();
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
    double outputPercent = getShooterOutputPercent();
    if (outputPercent <= 0.0) {
      m_autoShooterStatus = "BLOCKED: Shooter Output (%) is zero";
      return;
    }
    m_autoShooterTargetPercent = outputPercent;
    m_autoShooterStartTime = Timer.getFPGATimestamp();
    m_autoShooterRequested = true;
    requestAutonomousHoodMax();
    m_autoShooterStatus = "SPINNING: shooter percent + hood max target";
  }

  private boolean isAutonomousShooterReady() {
    return DriverStation.isAutonomousEnabled()
        && m_autoShooterRequested
        && m_autoHoodTargetPosition == HoodConstants.AUTO_FULL_RAISED_POSITION
        && m_hood.isHoodReady() && m_hood.isAtTargetPosition()
        && m_shooter.isReadyToFeed()
        && m_autoShooterStartTime >= 0.0
        && Timer.getFPGATimestamp() - m_autoShooterStartTime >= 1.0;
  }

  /** Stop only the feeder, without changing the running shooter or hood target. */
  private void stopAutonomousFeeder() {
    // Aborts any in-progress timed feed; the feed command owns its own cleanup.
    m_autoFeedDeadline = -1.0;
    m_feeder.stopFeeder();
  }

  private void stopAutonomousShooter() {
    stopAutonomousFeeder();
    m_autoShooterRequested = false;
    m_autoHoodTargetPosition = Double.NaN;
    m_autoShooterTargetPercent = 0.0;
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
      Pose2d cameraCapturePose =
          m_robotDrive.getPoseAtTime(result.getTimestampSeconds()).orElse(null);
      // V3 owns aim; V2 receives the exact same data for comparison only.
      m_hubTrackerV3.observe(
          result.getTargets(), DriverStation.getAlliance(),
          cameraCapturePose, result.getTimestampSeconds(), now);
      m_hubRelativeTarget.observe(
          result.getTargets(), DriverStation.getAlliance(),
          cameraCapturePose, result.getTimestampSeconds(), now);
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
        m_hubTrackerV3.clear(); // fixed odometry point is no longer valid
        m_visionResetStatus = "INITIALIZED FROM VERIFIED MULTITAG POSE";
        // Wait for the next validated frame before allowing auto-aim.
      }
      m_visionResetEntry.setBoolean(false);
    }
  }




  /** X toggles V3 aiming; no dashboard arm or full-field pose is required. */
  private String getHubRotationBlocker() {
    if (!DriverStation.isTeleopEnabled()) return "TELEOP NOT ENABLED";
    if (!m_visionAlignEnabled) return "PRESS X TO AIM";
    if (!m_robotDrive.isGyroReady()) return "NAVX NOT READY";
    if (!m_hubTrackerV3.hasLock(Timer.getFPGATimestamp()))
      return "NO VALID HUB TARGET";
    return "V3 TRACKING HUB CENTER";
  }

  private double getDriveRotationCommand() {
    if (!m_visionAlignEnabled) {
      m_lastHubTurnOutput = 0.0;
      return getTriggerRotation();
    }
    Optional<Translation2d> hub = m_hubTrackerV3.getHubInOdometry(
        Timer.getFPGATimestamp());
    if (!m_robotDrive.isGyroReady() || hub.isEmpty()) {
      m_hubCenterAlign.stop();
      m_lastHubTurnOutput = 0.0;
      return 0.0;
    }
    m_lastHubTurnOutput = m_hubCenterAlign.calculateDirectTurn(
        m_robotDrive.getPose(), m_robotDrive.getRobotRelativeSpeeds(),
        hub.orElseThrow(), true);
    return m_lastHubTurnOutput;
  }

  /** Toggle between robot-relative and field-relative translation control. */
  private void toggleFieldRelative() {
    m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
  }

  /** X starts/stops HUB-center tracking directly in Teleop. */
  private void toggleVisionAlign() {
    if (!DriverStation.isTeleopEnabled()) return;
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
    // Operator B controls only the shooter. A (feeder) may run concurrently.
    new Trigger(() -> DriverStation.isTeleopEnabled()
        && m_operatorController.getBButton())
        .whileTrue(Commands.runEnd(
            () -> {
              m_teleopShootCommandActive = true;
              m_shooter.setOutputPercent(getShooterOutputPercent());
            },
            () -> {
              m_shooter.stopShooter();
              m_teleopShootCommandActive = false;
            },
            m_shooter));

    // Operator A and the timed dashboard test are handled by the feeder's
    // default command, independently of the shooter-only B trigger.

    // Driver X/Y remain dedicated to HUB aim and field-relative driving.
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(new InstantCommand(this::toggleVisionAlign));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(this::toggleFieldRelative));

    // Operator X intakes forward; Y reverses. Y has priority if both are held,
    // so the two commands cannot repeatedly interrupt each other.
    new Trigger(() -> DriverStation.isTeleopEnabled()
        && m_operatorController.getXButton()
        && !m_operatorController.getYButton())
        .whileTrue(Commands.runEnd(
            m_intake::runIntake,
            m_intake::stopIntake,
            m_intake));

    new Trigger(() -> DriverStation.isTeleopEnabled()
        && m_operatorController.getYButton())
        .whileTrue(Commands.runEnd(
            m_intake::reverseIntake,
            m_intake::stopIntake,
            m_intake));

    // The operator left-stick Y axis controls intake-arm direction and speed
    // through the arm's default command, with a hold-to-move timeout.

    // Original right-bumper X-lock behavior.
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> {
          m_robotDrive.zeroHeading();
          m_hubRelativeTarget.clear(); // odometry heading reference changed
          m_hubTrackerV3.clear(); // fixed odometry point is no longer valid
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
        && m_hubTrackerV3.hasLock(now)) {
      Optional<Translation2d> hub = m_hubTrackerV3.getHubInOdometry(now);
      if (hub.isPresent()) {
        distance = HubDistanceCalculator.calculateFromLocalHub(
            m_robotDrive.getPose(), hub.get());
        if (distance.isPresent()) m_hoodPreviewSource = "VERIFIED V3 ODOM HUB";
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
   * V3 read-only geometry preview. This is intentionally visible BEFORE
   * certifying the camera, without changing any motor output/interlock.
   */
  private void updateHubAimPreview() {
    if (DriverStation.isTeleopEnabled() && m_visionAlignEnabled) return;
    if (!m_robotDrive.isGyroReady()) {
      m_hubCenterAlign.stop();
      return;
    }
    double now = Timer.getFPGATimestamp();
    Optional<Translation2d> fixedHub = m_hubTrackerV3.getHubInOdometry(now);
    if (fixedHub.isEmpty()) {
      m_hubCenterAlign.stop();
      return;
    }
    String source = m_hubTrackerV3.hasLiveTag(now)
        ? "V3 ODOM FIX (TAG)" : "V3 ODOM FIX (HOLD)";
    if (!m_directHubGeometryEntry.getBoolean(false))
      source += " - UNVERIFIED CAMERA";
    m_hubCenterAlign.previewOnly(m_robotDrive.getPose(), fixedHub.orElseThrow(), source);
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
    boolean aPressed = m_operatorController.getAButton();
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
      m_feederManualStatus = "OPERATOR A BUTTON: FORWARD";
    } else if (timedTest) {
      m_feederManualStatus = "DASHBOARD TEST: FORWARD (AUTO STOP)";
    } else {
      m_feederManualStatus = "IDLE: HOLD OPERATOR A OR PRESS FEEDER TEST (2 s)";
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
    if (!DriverStation.isTeleopEnabled()) {
      m_visionAlignEnabled = false;
      m_lastHubTurnOutput = 0.0;
    }
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
    return auto.finallyDo(interrupted -> {
      stopAutonomousShooter();
      stopAutonomousHoodTarget();
      stopAutonomousIntake();
      m_intakeArm.stopArm();
    });
  }
}
