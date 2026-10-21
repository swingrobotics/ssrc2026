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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.OptionalDouble;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HubConstants;
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
  private static final double kDefaultShooterPercent =
      ShooterConstants.DEFAULT_OUTPUT_PERCENT;
  private static final double kDefaultFeederPercent = Math.abs(FeederConstants.TEST_SPEED) * 100.0;

  // The original swerve/vision/autonomous subsystem, plus the ZIP's mechanism subsystems.
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // The SHOOTER feeder (CAN 11/14) may feed only after measured flywheel
  // stability in percent mode or target RPM stability in calibrated mode.
  private final FeederSubsystem m_feeder = new FeederSubsystem(this::isShooterFeederReady);
  private final HoodSubsystem m_hood = new HoodSubsystem();
  // Creates no motor controllers until both intake CAN IDs are assigned.
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // Dedicated intake ARM on an as-yet-unassigned Spark MAX CAN ID.
  private final IntakeArmSubsystem m_intakeArm = new IntakeArmSubsystem();

  private final GenericEntry m_driveSpeedEntry;
  private final GenericEntry m_shooterSpeedEntry;
  private final GenericEntry m_feederSpeedEntry;
  // Clean fixed-RPM calibration: use ONE tested motor RPM with manual hood
  // first, then two measured near/far hood endpoints for live HUB tracking.
  // Both modes default OFF; zero RPM and -1 raw hood positions are invalid.
  private final GenericEntry m_fixedRpmEnabledEntry;
  private final GenericEntry m_fixedRpmEntry;
  private final GenericEntry m_distanceHoodEnabledEntry;
  private final GenericEntry m_hoodDistanceP1Entry;
  private final GenericEntry m_hoodEncoderP1Entry;
  private final GenericEntry m_hoodDistanceP2Entry;
  private final GenericEntry m_hoodEncoderP2Entry;

  private double m_hubRawDistance = Double.NaN;
  private double m_hubFilteredDistance = Double.NaN;
  private double m_lastHubVisionTimestamp = -1.0;
  private Optional<Alliance> m_lastHubAlliance = Optional.empty();
  private String m_hubDistanceStatus = "No valid field pose";
  private boolean m_autoHoodTargetActive = false;
  private boolean m_autoHoodManualOverride = false;
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
  private double m_autoShooterTargetRpm = 0.0;
  private double m_autoShooterTargetPercent = 0.0;
  private double m_autoShooterStartTime = -1.0;
  private boolean m_autoShooterUseFixedRpm = false;
  private String m_autoShooterStatus = "Idle";
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
    m_shooterSpeedEntry = driveTab.add("Shooter Output (%)", kDefaultShooterPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0,
            "max", ShooterConstants.MAX_MOTOR_OUTPUT * 100.0)).getEntry();
    // Actual shaft RPM remains a measurement, not the manual speed control.
    driveTab.addDouble("Shooter Actual RPM", m_shooter::getActualRPM);
    driveTab.addBoolean("Feeder Active", m_feeder::isFeeding);
    m_feederSpeedEntry = driveTab.add("Feeder Speed (%)", kDefaultFeederPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
    driveTab.addBoolean("Field Relative", () -> m_fieldRelativeEnabled);
    driveTab.addBoolean("Tag Auto Align", () -> m_visionAlignEnabled);

    // Only the chosen calibration strategy: one FIXED motor-shaft RPM,
    // then two measured HUB distances with raw hood encoder endpoints.
    // Keep all automatic calibration OFF by default.
    ShuffleboardTab shotTab = Shuffleboard.getTab("Shot Tuning");
    m_fixedRpmEnabledEntry = shotTab.add("Fixed RPM Enabled", false).getEntry();
    m_fixedRpmEntry = shotTab.add("Fixed Shooter RPM", 0.0).getEntry();
    shotTab.addBoolean("Fixed RPM Valid", this::hasValidFixedRpm);
    m_distanceHoodEnabledEntry = shotTab.add("Distance Hood Enabled", false).getEntry();
    m_hoodDistanceP1Entry = shotTab.add("Min Shoot Distance (m)", 0.0).getEntry();
    m_hoodEncoderP1Entry = shotTab.add("Hood at Min Distance", -1.0).getEntry();
    m_hoodDistanceP2Entry = shotTab.add("Max Shoot Distance (m)", 0.0).getEntry();
    m_hoodEncoderP2Entry = shotTab.add("Hood at Max Distance", -1.0).getEntry();
    shotTab.addBoolean("Hood Calibration Valid", this::hasValidHoodCalibration);
    shotTab.addDouble("Calculated Hood Target", () -> getDistanceHoodTarget().orElse(-1.0));
    shotTab.addBoolean("Hood At Distance Target", this::isDistanceHoodAtTarget);
    shotTab.addBoolean("Hood Manual Override", () -> m_autoHoodManualOverride);
    shotTab.addString("Shot Setup Status", this::getShotSetupStatus);

    // Keep autonomous feed explicitly disarmed until robot testing.
    m_autoFeederEnabledEntry = shotTab.add("Autonomous Feeder Enabled", false).getEntry();
    m_autoFeedDurationEntry = shotTab.add("Autonomous Feed Duration (s)", 0.75)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.1, "max", 2.0)).getEntry();
    shotTab.addBoolean("Autonomous Shot Ready", this::isAutonomousShooterReady);
    shotTab.addString("Autonomous Shooter Status", () -> m_autoShooterStatus);
    shotTab.addString("Last Auto Shot Result", () -> m_autoLastShotResult);
    shotTab.addBoolean("Auto Robot Stopped", this::isAutonomousDriveStoppedNow);


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
    visionTab.addDouble("HUB Distance (m)",
        () -> Double.isFinite(getFreshHubDistance()) ? getFreshHubDistance() : -1.0);
    visionTab.addString("HUB Distance Status", () -> m_hubDistanceStatus);
    visionTab.addString("Alliance", () -> DriverStation.getAlliance()
        .map(Enum::name).orElse("UNKNOWN"));

    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    debugTab.addString("Vision Field Layout", () -> "2026 REBUILT Welded");
    debugTab.addString("Vision Camera", () -> VisionConstants.CAMERA_NAME);
    debugTab.addDouble("Vision Target Yaw", () -> m_visionTargetYawDegrees);
    debugTab.addDouble("Vision Pose Age (s)", this::getVisionPoseAgeSeconds);

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
    driveTab.add("Auto Chooser", m_autoChooser);
    driveTab.addString("Selected Auto", () -> {
      Command chosen = m_autoChooser.getSelected();
      return chosen != null ? chosen.getName() : "None";
    });

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> {
              // A gap between autonomous path commands must not execute
              // joystick-driven motion. PathPlanner overrides this default.
              if (!DriverStation.isTeleopEnabled()) {
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

    // The right stick has priority in teleop. During autonomous, never read
    // joystick input; optionally hold a validated, snapshotted shot position.
    m_hood.setDefaultCommand(
        new RunCommand(
            () -> {
              boolean autonomous = DriverStation.isAutonomousEnabled();
              double hoodInput = autonomous ? 0.0 : MathUtil.applyDeadband(
                  -m_driverController.getRightY(), OIConstants.kHoodDeadband);
              boolean bPressed = m_driverController.getBButton();
              boolean distanceTracking = isDistanceHoodEnabled() && isFixedRpmEnabled();
              if (!bPressed && !autonomous) {
                m_autoHoodManualOverride = false;
              }
              if (hoodInput != 0.0) {
                if (distanceTracking && bPressed) {
                  m_autoHoodManualOverride = true;
                }
                m_autoHoodTargetActive = false;
                m_hood.setManual(hoodInput * HoodConstants.TEST_MAX_SPEED);
                return;
              }

              // Auto hood moves only when FIXED RPM is selected and both
              // valid measured endpoints are supplied. No legacy RPM table.
              OptionalDouble target = OptionalDouble.empty();
              if (distanceTracking
                  && ((autonomous && m_autoShooterRequested && m_autoShooterUseFixedRpm)
                      || (DriverStation.isTeleopEnabled()
                          && bPressed && !m_autoHoodManualOverride))) {
                target = getDistanceHoodTarget();
              }

              if (target.isPresent() && m_hood.setTargetPosition(target.getAsDouble())) {
                m_autoHoodTargetActive = true;
                return;
              }
              if (m_autoHoodTargetActive) {
                m_autoHoodTargetActive = false;
                m_hood.captureCurrentPosition();
              } else {
                m_hood.setManual(0.0);
              }
            },
            m_hood));

    // Keeps motor ownership in one command (and avoids path-marker commands
    // with drive subsystem requirements interfering with path following).
    m_shooter.setDefaultCommand(new RunCommand(() -> {
      if (DriverStation.isAutonomousEnabled() && m_autoShooterRequested) {
        if (m_autoShooterUseFixedRpm) {
          m_shooter.setTargetRPM(m_autoShooterTargetRpm);
        } else {
          m_shooter.setOutputPercent(m_autoShooterTargetPercent);
        }
      } else if (m_shooter.isRunning()) {
        m_shooter.stopShooter();
      }
    }, m_shooter));
  }

  /** Read a 0-100% Shuffleboard slider and safely convert to an output multiplier. */
  private static double getOutputScale(GenericEntry entry, double defaultPercent) {
    double percent = entry.getDouble(defaultPercent);
    return Double.isFinite(percent) ? MathUtil.clamp(percent, 0.0, 100.0) / 100.0 : 0.0;
  }

  /** Read direct motor output percentage; does NOT convert RPM to duty cycle. */
  private static double getShooterOutputPercent(GenericEntry entry) {
    double percent = entry.getDouble(kDefaultShooterPercent);
    return Double.isFinite(percent)
        ? MathUtil.clamp(percent, 0.0, ShooterConstants.MAX_MOTOR_OUTPUT * 100.0)
        : 0.0;
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

  private double getAutoRobotLinearSpeed() {
    ChassisSpeeds speeds = m_robotDrive.getRobotRelativeSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
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
      m_autoShooterStatus = "BLOCKED: not enabled autonomous";
      return;
    }
    if (HoodConstants.CALIBRATION_MODE || !m_hood.isHoodReady()) {
      m_autoShooterStatus = "BLOCKED: hood encoder or calibration mode";
      return;
    }
    if (isDistanceHoodEnabled() && (!isFixedRpmEnabled() || !hasValidHoodCalibration())) {
      m_autoShooterStatus = "BLOCKED: fixed RPM + near/far hood measurements required";
      return;
    }

    double rpm = 0.0;
    double percent = 0.0;
    boolean rpmMode = isFixedRpmEnabled();
    if (rpmMode) {
      rpm = getValidatedFixedRpm();
      if (!Double.isFinite(rpm)) {
        m_autoShooterStatus = "BLOCKED: fixed shooter RPM not configured";
        return;
      }
    } else {
      percent = getShooterOutputPercent(m_shooterSpeedEntry);
      if (percent <= 0.0) {
        m_autoShooterStatus = "BLOCKED: manual shooter percent not configured";
        return;
      }
    }

    m_autoShooterUseFixedRpm = rpmMode;
    m_autoShooterTargetRpm = rpm;
    m_autoShooterTargetPercent = percent;
    m_autoShooterStartTime = Timer.getFPGATimestamp();
    m_autoShooterRequested = true;
    m_autoShooterStatus = rpmMode
        ? "SPINNING: fixed target RPM" : "SPINNING: manual output percent";
  }

  private boolean isAutonomousShooterReady() {
    if (!DriverStation.isAutonomousEnabled()
        || !m_autoShooterRequested
        || !m_hood.isHoodReady()
        || m_autoShooterUseFixedRpm != isFixedRpmEnabled()) {
      return false;
    }
    // If distance hood mode is toggled ON, require a valid fixed RPM and
    // fresh vision-based calibrated hood position before ever feeding.
    if (isDistanceHoodEnabled()
        && (!m_autoShooterUseFixedRpm || !isDistanceHoodAtTarget())) {
      return false;
    }
    if (m_autoShooterUseFixedRpm) {
      return m_shooter.isAtSpeed()
          && Math.abs(m_shooter.getTargetRPM() - m_autoShooterTargetRpm) <= 1.0;
    }
    // Manual % fallback only checks stable actual shaft speed, not accuracy.
    return m_shooter.getControlMode().equals("PERCENT")
        && m_shooter.isReadyToFeed()
        && m_autoShooterStartTime >= 0.0
        && Timer.getFPGATimestamp() - m_autoShooterStartTime >= 1.0;
  }

  private void stopAutonomousShooter() {
    m_feeder.stopFeeder();
    m_autoFeedDeadline = -1.0;
    m_autoShooterRequested = false;
    m_autoShooterTargetRpm = 0.0;
    m_autoShooterTargetPercent = 0.0;
    m_autoShooterStartTime = -1.0;
    m_autoShooterUseFixedRpm = false;
    m_shooter.stopShooter();
    m_autoShooterStatus = "STOPPED";
  }

  private Point getHoodDistancePoint1() {
    return new Point(
        m_hoodDistanceP1Entry.getDouble(Double.NaN),
        m_hoodEncoderP1Entry.getDouble(Double.NaN));
  }

  private Point getHoodDistancePoint2() {
    return new Point(
        m_hoodDistanceP2Entry.getDouble(Double.NaN),
        m_hoodEncoderP2Entry.getDouble(Double.NaN));
  }

  private boolean isFixedRpmEnabled() {
    return m_fixedRpmEnabledEntry.getBoolean(false);
  }

  private double getValidatedFixedRpm() {
    double rpm = m_fixedRpmEntry.getDouble(Double.NaN);
    return Double.isFinite(rpm) && rpm > 0.0 && rpm <= ShooterConstants.MAX_RPM
        ? rpm : Double.NaN;
  }

  private boolean hasValidFixedRpm() {
    return Double.isFinite(getValidatedFixedRpm());
  }

  private boolean isDistanceHoodEnabled() {
    return m_distanceHoodEnabledEntry.getBoolean(false);
  }

  private boolean hasValidHoodCalibration() {
    Point near = getHoodDistancePoint1();
    Point far = getHoodDistancePoint2();
    return HoodDistanceCalculator.areValidPair(near, far)
        && near.distanceMeters() < far.distanceMeters();
  }

  /**
   * Feeder interlock: verified speed always required. In the fully
   * calibrated mode, additionally require fresh HUB distance and actual
   * hood-at-target. Manual right-stick override blocks auto feeding.
   */
  private boolean isShooterFeederReady() {
    if (!m_shooter.isReadyToFeed()) {
      return false;
    }
    if (m_distanceHoodEnabledEntry == null || !isDistanceHoodEnabled()) {
      return true;
    }
    return isFixedRpmEnabled()
        && m_shooter.getControlMode().equals("RPM")
        && hasValidFixedRpm()
        && Math.abs(m_shooter.getTargetRPM() - getValidatedFixedRpm()) <= 1.0
        && !m_autoHoodManualOverride
        && isDistanceHoodAtTarget();
  }

  private String getShotSetupStatus() {
    if (!isFixedRpmEnabled()) {
      return isDistanceHoodEnabled()
          ? "BLOCKED: enable fixed RPM before distance hood"
          : "MANUAL: B=percent shooter, right stick=hood";
    }
    if (!hasValidFixedRpm()) {
      return "BLOCKED: enter tested fixed motor shaft RPM";
    }
    if (!isDistanceHoodEnabled()) {
      return "FIXED RPM: hold B, set hood manually for shot calibration";
    }
    if (!hasValidHoodCalibration()) {
      return "BLOCKED: enter measured min/max HUB distance and hood positions";
    }
    if (!Double.isFinite(getFreshHubDistance())) {
      return "BLOCKED: fresh valid PhotonVision pose required";
    }
    if (getDistanceHoodTarget().isEmpty()) {
      return "BLOCKED: HUB distance outside tested min/max range";
    }
    if (m_autoHoodManualOverride) {
      return "MANUAL OVERRIDE: release B to reset";
    }
    return isDistanceHoodAtTarget()
        ? "HOOD READY: shot trajectory still requires validation"
        : "TRACKING: hold B in teleop or Auto Shooter Start";
  }

  /** Invalidates stale pose/unknown alliance instead of reusing old geometry. */
  private double getFreshHubDistance() {
    if (getVisionPoseAgeSeconds() < 0.0
        || getVisionPoseAgeSeconds() > HubConstants.VISION_MAX_AGE_SECONDS
        || DriverStation.getAlliance().isEmpty()
        || !Double.isFinite(m_hubFilteredDistance)) {
      return Double.NaN;
    }
    return m_hubFilteredDistance;
  }

  private OptionalDouble getDistanceHoodTarget() {
    if (!hasValidHoodCalibration()) {
      return OptionalDouble.empty();
    }
    return HoodDistanceCalculator.interpolate(
        getFreshHubDistance(), getHoodDistancePoint1(), getHoodDistancePoint2());
  }

  /** This confirms encoder position, not that FUEL will land in the HUB. */
  private boolean isDistanceHoodAtTarget() {
    OptionalDouble target = getDistanceHoodTarget();
    return target.isPresent()
        && m_hood.isAtTargetPosition()
        && Math.abs(m_hood.getTargetPosition() - target.getAsDouble())
            <= HoodConstants.AUTO_TARGET_TOLERANCE;
  }

 

  /**
   * Process each newly accepted PhotonVision field pose exactly once.
   * Camera estimates are NOT odometry (fusion remains independently optional).
   * Do not hold a stale range across tag loss or alliance changes.
   */
  private void updateHubDistance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      m_hubRawDistance = Double.NaN;
      m_hubFilteredDistance = Double.NaN;
      m_lastHubVisionTimestamp = -1.0;
      m_lastHubAlliance = Optional.empty();
      m_hubDistanceStatus = "WAIT: choose alliance on Driver Station";
      return;
    }
    if (!alliance.equals(m_lastHubAlliance)) {
      m_hubFilteredDistance = Double.NaN;
      m_lastHubVisionTimestamp = -1.0;
      m_lastHubAlliance = alliance;
    }
    if (m_lastRawVisionPose == null
        || !isVisionPoseFresh()
        || getVisionPoseAgeSeconds() > HubConstants.VISION_MAX_AGE_SECONDS) {
      m_hubRawDistance = Double.NaN;
      m_hubFilteredDistance = Double.NaN;
      m_hubDistanceStatus = "BLOCKED: fresh PhotonVision pose required";
      return;
    }
    // Cannot compute a new distance from an old frame. The validity clock
    // keeps running even if the cached filtered value is temporarily retained.
    if (m_lastHubVisionTimestamp >= m_lastVisionMeasurementTimestamp) {
      return;
    }
    OptionalDouble measured = HubDistanceCalculator.calculate(
        m_lastRawVisionPose, alliance);
    if (measured.isEmpty()) {
      m_hubRawDistance = Double.NaN;
      m_hubFilteredDistance = Double.NaN;
      m_hubDistanceStatus = "BLOCKED: field pose out of bounds";
      return;
    }
    m_hubRawDistance = measured.getAsDouble();
    m_hubFilteredDistance = Double.isFinite(m_hubFilteredDistance)
        ? HubConstants.DISTANCE_FILTER_ALPHA * m_hubRawDistance
            + (1.0 - HubConstants.DISTANCE_FILTER_ALPHA) * m_hubFilteredDistance
        : m_hubRawDistance;
    m_lastHubVisionTimestamp = m_lastVisionMeasurementTimestamp;
    m_hubDistanceStatus = "FRESH: " + m_poseSolver + " pose -> alliance HUB center";
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
    // Hold A in TELEOP to request the shooter feeder. Percent output is
    // allowed only after measured motor speed settles; otherwise it stops.
    new Trigger(() -> DriverStation.isTeleopEnabled() && m_driverController.getAButton())
        .whileTrue(Commands.runEnd(
            () -> m_feeder.runFeeder(getOutputScale(m_feederSpeedEntry, kDefaultFeederPercent)),
            m_feeder::stopFeeder,
            m_feeder));

    // B remains hold-to-spin. Default is the ORIGINAL manual percent output;
    // optional, opt-in Auto Shot mode retains calibrated RPM control.
    new Trigger(() -> DriverStation.isTeleopEnabled() && m_driverController.getBButton())
        .whileTrue(Commands.runEnd(
            () -> {
              if (isFixedRpmEnabled()) {
                // Invalid RPM safely stops the shooter (no guessed default).
                m_shooter.setTargetRPM(getValidatedFixedRpm());
              } else {
                m_shooter.setOutputPercent(getShooterOutputPercent(m_shooterSpeedEntry));
              }
            },
            m_shooter::stopShooter,
            m_shooter));

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
    // The arm motor remains DISABLED until its unique CAN ID is configured.
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
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
  }

  /** Camera frames are read once per loop; Shuffleboard suppliers publish live values. */
  public void updateDashboard() {
    updateVisionTarget();
    updateHubDistance();
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
