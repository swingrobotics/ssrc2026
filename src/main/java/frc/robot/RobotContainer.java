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
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ShotCalculator.CalibrationPoint;
import frc.robot.ShotCalculator.ShotSetpoint;
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
  private final FeederSubsystem m_feeder = new FeederSubsystem(m_shooter::isReadyToFeed);
  private final HoodSubsystem m_hood = new HoodSubsystem();
  // Creates no motor controllers until both intake CAN IDs are assigned.
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // Dedicated intake ARM on an as-yet-unassigned Spark MAX CAN ID.
  private final IntakeArmSubsystem m_intakeArm = new IntakeArmSubsystem();

  private final GenericEntry m_driveSpeedEntry;
  private final GenericEntry m_shooterSpeedEntry;
  private final GenericEntry m_feederSpeedEntry;
  // Manual-distance shot tuning. Defaults are INVALID until measured values
  // are supplied, so switching on auto mode alone cannot start mechanism motion.
  private final GenericEntry m_autoShotEnabledEntry;
  private final GenericEntry m_manualShotDistanceEntry;
  private final GenericEntry m_shotP1DistanceEntry;
  private final GenericEntry m_shotP1RpmEntry;
  private final GenericEntry m_shotP1HoodEntry;
  private final GenericEntry m_shotP2DistanceEntry;
  private final GenericEntry m_shotP2RpmEntry;
  private final GenericEntry m_shotP2HoodEntry;
  private boolean m_autoHoodTargetActive = false;
  private boolean m_autoHoodManualOverride = false;
  // Autonomous commands set a request; subsystem default commands own the
  // motors continuously, so the request persists across paths and is always
  // cleared on end/cancellation. Shooter-feeder actions are separately armed.
  private final GenericEntry m_autoUseCalibratedShotEntry;
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
  private Optional<ShotSetpoint> m_autoShooterCalibratedSetpoint = Optional.empty();
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
    driveTab.addDouble("Shooter Command Output (%)", m_shooter::getOutputPercent);
    driveTab.addString("Shooter Control Mode", m_shooter::getControlMode);
    driveTab.addDouble("Shooter Actual RPM", m_shooter::getActualRPM);
    driveTab.addDouble("Shooter Error RPM", m_shooter::getErrorRPM);
    driveTab.addBoolean("Shooter RPM Ready", m_shooter::isAtSpeed);
    driveTab.addBoolean("Feeder Allowed", m_feeder::isFeedAllowed);
    driveTab.addBoolean("Feeder Command Active", m_feeder::isFeeding);
    driveTab.addBoolean("Feeder A Requested", m_driverController::getAButton);
    m_feederSpeedEntry = driveTab.add("Feeder Speed (%)", kDefaultFeederPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
    driveTab.addBoolean("Field Relative", () -> m_fieldRelativeEnabled);
    driveTab.addBoolean("Tag Auto Align", () -> m_visionAlignEnabled);

    // Calibration data starts INVALID (0 RPM / -1 hood encoder position).
    // Enter two MEASURED shots before auto control can become active.
    ShuffleboardTab shotTab = Shuffleboard.getTab("Shot Tuning");
    m_autoShotEnabledEntry = shotTab.add("Auto Shot Enabled", false).getEntry();
    m_manualShotDistanceEntry = shotTab.add("Manual Distance (m)", 2.0).getEntry();
    m_shotP1DistanceEntry = shotTab.add("Point 1 Distance (m)", 2.0).getEntry();
    m_shotP1RpmEntry = shotTab.add("Point 1 Motor RPM", 0.0).getEntry();
    m_shotP1HoodEntry = shotTab.add("Point 1 Hood Encoder", -1.0).getEntry();
    m_shotP2DistanceEntry = shotTab.add("Point 2 Distance (m)", 3.0).getEntry();
    m_shotP2RpmEntry = shotTab.add("Point 2 Motor RPM", 0.0).getEntry();
    m_shotP2HoodEntry = shotTab.add("Point 2 Hood Encoder", -1.0).getEntry();
    shotTab.addBoolean("Calibration Valid",
        () -> ShotCalculator.areValidPair(getCalibration1(), getCalibration2()));
    shotTab.addDouble("Calculated Motor RPM",
        () -> getShotSolution().map(ShotSetpoint::motorRpm).orElse(0.0));
    shotTab.addDouble("Calculated Hood Encoder",
        () -> getShotSolution().map(ShotSetpoint::hoodPosition).orElse(-1.0));
    shotTab.addBoolean("Hood Manual Override", () -> m_autoHoodManualOverride);
    shotTab.addBoolean("Hood At Target", m_hood::isAtTargetPosition);
    shotTab.addBoolean("Shot Settings Ready", this::isAutoShotReady);
    shotTab.addString("Auto Shot Status", this::getAutoShotStatus);
    // When OFF, autonomous uses the driver percent-output slider and holds
    // the hood's current position. When ON, it requires valid RPM calibration.
    m_autoUseCalibratedShotEntry =
        shotTab.add("Autonomous Use Calibration", false).getEntry();
    shotTab.addDouble("Autonomous Target RPM", () -> m_autoShooterTargetRpm);
    shotTab.addDouble("Autonomous Output (%)", () -> m_autoShooterTargetPercent);
    shotTab.addBoolean("Autonomous Shooter Running", () -> m_autoShooterRequested);
    shotTab.addBoolean("Autonomous Shot Ready", this::isAutonomousShooterReady);
    shotTab.addString("Autonomous Shooter Status", () -> m_autoShooterStatus);
    m_autoFeederEnabledEntry = shotTab.add("Autonomous Feeder Enabled", false).getEntry();
    m_autoFeedDurationEntry = shotTab.add("Autonomous Feed Duration (s)", 0.75)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.1, "max", 2.0)).getEntry();
    shotTab.addBoolean("Autonomous Feeder Active", m_feeder::isFeeding);
    shotTab.addDouble("Auto Robot Linear Speed (m/s)",
        this::getAutoRobotLinearSpeed);
    shotTab.addDouble("Auto Robot Angular Speed (deg/s)",
        () -> Math.toDegrees(m_robotDrive.getRobotRelativeSpeeds().omegaRadiansPerSecond));
    shotTab.addBoolean("Auto Robot Stopped", this::isAutonomousDriveStoppedNow);
    shotTab.addString("Last Auto Shot Result", () -> m_autoLastShotResult);
    driveTab.addBoolean("Auto Shot Enabled", this::isAutoShotEnabled);
    driveTab.addBoolean("Shot Settings Ready", this::isAutoShotReady);

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
              boolean teleopAutoRequested = DriverStation.isTeleopEnabled()
                  && isAutoShotEnabled() && m_driverController.getBButton();
              if (!m_driverController.getBButton() && !autonomous) {
                m_autoHoodManualOverride = false;
              }
              if (hoodInput != 0.0) {
                if (teleopAutoRequested) {
                  m_autoHoodManualOverride = true;
                }
                m_autoHoodTargetActive = false;
                m_hood.setManual(hoodInput * HoodConstants.TEST_MAX_SPEED);
                return;
              }
              Optional<ShotSetpoint> requestedShot = Optional.empty();
              if (autonomous && m_autoShooterRequested) {
                requestedShot = m_autoShooterCalibratedSetpoint;
              } else if (teleopAutoRequested && !m_autoHoodManualOverride) {
                requestedShot = getShotSolution();
              }
              if (requestedShot.isPresent()
                  && m_hood.setTargetPosition(requestedShot.get().hoodPosition())) {
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
        if (m_autoShooterCalibratedSetpoint.isPresent()) {
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
          // No ball-exit sensor: finishing the timer only proves the feed
          // command ended, NOT that every game piece was launched.
          m_feeder.stopFeeder();
          m_autoFeedDeadline = -1.0;
          m_autoLastShotResult = "FEED STOPPED: verify fuel cleared";
          m_autoShooterStatus = m_autoLastShotResult;
        }),
        Commands.waitSeconds(kAutoShootPostFeedPauseSeconds));
  }

  private void startAutonomousShooter() {
    stopAutonomousShooter();
    if (!DriverStation.isAutonomousEnabled()) {
      m_autoShooterStatus = "BLOCKED: not in enabled autonomous";
      return;
    }
    if (HoodConstants.CALIBRATION_MODE || !m_hood.isHoodReady()) {
      m_autoShooterStatus = "BLOCKED: hood encoder or calibration mode";
      return;
    }

    Optional<ShotSetpoint> shot = Optional.empty();
    double rpm = 0.0;
    double percent = 0.0;
    if (m_autoUseCalibratedShotEntry.getBoolean(false)) {
      shot = getShotSolution();
      if (shot.isEmpty()) {
        m_autoShooterStatus = "BLOCKED: no valid calibrated shot for distance";
        return;
      }
      rpm = shot.get().motorRpm();
      if (!Double.isFinite(rpm) || rpm <= 0.0 || rpm > ShooterConstants.MAX_RPM) {
        m_autoShooterStatus = "BLOCKED: invalid calibrated RPM";
        return;
      }
    } else {
      percent = getShooterOutputPercent(m_shooterSpeedEntry);
      if (!Double.isFinite(percent) || percent <= 0.0) {
        m_autoShooterStatus = "BLOCKED: invalid shooter output percent";
        return;
      }
    }

    m_autoShooterCalibratedSetpoint = shot;
    m_autoShooterTargetRpm = rpm;
    m_autoShooterTargetPercent = percent;
    m_autoShooterStartTime = Timer.getFPGATimestamp();
    m_autoShooterRequested = true;
    m_autoShooterStatus = shot.isPresent()
        ? "SPINNING: calibrated RPM and hood target"
        : "SPINNING: Drive output percent, hood holds current position";
  }

  private boolean isAutonomousShooterReady() {
    if (!DriverStation.isAutonomousEnabled()
        || !m_autoShooterRequested
        || !m_hood.isHoodReady()) {
      return false;
    }
    // Uncalibrated percentage mode is ONLY a spin-up check. It cannot
    // certify accuracy or a specific RPM. Require a minimum elapsed time
    // and a measured rotating shaft rather than claiming RPM readiness.
    if (m_autoShooterCalibratedSetpoint.isEmpty()) {
      return m_shooter.getControlMode().equals("PERCENT")
          && m_shooter.isReadyToFeed()
          && m_autoShooterStartTime >= 0.0
          && Timer.getFPGATimestamp() - m_autoShooterStartTime >= 1.0;
    }
    ShotSetpoint requested = m_autoShooterCalibratedSetpoint.get();
    return m_shooter.isAtSpeed()
        && Math.abs(m_shooter.getTargetRPM() - m_autoShooterTargetRpm) <= 1.0
        && m_hood.isAtTargetPosition()
        && Math.abs(m_hood.getTargetPosition() - requested.hoodPosition())
            <= HoodConstants.AUTO_TARGET_TOLERANCE;
  }

  private void stopAutonomousShooter() {
    m_feeder.stopFeeder();
    m_autoFeedDeadline = -1.0;
    m_autoShooterRequested = false;
    m_autoShooterTargetRpm = 0.0;
    m_autoShooterTargetPercent = 0.0;
    m_autoShooterStartTime = -1.0;
    m_autoShooterCalibratedSetpoint = Optional.empty();
    m_shooter.stopShooter();
    m_autoShooterStatus = "STOPPED";
  }

  /** Calibration points use observed MOTOR shaft RPM and raw hood encoder readings. */
  private CalibrationPoint getCalibration1() {
    return new CalibrationPoint(
        m_shotP1DistanceEntry.getDouble(Double.NaN),
        m_shotP1RpmEntry.getDouble(Double.NaN),
        m_shotP1HoodEntry.getDouble(Double.NaN));
  }

  private CalibrationPoint getCalibration2() {
    return new CalibrationPoint(
        m_shotP2DistanceEntry.getDouble(Double.NaN),
        m_shotP2RpmEntry.getDouble(Double.NaN),
        m_shotP2HoodEntry.getDouble(Double.NaN));
  }

  private boolean isAutoShotEnabled() {
    return m_autoShotEnabledEntry.getBoolean(false);
  }

  /** Only calibrated distances, never guessed or extrapolated setpoints. */
  private Optional<ShotSetpoint> getShotSolution() {
    return ShotCalculator.interpolate(
        m_manualShotDistanceEntry.getDouble(Double.NaN),
        getCalibration1(), getCalibration2());
  }

  private boolean isAutoShotReady() {
    if (!isAutoShotEnabled()
        || !m_driverController.getBButton()
        || m_autoHoodManualOverride
        || !m_autoHoodTargetActive) {
      return false;
    }
    Optional<ShotSetpoint> shot = getShotSolution();
    if (shot.isEmpty()) {
      return false;
    }
    return m_shooter.isAtSpeed()
        && m_hood.isAtTargetPosition()
        && Math.abs(m_shooter.getTargetRPM() - shot.get().motorRpm()) < 1.0
        && Math.abs(m_hood.getTargetPosition() - shot.get().hoodPosition())
            <= HoodConstants.AUTO_TARGET_TOLERANCE;
  }

  private String getAutoShotStatus() {
    if (!isAutoShotEnabled()) {
      return "OFF: manual RPM and hood control";
    }
    if (!ShotCalculator.areValidPair(getCalibration1(), getCalibration2())) {
      return "BLOCKED: enter 2 valid measured calibration points";
    }
    if (getShotSolution().isEmpty()) {
      return "BLOCKED: distance outside calibrated range";
    }
    if (m_autoHoodManualOverride) {
      return "MANUAL HOOD OVERRIDE: release B to reset";
    }
    if (!m_driverController.getBButton()) {
      return "ARMED: hold B to move hood and spin shooter";
    }
    if (HoodConstants.CALIBRATION_MODE) {
      return "BLOCKED: hood calibration mode";
    }
    if (!m_hood.isHoodReady()) {
      return "BLOCKED: hood encoder not ready";
    }
    if (isAutoShotReady()) {
      return "READY: RPM and hood at calculated targets";
    }
    return "WAIT: adjusting RPM / hood position";
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
              if (isAutoShotEnabled()) {
                // Invalid calibration or hood sensor blocks closed-loop RPM.
                double rpm = (!HoodConstants.CALIBRATION_MODE && m_hood.isHoodReady())
                    ? getShotSolution().map(ShotSetpoint::motorRpm).orElse(0.0)
                    : 0.0;
                m_shooter.setTargetRPM(rpm);
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
