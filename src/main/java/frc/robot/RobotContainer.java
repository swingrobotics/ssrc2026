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

  // Keep drivetrain, shooter/feeder, hood and intake mechanisms independent.
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  // The SHOOTER feeder (CAN 11/14) may feed only after measured flywheel
  // stability of real rotation before accepting feeder commands.
  private final FeederSubsystem m_feeder = new FeederSubsystem(m_shooter::isReadyToFeed);
  private final HoodSubsystem m_hood = new HoodSubsystem();
  // Intake roller controllers stay disabled until both roller CAN IDs are assigned.
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  // Intake arm is currently configured as CAN 15; verify hardware and limits.
  private final IntakeArmSubsystem m_intakeArm = new IntakeArmSubsystem();

  private final GenericEntry m_driveSpeedEntry;
  private final GenericEntry m_shooterSpeedEntry;
  private final GenericEntry m_feederSpeedEntry;
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
  private double m_autoShooterTargetPercent = 0.0;
  private double m_autoShooterStartTime = -1.0;
  private String m_autoShooterStatus = "Idle";
  private final GenericEntry m_visionFusionEntry;
  private final GenericEntry m_visionResetEntry;
  private final GenericEntry m_fieldVerifiedEntry;
  private final GenericEntry m_hubAimTrimEntry;
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
  private final HubCenterAlign m_hubCenterAlign = new HubCenterAlign();

  // Multitag HUB center aiming uses robot field pose, not individual tag yaw.

  // Driving mode. False keeps robot-relative translation by default.
  private boolean m_fieldRelativeEnabled = false;

  // Vision auto-align state.
  private boolean m_visionAlignEnabled = false;
  private double m_lastVisionPoseReceivedAt = -1.0;
  private double m_lastVisionMeasurementTimestamp = -1.0;
  private Pose2d m_lastRawVisionPose = null;
  private Pose2d m_latestGoodSeedPose = null;
  private double m_latestGoodSeedTimestamp = -1.0;
  private double m_lastVisionInnovationMeters = -1.0;
  private int m_lastVisionUsedTagCount = 0;
  private int m_acceptedVisionCount = 0;
  private int m_rejectedVisionCount = 0;
  private String m_visionGateStatus = "WAITING FOR CAMERA";

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
    // All operator speed settings use motor duty-cycle PERCENT only.
    driveTab.addBoolean("Feeder Active", m_feeder::isFeeding);
    m_feederSpeedEntry = driveTab.add("Feeder Speed (%)", kDefaultFeederPercent)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
    driveTab.addBoolean("Field Relative", () -> m_fieldRelativeEnabled);
    driveTab.addBoolean("Hub Center Align", () -> m_visionAlignEnabled);

    // No RPM setpoints or unmeasured hood calibration parameters.
    ShuffleboardTab shotTab = Shuffleboard.getTab("Shot Tuning");
    m_autoFeederEnabledEntry = shotTab.add("Autonomous Feeder Enabled", false).getEntry();
    m_autoFeedDurationEntry = shotTab.add("Autonomous Feed Duration (s)", 0.75)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0.1, "max", 2.0)).getEntry();
    shotTab.addBoolean("Autonomous Shot Ready", this::isAutonomousShooterReady);
    shotTab.addString("Autonomous Shooter Status", () -> m_autoShooterStatus);
    shotTab.addString("Last Auto Shot Result", () -> m_autoLastShotResult);
    shotTab.addBoolean("Auto Robot Stopped", this::isAutonomousDriveStoppedNow);

    // FRC 5024: validate distance/count and supply range-aware covariance.
    // FRC 6328: maintain one sensor-fused pose, with camera capture times.
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    m_fieldVerifiedEntry = visionTab.add("Field Camera Verified", false).getEntry();
    m_visionFusionEntry = visionTab.add("Fusion Enabled", true).getEntry();
    m_visionResetEntry = visionTab.add("Reset Pose (Disabled)", false).getEntry();
    visionTab.addBoolean("Pose Initialized", m_robotDrive::isFieldPoseInitialized);
    visionTab.addBoolean("NavX Ready", m_robotDrive::isGyroReady);
    visionTab.addBoolean("Fused Vision Fresh",
        () -> m_robotDrive.hasRecentVision(1.0));
    visionTab.addDouble("Fused Pose X (m)", () -> m_robotDrive.getPose().getX());
    visionTab.addDouble("Fused Pose Y (m)", () -> m_robotDrive.getPose().getY());
    visionTab.addDouble("Fused Heading (deg)",
        () -> m_robotDrive.getPose().getRotation().getDegrees());
    visionTab.addString("Pose Init Status", () -> m_visionResetStatus);
    visionTab.addString("Vision Gate Status", () -> m_visionGateStatus);
    visionTab.addDouble("Vision Accepted Frames", () -> m_acceptedVisionCount);
    visionTab.addDouble("Vision Rejected Frames", () -> m_rejectedVisionCount);
    visionTab.addDouble("Visible Tags Used", () -> m_lastVisionUsedTagCount);
    visionTab.addDouble("Vision/Fused Delta XY (m)",
        () -> m_lastVisionInnovationMeters);
    visionTab.addDouble("Tag ID", () -> m_lastTagId);
    visionTab.addBoolean("Raw Pose Fresh", this::isVisionPoseFresh);
    visionTab.addDouble("Raw Pose X (m)",
        () -> m_lastRawVisionPose == null ? -1.0 : m_lastRawVisionPose.getX());
    visionTab.addDouble("Raw Pose Y (m)",
        () -> m_lastRawVisionPose == null ? -1.0 : m_lastRawVisionPose.getY());
    visionTab.addDouble("Raw Heading (deg)",
        () -> m_lastRawVisionPose == null ? -999.0
            : m_lastRawVisionPose.getRotation().getDegrees());
    visionTab.addString("Pose Solver", () -> m_poseSolver);
    visionTab.addString("Alliance",
        () -> DriverStation.getAlliance().map(Enum::name).orElse("UNKNOWN"));
    visionTab.addString("HUB Align Status", m_hubCenterAlign::getStatus);
    visionTab.addDouble("HUB Error (deg)",
        m_hubCenterAlign::getHeadingErrorDegrees);
    visionTab.addBoolean("HUB Aligned",
        () -> m_visionAlignEnabled && m_hubCenterAlign.isAligned());
    m_hubAimTrimEntry = visionTab.add("Hub Aim Trim (deg)", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", -15.0, "max", 15.0)).getEntry();
    visionTab.addDouble("Hub Trim Applied (deg)", m_hubCenterAlign::getAimTrimDegrees);
    visionTab.addDouble("Hub Untrimmed Error (deg)",
        m_hubCenterAlign::getUntrimmedHeadingErrorDegrees);
    visionTab.addDouble("Hub Estimated Heading (deg)",
        m_hubCenterAlign::getEstimatedHeadingDegrees);
    visionTab.addDouble("Hub Desired Heading (deg)",
        m_hubCenterAlign::getDesiredHeadingDegrees);
    visionTab.addDouble("Hub Bearing (deg)", m_hubCenterAlign::getHubBearingDegrees);
    visionTab.addDouble("Hub Distance (m)", m_hubCenterAlign::getHubDistanceMeters);

    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    debugTab.addString("Vision Field Layout", () -> "2026 REBUILT Welded");
    debugTab.addString("Vision Camera", () -> VisionConstants.CAMERA_NAME);
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
        m_shooter.setOutputPercent(m_autoShooterTargetPercent);
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
    double percent = getShooterOutputPercent(m_shooterSpeedEntry);
    if (percent <= 0.0) {
      m_autoShooterStatus = "BLOCKED: invalid shooter percent";
      return;
    }
    m_autoShooterTargetPercent = percent;
    m_autoShooterStartTime = Timer.getFPGATimestamp();
    m_autoShooterRequested = true;
    m_autoShooterStatus = "SPINNING: percent output";
  }

  private boolean isAutonomousShooterReady() {
    return DriverStation.isAutonomousEnabled()
        && m_autoShooterRequested && m_hood.isHoodReady()
        && m_shooter.isReadyToFeed()
        && m_autoShooterStartTime >= 0.0
        && Timer.getFPGATimestamp() - m_autoShooterStartTime >= 1.0;
  }

  private void stopAutonomousShooter() {
    m_feeder.stopFeeder();
    m_autoFeedDeadline = -1.0;
    m_autoShooterRequested = false;
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
   * PhotonVision is a MEASUREMENT SOURCE, not the heading controller.
   * Every accepted pose enters WPILib's timestamped SwerveDrivePoseEstimator
   * (FRC 5024 / 6328 pattern). All visible field tags can contribute.
   * Initial field coordinates require a verified disabled multitag seed.
   */
  private void updateVisionTarget() {
    double now = Timer.getFPGATimestamp();
    for (var result : m_photonCamera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;

      var bestTarget = result.getBestTarget();
      if (bestTarget != null
          && VisionConstants.FIELD_LAYOUT.getTagPose(
              bestTarget.getFiducialId()).isPresent()) {
        m_lastTagId = bestTarget.getFiducialId();
      }

      Optional<EstimatedRobotPose> estimate =
          m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
      boolean multiTag = estimate.isPresent();

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
        m_rejectedVisionCount++;
        m_visionGateStatus = "NO VALID PNP RESULT";
        continue;
      }

      var measurement = estimate.get();
      double stamp = measurement.timestampSeconds;
      if (stamp <= m_lastVisionMeasurementTimestamp) continue;

      int used = measurement.targetsUsed == null
          ? 0 : (int) measurement.targetsUsed.stream()
              .mapToInt(t -> t.getFiducialId()).distinct().count();
      double avgDistance = measurement.targetsUsed == null ? Double.NaN
          : measurement.targetsUsed.stream()
              .mapToDouble(t -> t.getBestCameraToTarget()
                  .getTranslation().getNorm())
              .filter(Double::isFinite).average().orElse(Double.NaN);
      double ambiguity = bestTarget != null
          ? bestTarget.getPoseAmbiguity() : Double.NaN;
      Pose3d rawPose3d = measurement.estimatedPose;
      Pose2d rawPose = rawPose3d.toPose2d();

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
        m_visionResetStatus = "INITIALIZED FROM VERIFIED MULTITAG POSE";
        // Wait for the next validated frame before allowing auto-aim.
      }
      m_visionResetEntry.setBoolean(false);
    }
  }



  /** FRC 135-style point-facing profiled rotation from fused field pose. */
  private double getDriveRotationCommand() {
    m_hubCenterAlign.setAimTrimDegrees(m_hubAimTrimEntry.getDouble(0.0));
    if (!m_visionAlignEnabled) return getTriggerRotation();
    return m_hubCenterAlign.calculateTurn(
        m_robotDrive.getPose(),
        m_robotDrive.getRobotRelativeSpeeds(),
        DriverStation.getAlliance(),
        m_robotDrive.isFieldPoseInitialized()
            && m_fieldVerifiedEntry.getBoolean(false),
        m_robotDrive.isGyroReady(),
        m_robotDrive.hasRecentVision(1.0));
  }

  /** Toggle between robot-relative and field-relative translation control. */
  private void toggleFieldRelative() {
    m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
  }

  /** Toggle PhotonVision AprilTag auto-alignment on/off. */
  private void toggleVisionAlign() {
    m_visionAlignEnabled = !m_visionAlignEnabled;
    if (m_visionAlignEnabled) m_hubCenterAlign.start();
    else m_hubCenterAlign.stop();
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

    // Hold B for percent-only shooter output.
    new Trigger(() -> DriverStation.isTeleopEnabled() && m_driverController.getBButton())
        .whileTrue(Commands.runEnd(
            () -> {
              m_shooter.setOutputPercent(
                  getShooterOutputPercent(m_shooterSpeedEntry));
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
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
  }

  /** Camera frames are read once per loop; Shuffleboard suppliers publish live values. */
  public void updateDashboard() {
    // Keep trim preview and its settling reset current even while disabled.
    m_hubCenterAlign.setAimTrimDegrees(m_hubAimTrimEntry.getDouble(0.0));
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
