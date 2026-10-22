 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 package frc.robot;

 import com.pathplanner.lib.auto.AutoBuilder;
 import com.pathplanner.lib.auto.NamedCommands;
 import edu.wpi.first.apriltag.AprilTagFieldLayout;
 import edu.wpi.first.math.MathUtil;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Pose3d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.kinematics.ChassisSpeeds;
 import edu.wpi.first.networktables.GenericEntry;
 import edu.wpi.first.wpilibj.DriverStation;
 import edu.wpi.first.wpilibj.DriverStation.Alliance;
 import edu.wpi.first.wpilibj.Timer;
 import edu.wpi.first.wpilibj.XboxController;
 import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
 import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
 import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
 import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.Commands;
 import edu.wpi.first.wpilibj2.command.InstantCommand;
 import edu.wpi.first.wpilibj2.command.RunCommand;
 import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
 import frc.robot.Constants.DriveConstants;
 import frc.robot.Constants.FeederConstants;
 import frc.robot.Constants.HoodConstants;
 import frc.robot.Constants.HubConstants;
 import frc.robot.Constants.OIConstants;
 import frc.robot.Constants.ShooterConstants;
 import frc.robot.Constants.VisionConstants;
 import frc.robot.subsystems.DriveSubsystem;
 import frc.robot.subsystems.FeederSubsystem;
 import frc.robot.subsystems.HoodSubsystem;
 import frc.robot.subsystems.IntakeArmSubsystem;
 import frc.robot.subsystems.IntakeSubsystem;
 import frc.robot.subsystems.ShooterSubsystem;
 import java.util.Map;
 import java.util.Optional;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;

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
   // Robot subsystems and original controller mappings.
   private final DriveSubsystem m_robotDrive = new DriveSubsystem();
   private final ShooterSubsystem m_shooter = new ShooterSubsystem();
   private final FeederSubsystem m_feeder = new FeederSubsystem(m_shooter::isReadyToFeed);
   private final HoodSubsystem m_hood = new HoodSubsystem();
   private final IntakeSubsystem m_intake = new IntakeSubsystem();
   private final IntakeArmSubsystem m_intakeArm = new IntakeArmSubsystem();

   private final GenericEntry m_driveSpeedEntry;
   private final GenericEntry m_shooterSpeedEntry;
   private final GenericEntry m_feederSpeedEntry;
   private final GenericEntry m_autoFeederEnabledEntry;
   private final GenericEntry m_autoFeedDurationEntry;

   // Autonomous still uses percent-output shooting, not RPM control.
   private double m_autoFeedDeadline = -1.0;
   private double m_autoStopStableSince = -1.0;
   private String m_autoLastShotResult = "NOT RUN";
   private boolean m_autoShooterRequested = false;
   private double m_autoShooterTargetPercent = 0.0;
   private double m_autoShooterStartTime = -1.0;
   private String m_autoShooterStatus = "Idle";

   private final GenericEntry m_visionFusionEntry;
   private final GenericEntry m_visionResetEntry;
   private double m_lastTagId = -1.0;
   private String m_poseSolver = "None";
   private String m_visionResetStatus = "Not requested";

   // Hub centered multi-AprilTag aim (blue-origin 2026 Welded field frame).
   // Hub centers come from Constants.HubConstants; verify on the real field.
   // Launcher pointing direction is assumed to be robot +X (forward).
   // If launcher is reversed, change this to Math.PI after a mechanical check.
   private static final double LAUNCHER_HEADING_OFFSET_RAD = 0.0;
   private static final double HUB_VISION_MAX_AGE_S = 0.30;
   private static final double HUB_ALIGN_KP = 1.8; // rad/s per rad error
   private static final double HUB_ALIGN_KD = 0.18; // rad/s per rad/s turn
   private static final double HUB_ALIGN_MAX_NORMALIZED = 0.25;
   private static final double HUB_ALIGN_TOLERANCE_RAD = Math.toRadians(1.5);
   private static final double HUB_ALIGN_MAX_RATE_RAD_S = 0.12;
   private static final double HUB_ALIGN_SETTLE_S = 0.20;

   private boolean m_visionAlignEnabled = false;
   private boolean m_fieldRelativeEnabled = false;
   private Pose2d m_hubVisionPose = null;
   private Alliance m_hubVisionAlliance = null;
   private double m_hubVisionTimestamp = -1.0;
   private double m_gyroAtVisionRad = Double.NaN;
   private int m_hubVisibleTagCount = 0;
   private double m_hubHeadingErrorDeg = Double.NaN;
   private double m_hubAlignedSince = -1.0;
   private String m_hubAlignStatus = "OFF";

   // General vision pose telemetry and manual-disabled odometry reset.
   private double m_lastVisionPoseReceivedAt = -1.0;
   private double m_lastVisionMeasurementTimestamp = -1.0;
   private Pose2d m_lastRawVisionPose = null;
   // The driver's controller
   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
   // PathPlanner autonomous chooser.
   private final SendableChooser<Command> m_autoChooser;
   // PhotonVision camera nickname. Must exactly match the nickname in the PhotonVision UI.
   private final PhotonCamera m_photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
   private final PhotonPoseEstimator m_photonPoseEstimator =
       new PhotonPoseEstimator(VisionConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_CAMERA);

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
     driveTab.addBoolean("Feeder Active", m_feeder::isFeeding);
     m_feederSpeedEntry = driveTab.add("Feeder Speed (%)", kDefaultFeederPercent)
         .withWidget(BuiltInWidgets.kNumberSlider)
         .withProperties(Map.of("min", 0.0, "max", 100.0)).getEntry();
     driveTab.addBoolean("Field Relative", () -> m_fieldRelativeEnabled);
     driveTab.addBoolean("Tag Auto Align", () -> m_visionAlignEnabled);
     // Keep shooter output controlled solely in percent mode; hood is manual
     // until team finishes shooting-angle calibration in a later task.
     ShuffleboardTab shotTab = Shuffleboard.getTab("Shot Tuning");
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
     visionTab.addBoolean("Tag Visible", this::isVisionPoseFresh);
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
     visionTab.addBoolean("Hub Center Align ON", () -> m_visionAlignEnabled);
     visionTab.addDouble("Hub Tags Visible (fresh)", this::getFreshHubTagCount);
     visionTab.addDouble("Hub Center Heading Error (deg)",
         () -> Double.isFinite(m_hubHeadingErrorDeg) ? m_hubHeadingErrorDeg : -999.0);
     visionTab.addBoolean("Hub Center Aligned", this::isHubCenterAligned);
     visionTab.addString("Hub Center Align Status", () -> m_hubAlignStatus);
     visionTab.addString("Alliance", () -> DriverStation.getAlliance()
         .map(Enum::name).orElse("UNKNOWN"));
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
     // Manual hood control remains unchanged, with position hold on release.
     // Do not read joystick while autonomous is enabled.
     m_hood.setDefaultCommand(new RunCommand(() -> {
       double input = DriverStation.isTeleopEnabled()
           ? MathUtil.applyDeadband(-m_driverController.getRightY(), OIConstants.kHoodDeadband)
           : 0.0;
       m_hood.setManual(input * HoodConstants.TEST_MAX_SPEED);
     }, m_hood));
     // Keeps motor ownership in one command (and avoids path-marker commands
     // with drive subsystem requirements interfering with path following).
     // PathPlanner named commands request shooter output in percent only.
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
       m_autoShooterStatus = "BLOCKED: autonomous disabled";
       return;
     }
     if (HoodConstants.CALIBRATION_MODE || !m_hood.isHoodReady()) {
       m_autoShooterStatus = "BLOCKED: hood encoder not ready";
       return;
     }
     double percent = getShooterOutputPercent(m_shooterSpeedEntry);
     if (!Double.isFinite(percent) || percent <= 0.0) {
       m_autoShooterStatus = "BLOCKED: invalid shooter output percent";
       return;
     }
     m_autoShooterTargetPercent = percent;
     m_autoShooterStartTime = Timer.getFPGATimestamp();
     m_autoShooterRequested = true;
     m_autoShooterStatus = "SPINNING: shooter percent output";
   }

   private boolean isAutonomousShooterReady() {
     if (!DriverStation.isAutonomousEnabled()
         || !m_autoShooterRequested
         || !m_hood.isHoodReady()) {
       return false;
     }
     // This checks spin-up/rotation stability, not guaranteed shot accuracy.
     return m_shooter.isReadyToFeed()
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

   /** A real HUB tag (not a tower, outpost or trench tag). */
   private static boolean isHubTag(int id, Alliance alliance) {
     if (alliance == Alliance.Blue) {
       return id == 18 || id == 19 || id == 20 || id == 21
           || id == 24 || id == 25 || id == 26 || id == 27;
     }
     return id == 2 || id == 3 || id == 4 || id == 5
         || id == 8 || id == 9 || id == 10 || id == 11;
   }

   /**
    * Read each PhotonVision frame once. Multi-tag hub aiming REQUIRES a
    * vetted multi-tag pose AND at least TWO visible alliance HUB tags.
    * Single-tag poses remain available solely for debugging/optional fusion.
    */
   private void updateVisionTarget() {
     for (var result : m_photonCamera.getAllUnreadResults()) {
       if (!result.hasTargets()) {
         continue;
       }
       var bestTarget = result.getBestTarget();
       if (bestTarget != null
           && VisionConstants.FIELD_LAYOUT.getTagPose(bestTarget.getFiducialId()).isPresent()) {
         m_lastTagId = bestTarget.getFiducialId();
       }

       Optional<EstimatedRobotPose> estimate =
           m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
       boolean multiTag = estimate.isPresent();

       if (!multiTag && bestTarget != null
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
       double tolerance = VisionConstants.FIELD_BOUNDS_TOLERANCE_M;
       AprilTagFieldLayout layout = VisionConstants.FIELD_LAYOUT;

       // Reject stale, out-of-order, impossible, or tilted field poses.
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

       // Align only if at least two of the tags used in the multi-tag
       // pose solution are distinct tags on our alliance HUB.
       Optional<Alliance> alliance = DriverStation.getAlliance();
       if (multiTag && alliance.isPresent()
           && Timer.getFPGATimestamp() - timestamp <= HUB_VISION_MAX_AGE_S) {
         Alliance color = alliance.get();
         long count = measurement.targetsUsed.stream()
             .mapToInt(target -> target.getFiducialId())
             .distinct()
             .filter(id -> isHubTag(id, color))
             .count();
         if (count >= 2) {
           m_hubVisionPose = measuredPose;
           m_hubVisionTimestamp = timestamp;
           m_hubVisionAlliance = color;
           m_gyroAtVisionRad = Math.toRadians(m_robotDrive.getHeading());
           m_hubVisibleTagCount = (int) count;
         }
       }

       // Position fusion is separate and OFF by default.
       if (m_visionFusionEntry.getBoolean(false)) {
         m_robotDrive.addVisionMeasurement(
            measuredPose, timestamp,
            multiTag ? 0.5 : 1.5, // position uncertainty (m): conservative starting estimates
            1000.0); // large heading uncertainty (rad): trust navX for yaw
       }
     }

     // A disabled-only camera pose reset is still supported.
     if (m_visionResetEntry.getBoolean(false)) {
       if (DriverStation.isDisabled() && isVisionPoseFresh()
           && m_lastRawVisionPose != null) {
         m_robotDrive.resetOdometry(m_lastRawVisionPose);
         m_visionResetStatus = "Applied while Disabled";
       } else {
         m_visionResetStatus = "Rejected: disable and show a valid tag";
       }
       m_visionResetEntry.setBoolean(false);
     }
   }

   private boolean hasFreshHubPose() {
     double age = Timer.getFPGATimestamp() - m_hubVisionTimestamp;
     Optional<Alliance> alliance = DriverStation.getAlliance();
     return m_hubVisionPose != null && alliance.isPresent()
         && alliance.get() == m_hubVisionAlliance
         && m_hubVisibleTagCount >= 2
         && age >= 0.0 && age <= HUB_VISION_MAX_AGE_S;
   }

   private int getFreshHubTagCount() {
     return hasFreshHubPose() ? m_hubVisibleTagCount : 0;
   }

   private void resetHubAlign() {
     m_hubVisionPose = null;
     m_hubVisionTimestamp = -1.0;
     m_hubVisionAlliance = null;
     m_hubVisibleTagCount = 0;
     m_gyroAtVisionRad = Double.NaN;
     m_hubHeadingErrorDeg = Double.NaN;
     m_hubAlignedSince = -1.0;
     m_hubAlignStatus = m_visionAlignEnabled ? "WAITING FOR MULTI-HUB TAGS" : "OFF";
   }

   private boolean isHubCenterAligned() {
     return m_visionAlignEnabled && hasFreshHubPose()
         && m_hubAlignedSince >= 0.0
         && Timer.getFPGATimestamp() - m_hubAlignedSince >= HUB_ALIGN_SETTLE_S;
   }

   /** Return normalized CCW-positive rotation, DriveSubsystem scales it. */
   private double getDriveRotationCommand() {
     if (!m_visionAlignEnabled) {
       return getTriggerRotation();
     }
     if (!hasFreshHubPose()) {
       m_hubHeadingErrorDeg = Double.NaN;
       m_hubAlignedSince = -1.0;
       m_hubAlignStatus = "NO FRESH 2+ HUB TAG MULTI-POSE";
       return 0.0;
     }

     double gyroNow = Math.toRadians(m_robotDrive.getHeading());
     double turnRate = m_robotDrive.getRobotRelativeSpeeds().omegaRadiansPerSecond;
     if (!Double.isFinite(gyroNow) || !Double.isFinite(turnRate)
         || !Double.isFinite(m_gyroAtVisionRad)) {
       m_hubAlignedSince = -1.0;
       m_hubAlignStatus = "GYRO / WHEEL SPEED INVALID";
       return 0.0;
     }

     // Project the camera-estimated heading forward with the gyro delta.
     double gyroChange = MathUtil.angleModulus(gyroNow - m_gyroAtVisionRad);
     double heading = MathUtil.angleModulus(
         m_hubVisionPose.getRotation().getRadians() + gyroChange);
     Translation2d hubCenter = m_hubVisionAlliance == Alliance.Blue
         ? HubConstants.BLUE_CENTER_METERS : HubConstants.RED_CENTER_METERS;
     // Point the launcher (not necessarily robot center) toward HUB center.
     Translation2d launcherPosition = m_hubVisionPose.getTranslation().plus(
         HubConstants.SHOOTER_OFFSET_ROBOT_METERS.rotateBy(new Rotation2d(heading)));
     double dx = hubCenter.getX() - launcherPosition.getX();
     double dy = hubCenter.getY() - launcherPosition.getY();
     if (Math.hypot(dx, dy) < 0.2) {
       m_hubAlignedSince = -1.0;
       m_hubAlignStatus = "ROBOT TOO CLOSE TO HUB CENTER";
       return 0.0;
     }

     // Robot +X shooting direction assumed: change offset if reversed.
     double targetHeading = Math.atan2(dy, dx) - LAUNCHER_HEADING_OFFSET_RAD;
     double errorRad = MathUtil.angleModulus(targetHeading - heading);
     m_hubHeadingErrorDeg = Math.toDegrees(errorRad);

     if (Math.abs(errorRad) <= HUB_ALIGN_TOLERANCE_RAD
         && Math.abs(turnRate) <= HUB_ALIGN_MAX_RATE_RAD_S) {
       if (m_hubAlignedSince < 0.0) {
         m_hubAlignedSince = Timer.getFPGATimestamp();
       }
       m_hubAlignStatus = isHubCenterAligned() ? "ALIGNED" : "SETTLING";
       return 0.0;
     }

     m_hubAlignedSince = -1.0;
     m_hubAlignStatus = "TURNING TO HUB CENTER";
     double turnRadiansPerSecond = HUB_ALIGN_KP * errorRad - HUB_ALIGN_KD * turnRate;
     double normalized = turnRadiansPerSecond / DriveConstants.kMaxAngularSpeed;
     return MathUtil.clamp(normalized,
         -HUB_ALIGN_MAX_NORMALIZED, HUB_ALIGN_MAX_NORMALIZED);
   }

   /** Toggle between robot-relative and field-relative translation control. */
   private void toggleFieldRelative() {
     m_fieldRelativeEnabled = !m_fieldRelativeEnabled;
   }

   /** Toggle PhotonVision AprilTag auto-alignment on/off. */
   private void toggleVisionAlign() {
     m_visionAlignEnabled = !m_visionAlignEnabled;
     resetHubAlign();
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
     // B held: shooter is controlled ONLY by duty-cycle percent output.
     new Trigger(() -> DriverStation.isTeleopEnabled() && m_driverController.getBButton())
         .whileTrue(Commands.runEnd(
             () -> m_shooter.setOutputPercent(
                 getShooterOutputPercent(m_shooterSpeedEntry)),
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