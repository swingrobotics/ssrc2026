package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PHASE 1 TEST: remember ONE stationary, elevated AprilTag in a LOCAL X/Y frame.
 *
 * First accepted observations save the tag's position (not its instantaneous
 * camera yaw). A heading controller then points at that location even while
 * the tag is out of view. Reobservations correct the robot's X/Y drift;
 * other tag IDs are deliberately ignored until the memory is cleared.
 *
 * This is NOT yet the official 2026 field HUB targeting system.
 */
public class HubVision {
  private static final double FRAME_MAX_AGE_S = 0.60;
  private static final double SINGLE_TAG_MAX_AMBIGUITY = 0.18;
  private static final double TAG_MIN_DISTANCE_M = 0.30;
  private static final double TAG_MAX_DISTANCE_M = 6.00;
  private static final double LOCK_MAX_ANGULAR_OUTPUT = 0.35;
  private static final double MANUAL_ROTATION_OVERRIDE = 0.12;

  // Initial world-coordinate tag observations must agree before storing target.
  private static final int INITIAL_OBSERVATIONS = 3;
  private static final double INITIAL_POSITION_TOLERANCE_M = 0.18;

  // After BUMP or suspicious odometry drift, compare VISION-ODOMETRY offsets
  // across separate frames, rather than comparing moving robot positions.
  private static final int RECOVERY_OBSERVATIONS = 3;
  private static final double RECOVERY_OFFSET_TOLERANCE_M = 0.25;
  private static final double SMALL_CORRECTION_M = 0.75;
  private static final double MAX_RECOVERY_CORRECTION_M = 2.50;

  private final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private final Transform3d robotToCamera = new Transform3d(
      new Translation3d(
          VisionConstants.CAMERA_FORWARD_M,
          VisionConstants.CAMERA_LEFT_M,
          VisionConstants.CAMERA_UP_M),
      new Rotation3d(
          Math.toRadians(VisionConstants.CAMERA_ROLL_DEG),
          Math.toRadians(VisionConstants.CAMERA_PITCH_DEG),
          Math.toRadians(VisionConstants.CAMERA_YAW_DEG)));
  private final PIDController headingPid = new PIDController(2.5, 0.0, 0.08);

  // Remembered ID and location are immutable until an intentional clear.
  private int rememberedTagId = -1;
  private Translation2d rememberedTagWorld = null;

  private int pendingTagId = -1;
  private Translation2d pendingTagWorld = null;
  private int pendingInitialFrames = 0;
  private Translation2d pendingCorrection = null;
  private int pendingRecoveryFrames = 0;

  private int lastVisibleTagId = -1;
  private int lastVisibleTagCount = 0;
  private double lastMatchingTagSeenSec = -1.0;
  private double lastAcceptedVisionSec = -1.0;
  private double latestMatchingYawDeg = 0.0;
  private String status = "WAITING_FOR_CAMERA_CALIBRATION";
  private long lastPoseFrameRevision = 0;

  public HubVision() {
    headingPid.enableContinuousInput(-Math.PI, Math.PI);
    headingPid.setTolerance(Math.toRadians(2.5));
    SmartDashboard.putBoolean("Tag Clear Memory", false);
    SmartDashboard.putBoolean("Tag Camera Mount Verified", VisionConstants.CAMERA_MOUNT_VERIFIED);
  }

  public void resetHeadingController() {
    headingPid.reset();
  }

  /** Delete the target so the next stable valid tag can be learned. */
  public void clearMemory() {
    rememberedTagId = -1;
    rememberedTagWorld = null;
    resetCandidates();
    lastMatchingTagSeenSec = -1.0;
    lastAcceptedVisionSec = -1.0;
    headingPid.reset();
    status = "MEMORY_CLEARED";
  }

  private void resetCandidates() {
    pendingTagId = -1;
    pendingTagWorld = null;
    pendingInitialFrames = 0;
    pendingCorrection = null;
    pendingRecoveryFrames = 0;
  }

  /** Called once per robotPeriodic. No other class reads PhotonCamera frames. */
  public void update(DriveSubsystem drive) {
    double now = Timer.getFPGATimestamp();
    if (SmartDashboard.getBoolean("Tag Clear Memory", false)) {
      clearMemory();
      SmartDashboard.putBoolean("Tag Clear Memory", false);
    }
    if (lastPoseFrameRevision != drive.getPoseFrameRevision()) {
      // An AutoBuilder/resetHeading pose reset changes the LOCAL frame.
      clearMemory();
      lastPoseFrameRevision = drive.getPoseFrameRevision();
      status = "POSE_FRAME_CHANGED_RELEARN";
    }

    for (var frame : camera.getAllUnreadResults()) {
      updateVisibleData(frame, now);
      if (!VisionConstants.CAMERA_MOUNT_VERIFIED) {
        status = "CAMERA_MOUNT_NOT_VERIFIED";
        continue;
      }
      if (!drive.isGyroReady() || drive.isBumpActive()) {
        continue;
      }

      double stamp = frame.getTimestampSeconds();
      if (!Double.isFinite(stamp) || now - stamp > FRAME_MAX_AGE_S
          || stamp > now + 0.10) {
        continue;
      }
      Optional<Pose2d> historicalPose = drive.getPoseAt(stamp);
      if (historicalPose.isEmpty()) {
        continue;
      }

      // Once a tag is remembered, ignore ALL other IDs, even if another
      // AprilTag appears closer or becomes PhotonVision's "best" target.
      for (PhotonTrackedTarget tag : frame.getTargets()) {
        if (rememberedTagId >= 0 && tag.getFiducialId() != rememberedTagId) {
          continue;
        }
        if (!validTag(tag)) {
          continue;
        }

        Translation2d robotToTag = measureRobotToTag(tag);
        if (robotToTag == null) {
          continue;
        }
        Pose2d sampled = historicalPose.get();
        Translation2d measuredWorldTag = sampled.getTranslation().plus(
            robotToTag.rotateBy(sampled.getRotation()));

        if (rememberedTagWorld == null) {
          learnTag(tag.getFiducialId(), measuredWorldTag, drive, now);
        } else if (tag.getFiducialId() == rememberedTagId) {
          correctDrift(drive, sampled, robotToTag, stamp, now);
        }
        // Only one tag can be learned/used per frame for this phase.
        break;
      }
    }

    if (!VisionConstants.CAMERA_MOUNT_VERIFIED) {
      status = "CAMERA_MOUNT_NOT_VERIFIED";
    } else if (!drive.isGyroReady()) {
      status = "NAVX_NOT_READY";
    } else if (drive.isBumpActive()) {
      status = "BUMP_CROSSING";
    } else if (drive.needsBumpRecovery() && rememberedTagWorld != null) {
      status = "AWAITING_TAG_RECOVERY";
    }
    publish(drive, now);
  }

  private void updateVisibleData(PhotonPipelineResult frame, double now) {
    int count = 0;
    lastVisibleTagId = -1;
    for (PhotonTrackedTarget tag : frame.getTargets()) {
      if (tag.getFiducialId() <= 0) {
        continue;
      }
      count++;
      if (lastVisibleTagId < 0) {
        lastVisibleTagId = tag.getFiducialId();
      }
      if (rememberedTagId >= 0 && tag.getFiducialId() == rememberedTagId) {
        lastMatchingTagSeenSec = now;
        latestMatchingYawDeg = tag.getYaw();
      }
    }
    lastVisibleTagCount = count;
  }

  private boolean validTag(PhotonTrackedTarget tag) {
    double ambiguity = tag.getPoseAmbiguity();
    if (tag.getFiducialId() <= 0 || !Double.isFinite(ambiguity)
        || ambiguity < 0.0 || ambiguity > SINGLE_TAG_MAX_AMBIGUITY) {
      return false;
    }
    Transform3d transform = tag.getBestCameraToTarget();
    double distance = transform.getTranslation().getNorm();
    return Double.isFinite(distance)
        && distance >= TAG_MIN_DISTANCE_M && distance <= TAG_MAX_DISTANCE_M;
  }

  /**
   * This full 3D transform includes camera height, camera rotation, and target
   * height. We deliberately discard Z ONLY AFTER transforming to ROBOT frame.
   */
  private Translation2d measureRobotToTag(PhotonTrackedTarget tag) {
    Pose3d targetRelativeToRobot = new Pose3d()
        .transformBy(robotToCamera)
        .transformBy(tag.getBestCameraToTarget());
    if (!Double.isFinite(targetRelativeToRobot.getX())
        || !Double.isFinite(targetRelativeToRobot.getY())
        || !Double.isFinite(targetRelativeToRobot.getZ())) {
      return null;
    }
    return new Translation2d(targetRelativeToRobot.getX(), targetRelativeToRobot.getY());
  }

  private void learnTag(int id, Translation2d observation,
      DriveSubsystem drive, double now) {
    if (pendingTagId != id || pendingTagWorld == null
        || pendingTagWorld.getDistance(observation) > INITIAL_POSITION_TOLERANCE_M) {
      pendingTagId = id;
      pendingTagWorld = observation;
      pendingInitialFrames = 1;
    } else {
      pendingInitialFrames++;
      // Moving average is only used before the memory is locked.
      pendingTagWorld = pendingTagWorld.interpolate(
          observation, 1.0 / pendingInitialFrames);
    }

    status = "LEARNING_TAG";
    if (pendingInitialFrames >= INITIAL_OBSERVATIONS) {
      rememberedTagId = id;
      rememberedTagWorld = pendingTagWorld;
      drive.enableLocalPoseFrame();
      resetCandidates();
      lastAcceptedVisionSec = now;
      lastMatchingTagSeenSec = now;
      status = "TAG_REMEMBERED";
    }
  }

  private void correctDrift(DriveSubsystem drive, Pose2d sampledPose,
      Translation2d robotToTag, double timestamp, double now) {
    // Measured robot field position = stored tag field position minus rotated
    // measured robot->tag vector. Gyro heading is preferred for 2D yaw.
    Translation2d measuredRobotPosition = rememberedTagWorld.minus(
        robotToTag.rotateBy(sampledPose.getRotation()));
    Translation2d correction = measuredRobotPosition.minus(sampledPose.getTranslation());
    double residual = correction.getNorm();
    if (!Double.isFinite(residual) || residual > MAX_RECOVERY_CORRECTION_M) {
      pendingCorrection = null;
      pendingRecoveryFrames = 0;
      status = "VISION_OUTLIER";
      return;
    }

    boolean needsRecovery = drive.needsBumpRecovery()
        || residual > SMALL_CORRECTION_M;
    if (needsRecovery) {
      if (pendingCorrection == null
          || pendingCorrection.getDistance(correction) > RECOVERY_OFFSET_TOLERANCE_M) {
        pendingCorrection = correction;
        pendingRecoveryFrames = 1;
      } else {
        pendingCorrection = pendingCorrection.interpolate(
            correction, 1.0 / (++pendingRecoveryFrames));
      }
      status = "VERIFYING_RECOVERY";
      if (pendingRecoveryFrames < RECOVERY_OBSERVATIONS) {
        return;
      }

      // Apply the confirmed camera->odometry offset to the CURRENT pose;
      // never reset the robot to a stale frame timestamp.
      Pose2d current = drive.getPose();
      Pose2d relocalized = new Pose2d(
          current.getTranslation().plus(pendingCorrection),
          current.getRotation());
      drive.relocalizeFromRememberedTag(relocalized);
      pendingCorrection = null;
      pendingRecoveryFrames = 0;
      lastAcceptedVisionSec = now;
      status = "TAG_RELOCALIZED";
      return;
    }

    // Normal, small correction is fused at capture time.
    drive.addVisionPose(
        new Pose2d(measuredRobotPosition, sampledPose.getRotation()),
        timestamp,
        false);
    lastAcceptedVisionSec = now;
    pendingCorrection = null;
    pendingRecoveryFrames = 0;
    status = "SINGLE_TAG_FUSED";
  }

  public boolean isTagVisible() {
    return rememberedTagId >= 0 && lastMatchingTagSeenSec >= 0.0
        && Timer.getFPGATimestamp() - lastMatchingTagSeenSec < 0.25;
  }

  public boolean canAim(DriveSubsystem drive) {
    return rememberedTagWorld != null
        && drive.isPoseInitialized()
        && drive.isGyroReady()
        && !drive.isBumpActive()
        && !drive.needsBumpRecovery();
  }

  /** Trigger rotation has priority; otherwise calculate heading to saved X/Y. */
  public double calculateRotation(DriveSubsystem drive, double manualRotation) {
    if (Math.abs(manualRotation) > MANUAL_ROTATION_OVERRIDE) {
      headingPid.reset();
      return manualRotation;
    }
    if (!canAim(drive)) {
      headingPid.reset();
      return 0.0;
    }
    Pose2d pose = drive.getPose();
    Translation2d delta = rememberedTagWorld.minus(pose.getTranslation());
    if (delta.getNorm() < 0.25) {
      headingPid.reset();
      return 0.0;
    }
    double desiredHeading = Math.atan2(delta.getY(), delta.getX());
    double omega = headingPid.calculate(pose.getRotation().getRadians(), desiredHeading);
    return MathUtil.clamp(
        omega / DriveConstants.kMaxAngularSpeed,
        -LOCK_MAX_ANGULAR_OUTPUT,
        LOCK_MAX_ANGULAR_OUTPUT);
  }

  private void publish(DriveSubsystem drive, double now) {
    SmartDashboard.putString("Tag Memory Status", status);
    SmartDashboard.putNumber("Tag Remembered ID", rememberedTagId);
    SmartDashboard.putBoolean("Tag Remembered", rememberedTagWorld != null);
    SmartDashboard.putNumber("Tag Visible Count", lastVisibleTagCount);
    SmartDashboard.putNumber("Tag Last Visible ID", lastVisibleTagId);
    SmartDashboard.putBoolean("Tag Visible", isTagVisible());
    SmartDashboard.putNumber("Tag Target Yaw", isTagVisible() ? latestMatchingYawDeg : 0.0);
    SmartDashboard.putBoolean("Tag Lock Ready", canAim(drive));
    SmartDashboard.putNumber("Tag Last Correction Age S",
        lastAcceptedVisionSec >= 0.0 ? now - lastAcceptedVisionSec : -1.0);

    // Keep legacy vision detection topics for the existing dashboard.
    SmartDashboard.putBoolean("Vision Target Visible", isTagVisible());
    SmartDashboard.putNumber("Vision Target Yaw", isTagVisible() ? latestMatchingYawDeg : 0.0);

    if (rememberedTagWorld != null) {
      var pose = drive.getPose();
      Translation2d delta = rememberedTagWorld.minus(pose.getTranslation());
      double targetAngle = Math.atan2(delta.getY(), delta.getX());
      SmartDashboard.putNumber("Tag Memory X", rememberedTagWorld.getX());
      SmartDashboard.putNumber("Tag Memory Y", rememberedTagWorld.getY());
      SmartDashboard.putNumber("Tag Distance M", delta.getNorm());
      SmartDashboard.putNumber("Tag Target Heading Deg", Math.toDegrees(targetAngle));
      SmartDashboard.putNumber("Tag Heading Error Deg",
          Math.toDegrees(MathUtil.angleModulus(
              targetAngle - pose.getRotation().getRadians())));
    } else {
      SmartDashboard.putNumber("Tag Memory X", 0.0);
      SmartDashboard.putNumber("Tag Memory Y", 0.0);
      SmartDashboard.putNumber("Tag Distance M", -1.0);
      SmartDashboard.putNumber("Tag Target Heading Deg", 0.0);
      SmartDashboard.putNumber("Tag Heading Error Deg", 0.0);
    }
  }
}
