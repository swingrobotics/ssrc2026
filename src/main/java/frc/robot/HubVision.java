package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * HUB LOCK: fixed field-coordinate HUB aiming plus PhotonVision field localization.
 *
 * The camera is consumed ONCE per robot loop. The 2026 multi-tag solver runs on
 * PhotonVision; single-tag pose estimation is a filtered fallback. Neither
 * raw yaw nor tag switching changes which HUB is targeted.
 */
public class HubVision {
  private static final double MAX_FRAME_AGE_SEC = 0.75;
  private static final double MAX_INITIAL_Z_M = 0.35;
  private static final double MAX_POSE_JUMP_M = 1.20;
  private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.15;
  private static final double CANDIDATE_MATCH_M = 0.45;
  private static final double CANDIDATE_MATCH_RAD = Math.toRadians(15.0);
  private static final double MANUAL_ROTATION_OVERRIDE = 0.12;

  private final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private final AprilTagFieldLayout field =
      AprilTagFieldLayout.loadField(VisionConstants.FIELD);
  private final PhotonPoseEstimator estimator = new PhotonPoseEstimator(
      field,
      new Transform3d(
          new Translation3d(
              VisionConstants.CAMERA_FORWARD_M,
              VisionConstants.CAMERA_LEFT_M,
              VisionConstants.CAMERA_UP_M),
          new Rotation3d(
              Math.toRadians(VisionConstants.CAMERA_ROLL_DEG),
              Math.toRadians(VisionConstants.CAMERA_PITCH_DEG),
              Math.toRadians(VisionConstants.CAMERA_YAW_DEG))));
  private final PIDController headingPid = new PIDController(2.5, 0.0, 0.08);

  private double latestYawDeg = 0.0;
  private double lastTagSeenSec = -1.0;
  private double lastVisionAcceptedSec = -1.0;
  private int lastVisibleTagCount = 0;
  private int pendingCount = 0;
  private Pose2d pendingPose = null;
  private String localizationStatus = "WAITING_FOR_CAMERA_CALIBRATION";

  public HubVision() {
    headingPid.enableContinuousInput(-Math.PI, Math.PI);
    headingPid.setTolerance(Math.toRadians(2.5));
    SmartDashboard.putBoolean("Hub Camera Mount Verified", VisionConstants.CAMERA_MOUNT_VERIFIED);
  }

  public void resetHeadingController() {
    headingPid.reset();
  }

  /** Read every unread frame exactly once; preserve latency-compensated timestamps. */
  public void update(DriveSubsystem drive) {
    double now = Timer.getFPGATimestamp();
    if (!VisionConstants.CAMERA_MOUNT_VERIFIED) {
      // Still consume frames and retain basic detection telemetry.
      for (var frame : camera.getAllUnreadResults()) {
        recordTargets(frame, now);
      }
      localizationStatus = "CAMERA_MOUNT_NOT_VERIFIED";
      publish(drive, now);
      return;
    }

    for (var frame : camera.getAllUnreadResults()) {
      recordTargets(frame, now);
      if (!frame.hasTargets() || !drive.isGyroReady() || drive.isBumpActive()) {
        continue;
      }

      boolean multi = frame.getMultiTagResult().isPresent();
      Optional<EstimatedRobotPose> estimated = estimator.estimateCoprocMultiTagPose(frame);
      if (estimated.isEmpty()) {
        multi = false;
        estimated = estimator.estimateLowestAmbiguityPose(frame);
      }
      if (estimated.isEmpty()) {
        continue;
      }
      EstimatedRobotPose e = estimated.get();
      if (!multi && !isGoodSingleTag(e)) {
        continue;
      }

      double timestamp = e.timestampSeconds;
      if (!Double.isFinite(timestamp) || timestamp > now + 0.10
          || now - timestamp > MAX_FRAME_AGE_SEC) {
        continue;
      }
      Pose3d pose3d = e.estimatedPose;
      Pose2d pose = pose3d.toPose2d();
      if (!isFieldPlausible(pose3d, pose)) {
        continue;
      }

      if (!drive.isPoseInitialized()) {
        // Require consistent single-tag readings before the first pose lock.
        if (multi || confirmPose(pose, 2)) {
          drive.resetOdometry(pose);
          lastVisionAcceptedSec = now;
          pendingPose = null;
          pendingCount = 0;
          localizationStatus = "INITIALIZED_BY_TAG";
        }
        continue;
      }

      double deviation = drive.getPose().getTranslation().getDistance(
          pose.getTranslation());
      if (deviation > MAX_POSE_JUMP_M) {
        // Do not allow an outlier to teleport the robot. After a BUMP,
        // consistent repeated frames can explicitly reinitialize the estimate.
        if (confirmPose(pose, multi ? 3 : 5)
            && !drive.isBumpActive()) {
          drive.resetOdometry(pose);
          lastVisionAcceptedSec = now;
          pendingPose = null;
          pendingCount = 0;
          localizationStatus = "RELOCALIZED";
        }
        continue;
      }

      // Sensor fusion uses the camera's capture time, not arrival time.
      drive.addVisionPose(pose, timestamp, multi);
      lastVisionAcceptedSec = now;
      pendingPose = null;
      pendingCount = 0;
      localizationStatus = multi ? "MULTITAG" : "SINGLETAG";
    }
    publish(drive, now);
  }

  private void recordTargets(PhotonPipelineResult frame, double now) {
    int validCount = 0;
    for (var target : frame.getTargets()) {
      if (target.getFiducialId() > 0) {
        validCount++;
      }
    }
    lastVisibleTagCount = validCount;
    if (validCount > 0) {
      lastTagSeenSec = now;
      var target = frame.getBestTarget();
      if (target != null && target.getFiducialId() > 0) {
        latestYawDeg = target.getYaw();
      }
    }
  }

  private boolean isGoodSingleTag(EstimatedRobotPose pose) {
    return pose.targetsUsed != null
        && pose.targetsUsed.size() == 1
        && pose.targetsUsed.get(0).getFiducialId() > 0
        && pose.targetsUsed.get(0).getPoseAmbiguity() >= 0.0
        && pose.targetsUsed.get(0).getPoseAmbiguity() < MAX_SINGLE_TAG_AMBIGUITY;
  }

  private boolean isFieldPlausible(Pose3d pose, Pose2d planar) {
    return Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getZ())
        && Double.isFinite(pose.getRotation().getX())
        && Double.isFinite(pose.getRotation().getY())
        && Math.abs(pose.getZ()) < MAX_INITIAL_Z_M
        && Math.abs(pose.getRotation().getX()) < Math.toRadians(20.0)
        && Math.abs(pose.getRotation().getY()) < Math.toRadians(20.0)
        && planar.getX() > -0.2 && planar.getX() < field.getFieldLength() + 0.2
        && planar.getY() > -0.2 && planar.getY() < field.getFieldWidth() + 0.2;
  }

  /** Consecutive independent observations must agree before a large pose reset. */
  private boolean confirmPose(Pose2d pose, int required) {
    if (pendingPose != null
        && pose.getTranslation().getDistance(pendingPose.getTranslation()) < CANDIDATE_MATCH_M
        && Math.abs(MathUtil.angleModulus(
            pose.getRotation().minus(pendingPose.getRotation()).getRadians()))
            < CANDIDATE_MATCH_RAD) {
      pendingCount++;
    } else {
      pendingPose = pose;
      pendingCount = 1;
    }
    return pendingCount >= required;
  }

  public boolean isTagVisible() {
    return lastTagSeenSec >= 0.0
        && Timer.getFPGATimestamp() - lastTagSeenSec <= 0.25;
  }

  public double getLastTargetYawDeg() {
    return isTagVisible() ? latestYawDeg : 0.0;
  }

  /** HUB center in the common BLUE-origin field coordinate system. */
  public Optional<Translation2d> getAllianceHub() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return Optional.empty();
    }
    double x = alliance.get() == DriverStation.Alliance.Blue
        ? VisionConstants.HUB_FROM_ALLIANCE_WALL_M
        : field.getFieldLength() - VisionConstants.HUB_FROM_ALLIANCE_WALL_M;
    return Optional.of(new Translation2d(x, field.getFieldWidth() / 2.0));
  }

  public boolean canAim(DriveSubsystem drive) {
    return drive.isPoseInitialized()
        && drive.isGyroReady()
        && !drive.isBumpActive()
        && !drive.needsBumpRecovery()
        && getAllianceHub().isPresent();
  }

  public double calculateRotation(DriveSubsystem drive, double manualRotation) {
    if (Math.abs(manualRotation) > MANUAL_ROTATION_OVERRIDE) {
      headingPid.reset();
      return manualRotation;
    }
    if (!canAim(drive)) {
      headingPid.reset();
      return 0.0;
    }
    var pose = drive.getPose();
    var hub = getAllianceHub().orElseThrow();
    double desired = Math.atan2(hub.getY() - pose.getY(), hub.getX() - pose.getX());
    double omegaRadPerSecond =
        headingPid.calculate(pose.getRotation().getRadians(), desired);
    return MathUtil.clamp(
        omegaRadPerSecond / DriveConstants.kMaxAngularSpeed,
        -0.6,
        0.6);
  }

  private void publish(DriveSubsystem drive, double now) {
    boolean visible = isTagVisible();
    SmartDashboard.putBoolean("Vision Target Visible", visible);
    SmartDashboard.putNumber("Vision Target Yaw", getLastTargetYawDeg());
    SmartDashboard.putNumber("Vision Tag Count", visible ? lastVisibleTagCount : 0);
    SmartDashboard.putString("Hub Localization Status", localizationStatus);
    SmartDashboard.putBoolean("Hub Lock Ready", canAim(drive));
    SmartDashboard.putNumber(
        "Vision Pose Age Sec",
        lastVisionAcceptedSec >= 0.0 ? now - lastVisionAcceptedSec : -1.0);

    var hub = getAllianceHub();
    if (hub.isPresent()) {
      var point = hub.get();
      var pose = drive.getPose();
      SmartDashboard.putNumber("Hub Target X", point.getX());
      SmartDashboard.putNumber("Hub Target Y", point.getY());
      SmartDashboard.putNumber("Hub Distance M", drive.isPoseInitialized()
          ? pose.getTranslation().getDistance(point) : -1.0);
      double heading = Math.atan2(point.getY() - pose.getY(), point.getX() - pose.getX());
      SmartDashboard.putNumber("Hub Target Heading", Math.toDegrees(heading));
      SmartDashboard.putNumber("Hub Heading Error",
          drive.isPoseInitialized()
              ? Math.toDegrees(MathUtil.angleModulus(
                  heading - pose.getRotation().getRadians()))
              : 0.0);
    } else {
      SmartDashboard.putNumber("Hub Distance M", -1.0);
    }
  }
}
