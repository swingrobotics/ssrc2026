package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Second-generation AprilTag-to-HUB tracker, independent of initial field pose.
 * Individual PnP tag transforms are converted to the physical HUB center,
 * then projected into capture-time odometry for latency compensation.
 * All output is geometry only: this class never commands a motor.
 */
public final class HubAimTargetV2 {
  private static final double FRAME_MAX_AGE_S = 0.30;
  public static final double TARGET_MAX_AGE_S = 0.25;
  private static final double MAX_AMBIGUITY = 0.20;
  private static final double MAX_TAG_RANGE_M = 5.5;
  private static final double TAG_CONSENSUS_M = 0.35;
  private static final double MAX_TARGET_STEP_M = 0.70;
  private static final double SMOOTHING = 0.40;

  private final AprilTagFieldLayout layout;
  private Translation2d hubInOdometry = null;
  private double lastCaptureTime = -1.0;
  private int lastTagCount = 0;
  private int lastHubTagId = -1;
  private Alliance targetAlliance = null;
  private int rejectedFrames = 0;
  private String status = "WAITING FOR HUB TAG";

  private record Candidate(Translation2d hub, Alliance alliance, int id, double weight) {}

  public HubAimTargetV2(AprilTagFieldLayout layout) {
    this.layout = layout;
  }

  public static Optional<Alliance> allianceForTag(int id) {
    return switch (id) {
      case 18, 19, 20, 21, 24, 25, 26, 27 -> Optional.of(Alliance.Blue);
      case 2, 3, 4, 5, 8, 9, 10, 11 -> Optional.of(Alliance.Red);
      default -> Optional.empty();
    };
  }

  /**
   * Tag yaw is not the aim point. Use Robot->Camera, Camera->Tag, and
   * official field-layout Tag->HUB transforms to obtain robot-relative HUB XY.
   */
  public static Optional<Translation2d> hubRelativeToRobot(
      int tagId, Transform3d cameraToTag, AprilTagFieldLayout layout,
      Transform3d robotToCamera) {
    if (cameraToTag == null || layout == null || robotToCamera == null) {
      return Optional.empty();
    }
    Optional<Alliance> alliance = allianceForTag(tagId);
    Optional<Pose3d> tag = layout.getTagPose(tagId);
    if (alliance.isEmpty() || tag.isEmpty()) return Optional.empty();

    Translation3d measured = cameraToTag.getTranslation();
    double range = measured.getNorm();
    if (!Double.isFinite(range) || range < 0.15 || range > MAX_TAG_RANGE_M) {
      return Optional.empty();
    }
    Translation2d hubXY = alliance.get() == Alliance.Blue
        ? HubConstants.BLUE_CENTER_METERS : HubConstants.RED_CENTER_METERS;
    Pose3d hubAtTagHeight = new Pose3d(
        new Translation3d(hubXY.getX(), hubXY.getY(), tag.get().getZ()),
        new Rotation3d());
    Transform3d tagToHub = new Transform3d(tag.get(), hubAtTagHeight);
    Translation2d relative = new Pose3d()
        .transformBy(robotToCamera)
        .transformBy(cameraToTag)
        .transformBy(tagToHub)
        .getTranslation().toTranslation2d();
    return Double.isFinite(relative.getX()) && Double.isFinite(relative.getY())
        && relative.getNorm() > 0.40 && relative.getNorm() < 8.0
        ? Optional.of(relative) : Optional.empty();
  }

  public void observe(List<PhotonTrackedTarget> targets,
      Optional<Alliance> knownAlliance, Pose2d poseAtCapture,
      double timestamp, double now) {
    if (targets == null || targets.isEmpty()) {
      status = "NO HUB TAGS";
      return;
    }
    if (knownAlliance == null || poseAtCapture == null
        || !Double.isFinite(poseAtCapture.getX())
        || !Double.isFinite(poseAtCapture.getY())
        || !Double.isFinite(poseAtCapture.getRotation().getRadians())
        || !Double.isFinite(timestamp) || !Double.isFinite(now)
        || timestamp <= lastCaptureTime
        || timestamp - now > 0.05 || now - timestamp > FRAME_MAX_AGE_S) {
      reject("STALE FRAME OR MISSING CAPTURE-TIME ODOMETRY");
      return;
    }

    List<Candidate> candidates = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      Optional<Alliance> candidateAlliance = allianceForTag(target.getFiducialId());
      if (candidateAlliance.isEmpty() || (knownAlliance.isPresent()
          && candidateAlliance.get() != knownAlliance.get())) continue;
      double ambiguity = target.getPoseAmbiguity();
      if (!Double.isFinite(ambiguity) || ambiguity < 0
          || ambiguity > MAX_AMBIGUITY) continue;
      Optional<Translation2d> relative = hubRelativeToRobot(
          target.getFiducialId(), target.getBestCameraToTarget(),
          layout, VisionConstants.ROBOT_TO_CAMERA);
      if (relative.isEmpty()) continue;
      Translation2d point = poseAtCapture.getTranslation().plus(
          relative.get().rotateBy(poseAtCapture.getRotation()));
      if (!Double.isFinite(point.getX()) || !Double.isFinite(point.getY())) continue;
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();
      double weight = 1.0 / (0.05 + ambiguity + 0.025 * distance * distance);
      candidates.add(new Candidate(
          point, candidateAlliance.get(), target.getFiducialId(), weight));
    }
    if (candidates.isEmpty()) {
      reject("NO VALID INDIVIDUAL 3D HUB TAG");
      return;
    }

    // If Driver Station has no alliance, never mix observations of two HUBs.
    if (knownAlliance.isEmpty()) {
      Alliance firstColor = candidates.get(0).alliance();
      if (candidates.stream().anyMatch(t -> t.alliance() != firstColor)) {
        reject("MIXED ALLIANCE HUB TAGS");
        return;
      }
    }
    Candidate seed = candidates.stream().min(Comparator.comparingDouble(
        c -> candidates.stream().mapToDouble(
            other -> c.hub().getDistance(other.hub())).sum())).orElseThrow();
    List<Candidate> consensus = candidates.stream().filter(
        c -> c.hub().getDistance(seed.hub()) <= TAG_CONSENSUS_M).toList();
    if (candidates.size() > 1 && consensus.size() < 2) {
      reject("HUB TAGS DISAGREE");
      return;
    }

    double x = 0, y = 0, totalWeight = 0;
    for (Candidate c : consensus) {
      x += c.hub().getX() * c.weight();
      y += c.hub().getY() * c.weight();
      totalWeight += c.weight();
    }
    Translation2d fresh = new Translation2d(x / totalWeight, y / totalWeight);
    if (isFresh(now) && fresh.getDistance(hubInOdometry) > MAX_TARGET_STEP_M) {
      reject("HUB POSITION JUMP");
      return;
    }
    if (isFresh(now)) fresh = hubInOdometry.interpolate(fresh, SMOOTHING);
    hubInOdometry = fresh;
    lastCaptureTime = timestamp;
    lastHubTagId = seed.id();
    lastTagCount = consensus.size();
    targetAlliance = seed.alliance();
    status = "VALID HUB CENTER: " + consensus.size() + " TAG(S)";
  }

  private void reject(String reason) {
    status = reason;
    rejectedFrames++;
    // Never keep output from a newly discovered invalid geometry disagreement.
    if ("HUB TAGS DISAGREE".equals(reason) || "MIXED ALLIANCE HUB TAGS".equals(reason)
        || "HUB POSITION JUMP".equals(reason)) {
      hubInOdometry = null;
      lastCaptureTime = -1.0;
    }
  }

  public Optional<Translation2d> getHubInOdometryFrame(double now) {
    return isFresh(now) ? Optional.of(hubInOdometry) : Optional.empty();
  }
  public boolean isFresh(double now) {
    return hubInOdometry != null && lastCaptureTime >= 0.0
        && Double.isFinite(now) && now >= lastCaptureTime
        && now - lastCaptureTime <= TARGET_MAX_AGE_S;
  }
  public void clear() {
    hubInOdometry = null;
    lastCaptureTime = -1.0;
    lastTagCount = 0;
    lastHubTagId = -1;
    targetAlliance = null;
    status = "WAITING FOR HUB TAG";
  }
  public String getStatus() { return status; }
  public int getLastTagCount() { return lastTagCount; }
  public int getLastHubTagId() { return lastHubTagId; }
  public int getRejectedFrames() { return rejectedFrames; }
  public String getTargetAlliance() {
    return targetAlliance == null ? "UNKNOWN" : targetAlliance.name();
  }
  public double getLastAgeSeconds(double now) {
    return lastCaptureTime < 0 ? -1.0 : now - lastCaptureTime;
  }
}
