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
 * Local HUB aiming without global field pose initialization.
 *
 * <p>Based on FRC 4107's single-tag cameraToTag -> tagToHub chain. The tag to
 * hub transform is derived from WPILib's actual 2026 field tag Pose3d, not
 * hand-tuned offsets. Each tag independently estimates the same physical HUB
 * center. Measurements are converted into an odometry-local frame using the
 * timestamped drivetrain pose, so robot translation/rotation is compensated
 * between PhotonVision frames (WPILib TimeInterpolatableBuffer principle).
 */
public final class HubRelativeTarget {
  public static final double MAX_TARGET_AGE_SECONDS = 0.30;
  private static final double MAX_FRAME_AGE_SECONDS = 0.35;
  private static final double MAX_TAG_RANGE_METERS = 5.5;
  private static final double MAX_POSE_AMBIGUITY = 0.30;
  private static final double CONSENSUS_RADIUS_METERS = 0.42;
  private static final double MAX_SINGLE_FRAME_STEP_METERS = 0.95;
  private static final double FILTER_ALPHA = 0.38;

  private final AprilTagFieldLayout fieldLayout;
  private Translation2d hubInOdometryFrame = null;
  private double lastTime = -1.0;
  private int lastTagCount = 0;
  private int lastHubTagId = -1;
  private Alliance targetAlliance = null;
  private String status = "WAITING FOR HUB TAG";
  private int rejectedFrames = 0;

  public HubRelativeTarget(AprilTagFieldLayout fieldLayout) {
    this.fieldLayout = fieldLayout;
  }

  public static Optional<Alliance> allianceForTag(int id) {
    return switch (id) {
      case 18, 19, 20, 21, 24, 25, 26, 27 -> Optional.of(Alliance.Blue);
      case 2, 3, 4, 5, 8, 9, 10, 11 -> Optional.of(Alliance.Red);
      default -> Optional.empty();
    };
  }

  /**
   * Calculate the HUB's center XY in ROBOT coordinates from ONE tracked tag.
   * It does not need global robot pose or an initialized field origin.
   *
   * The known tag-to-hub transform is from the official AprilTag layout;
   * its Z is deliberately the tag's own height because we aim horizontally.
   * All tag sides and the camera's real 3-D pitch/yaw are accounted for by
   * Transform3d composition, unlike directly using target.getYaw().
   */
  public static Optional<Translation2d> hubRelativeToRobot(
      int tagId, Transform3d cameraToTag, AprilTagFieldLayout layout,
      Transform3d robotToCamera) {
    Optional<Alliance> alliance = allianceForTag(tagId);
    Optional<Pose3d> tagPoseOpt = layout.getTagPose(tagId);
    if (alliance.isEmpty() || tagPoseOpt.isEmpty() || cameraToTag == null) {
      return Optional.empty();
    }

    Translation3d cameraTranslation = cameraToTag.getTranslation();
    double tagRange = cameraTranslation.getNorm();
    if (!Double.isFinite(tagRange) || tagRange < 0.15
        || tagRange > MAX_TAG_RANGE_METERS) {
      // PhotonVision can leave per-tag cameraToTarget = zero when MultiTag
      // disables "Always Do Single-Target Estimation".
      return Optional.empty();
    }

    Pose3d tagFieldPose = tagPoseOpt.get();
    Translation2d xy = alliance.get() == Alliance.Blue
        ? HubConstants.BLUE_CENTER_METERS : HubConstants.RED_CENTER_METERS;
    Pose3d hubFieldPose = new Pose3d(
        new Translation3d(xy.getX(), xy.getY(), tagFieldPose.getZ()),
        new Rotation3d());
    Transform3d tagToHub = new Transform3d(tagFieldPose, hubFieldPose);
    Pose3d hubRobotPose = new Pose3d()
        .transformBy(robotToCamera)
        .transformBy(cameraToTag)
        .transformBy(tagToHub);
    Translation2d hubRobot = hubRobotPose.getTranslation().toTranslation2d();
    return Double.isFinite(hubRobot.getX()) && Double.isFinite(hubRobot.getY())
        && hubRobot.getNorm() > 0.25 && hubRobot.getNorm() < 8.0
            ? Optional.of(hubRobot) : Optional.empty();
  }

  private record Candidate(
      Translation2d hubOdom, Alliance alliance, int id, double weight) {}

  /** Read one camera frame; only valid individual SolvePnP tag poses participate. */
  public void observe(
      List<PhotonTrackedTarget> targets, Optional<Alliance> knownAlliance,
      Pose2d poseAtCapture, double captureTime, double now) {
    if (targets == null || targets.isEmpty()) {
      status = "NO TAGS IN FRAME";
      return;
    }
    if (poseAtCapture == null || !Double.isFinite(captureTime)
        || !Double.isFinite(now) || now - captureTime > MAX_FRAME_AGE_SECONDS
        || captureTime - now > 0.05 || captureTime <= lastTime) {
      status = "STALE OR UNSAMPLED CAMERA FRAME";
      rejectedFrames++;
      return;
    }

    List<Candidate> candidates = new ArrayList<>();
    for (PhotonTrackedTarget target : targets) {
      int id = target.getFiducialId();
      Optional<Alliance> tagAlliance = allianceForTag(id);
      if (tagAlliance.isEmpty()) continue;
      if (knownAlliance.isPresent() && knownAlliance.get() != tagAlliance.get()) {
        continue;
      }
      double ambiguity = target.getPoseAmbiguity();
      if (!Double.isFinite(ambiguity) || ambiguity < 0.0
          || ambiguity > MAX_POSE_AMBIGUITY) continue;
      var relative = hubRelativeToRobot(
          id, target.getBestCameraToTarget(), fieldLayout,
          VisionConstants.ROBOT_TO_CAMERA);
      if (relative.isEmpty()) continue;
      Translation2d hubOdom = poseAtCapture.getTranslation().plus(
          relative.get().rotateBy(poseAtCapture.getRotation()));
      double range = target.getBestCameraToTarget().getTranslation().getNorm();
      double weight = 1.0 / (0.06 + ambiguity + 0.02 * range * range);
      candidates.add(new Candidate(hubOdom, tagAlliance.get(), id, weight));
    }

    if (candidates.isEmpty()) {
      status = "NO VALID HUB 3D TAG: ENABLE ALWAYS DO SINGLE-TARGET ESTIMATION";
      rejectedFrames++;
      return;
    }

    // Without DS alliance, select the strongest valid HUB target's color
    // rather than making up a Blue or Red assumption.
    Candidate strongest = candidates.stream()
        .max(Comparator.comparingDouble(Candidate::weight)).orElseThrow();
    Alliance selected = knownAlliance.orElse(strongest.alliance());
    candidates.removeIf(c -> c.alliance() != selected);

    // Find the most internally consistent candidate (medoid).
    Candidate seed = candidates.stream().min(Comparator.comparingDouble(c ->
        candidates.stream().mapToDouble(other ->
            c.hubOdom().getDistance(other.hubOdom())).sum())).orElseThrow();
    Candidate consensusSeed = seed;
    List<Candidate> consistent = candidates.stream()
        .filter(c -> c.hubOdom().getDistance(consensusSeed.hubOdom())
            <= CONSENSUS_RADIUS_METERS).toList();
    if (consistent.isEmpty()) {
      status = "HUB TAGS DISAGREE";
      rejectedFrames++;
      return;
    }
    // If two tags disagree, trust the one with better individual confidence;
    // never average wildly separated field locations.
    if (consistent.size() == 1 && candidates.size() == 2) {
      seed = candidates.stream()
          .max(Comparator.comparingDouble(Candidate::weight)).orElseThrow();
      consistent = List.of(seed);
    }

    double sx = 0.0, sy = 0.0, weightSum = 0.0;
    for (Candidate c : consistent) {
      sx += c.hubOdom().getX() * c.weight();
      sy += c.hubOdom().getY() * c.weight();
      weightSum += c.weight();
    }
    Translation2d candidateHub = new Translation2d(
        sx / weightSum, sy / weightSum);

    if (hubInOdometryFrame != null && isFresh(now)
        && candidateHub.getDistance(hubInOdometryFrame) > MAX_SINGLE_FRAME_STEP_METERS) {
      status = "REJECTED HUB POSITION JUMP";
      rejectedFrames++;
      return;
    }
    if (hubInOdometryFrame != null && isFresh(now)) {
      candidateHub = hubInOdometryFrame.interpolate(candidateHub, FILTER_ALPHA);
    }

    hubInOdometryFrame = candidateHub;
    lastTime = captureTime;
    targetAlliance = selected;
    lastHubTagId = seed.id();
    lastTagCount = consistent.size();
    status = "DIRECT HUB TARGET VALID: " + consistent.size() + " TAG(S)";
  }

  public Optional<Translation2d> getHubInOdometryFrame(double now) {
    return isFresh(now) ? Optional.of(hubInOdometryFrame) : Optional.empty();
  }

  public void clear() {
    hubInOdometryFrame = null;
    lastTime = -1.0;
    lastTagCount = 0;
    targetAlliance = null;
    status = "WAITING FOR HUB TAG";
  }

  public boolean isFresh(double now) {
    return hubInOdometryFrame != null && lastTime >= 0.0
        && now - lastTime >= 0.0
        && now - lastTime <= MAX_TARGET_AGE_SECONDS;
  }

  public String getStatus() { return status; }
  public int getLastTagCount() { return lastTagCount; }
  public int getLastHubTagId() { return lastHubTagId; }
  public String getTargetAlliance() {
    return targetAlliance == null ? "UNKNOWN" : targetAlliance.name();
  }
  public int getRejectedFrames() { return rejectedFrames; }
  public double getLastAgeSeconds(double now) {
    return lastTime < 0.0 ? -1.0 : now - lastTime;
  }
}
