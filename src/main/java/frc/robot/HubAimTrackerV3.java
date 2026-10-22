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
 * V3: lock the physical HUB center in LOCAL swerve-odometry coordinates.
 * A fresh individual AprilTag fixes the center; a moving chassis changes the
 * robot-to-HUB bearing but NEVER moves the stored target with the chassis.
 * Old V2, global pose initialization, and a coprocessor MultiTag solution
 * are NOT dependencies of this tracker.
 */
public final class HubAimTrackerV3 {
  public static final double MAX_HOLD_SECONDS = 0.80;
  public static final double LIVE_TAG_SECONDS = 0.25;
  private static final double MAX_FRAME_AGE_SECONDS = 0.35;
  private static final double MAX_AMBIGUITY = 0.20;
  private static final double MAX_TAG_RANGE = 5.5;
  private static final double TAG_CONSENSUS = 0.35;
  private static final double MAX_FIX_JUMP = 0.70;
  private static final double FIX_ALPHA = 0.35;

  private final AprilTagFieldLayout layout;
  private Translation2d hubOdom = null;
  private Alliance lockedAlliance = null;
  private double lastFixTimestamp = -1.0;
  private double lastProcessedTimestamp = -1.0;
  private int tagCount = 0;
  private int rejectionCount = 0;
  private String lastRejection = "NONE";
  private String status = "WAITING FOR HUB TAG";

  private record Fix(Translation2d hub, Alliance alliance, double weight) {}

  public HubAimTrackerV3(AprilTagFieldLayout fieldLayout) {
    layout = fieldLayout;
  }

  public static Optional<Alliance> hubAlliance(int id) {
    return switch (id) {
      case 18, 19, 20, 21, 24, 25, 26, 27 -> Optional.of(Alliance.Blue);
      case 2, 3, 4, 5, 8, 9, 10, 11 -> Optional.of(Alliance.Red);
      default -> Optional.empty();
    };
  }

  /** Tag pose and tag facing, not a naive average of the visible tag centers. */
  public Optional<Translation2d> hubRelativeToRobot(
      int tagId, Transform3d cameraToTag) {
    if (layout == null || cameraToTag == null) return Optional.empty();
    Optional<Alliance> alliance = hubAlliance(tagId);
    Optional<Pose3d> tag = layout.getTagPose(tagId);
    if (alliance.isEmpty() || tag.isEmpty()) return Optional.empty();
    Translation3d v = cameraToTag.getTranslation();
    double range = v.getNorm();
    if (!Double.isFinite(v.getX()) || !Double.isFinite(v.getY())
        || !Double.isFinite(v.getZ())
        || !Double.isFinite(range) || range < 0.15 || range > MAX_TAG_RANGE)
      return Optional.empty();
    Translation2d center = alliance.get() == Alliance.Blue
        ? HubConstants.BLUE_CENTER_METERS : HubConstants.RED_CENTER_METERS;
    Pose3d hubAtTagHeight = new Pose3d(
        new Translation3d(center.getX(), center.getY(), tag.get().getZ()),
        new Rotation3d());
    Transform3d tagToHub = new Transform3d(tag.get(), hubAtTagHeight);
    Translation2d relative = new Pose3d()
        .transformBy(VisionConstants.ROBOT_TO_CAMERA)
        .transformBy(cameraToTag)
        .transformBy(tagToHub)
        .getTranslation().toTranslation2d();
    return Double.isFinite(relative.getX()) && Double.isFinite(relative.getY())
        && relative.getNorm() > 0.40 && relative.getNorm() < 8.0
        ? Optional.of(relative) : Optional.empty();
  }

  /** PhotonVision frames are processed against odometry at CAPTURE time. */
  public void observe(List<PhotonTrackedTarget> targets,
      Optional<Alliance> currentAlliance, Pose2d robotPoseAtCapture,
      double timestamp, double now) {
    if (currentAlliance == null) {
      reject("INVALID ALLIANCE", false);
      return;
    }
    if (lockedAlliance != null && currentAlliance.isPresent()
        && currentAlliance.get() != lockedAlliance) clear();
    if (targets == null || targets.isEmpty()) {
      status = "NO HUB TAG VISIBLE";
      return;
    }
    if (robotPoseAtCapture == null
        || !Double.isFinite(robotPoseAtCapture.getX())
        || !Double.isFinite(robotPoseAtCapture.getY())
        || !Double.isFinite(robotPoseAtCapture.getRotation().getRadians())
        || !Double.isFinite(timestamp) || !Double.isFinite(now)
        || timestamp <= lastProcessedTimestamp
        || timestamp - now > 0.05 || now - timestamp > MAX_FRAME_AGE_SECONDS) {
      reject("BAD TIMESTAMP OR CAPTURE ODOMETRY", false);
      return;
    }
    lastProcessedTimestamp = timestamp;
    List<Fix> candidates = new ArrayList<>();
    for (PhotonTrackedTarget t : targets) {
      Optional<Alliance> color = hubAlliance(t.getFiducialId());
      if (color.isEmpty()
          || currentAlliance.isPresent() && color.get() != currentAlliance.get()
          || lockedAlliance != null && color.get() != lockedAlliance)
        continue;
      double ambiguity = t.getPoseAmbiguity();
      if (!Double.isFinite(ambiguity) || ambiguity < 0
          || ambiguity > MAX_AMBIGUITY) continue;
      Optional<Translation2d> hubRobot = hubRelativeToRobot(
          t.getFiducialId(), t.getBestCameraToTarget());
      if (hubRobot.isEmpty()) continue;
      Translation2d fix = robotPoseAtCapture.getTranslation().plus(
          hubRobot.get().rotateBy(robotPoseAtCapture.getRotation()));
      if (!Double.isFinite(fix.getX()) || !Double.isFinite(fix.getY())) continue;
      double range = t.getBestCameraToTarget().getTranslation().getNorm();
      double weight = 1.0 / (0.05 + ambiguity + 0.025 * range * range);
      candidates.add(new Fix(fix, color.get(), weight));
    }
    if (candidates.isEmpty()) {
      reject("NO VALID HUB TAG GEOMETRY", false);
      return;
    }
    Fix reference = candidates.stream().min(Comparator.comparingDouble(
        a -> candidates.stream().mapToDouble(
            b -> a.hub().getDistance(b.hub())).sum())).orElseThrow();
    List<Fix> agreed = candidates.stream()
        .filter(c -> c.alliance() == reference.alliance()
            && c.hub().getDistance(reference.hub()) <= TAG_CONSENSUS)
        .toList();
    if (candidates.size() > 1 && agreed.size() < 2) {
      reject("TAG CENTERS DISAGREE", true);
      return;
    }
    double x = 0, y = 0, weight = 0;
    for (Fix c : agreed) {
      x += c.hub().getX() * c.weight();
      y += c.hub().getY() * c.weight();
      weight += c.weight();
    }
    Translation2d estimate = new Translation2d(x / weight, y / weight);
    if (!Double.isFinite(estimate.getX()) || !Double.isFinite(estimate.getY())) {
      reject("INVALID HUB CENTER", true);
      return;
    }
    if (hasLock(now) && estimate.getDistance(hubOdom) > MAX_FIX_JUMP) {
      reject("HUB FIX JUMP", true);
      return;
    }
    if (hasLock(now)) estimate = hubOdom.interpolate(estimate, FIX_ALPHA);
    hubOdom = estimate;
    lockedAlliance = reference.alliance();
    lastFixTimestamp = timestamp;
    tagCount = agreed.size();
    status = "TAG FIX";
  }

  private void reject(String reason, boolean invalidate) {
    lastRejection = reason;
    rejectionCount++;
    if (invalidate) clear();
    status = reason;
  }

  /** Call this on deliberate gyro/odometry frame resets. */
  public void clear() {
    hubOdom = null;
    lockedAlliance = null;
    lastFixTimestamp = -1.0;
    tagCount = 0;
    status = "WAITING FOR HUB TAG";
  }

  /** A forgotten tag is tracked ONLY for 0.80 seconds from its last fix. */
  public boolean hasLock(double now) {
    return hubOdom != null && lastFixTimestamp >= 0.0
        && Double.isFinite(now) && now >= lastFixTimestamp
        && now - lastFixTimestamp <= MAX_HOLD_SECONDS;
  }

  public boolean hasLiveTag(double now) {
    return hasLock(now) && now - lastFixTimestamp <= LIVE_TAG_SECONDS;
  }

  public Optional<Translation2d> getHubInOdometry(double now) {
    return hasLock(now) ? Optional.of(hubOdom) : Optional.empty();
  }

  /** Translation updates as the robot moves, using a fixed odometry target. */
  public Optional<Translation2d> getHubInRobot(Pose2d robotPose, double now) {
    if (!hasLock(now) || robotPose == null
        || !Double.isFinite(robotPose.getX()) || !Double.isFinite(robotPose.getY())
        || !Double.isFinite(robotPose.getRotation().getRadians()))
      return Optional.empty();
    return Optional.of(hubOdom.minus(robotPose.getTranslation())
        .rotateBy(robotPose.getRotation().unaryMinus()));
  }

  public double getDistanceMeters(Pose2d robotPose, double now) {
    return getHubInRobot(robotPose, now)
        .map(Translation2d::getNorm).orElse(-1.0);
  }

  public double getBearingDegrees(Pose2d robotPose, double now) {
    return getHubInRobot(robotPose, now)
        .map(v -> Math.toDegrees(Math.atan2(v.getY(), v.getX())))
        .orElse(-999.0);
  }

  public String getStatus(double now) {
    if (!hasLock(now)) return "NO HUB LOCK: " + status;
    return hasLiveTag(now) ? "TAG FIX (" + tagCount + " TAGS)"
        : "ODOMETRY HOLD (NO TAG)";
  }

  public int getTagCount() { return tagCount; }
  public int getRejectionCount() { return rejectionCount; }
  public String getLastRejection() { return lastRejection; }
  public String getAlliance() {
    return lockedAlliance == null ? "NONE" : lockedAlliance.name();
  }
  public double getTargetAge(double now) {
    return lastFixTimestamp < 0.0 ? -1.0 : now - lastFixTimestamp;
  }
  public double getHubX(double now) {
    return hasLock(now) ? hubOdom.getX() : -999.0;
  }
  public double getHubY(double now) {
    return hasLock(now) ? hubOdom.getY() : -999.0;
  }
}
