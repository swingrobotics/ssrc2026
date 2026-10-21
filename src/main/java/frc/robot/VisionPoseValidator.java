package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * Stateless quality gate for PhotonVision field-pose measurements.
 * FRC 5024 VisionSubsystem and BCHRobotics 2026 Vision use the same
 * principles: timestamp and field bounds, ambiguity/distance rejection,
 * odometry innovation gates, covariance scaled by range and tag count.
 */
public final class VisionPoseValidator {
  private static final double MAX_FRAME_AGE_SECONDS = 0.35;
  private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20;
  private static final double MAX_SINGLE_TAG_DISTANCE_M = 3.0;
  private static final double MAX_MULTI_TAG_DISTANCE_M = 6.0;
  private static final double MAX_ROBOT_HEIGHT_METERS = 0.5;
  private static final double MAX_CAMERA_TILT_DEGREES = 25.0;
  private static final double MAX_LINEAR_SPEED_MPS = 3.5;
  private static final double MAX_ANGULAR_SPEED_RPS = Math.toRadians(120.0);
  private static final double MAX_MULTI_TAG_TRANSLATION_ERROR_M = 1.5;
  private static final double MAX_SINGLE_TAG_TRANSLATION_ERROR_M = 0.70;
  private static final double MAX_MULTI_TAG_HEADING_ERROR_RAD = Math.toRadians(25.0);

  private VisionPoseValidator() {}

  public record Decision(
      boolean accepted, boolean canInitialize,
      String reason, double xyStdDevMeters, double thetaStdDevRadians) {
    public static Decision reject(String reason) {
      return new Decision(false, false, reason, Double.NaN, Double.NaN);
    }
  }

  /**
   * Validate a camera field pose. Set poseInitialized=false before the
   * first verified Disabled-mode multi-tag pose reset. Single tag is never
   * allowed to establish an initial global pose.
   */
  public static Decision validate(
      Pose3d estimate3d, double captureTime, double now,
      Pose2d fusedPose, boolean poseInitialized,
      boolean multiTag, int usedTagCount, double averageTagDistance,
      double ambiguity, double linearSpeed, double angularSpeed,
      double fieldLength, double fieldWidth, boolean gyroReady) {

    if (!gyroReady) return Decision.reject("NAVX DISCONNECTED OR CALIBRATING");
    if (estimate3d == null || fusedPose == null || usedTagCount < 1)
      return Decision.reject("INVALID FRAME OR NO TAGS");
    double age = now - captureTime;
    if (!Double.isFinite(age) || age < -0.05 || age > MAX_FRAME_AGE_SECONDS)
      return Decision.reject("STALE OR FUTURE FRAME");
    double x = estimate3d.getX(), y = estimate3d.getY(), z = estimate3d.getZ();
    if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)
        || !Double.isFinite(estimate3d.getRotation().getX())
        || !Double.isFinite(estimate3d.getRotation().getY())
        || !Double.isFinite(estimate3d.getRotation().getZ())
        || x < -0.15 || y < -0.15
        || x > fieldLength + 0.15 || y > fieldWidth + 0.15
        || Math.abs(z) > MAX_ROBOT_HEIGHT_METERS
        || Math.abs(estimate3d.getRotation().getX())
            > Math.toRadians(MAX_CAMERA_TILT_DEGREES)
        || Math.abs(estimate3d.getRotation().getY())
            > Math.toRadians(MAX_CAMERA_TILT_DEGREES)) {
      return Decision.reject("POSE OUTSIDE PHYSICAL FIELD LIMITS");
    }
    if (!Double.isFinite(averageTagDistance)
        || averageTagDistance < 0.05
        || averageTagDistance > (multiTag
            ? MAX_MULTI_TAG_DISTANCE_M : MAX_SINGLE_TAG_DISTANCE_M)) {
      return Decision.reject("TAG TOO FAR OR INVALID RANGE");
    }
    if (!Double.isFinite(linearSpeed) || !Double.isFinite(angularSpeed)
        || linearSpeed > MAX_LINEAR_SPEED_MPS
        || Math.abs(angularSpeed) > MAX_ANGULAR_SPEED_RPS) {
      return Decision.reject("ROBOT MOVING TOO FAST FOR VISION");
    }
    if (!multiTag && (!Double.isFinite(ambiguity)
        || ambiguity < 0.0 || ambiguity > MAX_SINGLE_TAG_AMBIGUITY)) {
      return Decision.reject("AMBIGUOUS SINGLE-TAG MEASUREMENT");
    }
    if (multiTag && usedTagCount < 2)
      return Decision.reject("MULTITAG SOLUTION USES FEWER THAN TWO TAGS");

    // Initial camera pose only establishes a global origin while Disabled,
    // after the operator explicitly verifies field layout and camera mount.
    if (!poseInitialized && !multiTag)
      return Decision.reject("NEED INITIAL MULTITAG POSE");

    if (poseInitialized) {
      double poseError = estimate3d.toPose2d().getTranslation()
          .getDistance(fusedPose.getTranslation());
      double limit = multiTag
          ? MAX_MULTI_TAG_TRANSLATION_ERROR_M
          : MAX_SINGLE_TAG_TRANSLATION_ERROR_M;
      if (poseError > limit) return Decision.reject("VISION/ODOMETRY POSITION DISAGREE");
      if (multiTag && Math.abs(MathUtil.angleModulus(
          estimate3d.toPose2d().getRotation().getRadians()
              - fusedPose.getRotation().getRadians())) > MAX_MULTI_TAG_HEADING_ERROR_RAD) {
        return Decision.reject("VISION/NAVX HEADING DISAGREE");
      }
    }

    // Observations farther away and using fewer tags are less trusted;
    // theta uncertainty deliberately VERY high to retain navX heading.
    double xy = MathUtil.clamp(
        (multiTag ? 0.10 : 0.70)
            + 0.08 * averageTagDistance * averageTagDistance
                / Math.sqrt(usedTagCount), 0.12, 2.5);
    return new Decision(true, multiTag && usedTagCount >= 2,
        "ACCEPTED " + (multiTag ? "MULTITAG" : "SINGLE TAG"), xy, 1000.0);
  }
}
