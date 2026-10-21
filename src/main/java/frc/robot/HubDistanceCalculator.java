package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Horizontal distance from the robot's configured launcher position to the
 * CENTER of its alliance's 2026 REBUILT HUB (not to any AprilTag).
 *
 * Works in the WPILib blue-origin field coordinates for both alliances.
 * Caller must validate that the source pose is recent and trustworthy.
 */
public final class HubDistanceCalculator {
  private HubDistanceCalculator() {}

  public static OptionalDouble calculate(Pose2d fieldPose, Optional<Alliance> alliance) {
    if (fieldPose == null || alliance == null || alliance.isEmpty()) {
      return OptionalDouble.empty();
    }
    if (!Double.isFinite(fieldPose.getX())
        || !Double.isFinite(fieldPose.getY())
        || !Double.isFinite(fieldPose.getRotation().getRadians())) {
      return OptionalDouble.empty();
    }
    double tolerance = VisionConstants.FIELD_BOUNDS_TOLERANCE_M;
    if (fieldPose.getX() < -tolerance
        || fieldPose.getX() > VisionConstants.FIELD_LAYOUT.getFieldLength() + tolerance
        || fieldPose.getY() < -tolerance
        || fieldPose.getY() > VisionConstants.FIELD_LAYOUT.getFieldWidth() + tolerance) {
      return OptionalDouble.empty();
    }
    Translation2d target = alliance.get() == Alliance.Red
        ? HubConstants.RED_CENTER_METERS : HubConstants.BLUE_CENTER_METERS;
    Translation2d launcher = fieldPose.getTranslation().plus(
        HubConstants.SHOOTER_OFFSET_ROBOT_METERS.rotateBy(fieldPose.getRotation()));
    double distance = target.getDistance(launcher);
    return Double.isFinite(distance) && distance > 0.0
        ? OptionalDouble.of(distance) : OptionalDouble.empty();
  }
}
