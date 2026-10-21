package frc.robot;

import frc.robot.Constants.HoodConstants;
import java.util.OptionalDouble;

/**
 * Two-point calibration for DISTANCE -> RAW HOOD ENCODER POSITION.
 *
 * Does not change shooter duty cycle or motor RPM. Never extrapolates,
 * and never supplies guessed endpoints: both points must be measured.
 * The raw DutyCycleEncoder position is not a physical launch angle.
 */
public final class HoodDistanceCalculator {
  private HoodDistanceCalculator() {}

  public record Point(double distanceMeters, double hoodEncoderPosition) {}

  public static boolean isValidPoint(Point point) {
    return point != null
        && Double.isFinite(point.distanceMeters())
        && point.distanceMeters() > 0.0
        && Double.isFinite(point.hoodEncoderPosition())
        && point.hoodEncoderPosition() >= HoodConstants.MIN_POSITION
        && point.hoodEncoderPosition() <= HoodConstants.MAX_POSITION;
  }

  public static boolean areValidPair(Point first, Point second) {
    return isValidPoint(first) && isValidPoint(second)
        && Math.abs(first.distanceMeters() - second.distanceMeters()) > 1e-5;
  }

  public static OptionalDouble interpolate(
      double distanceMeters, Point first, Point second) {
    if (!Double.isFinite(distanceMeters) || !areValidPair(first, second)) {
      return OptionalDouble.empty();
    }
    Point near = first.distanceMeters() < second.distanceMeters() ? first : second;
    Point far = first.distanceMeters() < second.distanceMeters() ? second : first;
    if (distanceMeters < near.distanceMeters() || distanceMeters > far.distanceMeters()) {
      return OptionalDouble.empty();
    }
    double t = (distanceMeters - near.distanceMeters())
        / (far.distanceMeters() - near.distanceMeters());
    double target = near.hoodEncoderPosition()
        + t * (far.hoodEncoderPosition() - near.hoodEncoderPosition());
    return Double.isFinite(target) ? OptionalDouble.of(target) : OptionalDouble.empty();
  }
}
