package frc.robot;

import java.util.Optional;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Interpolates measured shooter RPM and absolute hood-encoder position between
 * two verified calibration points. No extrapolation or guessed shot data.
 *
 * Distances are in meters, RPM is LEADER MOTOR SHAFT RPM (not wheel RPM),
 * and hood positions are the raw DutyCycleEncoder fractional-turn reading.
 */
public final class ShotCalculator {
  private ShotCalculator() {}

  public record CalibrationPoint(double distanceMeters, double motorRpm, double hoodPosition) {}

  public record ShotSetpoint(double motorRpm, double hoodPosition) {}

  public static boolean isValidCalibration(CalibrationPoint point) {
    return point != null
        && Double.isFinite(point.distanceMeters())
        && point.distanceMeters() > 0.0
        && Double.isFinite(point.motorRpm())
        && point.motorRpm() > 0.0
        && point.motorRpm() <= ShooterConstants.MAX_RPM
        && Double.isFinite(point.hoodPosition())
        && point.hoodPosition() >= HoodConstants.MIN_POSITION
        && point.hoodPosition() <= HoodConstants.MAX_POSITION;
  }

  public static boolean areValidPair(CalibrationPoint first, CalibrationPoint second) {
    return isValidCalibration(first)
        && isValidCalibration(second)
        && Math.abs(first.distanceMeters() - second.distanceMeters()) > 1e-6;
  }

  /**
   * Returns empty when calibration is missing, input is invalid or outside
   * the calibrated distance interval. Endpoints are included.
   */
  public static Optional<ShotSetpoint> interpolate(
      double distanceMeters, CalibrationPoint first, CalibrationPoint second) {
    if (!areValidPair(first, second) || !Double.isFinite(distanceMeters)) {
      return Optional.empty();
    }

    CalibrationPoint near = first.distanceMeters() < second.distanceMeters() ? first : second;
    CalibrationPoint far = first.distanceMeters() < second.distanceMeters() ? second : first;
    if (distanceMeters < near.distanceMeters() || distanceMeters > far.distanceMeters()) {
      return Optional.empty();
    }

    double t = (distanceMeters - near.distanceMeters())
        / (far.distanceMeters() - near.distanceMeters());
    double rpm = near.motorRpm() + t * (far.motorRpm() - near.motorRpm());
    double hoodPosition = near.hoodPosition()
        + t * (far.hoodPosition() - near.hoodPosition());
    if (!Double.isFinite(rpm) || !Double.isFinite(hoodPosition)) {
      return Optional.empty();
    }

    return Optional.of(new ShotSetpoint(rpm, hoodPosition));
  }
}
