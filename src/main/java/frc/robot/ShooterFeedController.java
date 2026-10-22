package frc.robot;

/**
 * Pure feeder direction logic; shooter speed is a measured shaft RPM,
 * never inferred from the motor duty-cycle percentage.
 */
public final class ShooterFeedController {
  private ShooterFeedController() {}

  public enum Direction { STOP, REVERSE, FORWARD }

  /**
   * Reverse until near a measured/calibrated target RPM; then feed forward.
   * Once forward, a second tolerance band prevents direction chatter when
   * loading the flywheel drops RPM slightly.
   * An unconfigured target or invalid sensor reading always stops the feeder.
   */
  public static Direction select(boolean shooterRequested, double measuredRPM,
      double targetRPM, double toleranceRPM, boolean previouslyForward) {
    if (!shooterRequested
        || !Double.isFinite(measuredRPM) || measuredRPM < 0.0
        || !Double.isFinite(targetRPM) || !Double.isFinite(toleranceRPM)
        || toleranceRPM <= 0.0 || targetRPM <= 2.0 * toleranceRPM) {
      return Direction.STOP;
    }

    double threshold = targetRPM - (previouslyForward ? 2.0 : 1.0) * toleranceRPM;
    return measuredRPM >= threshold ? Direction.FORWARD : Direction.REVERSE;
  }
}
