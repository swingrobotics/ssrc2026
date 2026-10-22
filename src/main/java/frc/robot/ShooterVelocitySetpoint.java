package frc.robot;

import frc.robot.Constants.ShooterConstants;

/** Pure fail-closed validation for the SPARK shooter RPM setpoint. */
public final class ShooterVelocitySetpoint {
  private ShooterVelocitySetpoint() {}

  /** Zero means stop; out-of-range/nonfinite values are rejected, not clamped. */
  public static double sanitizeRPM(double requestedRPM) {
    return Double.isFinite(requestedRPM)
        && requestedRPM > 0.0 && requestedRPM <= ShooterConstants.MAX_TARGET_RPM
            ? requestedRPM : 0.0;
  }

  /** Validates both underspeed and overspeed before reporting shot readiness. */
  public static boolean isAtSpeed(double actualRPM, double targetRPM, double toleranceRPM) {
    return Double.isFinite(actualRPM) && actualRPM >= 0.0
        && sanitizeRPM(targetRPM) > 0.0
        && Double.isFinite(toleranceRPM) && toleranceRPM > 0.0
        && targetRPM > toleranceRPM
        && Math.abs(actualRPM - targetRPM) <= toleranceRPM;
  }
}
