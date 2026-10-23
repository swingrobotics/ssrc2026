package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import org.junit.jupiter.api.Test;

class ShooterPercentOutputTest {
  @Test
  void configuredPercentStaysWithinPhysicalMotorOutputLimits() {
    // The team may tune this percentage; do not hard-code the old 50/75 values.
    assertTrue(ShooterConstants.MAX_MOTOR_OUTPUT > 0.0);
    assertTrue(ShooterConstants.MAX_MOTOR_OUTPUT <= 1.0);
    assertTrue(ShooterConstants.DEFAULT_OUTPUT_PERCENT >= 0.0);
    assertTrue(ShooterConstants.DEFAULT_OUTPUT_PERCENT
        <= ShooterConstants.MAX_MOTOR_OUTPUT * 100.0);
    assertEquals(40, ShooterConstants.CURRENT_LIMIT_AMPS);
  }

  @Test
  void percentOutputSanitization() {
    assertEquals(Math.min(50.0, ShooterConstants.MAX_MOTOR_OUTPUT * 100.0),
        ShooterSubsystem.sanitizeOutputPercent(50.0), 1e-9);
    assertEquals(ShooterConstants.MAX_MOTOR_OUTPUT * 100.0,
        ShooterSubsystem.sanitizeOutputPercent(100.0), 1e-9);
    assertEquals(ShooterConstants.DEFAULT_OUTPUT_PERCENT,
        ShooterSubsystem.sanitizeOutputPercent(
            ShooterConstants.DEFAULT_OUTPUT_PERCENT), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(0.0), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(-20.0), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(Double.NaN), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(Double.POSITIVE_INFINITY), 1e-9);
  }
}
