package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import org.junit.jupiter.api.Test;

class ShooterPercentOutputTest {
  @Test
  void restoresHistoricalDefaultAndLimitWithoutOverwriting40Amps() {
    assertEquals(50.0, ShooterConstants.DEFAULT_OUTPUT_PERCENT, 1e-9);
    assertEquals(75.0, ShooterConstants.MAX_MOTOR_OUTPUT * 100.0, 1e-9);
    assertEquals(40, ShooterConstants.CURRENT_LIMIT_AMPS);
  }

  @Test
  void percentOutputSanitization() {
    assertEquals(50.0, ShooterSubsystem.sanitizeOutputPercent(50.0), 1e-9);
    assertEquals(75.0, ShooterSubsystem.sanitizeOutputPercent(100.0), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(0.0), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(-20.0), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(Double.NaN), 1e-9);
    assertEquals(0.0, ShooterSubsystem.sanitizeOutputPercent(Double.POSITIVE_INFINITY), 1e-9);
  }
}
