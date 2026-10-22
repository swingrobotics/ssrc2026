package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;
import frc.robot.Constants.ShooterConstants;

class ShooterVelocitySetpointTest {
  @Test
  void zeroInvalidOrTooHighTargetIsDisarmed() {
    assertEquals(0.0, ShooterVelocitySetpoint.sanitizeRPM(0.0));
    assertEquals(0.0, ShooterVelocitySetpoint.sanitizeRPM(-20.0));
    assertEquals(0.0, ShooterVelocitySetpoint.sanitizeRPM(Double.NaN));
    assertEquals(0.0, ShooterVelocitySetpoint.sanitizeRPM(Double.POSITIVE_INFINITY));
    assertEquals(0.0, ShooterVelocitySetpoint.sanitizeRPM(ShooterConstants.MAX_TARGET_RPM + 1));
    assertEquals(4500.0, ShooterVelocitySetpoint.sanitizeRPM(4500.0));
  }

  @Test
  void readinessChecksBothSidesOfBand() {
    assertTrue(ShooterVelocitySetpoint.isAtSpeed(3910, 4000, 100));
    assertTrue(ShooterVelocitySetpoint.isAtSpeed(4090, 4000, 100));
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(3899, 4000, 100));
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(4101, 4000, 100));
  }

  @Test
  void invalidRPMAndToleranceNeverBecomeReady() {
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(Double.NaN, 4000, 100));
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(-4000, 4000, 100));
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(0, 0, 100));
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(4000, 4000, Double.NaN));
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(4000, 4000, 0));
    assertFalse(ShooterVelocitySetpoint.isAtSpeed(4000, 4000, 4000));
  }
}
