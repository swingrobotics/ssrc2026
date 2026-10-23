package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.HoodConstants;
import org.junit.jupiter.api.Test;

class AutoHoodTargetTest {
  @Test
  void pointBlankAutoUsesMeasuredFullyRaisedMinimumEncoderEndpoint() {
    assertTrue(Double.isFinite(HoodConstants.AUTO_FULL_RAISED_POSITION));
    assertEquals(HoodConstants.MIN_POSITION,
        HoodConstants.AUTO_FULL_RAISED_POSITION, 1e-9);
    assertTrue(HoodConstants.AUTO_FULL_RAISED_POSITION
        >= HoodConstants.MIN_POSITION);
    assertTrue(HoodConstants.AUTO_FULL_RAISED_POSITION
        <= HoodConstants.MAX_POSITION);
  }
}
