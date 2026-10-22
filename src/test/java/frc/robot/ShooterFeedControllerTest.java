package frc.robot;

import static frc.robot.ShooterFeedController.Direction.FORWARD;
import static frc.robot.ShooterFeedController.Direction.REVERSE;
import static frc.robot.ShooterFeedController.Direction.STOP;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

class ShooterFeedControllerTest {
  @Test
  void rampsInReverseThenFeedsForwardAtTargetBand() {
    assertEquals(REVERSE, ShooterFeedController.select(true, 500, 3000, 150, false));
    assertEquals(REVERSE, ShooterFeedController.select(true, 2849, 3000, 150, false));
    assertEquals(FORWARD, ShooterFeedController.select(true, 2850, 3000, 150, false));
  }

  @Test
  void appliesHysteresisWhileForward() {
    assertEquals(FORWARD, ShooterFeedController.select(true, 2750, 3000, 150, true));
    assertEquals(REVERSE, ShooterFeedController.select(true, 2699, 3000, 150, true));
  }

  @Test
  void reverseOnlyWhenTargetUncalibratedAndStopForInvalidReadings() {
    assertEquals(STOP, ShooterFeedController.select(false, 3000, 3000, 150, true));
    assertEquals(REVERSE, ShooterFeedController.select(true, 0, 0, 150, false));
    assertEquals(REVERSE, ShooterFeedController.select(true, 500, 0, 150, true));
    assertEquals(STOP, ShooterFeedController.select(true, 500, -1, 150, false));
    assertEquals(STOP, ShooterFeedController.select(true, 500, Double.NaN, 150, false));
    assertEquals(STOP, ShooterFeedController.select(true, 500, 200, 150, false));
    assertEquals(STOP, ShooterFeedController.select(true, Double.NaN, 3000, 150, false));
    assertEquals(STOP, ShooterFeedController.select(true, -100, 3000, 150, false));
    assertEquals(STOP, ShooterFeedController.select(true, 500, 3000, Double.NaN, false));
  }
}
