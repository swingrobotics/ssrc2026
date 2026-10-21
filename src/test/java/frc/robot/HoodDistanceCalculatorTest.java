package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.HoodDistanceCalculator.Point;
import org.junit.jupiter.api.Test;

/** Pure hood interpolation tests: numbers here are NOT field calibrations. */
class HoodDistanceCalculatorTest {
  private static final Point NEAR = new Point(2.0, 0.220);
  private static final Point FAR = new Point(4.0, 0.360);

  @Test
  void interpolatesHoodWithoutShooterRpm() {
    assertEquals(0.220, HoodDistanceCalculator.interpolate(2.0, NEAR, FAR)
        .orElseThrow(), 1e-9);
    assertEquals(0.290, HoodDistanceCalculator.interpolate(3.0, NEAR, FAR)
        .orElseThrow(), 1e-9);
    assertEquals(0.360, HoodDistanceCalculator.interpolate(4.0, NEAR, FAR)
        .orElseThrow(), 1e-9);
    assertEquals(0.290, HoodDistanceCalculator.interpolate(3.0, FAR, NEAR)
        .orElseThrow(), 1e-9);
  }

  @Test
  void doesNotExtrapolateOrGuessMissingMeasurements() {
    assertFalse(HoodDistanceCalculator.interpolate(1.99, NEAR, FAR).isPresent());
    assertFalse(HoodDistanceCalculator.interpolate(4.01, NEAR, FAR).isPresent());
    assertFalse(HoodDistanceCalculator.interpolate(Double.NaN, NEAR, FAR).isPresent());
    assertFalse(HoodDistanceCalculator.interpolate(
        3.0, new Point(2.0, -1.0), FAR).isPresent());
  }

  @Test
  void rejectsDuplicateDistancesAndHoodLimits() {
    assertFalse(HoodDistanceCalculator.areValidPair(NEAR, new Point(2.0, 0.36)));
    assertFalse(HoodDistanceCalculator.areValidPair(NEAR, new Point(4.0, 0.5)));
    assertTrue(HoodDistanceCalculator.areValidPair(NEAR, FAR));
  }
}
