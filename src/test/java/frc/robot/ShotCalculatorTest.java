package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.ShotCalculator.CalibrationPoint;
import org.junit.jupiter.api.Test;

/** Pure interpolation tests: no robot hardware or sensors required. */
class ShotCalculatorTest {
  private static final CalibrationPoint NEAR = new CalibrationPoint(2.0, 1200.0, 0.25);
  private static final CalibrationPoint FAR = new CalibrationPoint(3.0, 2400.0, 0.35);

  @Test
  void interpolatesEndpointsAndMidpoint() {
    var near = ShotCalculator.interpolate(2.0, NEAR, FAR).orElseThrow();
    assertEquals(1200.0, near.motorRpm(), 1e-9);
    assertEquals(0.25, near.hoodPosition(), 1e-9);

    var mid = ShotCalculator.interpolate(2.5, NEAR, FAR).orElseThrow();
    assertEquals(1800.0, mid.motorRpm(), 1e-9);
    assertEquals(0.30, mid.hoodPosition(), 1e-9);

    var far = ShotCalculator.interpolate(3.0, NEAR, FAR).orElseThrow();
    assertEquals(2400.0, far.motorRpm(), 1e-9);
    assertEquals(0.35, far.hoodPosition(), 1e-9);
  }

  @Test
  void supportsReversedCalibrationPointOrder() {
    assertEquals(
        1800.0,
        ShotCalculator.interpolate(2.5, FAR, NEAR).orElseThrow().motorRpm(),
        1e-9);
  }

  @Test
  void rejectsUncalibratedAndOutOfRangeRequests() {
    assertFalse(ShotCalculator.interpolate(2.5,
        new CalibrationPoint(2.0, 0.0, -1.0), FAR).isPresent());
    assertFalse(ShotCalculator.interpolate(1.99, NEAR, FAR).isPresent());
    assertFalse(ShotCalculator.interpolate(3.01, NEAR, FAR).isPresent());
    assertFalse(ShotCalculator.interpolate(Double.NaN, NEAR, FAR).isPresent());
    assertFalse(ShotCalculator.interpolate(Double.POSITIVE_INFINITY, NEAR, FAR).isPresent());
  }

  @Test
  void rejectsInvalidCalibrationPoints() {
    assertFalse(ShotCalculator.areValidPair(NEAR,
        new CalibrationPoint(2.0, 2400.0, 0.35)));
    assertFalse(ShotCalculator.areValidPair(NEAR,
        new CalibrationPoint(3.0, 9000.0, 0.35)));
    assertFalse(ShotCalculator.areValidPair(NEAR,
        new CalibrationPoint(3.0, 2200.0, 0.5)));
    assertTrue(ShotCalculator.areValidPair(NEAR, FAR));
  }
}
