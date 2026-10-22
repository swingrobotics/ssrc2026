package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.Test;

class TeleopFeederControlTest {
  @Test
  void aFeedsWithoutRequiringShooterOrB() {
    assertEquals(0.70, TeleopFeederControl.selectOutput(true, true, false, 70.0), 1e-9);
    assertEquals(0.70, TeleopFeederControl.selectOutput(true, true, true, 70.0), 1e-9);
    assertEquals(0.0, TeleopFeederControl.selectOutput(true, false, false, 70.0), 1e-9);
  }

  @Test
  void dashboardTestIsIndependentAndOnlyDuringTeleop() {
    assertEquals(0.45, TeleopFeederControl.selectOutput(true, false, true, 45.0), 1e-9);
    assertEquals(0.0, TeleopFeederControl.selectOutput(false, true, true, 90.0), 1e-9);
  }

  @Test
  void testsStopExactlyAtDeadlineAndNeverExtendAutomatically() {
    assertTrue(TeleopFeederControl.isDashboardTestActive(10.0, 12.0));
    assertTrue(TeleopFeederControl.isDashboardTestActive(11.99, 12.0));
    assertFalse(TeleopFeederControl.isDashboardTestActive(12.0, 12.0));
    assertFalse(TeleopFeederControl.isDashboardTestActive(10.0, -1.0));
    assertFalse(TeleopFeederControl.isDashboardTestActive(Double.NaN, 12.0));
  }

  @Test
  void invalidOrZeroSliderValuesStopTheFeeder() {
    assertEquals(0.0, TeleopFeederControl.selectOutput(true,true,false,0.0), 1e-9);
    assertEquals(0.0, TeleopFeederControl.selectOutput(true,true,false,Double.NaN), 1e-9);
    assertEquals(0.0, TeleopFeederControl.selectOutput(true,true,false,-20), 1e-9);
    assertEquals(1.0, TeleopFeederControl.selectOutput(true,true,false,120), 1e-9);
  }
}
