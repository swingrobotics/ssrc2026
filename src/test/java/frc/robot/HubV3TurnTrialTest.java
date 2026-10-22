package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

class HubV3TurnTrialTest {
  @Test
  void rotationOutputNeverExceedsSlowTrialLimit() {
    assertEquals(0.08, RobotContainer.capV3TrialTurn(0.9), 1e-9);
    assertEquals(-0.08, RobotContainer.capV3TrialTurn(-0.9), 1e-9);
    assertEquals(0.02, RobotContainer.capV3TrialTurn(0.02), 1e-9);
    assertEquals(0.0, RobotContainer.capV3TrialTurn(Double.NaN), 1e-9);
    assertEquals(0.0, RobotContainer.capV3TrialTurn(Double.POSITIVE_INFINITY), 1e-9);
  }
}
