package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.OIConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

class OperatorControlsTest {
  @Test
  void operatorUsesASeparateControllerPortAndSafeArmScaling() {
    assertEquals(0, OIConstants.kDriverControllerPort);
    assertEquals(1, OIConstants.kOperatorControllerPort);
    assertTrue(OIConstants.kIntakeArmDeadband > 0.0);
    assertTrue(OIConstants.kIntakeArmDeadband < 1.0);
    assertTrue(IntakeArmConstants.MAX_OUTPUT <= 0.20);
    assertTrue(IntakeArmConstants.MAX_CONTINUOUS_RUN_SECONDS <= 1.5);
  }

  @Test
  void mechanismsAreAssignedToOperatorNotDriver() throws IOException {
    String container = Files.readString(
        Path.of("src", "main", "java", "frc", "robot", "RobotContainer.java"));
    assertTrue(container.contains("m_operatorController.getAButton()"));
    assertTrue(container.contains("m_operatorController.getBButton()"));
    assertTrue(container.contains("m_operatorController.getXButton()"));
    assertTrue(container.contains("m_operatorController.getYButton()"));
    assertTrue(container.contains("&& !m_operatorController.getYButton()"));
    assertTrue(container.contains("m_intake::reverseIntake"));
    assertTrue(container.contains("m_operatorController.getLeftY()"));
    assertFalse(container.contains("m_driverController.getAButton()"));
    assertFalse(container.contains("m_driverController.getBButton()"));
    assertFalse(container.contains("m_driverController.getLeftBumper()"));
    assertFalse(container.contains("m_driverController.getPOV()"));
  }
}
