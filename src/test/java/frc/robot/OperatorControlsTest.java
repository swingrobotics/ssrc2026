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
  void singleControllerPortRetainsArmSafetyLimits() {
    assertEquals(0, OIConstants.kDriverControllerPort);
    assertTrue(IntakeArmConstants.MAX_OUTPUT <= 0.20);
    assertTrue(IntakeArmConstants.MAX_CONTINUOUS_RUN_SECONDS <= 1.5);
  }

  @Test
  void allTeleopMechanismsUseDriverController() throws IOException {
    String container = Files.readString(
        Path.of("src", "main", "java", "frc", "robot", "RobotContainer.java"));
    assertFalse(container.contains("m_operatorController"));
    assertFalse(container.contains("new XboxController(OIConstants.kOperatorControllerPort)"));

    assertTrue(container.contains("m_driverController.getAButton()"));
    assertTrue(container.contains("m_driverController.getBButton()"));
    assertTrue(container.contains("XboxController.Button.kX.value"));
    assertTrue(container.contains("XboxController.Button.kY.value"));

    // LB = intake forward, POV left = reverse, POV up/down = intake arm.
    assertTrue(container.contains("m_driverController.getLeftBumper()"));
    assertTrue(container.contains("m_driverController.getPOV() != 270"));
    assertTrue(container.contains("m_driverController.getPOV() == 270"));
    assertTrue(container.contains("m_intake::runIntake"));
    assertTrue(container.contains("m_intake::reverseIntake"));
    assertTrue(container.contains("m_driverController.getPOV() == 0"));
    assertTrue(container.contains("m_driverController.getPOV() == 180"));
    assertTrue(container.contains("m_intakeArm::moveForward"));
    assertTrue(container.contains("m_intakeArm::moveBackward"));
  }
}
