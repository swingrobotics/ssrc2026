package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HubConstants;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** Pure field-geometry tests; not physical accuracy verification. */
class HubDistanceCalculatorTest {
  @Test
  void usesSelectedAllianceHubCenter() {
    var blue = HubDistanceCalculator.calculate(
        new Pose2d(2.625, 4.033, new Rotation2d()), Optional.of(Alliance.Blue));
    assertEquals(2.0, blue.orElseThrow(), 1e-9);

    var red = HubDistanceCalculator.calculate(
        new Pose2d(13.920, 4.033, new Rotation2d()), Optional.of(Alliance.Red));
    assertEquals(2.0, red.orElseThrow(), 1e-9);
  }

  @Test
  void calculatesLocalTagBasedHubDistanceWithoutFieldPose() {
    var robot = new Pose2d(1.0, 2.0, new Rotation2d());
    var hub = new Translation2d(4.0, 2.0);
    assertEquals(3.0, HubDistanceCalculator.calculateFromLocalHub(robot, hub)
        .orElseThrow(), 1e-9);
    assertFalse(HubDistanceCalculator.calculateFromLocalHub(
        robot, new Translation2d(Double.NaN, 2.0)).isPresent());
    assertFalse(HubDistanceCalculator.calculateFromLocalHub(
        robot, new Translation2d(1.0, 2.0)).isPresent());
  }

  @Test
  void rejectsUnknownAllianceInvalidAndOutOfFieldPoses() {
    var robot = new Pose2d(2.625, 4.033, new Rotation2d());
    assertFalse(HubDistanceCalculator.calculate(robot, Optional.empty()).isPresent());
    assertFalse(HubDistanceCalculator.calculate(
        new Pose2d(-10.0, 4.033, new Rotation2d()),
        Optional.of(Alliance.Blue)).isPresent());
    assertFalse(HubDistanceCalculator.calculate(
        new Pose2d(Double.NaN, 4.033, new Rotation2d()),
        Optional.of(Alliance.Blue)).isPresent());
    assertTrue(HubDistanceCalculator.calculate(robot, Optional.of(Alliance.Blue)).isPresent());
  }
}
