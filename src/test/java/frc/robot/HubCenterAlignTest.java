package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

/** Pure geometric tests for the new fused-pose HUB controller. */
class HubCenterAlignTest {
  @Test
  void aimsFromRobotFieldPoseToHubCenter() {
    Pose2d robot = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(20));
    Rotation2d aim = HubCenterAlign.desiredHeading(
        robot, new Translation2d(5.0, 4.0),
        new Translation2d(), Rotation2d.kZero, 0.0);
    assertEquals(45.0, aim.getDegrees(), 1e-6);
  }

  @Test
  void updatesAimAngleWithLateralTranslation() {
    Pose2d moved = new Pose2d(3.0, 3.0, Rotation2d.kZero);
    Rotation2d aim = HubCenterAlign.desiredHeading(
        moved, new Translation2d(5.0, 4.0),
        new Translation2d(), Rotation2d.kZero, 0.0);
    assertEquals(Math.toDegrees(Math.atan2(1.0, 2.0)), aim.getDegrees(), 1e-6);
  }

  @Test
  void launcherYawAndTrimAreAppliedWithCorrectSigns() {
    Pose2d robot = Pose2d.kZero;
    Rotation2d aim = HubCenterAlign.desiredHeading(
        robot, new Translation2d(5.0, 0.0),
        new Translation2d(), Rotation2d.fromDegrees(15), 5.0);
    assertEquals(-10.0, aim.getDegrees(), 1e-6);
  }

  @Test
  void launcherOffsetInFieldSpaceChangesBearing() {
    Pose2d robot = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(90));
    Rotation2d aim = HubCenterAlign.desiredHeading(
        robot, new Translation2d(4.0, 4.0),
        new Translation2d(1.0, 0.0), Rotation2d.kZero, 0.0);
    assertEquals(Math.toDegrees(Math.atan2(2.0, 3.0)), aim.getDegrees(), 1e-6);
  }

  @Test
  void anglesWrapContinuously() {
    Rotation2d aim = HubCenterAlign.desiredHeading(
        Pose2d.kZero, new Translation2d(-1.0, 0.01),
        new Translation2d(), Rotation2d.kZero, 12.0);
    assertTrue(aim.getDegrees() < -165.0);
  }

  @Test
  void redAndBlueUseCorrectHubCoordinates() {
    assertEquals(Constants.HubConstants.RED_CENTER_METERS,
        HubCenterAlign.hubForAlliance(Alliance.Red));
    assertEquals(Constants.HubConstants.BLUE_CENTER_METERS,
        HubCenterAlign.hubForAlliance(Alliance.Blue));
  }
}
