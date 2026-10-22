package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.VisionConstants;
import org.junit.jupiter.api.Test;

/** 3D geometry based on actual official 2026 HUB AprilTag field poses. */
class HubRelativeTargetTest {
  private void checkSameHubCenterFromTag(
      int id, Translation2d expectedHub, Translation3d robotFieldTranslation,
      Rotation3d robotRotation) {
    Pose3d robotField = new Pose3d(robotFieldTranslation, robotRotation);
    Pose3d cameraField = robotField.transformBy(VisionConstants.ROBOT_TO_CAMERA);
    Pose3d tagField = VisionConstants.FIELD_LAYOUT.getTagPose(id).orElseThrow();
    Transform3d cameraToTag = new Transform3d(cameraField, tagField);

    var hubRobot = HubRelativeTarget.hubRelativeToRobot(
        id, cameraToTag, VisionConstants.FIELD_LAYOUT,
        VisionConstants.ROBOT_TO_CAMERA);
    assertTrue(hubRobot.isPresent(), "3D target should be solvable: id " + id);

    // Robot-relative XY direction is independent of the robot's global
    // localization initialization (or absolute coordinate origin).
    Translation2d expectedRobot = expectedHub.minus(
        robotFieldTranslation.toTranslation2d()).rotateBy(
            robotField.getRotation().toRotation2d().unaryMinus());
    assertEquals(expectedRobot.getX(), hubRobot.get().getX(), 1e-5);
    assertEquals(expectedRobot.getY(), hubRobot.get().getY(), 1e-5);
  }

  @Test
  void blueHubCanBeSolvedFromOneTagOnDifferentFaces() {
    for (int id : new int[] {18, 19, 20, 21, 24, 25, 26, 27}) {
      checkSameHubCenterFromTag(
          id, HubConstants.BLUE_CENTER_METERS,
          new Translation3d(3.4, 4.1, 0.0),
          new Rotation3d(0, 0, Math.toRadians(25)));
    }
  }

  @Test
  void redHubCanBeSolvedFromOneTagOnDifferentFaces() {
    for (int id : new int[] {2, 3, 4, 5, 8, 9, 10, 11}) {
      checkSameHubCenterFromTag(
          id, HubConstants.RED_CENTER_METERS,
          new Translation3d(13.0, 4.1, 0.0),
          new Rotation3d(0, 0, Math.toRadians(-45)));
    }
  }

  @Test
  void zeroCameraTargetTransformIsRejected() {
    assertFalse(HubRelativeTarget.hubRelativeToRobot(
        18, Transform3d.kZero, VisionConstants.FIELD_LAYOUT,
        VisionConstants.ROBOT_TO_CAMERA).isPresent());
  }

  @Test
  void nonHubTagDoesNotBecomeAHubAimTarget() {
    assertFalse(HubRelativeTarget.hubRelativeToRobot(
        1, new Transform3d(1, 0, 0, Rotation3d.kZero),
        VisionConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_CAMERA).isPresent());
  }

  @Test
  void allianceComesFromDetectedHubTagIdWithoutDriverStationAlliance() {
    assertEquals(Alliance.Blue, HubRelativeTarget.allianceForTag(18).orElseThrow());
    assertEquals(Alliance.Red, HubRelativeTarget.allianceForTag(2).orElseThrow());
    assertTrue(HubRelativeTarget.allianceForTag(1).isEmpty());
  }

  @Test
  void knownHubCenterIsIndependentOfRobotPitchAndCameraPitch() {
    checkSameHubCenterFromTag(
        18, HubConstants.BLUE_CENTER_METERS,
        new Translation3d(3.2, 3.8, 0.0),
        new Rotation3d(0.0, 0.0, 0.0));
  }
}
