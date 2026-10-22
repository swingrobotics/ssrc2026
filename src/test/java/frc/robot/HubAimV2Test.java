package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HubConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

class HubAimV2Test {
  private static List<TargetCorner> corners() {
    return List.of(new TargetCorner(0, 0), new TargetCorner(1, 0),
        new TargetCorner(1, 1), new TargetCorner(0, 1));
  }
  private static PhotonTrackedTarget tag(int id, Pose3d robot) {
    Transform3d cameraToTag = new Transform3d(
        robot.transformBy(VisionConstants.ROBOT_TO_CAMERA),
        VisionConstants.FIELD_LAYOUT.getTagPose(id).orElseThrow());
    return new PhotonTrackedTarget(0, 0, 0, 0, id, -1, 0.0f,
        cameraToTag, Transform3d.kZero, 0.05,
        corners(), new ArrayList<>());
  }

  @Test
  void everyOfficialHubTagComputesItsOwnAllianceHubCenter() {
    int[] ids = {18,19,20,21,24,25,26,27,2,3,4,5,8,9,10,11};
    for (int id : ids) {
      boolean blue = HubAimTargetV2.allianceForTag(id).orElseThrow() == Alliance.Blue;
      Pose3d robot = new Pose3d(blue ? 3.4 : 13.0, 4.1, 0,
          new Rotation3d(0, 0, Math.toRadians(20)));
      Transform3d cameraToTag = new Transform3d(
          robot.transformBy(VisionConstants.ROBOT_TO_CAMERA),
          VisionConstants.FIELD_LAYOUT.getTagPose(id).orElseThrow());
      Translation2d actual = HubAimTargetV2.hubRelativeToRobot(id, cameraToTag,
          VisionConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_CAMERA).orElseThrow();
      Translation2d expected = (blue
          ? HubConstants.BLUE_CENTER_METERS : HubConstants.RED_CENTER_METERS)
          .minus(robot.getTranslation().toTranslation2d())
          .rotateBy(Rotation2d.fromDegrees(-20));
      assertEquals(expected.getX(), actual.getX(), 1e-5);
      assertEquals(expected.getY(), actual.getY(), 1e-5);
    }
  }

  @Test
  void rejectsAbsentPoseAndNonHubTargets() {
    assertFalse(HubAimTargetV2.hubRelativeToRobot(18, Transform3d.kZero,
        VisionConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_CAMERA).isPresent());
    assertFalse(HubAimTargetV2.hubRelativeToRobot(1,
        new Transform3d(1, 0, 0, Rotation3d.kZero),
        VisionConstants.FIELD_LAYOUT, VisionConstants.ROBOT_TO_CAMERA).isPresent());
  }

  @Test
  void singleTagWorksWithoutFieldSeedButExpiresAtQuarterSecond() {
    HubAimTargetV2 tracker = new HubAimTargetV2(VisionConstants.FIELD_LAYOUT);
    tracker.observe(List.of(tag(18, new Pose3d(3.4,4.1,0,new Rotation3d()))),
        Optional.of(Alliance.Blue), Pose2d.kZero, 30.0, 30.05);
    assertTrue(tracker.isFresh(30.10));
    assertEquals(1, tracker.getLastTagCount());
    assertFalse(tracker.isFresh(30.26));
    assertTrue(tracker.getHubInOdometryFrame(30.26).isEmpty());
  }

  @Test
  void twoConflictingTagsAndStaleCameraFramesAreRejected() {
    Pose3d robot = new Pose3d(3.4,4.1,0,new Rotation3d());
    PhotonTrackedTarget good = tag(18, robot);
    PhotonTrackedTarget other = tag(19, robot);
    Transform3d wrong = new Transform3d(
        other.getBestCameraToTarget().getTranslation().plus(
            new Translation3d(1.2, 0, 0)),
        other.getBestCameraToTarget().getRotation());
    PhotonTrackedTarget bad = new PhotonTrackedTarget(0,0,0,0,
        19,-1,0.0f,wrong,Transform3d.kZero,0.05,
        corners(),new ArrayList<>());
    HubAimTargetV2 tracker = new HubAimTargetV2(VisionConstants.FIELD_LAYOUT);
    tracker.observe(List.of(good,bad), Optional.of(Alliance.Blue),
        Pose2d.kZero,40.0,40.05);
    assertFalse(tracker.isFresh(40.1));
    tracker.observe(List.of(good), Optional.of(Alliance.Blue),
        Pose2d.kZero,40.0,40.6);
    assertFalse(tracker.isFresh(40.6));
  }

  @Test
  void perTagDiagnosticsIdentifyInvalidAmbiguityWithoutRelaxingTheGate() {
    Pose3d robot = new Pose3d(3.4, 4.1, 0, new Rotation3d());
    PhotonTrackedTarget good = tag(25, robot);
    PhotonTrackedTarget missingAmbiguity = new PhotonTrackedTarget(0,0,0,0,
        25,-1,0.0f,good.getBestCameraToTarget(),Transform3d.kZero,-1.0,
        corners(),new ArrayList<>());
    HubAimTargetV2 tracker = new HubAimTargetV2(VisionConstants.FIELD_LAYOUT);
    tracker.observe(List.of(missingAmbiguity), Optional.of(Alliance.Blue),
        Pose2d.kZero, 10.0, 10.05);
    assertEquals("NO VALID INDIVIDUAL 3D HUB TAG", tracker.getStatus());
    assertTrue(tracker.getTag25Diagnostics().contains("INVALID_AMBIGUITY"));
    assertTrue(tracker.getTag25Diagnostics().contains("ambiguity=-1.00000"));
    assertEquals(0, tracker.getValidCandidateCount());
    assertTrue(tracker.getTag26Diagnostics().contains("NOT_SEEN_THIS_FRAME"));
    assertEquals("NO VALID INDIVIDUAL 3D HUB TAG", tracker.getLastRejectionReason());
  }

  @Test
  void diagnosticsExposeMissing3DTransformAndValidIndividualTag() {
    Pose3d robot = new Pose3d(3.4, 4.1, 0, new Rotation3d());
    PhotonTrackedTarget missing = new PhotonTrackedTarget(0,0,0,0,
        26,-1,0.0f,Transform3d.kZero,Transform3d.kZero,0.05,
        corners(),new ArrayList<>());
    HubAimTargetV2 tracker = new HubAimTargetV2(VisionConstants.FIELD_LAYOUT);
    tracker.observe(List.of(missing), Optional.of(Alliance.Blue),
        Pose2d.kZero, 20.0, 20.05);
    assertTrue(tracker.getTag26Diagnostics().contains("INVALID_TRANSFORM"));
    assertEquals(0, tracker.getValidCandidateCount());
    tracker.observe(List.of(tag(25, robot)), Optional.of(Alliance.Blue),
        Pose2d.kZero, 20.10, 20.15);
    assertTrue(tracker.getTag25Diagnostics().contains("=> ACCEPTED"));
    assertEquals(1, tracker.getValidCandidateCount());
    assertTrue(tracker.isFresh(20.16));
  }

  @Test
  void diagnosticsReportAllianceMismatchAndStaleFrames() {
    Pose3d robot = new Pose3d(3.4, 4.1, 0, new Rotation3d());
    HubAimTargetV2 tracker = new HubAimTargetV2(VisionConstants.FIELD_LAYOUT);
    tracker.observe(List.of(tag(25, robot)), Optional.of(Alliance.Red),
        Pose2d.kZero, 30.0, 30.05);
    assertTrue(tracker.getTag25Diagnostics().contains("ALLIANCE_MISMATCH"));
    tracker.observe(List.of(tag(26, robot)), Optional.of(Alliance.Blue),
        Pose2d.kZero, 30.10, 30.90);
    assertTrue(tracker.getTag26Diagnostics().contains("STALE_FRAME"));
  }

  @Test
  void rotationWrapsAndFollowsTranslation() {
    assertEquals(45.0, HubAimHeadingV2.desiredHeading(
        Pose2d.kZero,new Translation2d(2,2),Translation2d.kZero,
        Rotation2d.kZero,0).getDegrees(),1e-6);
    assertEquals(Math.toDegrees(Math.atan2(1,2)), HubAimHeadingV2.desiredHeading(
        new Pose2d(0,1,Rotation2d.kZero),new Translation2d(2,2),
        Translation2d.kZero,Rotation2d.kZero,0).getDegrees(),1e-6);
    assertEquals(-10.0, HubAimHeadingV2.desiredHeading(
        Pose2d.kZero,new Translation2d(4,0),Translation2d.kZero,
        Rotation2d.fromDegrees(15),5).getDegrees(),1e-6);
    assertTrue(HubAimHeadingV2.desiredHeading(
        Pose2d.kZero,new Translation2d(-1,0.01),Translation2d.kZero,
        Rotation2d.kZero,12).getDegrees() < -165.0);
  }
}
