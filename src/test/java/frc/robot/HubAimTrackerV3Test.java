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

class HubAimTrackerV3Test {
  private static List<TargetCorner> corners() {
    return List.of(new TargetCorner(0, 0), new TargetCorner(1, 0),
        new TargetCorner(1, 1), new TargetCorner(0, 1));
  }

  private static PhotonTrackedTarget tag(int id, Pose3d fieldRobot) {
    Transform3d xform = new Transform3d(
        fieldRobot.transformBy(VisionConstants.ROBOT_TO_CAMERA),
        VisionConstants.FIELD_LAYOUT.getTagPose(id).orElseThrow());
    return new PhotonTrackedTarget(0,0,0,0,id,-1,0.0f,
        xform, Transform3d.kZero, 0.05, corners(), new ArrayList<>());
  }

  @Test
  void fixedHubIsIndependentOfRobotTranslationAndTracksBriefOcclusion() {
    Pose3d fieldRobot = new Pose3d(3.4,4.1,0,
        new Rotation3d(0,0,Math.toRadians(20)));
    Pose2d capture = new Pose2d(0,0,Rotation2d.fromDegrees(20));
    HubAimTrackerV3 v3 = new HubAimTrackerV3(VisionConstants.FIELD_LAYOUT);
    v3.observe(List.of(tag(25, fieldRobot), tag(26, fieldRobot)),
        Optional.of(Alliance.Blue), capture, 100.0, 100.05);
    assertEquals(2, v3.getTagCount());
    assertTrue(v3.hasLiveTag(100.1));
    Translation2d target = v3.getHubInOdometry(100.1).orElseThrow();
    Translation2d expected = HubConstants.BLUE_CENTER_METERS
        .minus(fieldRobot.getTranslation().toTranslation2d());
    assertEquals(expected.getX(), target.getX(), 1e-5);
    assertEquals(expected.getY(), target.getY(), 1e-5);

    Pose2d moved = new Pose2d(0, 1.0, Rotation2d.fromDegrees(20));
    assertEquals(target, v3.getHubInOdometry(100.60).orElseThrow());
    Translation2d relative = v3.getHubInRobot(moved, 100.60).orElseThrow();
    Translation2d expectedRelative = target.minus(moved.getTranslation())
        .rotateBy(moved.getRotation().unaryMinus());
    assertEquals(expectedRelative.getX(), relative.getX(), 1e-5);
    assertEquals(expectedRelative.getY(), relative.getY(), 1e-5);
    assertEquals("ODOMETRY HOLD (NO TAG)", v3.getStatus(100.60));
    assertFalse(v3.getHubInOdometry(100.81).isPresent());
  }

  @Test
  void singleHubTagWorksWithUninitializedGlobalFieldPose() {
    HubAimTrackerV3 v3 = new HubAimTrackerV3(VisionConstants.FIELD_LAYOUT);
    v3.observe(List.of(tag(25, new Pose3d(3.4,4.1,0,new Rotation3d()))),
        Optional.of(Alliance.Blue), Pose2d.kZero, 5.0, 5.05);
    assertTrue(v3.hasLock(5.2));
    assertEquals(1, v3.getTagCount());
  }

  @Test
  void allianceMismatchAndAllianceChangeNeverKeepOpponentHub() {
    HubAimTrackerV3 v3 = new HubAimTrackerV3(VisionConstants.FIELD_LAYOUT);
    PhotonTrackedTarget blue = tag(25,new Pose3d(3.4,4.1,0,new Rotation3d()));
    v3.observe(List.of(blue), Optional.of(Alliance.Red),
        Pose2d.kZero, 10.0, 10.05);
    assertFalse(v3.hasLock(10.1));
    v3.observe(List.of(blue), Optional.of(Alliance.Blue),
        Pose2d.kZero, 10.2, 10.25);
    assertTrue(v3.hasLock(10.3));
    v3.observe(List.of(blue), Optional.of(Alliance.Red),
        Pose2d.kZero, 10.4, 10.45);
    assertFalse(v3.hasLock(10.5));
  }

  @Test
  void conflictingTagsFailClosedRatherThanAveragingIncompatibleCenters() {
    HubAimTrackerV3 v3 = new HubAimTrackerV3(VisionConstants.FIELD_LAYOUT);
    Pose3d robot = new Pose3d(3.4,4.1,0,new Rotation3d());
    PhotonTrackedTarget good = tag(25,robot);
    PhotonTrackedTarget other = tag(26,robot);
    Transform3d shifted = new Transform3d(
        other.getBestCameraToTarget().getTranslation().plus(
            new Translation3d(1.2,0,0)),
        other.getBestCameraToTarget().getRotation());
    PhotonTrackedTarget bad = new PhotonTrackedTarget(0,0,0,0,26,-1,0,
        shifted,Transform3d.kZero,0.05,corners(),new ArrayList<>());
    v3.observe(List.of(good,bad),Optional.of(Alliance.Blue),
        Pose2d.kZero,20.0,20.05);
    assertFalse(v3.hasLock(20.1));
    assertEquals("TAG CENTERS DISAGREE",v3.getLastRejection());
  }

  @Test
  void staleAndMissingOdometryMeasurementsCannotAcquireTarget() {
    HubAimTrackerV3 v3 = new HubAimTrackerV3(VisionConstants.FIELD_LAYOUT);
    PhotonTrackedTarget blue = tag(25,new Pose3d(3.4,4.1,0,new Rotation3d()));
    v3.observe(List.of(blue),Optional.of(Alliance.Blue),
        Pose2d.kZero,30,30.5);
    assertFalse(v3.hasLock(30.5));
    v3.observe(List.of(blue),Optional.of(Alliance.Blue),
        null,31,31.05);
    assertFalse(v3.hasLock(31.1));
    assertEquals(-999.0,v3.getBearingDegrees(Pose2d.kZero,31.1));
  }

  @Test
  void explicitFrameResetInvalidatesFixedTarget() {
    HubAimTrackerV3 v3 = new HubAimTrackerV3(VisionConstants.FIELD_LAYOUT);
    v3.observe(List.of(tag(25,new Pose3d(3.4,4.1,0,new Rotation3d()))),
        Optional.of(Alliance.Blue),Pose2d.kZero,40,40.05);
    assertTrue(v3.hasLock(40.1));
    v3.clear();
    assertFalse(v3.hasLock(40.2));
  }
}
