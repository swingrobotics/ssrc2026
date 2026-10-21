package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

/** The camera must not overwrite a valid gyro/odometry pose with bad PnP. */
class VisionPoseValidatorTest {
  private static final double NOW = 100.0;
  private static final double CAPTURE = 99.95;
  private static final double FIELD_LENGTH = 16.5;
  private static final double FIELD_WIDTH = 8.1;

  private VisionPoseValidator.Decision validate(
      Pose3d vision, Pose2d fused, boolean initialized, boolean multitag,
      int tags, double distance, double ambiguity) {
    return VisionPoseValidator.validate(
        vision, CAPTURE, NOW, fused, initialized, multitag,
        tags, distance, ambiguity, 0.0, 0.0,
        FIELD_LENGTH, FIELD_WIDTH, true);
  }

  @Test
  void qualityMultitagCanInitializeFieldPose() {
    var decision = validate(
        new Pose3d(4, 4, 0, new Rotation3d()),
        Pose2d.kZero, false, true, 3, 2.0, -1.0);
    assertTrue(decision.accepted());
    assertTrue(decision.canInitialize());
  }

  @Test
  void singleTagCannotDefineInitialGlobalHeading() {
    var decision = validate(
        new Pose3d(4, 4, 0, new Rotation3d()),
        Pose2d.kZero, false, false, 1, 2.0, 0.1);
    assertFalse(decision.accepted());
  }

  @Test
  void validSingleTagCanCorrectExistingFusedPose() {
    var decision = validate(
        new Pose3d(4.1, 4, 0, new Rotation3d()),
        new Pose2d(4, 4, edu.wpi.first.math.geometry.Rotation2d.kZero),
        true, false, 1, 2.0, 0.1);
    assertTrue(decision.accepted());
    assertFalse(decision.canInitialize());
  }

  @Test
  void outlierPoseIsRejectedInsteadOfJumpingFieldPosition() {
    var decision = validate(
        new Pose3d(10, 4, 0, new Rotation3d()),
        new Pose2d(4, 4, edu.wpi.first.math.geometry.Rotation2d.kZero),
        true, true, 3, 2.0, -1.0);
    assertFalse(decision.accepted());
    assertEquals("VISION/ODOMETRY POSITION DISAGREE", decision.reason());
  }

  @Test
  void outOfFieldPoseRejected() {
    var decision = validate(
        new Pose3d(-2.0, 4, 0, new Rotation3d()),
        Pose2d.kZero, false, true, 2, 2.0, -1.0);
    assertFalse(decision.accepted());
  }

  @Test
  void moreTagsProduceSmallerMeasurementUncertainty() {
    var pose = new Pose3d(4, 4, 0, new Rotation3d());
    double twoTag = validate(pose, Pose2d.kZero, false, true, 2, 2.0, -1.0)
        .xyStdDevMeters();
    double fourTag = validate(pose, Pose2d.kZero, false, true, 4, 2.0, -1.0)
        .xyStdDevMeters();
    assertTrue(fourTag < twoTag);
  }

  @Test
  void staleVisionAndCalibratingGyroAreRejected() {
    var vision = new Pose3d(4, 4, 0, new Rotation3d());
    assertFalse(VisionPoseValidator.validate(
        vision, 98.0, NOW, Pose2d.kZero, false, true,
        2, 2.0, -1.0, 0, 0, FIELD_LENGTH, FIELD_WIDTH, true).accepted());
    assertFalse(VisionPoseValidator.validate(
        vision, CAPTURE, NOW, Pose2d.kZero, false, true,
        2, 2.0, -1.0, 0, 0, FIELD_LENGTH, FIELD_WIDTH, false).accepted());
  }
}
