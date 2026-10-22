package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;
import org.junit.jupiter.api.Test;

/**
 * Static coordinate-convention regression tests, NOT hardware calibration.
 * Physical sensor is inverted; PhotonVision Input Orientation is expected
 * to rotate the image 180 degrees before AprilTag SolvePnP.
 */
class CameraMountGeometryTest {
  @Test
  void upTwentyFiveDegreesUsesNegativeWpilibPitch() {
    Translation3d opticalAxisRobot = new Translation3d(1, 0, 0)
        .rotateBy(VisionConstants.ROBOT_TO_CAMERA.getRotation());
    double actualElevation = Math.toDegrees(Math.atan2(
        opticalAxisRobot.getZ(),
        Math.hypot(opticalAxisRobot.getX(), opticalAxisRobot.getY())));
    assertEquals(25.0, actualElevation, 1e-8);
    assertEquals(0.0, opticalAxisRobot.getY(), 1e-8);
  }

  @Test
  void processedImageIsExpectedToBeUprightNotDoubleRolled() {
    assertEquals(180.0,
        VisionConstants.EXPECTED_PHOTONVISION_IMAGE_ORIENTATION_DEG, 1e-8);
    assertEquals(0.0, VisionConstants.CAMERA_ROLL_DEG, 1e-8);
    assertEquals(0.0, VisionConstants.CAMERA_YAW_DEG, 1e-8);
  }

  @Test
  void knownHeightTargetRoundTripsThroughRobotCamera3dTransform() {
    Pose3d knownTagRobot = new Pose3d(
        new Translation3d(2.0, 0.3, 1.124),
        new Rotation3d(0, 0, Math.PI));
    Pose3d cameraRobot = new Pose3d().transformBy(
        VisionConstants.ROBOT_TO_CAMERA);
    Transform3d idealCameraToTag = new Transform3d(cameraRobot, knownTagRobot);
    Pose3d recoveredTag = new Pose3d()
        .transformBy(VisionConstants.ROBOT_TO_CAMERA)
        .transformBy(idealCameraToTag);
    assertEquals(1.124, recoveredTag.getZ(), 1e-8);
    assertEquals(2.0, recoveredTag.getX(), 1e-8);
    assertEquals(0.3, recoveredTag.getY(), 1e-8);
    assertTrue(Double.isFinite(recoveredTag.getRotation().getZ()));
  }
}
