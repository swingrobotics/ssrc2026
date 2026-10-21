package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

/** Only the correct alliance's HUB tags can authorize HUB center aiming. */
class HubCenterAlignTest {
  @Test
  void acceptsCorrectAllianceHubTags() {
    for (int id : new int[] {18, 19, 20, 21, 24, 25, 26, 27}) {
      assertTrue(HubCenterAlign.isAllianceHubTag(id, Alliance.Blue));
      assertFalse(HubCenterAlign.isAllianceHubTag(id, Alliance.Red));
    }
    for (int id : new int[] {2, 3, 4, 5, 8, 9, 10, 11}) {
      assertTrue(HubCenterAlign.isAllianceHubTag(id, Alliance.Red));
      assertFalse(HubCenterAlign.isAllianceHubTag(id, Alliance.Blue));
    }
  }

  @Test
  void positiveTrimTurnsTargetCounterclockwise() {
    assertEquals(
        Math.toRadians(6.0),
        HubCenterAlign.desiredHeadingRadians(0.0, 0.0, 6.0),
        1e-9);
    assertEquals(
        Math.toRadians(-6.0),
        HubCenterAlign.desiredHeadingRadians(0.0, 0.0, -6.0),
        1e-9);
  }

  @Test
  void launcherYawOffsetAndHeadingWrapping() {
    assertEquals(
        Math.toRadians(-10.0),
        HubCenterAlign.desiredHeadingRadians(
            Math.toRadians(15.0), Math.toRadians(25.0), 0.0),
        1e-9);
    assertEquals(
        Math.toRadians(-171.0),
        HubCenterAlign.desiredHeadingRadians(Math.toRadians(179.0), 0.0, 10.0),
        1e-9);
  }

  @Test
  void excessiveTrimIsBoundedAndInvalidTrimResetsToZero() {
    HubCenterAlign align = new HubCenterAlign();
    align.setAimTrimDegrees(100.0);
    assertEquals(15.0, align.getAimTrimDegrees(), 1e-9);
    align.setAimTrimDegrees(-100.0);
    assertEquals(-15.0, align.getAimTrimDegrees(), 1e-9);
    align.setAimTrimDegrees(Double.NaN);
    assertEquals(0.0, align.getAimTrimDegrees(), 1e-9);
  }

  @Test
  void ignoresNonHubTags() {
    for (int id : new int[] {-1, 0, 1, 6, 13, 17, 28, 99}) {
      assertFalse(HubCenterAlign.isAllianceHubTag(id, Alliance.Blue));
      assertFalse(HubCenterAlign.isAllianceHubTag(id, Alliance.Red));
    }
  }
}
