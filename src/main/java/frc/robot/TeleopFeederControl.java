package frc.robot;

import edu.wpi.first.math.MathUtil;

/** Pure decision logic for the independent TELEOP A/dashboard feeder controls. */
public final class TeleopFeederControl {
  private TeleopFeederControl() {}

  public static final double DASHBOARD_TEST_SECONDS = 2.0;

  public static boolean isDashboardTestActive(double now, double deadline) {
    return Double.isFinite(now) && Double.isFinite(deadline)
        && deadline >= 0.0 && now >= 0.0 && now < deadline;
  }

  public static double selectOutput(
      boolean teleopEnabled, boolean aPressed,
      boolean dashboardTestActive, double percentOutput) {
    if (!teleopEnabled || !(aPressed || dashboardTestActive)
        || !Double.isFinite(percentOutput)) return 0.0;
    return MathUtil.clamp(percentOutput, 0.0, 100.0) / 100.0;
  }
}
