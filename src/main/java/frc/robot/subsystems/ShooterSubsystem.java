package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Percent-output ONLY shooter. Motors 10 and 12 remain hardware followers
 * of motor 9. The encoder is read solely to detect settled rotation before
 * feeding, never to command velocity or convert RPM into motor output.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooter9 =
      new SparkMax(ShooterConstants.SHOOTER_9_ID, MotorType.kBrushless);
  private final SparkMax shooter10 =
      new SparkMax(ShooterConstants.SHOOTER_10_ID, MotorType.kBrushless);
  private final SparkMax shooter12 =
      new SparkMax(ShooterConstants.SHOOTER_12_ID, MotorType.kBrushless);
  private final RelativeEncoder shooter9Encoder = shooter9.getEncoder();

  // Internal encoder safety thresholds, NOT shooter speed control settings.
  private static final double MIN_MEASURED_SHAFT_RPM = 300.0;
  private static final double MIN_SPINUP_SECONDS = 0.75;
  private static final int REQUIRED_SETTLED_CYCLES = 8;

  private double commandedPercent = 0.0;
  private double startTimestamp = -1.0;
  private double previousMeasuredRPM = Double.NaN;
  private int settledCycles = 0;

  public ShooterSubsystem() {
    SparkMaxConfig leader = new SparkMaxConfig();
    leader.inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    shooter9.configure(leader, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig follower10 = new SparkMaxConfig();
    follower10.follow(shooter9, false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    shooter10.configure(follower10, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig follower12 = new SparkMaxConfig();
    follower12.follow(shooter9, true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    shooter12.configure(follower12, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    stopShooter();

    var shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addDouble("Shooter Command (%)", this::getOutputPercent)
        .withPosition(0, 2).withSize(2, 1);
    shooterTab.addBoolean("Shooter Running", this::isRunning)
        .withPosition(4, 2);
    shooterTab.addDouble("Shooter Applied (%)", this::getAppliedOutputPercent)
        .withPosition(0, 5).withSize(2, 1);
    shooterTab.addBoolean("Shooter Speed Stable", this::isReadyToFeed)
        .withPosition(6, 1).withSize(2, 1);

    var diagnostics = Shuffleboard.getTab("Mechanism Diagnostics");
    diagnostics.addDouble("Shooter CAN 9 Current (A)", shooter9::getOutputCurrent);
    diagnostics.addDouble("Shooter CAN 10 Current (A)", shooter10::getOutputCurrent);
    diagnostics.addDouble("Shooter CAN 12 Current (A)", shooter12::getOutputCurrent);
    diagnostics.addDouble("Shooter CAN 9 Bus (V)", shooter9::getBusVoltage);
    diagnostics.addDouble("Shooter CAN 9 Applied (V)",
        () -> shooter9.getBusVoltage() * shooter9.getAppliedOutput());
  }

  /**
   * Request genuine motor duty-cycle percentage (0-75), preserving original
   * negative leader direction. Disable or invalid commands always stop.
   */
  /** Simple finite clamp shared by B control and PathPlanner. */
  public static double sanitizeOutputPercent(double value) {
    return Double.isFinite(value)
        ? MathUtil.clamp(value, 0.0, ShooterConstants.MAX_MOTOR_OUTPUT * 100.0)
        : 0.0;
  }

  public void setOutputPercent(double requestedPercent) {
    if (!DriverStation.isEnabled()
        || !Double.isFinite(requestedPercent)
        || requestedPercent <= 0.0) {
      stopShooter();
      return;
    }

    double percent = sanitizeOutputPercent(requestedPercent);
    if (!isRunning() || Math.abs(percent - commandedPercent) > 0.1) {
      startTimestamp = Timer.getFPGATimestamp();
      previousMeasuredRPM = Double.NaN;
      settledCycles = 0;
    }
    commandedPercent = percent;
    shooter9.set(ShooterConstants.MOTOR_DIRECTION * percent / 100.0);
  }

  public void runShooter() {
    setOutputPercent(ShooterConstants.DEFAULT_OUTPUT_PERCENT);
  }

  public double getOutputPercent() {
    return commandedPercent;
  }

  public boolean isRunning() {
    return commandedPercent > 0.0 && DriverStation.isEnabled();
  }

  /** Signed actual motor output; not a target RPM. */
  public double getAppliedOutputPercent() {
    return shooter9.getAppliedOutput() * 100.0;
  }

  /** For internal safety check only: positive in normal shooting direction. */
  public double getActualRPM() {
    return shooter9Encoder.getVelocity() * ShooterConstants.MOTOR_DIRECTION;
  }

  /**
   * Readiness means sustained measured rotation, NOT a guaranteed shot.
   * We do not maintain a target shaft RPM in this percent-output design.
   */
  public boolean isReadyToFeed() {
    return DriverStation.isEnabled()
        && isRunning()
        && startTimestamp >= 0.0
        && Timer.getFPGATimestamp() - startTimestamp >= MIN_SPINUP_SECONDS
        && settledCycles >= REQUIRED_SETTLED_CYCLES
        && Double.isFinite(getActualRPM())
        && getActualRPM() >= MIN_MEASURED_SHAFT_RPM;
  }

  public void stopShooter() {
    commandedPercent = 0.0;
    startTimestamp = -1.0;
    settledCycles = 0;
    previousMeasuredRPM = Double.NaN;
    shooter9.stopMotor();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      if (isRunning()) {
        stopShooter();
      }
      return;
    }
    double measuredRPM = getActualRPM();
    if (!isRunning() || !Double.isFinite(measuredRPM)
        || measuredRPM < MIN_MEASURED_SHAFT_RPM) {
      settledCycles = 0;
      previousMeasuredRPM = Double.NaN;
      return;
    }

    double allowedVariation = Math.max(120.0, Math.abs(measuredRPM) * 0.08);
    if (Double.isFinite(previousMeasuredRPM)
        && Math.abs(measuredRPM - previousMeasuredRPM) <= allowedVariation) {
      settledCycles = Math.min(settledCycles + 1, REQUIRED_SETTLED_CYCLES);
    } else {
      settledCycles = 0;
    }
    previousMeasuredRPM = measuredRPM;
  }
}
