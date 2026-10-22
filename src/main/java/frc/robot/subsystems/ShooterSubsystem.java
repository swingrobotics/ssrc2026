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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Percent-output-only shooter; measured shaft RPM is used for feeder direction,
 * never to convert an RPM target into motor duty cycle.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooter9 =
      new SparkMax(ShooterConstants.SHOOTER_9_ID, MotorType.kBrushless);
  private final SparkMax shooter10 =
      new SparkMax(ShooterConstants.SHOOTER_10_ID, MotorType.kBrushless);
  private final SparkMax shooter12 =
      new SparkMax(ShooterConstants.SHOOTER_12_ID, MotorType.kBrushless);
  private final RelativeEncoder shooter9Encoder = shooter9.getEncoder();
  private double commandedPercent = 0.0;

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
    shooterTab.addDouble("Shooter Command (%)", this::getOutputPercent);
    shooterTab.addBoolean("Shooter Running", this::isRunning);
  }

  /** Request percent output (0-75), preserving the original negative direction. */
  public void setOutputPercent(double requestedPercent) {
    if (DriverStation.isDisabled()
        || !Double.isFinite(requestedPercent)
        || requestedPercent <= 0.0) {
      stopShooter();
      return;
    }
    commandedPercent = MathUtil.clamp(
        requestedPercent, 0.0, ShooterConstants.MAX_MOTOR_OUTPUT * 100.0);
    shooter9.set(ShooterConstants.MOTOR_DIRECTION * commandedPercent / 100.0);
  }

  public void runShooter() {
    setOutputPercent(ShooterConstants.DEFAULT_OUTPUT_PERCENT);
  }

  public double getOutputPercent() {
    return commandedPercent;
  }

  public boolean isRunning() {
    return commandedPercent > 0.0;
  }

  /** Encoder shaft RPM; positive indicates normal shooting direction. */
  public double getActualRPM() {
    return shooter9Encoder.getVelocity() * ShooterConstants.MOTOR_DIRECTION;
  }

  /** Positive measured RPM must reach the calibrated target minus tolerance. */
  public boolean isAtTargetRPM(double targetRPM, double toleranceRPM) {
    double rpm = getActualRPM();
    return DriverStation.isEnabled() && isRunning()
        && Double.isFinite(targetRPM) && Double.isFinite(toleranceRPM)
        && targetRPM > toleranceRPM && toleranceRPM > 0.0
        && Double.isFinite(rpm) && rpm >= targetRPM - toleranceRPM;
  }

  public void stopShooter() {
    commandedPercent = 0.0;
    shooter9.stopMotor();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && isRunning()) {
      stopShooter();
    }
  }
}
