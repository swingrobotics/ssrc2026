package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Dual-mode control for shooter motor 9 (leader): driver percent output
 * and optional closed-loop velocity RPM for measured distance calibration.
 * Motors 10 and 12 retain their existing hardware follower relationships:
 * 10 follows the leader, 12 follows in the opposite direction.
 *
 * All displayed RPM values refer to MOTOR SHAFT RPM, not shooter wheel RPM
 * when an external gear or belt reduction is present. The follower motors
 * follow the leader OUTPUT; they do not have their own velocity loops.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooter9 =
      new SparkMax(ShooterConstants.SHOOTER_9_ID, MotorType.kBrushless);
  private final SparkMax shooter10 =
      new SparkMax(ShooterConstants.SHOOTER_10_ID, MotorType.kBrushless);
  private final SparkMax shooter12 =
      new SparkMax(ShooterConstants.SHOOTER_12_ID, MotorType.kBrushless);

  private final SparkClosedLoopController velocityController =
      shooter9.getClosedLoopController();
  private final RelativeEncoder shooter9Encoder = shooter9.getEncoder();
  private final RelativeEncoder shooter10Encoder = shooter10.getEncoder();
  private final RelativeEncoder shooter12Encoder = shooter12.getEncoder();

  private enum ControlMode { STOPPED, PERCENT, RPM }

  private ControlMode mode = ControlMode.STOPPED;
  private double targetRPM = 0.0;
  private double commandedOutputPercent = 0.0;
  private int stableCycles = 0;

  public ShooterSubsystem() {
    SparkMaxConfig shooter9Config = new SparkMaxConfig();
    shooter9Config.inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    shooter9Config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.VELOCITY_KP, 0.0, 0.0)
        .outputRange(-ShooterConstants.MAX_MOTOR_OUTPUT, ShooterConstants.MAX_MOTOR_OUTPUT);
    shooter9Config.closedLoop.feedForward.kV(ShooterConstants.VELOCITY_KV);
    shooter9.configure(shooter9Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig shooter10Config = new SparkMaxConfig();
    shooter10Config.follow(shooter9, false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    shooter10.configure(shooter10Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig shooter12Config = new SparkMaxConfig();
    shooter12Config.follow(shooter9, true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS);
    shooter12.configure(shooter12Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    stopShooter();

    // SmartDashboard topics do not automatically appear as tiles on the
    // Shuffleboard Drive tab. Publish live suppliers on a dedicated tab.
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addString("Control Mode", this::getControlMode);
    shooterTab.addDouble("Command Output (%)", this::getOutputPercent);
    shooterTab.addDouble("Target RPM", this::getTargetRPM);
    shooterTab.addDouble("Actual RPM", this::getActualRPM);
    shooterTab.addDouble("Error RPM", this::getErrorRPM);
    shooterTab.addDouble("Motor 10 RPM", this::getMotor10RPM);
    shooterTab.addDouble("Motor 12 RPM", this::getMotor12RPM);
    shooterTab.addBoolean("At Target RPM", this::isAtSpeed);
    shooterTab.addBoolean("Running", this::isRunning);

    publishTelemetry();
  }

  /**
   * Original operator mode: output is a genuine motor duty-cycle percentage,
   * not an RPM converted into a percentage. A request of 50 means -0.50 to
   * the leader, matching the team's original shooting direction.
   * Clamped to the existing 75% output cap.
   */
  public void setOutputPercent(double requestedPercent) {
    if (DriverStation.isDisabled()
        || !Double.isFinite(requestedPercent)
        || requestedPercent <= 0.0) {
      stopShooter();
      return;
    }

    double percent = MathUtil.clamp(requestedPercent, 0.0,
        ShooterConstants.MAX_MOTOR_OUTPUT * 100.0);
    mode = ControlMode.PERCENT;
    targetRPM = 0.0;
    stableCycles = 0;
    commandedOutputPercent = percent;
    shooter9.set(ShooterConstants.MOTOR_DIRECTION * percent / 100.0);
  }

  /**
   * Optional closed-loop motor-shaft RPM mode for calibrated shots.
   * Negative motor rotation from the original setup is preserved.
   * Invalid, zero or disabled requests stop the mechanism.
   */
  public void setTargetRPM(double requestedRPM) {
    if (DriverStation.isDisabled()
        || !Double.isFinite(requestedRPM)
        || requestedRPM <= 0.0) {
      stopShooter();
      return;
    }

    double safeRPM = MathUtil.clamp(requestedRPM, 0.0, ShooterConstants.MAX_RPM);
    if (mode != ControlMode.RPM || Math.abs(safeRPM - targetRPM) > 0.01) {
      stableCycles = 0;
    }
    mode = ControlMode.RPM;
    commandedOutputPercent = 0.0;
    targetRPM = safeRPM;
    velocityController.setSetpoint(
        ShooterConstants.MOTOR_DIRECTION * targetRPM,
        ControlType.kVelocity);
  }

  public void runShooter() {
    setOutputPercent(ShooterConstants.DEFAULT_OUTPUT_PERCENT);
  }

  public String getControlMode() {
    return mode.name();
  }

  /** Percent of full motor-controller output, not percent of target RPM. */
  public double getOutputPercent() {
    return commandedOutputPercent;
  }

  public boolean isRunning() {
    return mode != ControlMode.STOPPED;
  }

  /** Actual leader shaft RPM, positive in the team's normal shooting direction. */
  public double getActualRPM() {
    return shooter9Encoder.getVelocity() * ShooterConstants.MOTOR_DIRECTION;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getErrorRPM() {
    double actualRPM = getActualRPM();
    return Double.isFinite(actualRPM) ? targetRPM - actualRPM : 0.0;
  }

  public double getMotor10RPM() {
    return shooter10Encoder.getVelocity() * ShooterConstants.MOTOR_DIRECTION;
  }

  public double getMotor12RPM() {
    return -shooter12Encoder.getVelocity() * ShooterConstants.MOTOR_DIRECTION;
  }

  /** Ready after stable leader RPM; this does NOT yet gate the feeder. */
  public boolean isAtSpeed() {
    return mode == ControlMode.RPM && targetRPM > 0.0
        && stableCycles >= ShooterConstants.READY_STABLE_CYCLES;
  }

  /** Stop the leader; its two hardware followers will stop with it. */
  public void stopShooter() {
    mode = ControlMode.STOPPED;
    targetRPM = 0.0;
    commandedOutputPercent = 0.0;
    stableCycles = 0;
    shooter9.stopMotor();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && isRunning()) {
      stopShooter();
    }

    double actualRPM = getActualRPM();
    if (mode == ControlMode.RPM && targetRPM > 0.0
        && Double.isFinite(actualRPM)
        && Math.abs(targetRPM - actualRPM) <= ShooterConstants.RPM_TOLERANCE) {
      stableCycles = Math.min(
          stableCycles + 1, ShooterConstants.READY_STABLE_CYCLES);
    } else {
      stableCycles = 0;
    }
    publishTelemetry();
  }

  private void publishTelemetry() {
    double actualRPM = getActualRPM();
    SmartDashboard.putString("Shooter Control Mode", getControlMode());
    SmartDashboard.putNumber("Shooter Command Output (%)", getOutputPercent());
    SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
    SmartDashboard.putNumber("Shooter Actual RPM", actualRPM);
    SmartDashboard.putNumber("Shooter Error RPM", getErrorRPM());
    SmartDashboard.putNumber("Shooter Motor 10 RPM", getMotor10RPM());
    SmartDashboard.putNumber("Shooter Motor 12 RPM", getMotor12RPM());
    SmartDashboard.putBoolean("Shooter Running", isRunning());
    SmartDashboard.putBoolean("Shooter At Target RPM", isAtSpeed());
  }
}
