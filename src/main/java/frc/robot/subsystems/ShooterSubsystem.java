package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ShooterVelocitySetpoint;

/**
 * Velocity closed-loop shooter: CAN 9 owns the encoder and velocity PID;
 * CAN 10 follows in-phase, CAN 12 follows inverted as in the original wiring.
 * The SPARK PID runs on board, with kV feedforward in volts per motor RPM,
 * 12 V voltage compensation, and independent 20 A current limits.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooter9 =
      new SparkMax(ShooterConstants.SHOOTER_9_ID, MotorType.kBrushless);
  private final SparkMax shooter10 =
      new SparkMax(ShooterConstants.SHOOTER_10_ID, MotorType.kBrushless);
  private final SparkMax shooter12 =
      new SparkMax(ShooterConstants.SHOOTER_12_ID, MotorType.kBrushless);
  private final RelativeEncoder shooter9Encoder = shooter9.getEncoder();
  private final SparkClosedLoopController velocityController = shooter9.getClosedLoopController();
  private double requestedTargetRpm = 0.0;

  public ShooterSubsystem() {
    SparkMaxConfig leader = new SparkMaxConfig();
    leader.inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS)
        .voltageCompensation(ShooterConstants.NOMINAL_VOLTAGE)
        .closedLoopRampRate(0.25);

    leader.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShooterConstants.VELOCITY_KP,
            ShooterConstants.VELOCITY_KI, ShooterConstants.VELOCITY_KD)
        .outputRange(-ShooterConstants.MAX_MOTOR_OUTPUT, ShooterConstants.MAX_MOTOR_OUTPUT)
        .feedForward.kV(ShooterConstants.VELOCITY_KV_VOLTS_PER_RPM);

    shooter9.configure(leader, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig follower10 = new SparkMaxConfig();
    follower10.follow(shooter9, false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS)
        .voltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);
    shooter10.configure(follower10, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig follower12 = new SparkMaxConfig();
    follower12.follow(shooter9, true)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT_AMPS)
        .voltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);
    shooter12.configure(follower12, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    stopShooter();

    var shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addDouble("Shooter Command RPM", this::getTargetRPM)
        .withPosition(0, 2).withSize(2, 1);
    shooterTab.addBoolean("Shooter Running", this::isRunning)
        .withPosition(4, 2);
    shooterTab.addDouble("Shooter Applied (%)", this::getAppliedOutputPercent)
        .withPosition(0, 5).withSize(2, 1);
    shooterTab.addDouble("Shooter RPM Error", this::getRPMError)
        .withPosition(2, 5).withSize(2, 1);

    var diagnostics = Shuffleboard.getTab("Mechanism Diagnostics");
    diagnostics.addDouble("Shooter CAN 9 Current (A)", shooter9::getOutputCurrent);
    diagnostics.addDouble("Shooter CAN 10 Current (A)", shooter10::getOutputCurrent);
    diagnostics.addDouble("Shooter CAN 12 Current (A)", shooter12::getOutputCurrent);
    diagnostics.addDouble("Shooter CAN 9 Bus (V)", shooter9::getBusVoltage);
    diagnostics.addDouble("Shooter CAN 9 Applied (V)",
        () -> shooter9.getBusVoltage() * shooter9.getAppliedOutput());
  }

  /**
   * Configure closed-loop motor RPM, not percentage. A negative motor-native
   * setpoint preserves the original shooter rotation direction.
   * Zero, out-of-range, and nonfinite targets stop the shooter.
   */
  public void setTargetRPM(double targetRpm) {
    double safeRpm = ShooterVelocitySetpoint.sanitizeRPM(targetRpm);
    if (!DriverStation.isEnabled() || safeRpm <= 0.0) {
      stopShooter();
      return;
    }
    requestedTargetRpm = safeRpm;
    velocityController.setSetpoint(
        ShooterConstants.MOTOR_DIRECTION * safeRpm, ControlType.kVelocity);
  }

  public double getTargetRPM() {
    return requestedTargetRpm;
  }

  public boolean isRunning() {
    return requestedTargetRpm > 0.0 && DriverStation.isEnabled();
  }

  /** Motor-shaft RPM, positive when spinning in the original shooting direction. */
  public double getActualRPM() {
    return shooter9Encoder.getVelocity() * ShooterConstants.MOTOR_DIRECTION;
  }

  public double getAppliedOutputPercent() {
    return shooter9.getAppliedOutput() * 100.0;
  }

  public double getRPMError() {
    return isRunning() ? requestedTargetRpm - getActualRPM() : 0.0;
  }

  /** Never claim readiness for a different or stale target, or overspeed. */
  public boolean isAtTargetRPM(double targetRpm, double toleranceRpm) {
    return isRunning() && Double.compare(
        requestedTargetRpm, ShooterVelocitySetpoint.sanitizeRPM(targetRpm)) == 0
        && ShooterVelocitySetpoint.isAtSpeed(
            getActualRPM(), requestedTargetRpm, toleranceRpm);
  }

  public void stopShooter() {
    requestedTargetRpm = 0.0;
    shooter9.stopMotor();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && requestedTargetRpm != 0.0) {
      stopShooter();
    }
  }
}
