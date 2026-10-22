package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/**
 * Two-motor shooter-side feeder: CAN 11 leader, CAN 14 inverted follower.
 * Signed commands select travel direction. Callers own shooter RPM decisions;
 * this subsystem still stops when disabled or given an invalid command.
 */
public class FeederSubsystem extends SubsystemBase {
  private final SparkMax feeder11 =
      new SparkMax(FeederConstants.FEEDER_11_ID, MotorType.kBrushless);
  private final SparkMax feeder14 =
      new SparkMax(FeederConstants.FEEDER_14_ID, MotorType.kBrushless);
  private double commandedOutput = 0.0;

  public FeederSubsystem() {
    SparkMaxConfig feeder11Config = new SparkMaxConfig();
    feeder11Config.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    feeder11.configure(feeder11Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig feeder14Config = new SparkMaxConfig();
    feeder14Config.follow(feeder11, true).idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    feeder14.configure(feeder14Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    stopFeeder();
    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addDouble("Feeder Command (%)", () -> commandedOutput * 100.0)
        .withPosition(2, 2).withSize(2, 1);
    ShuffleboardTab debugTab = Shuffleboard.getTab("Mechanism Diagnostics");
    debugTab.addBoolean("Feeder Enabled", DriverStation::isEnabled);
    debugTab.addBoolean("Feeder Command Active", this::isFeeding);
    // Compare the command with real SPARK feedback to distinguish input,
    // dashboard speed, missing hardware and motor/controller problems.
    debugTab.addDouble("Feeder CAN 11 Applied (%)",
        () -> feeder11.getAppliedOutput() * 100.0);
    debugTab.addDouble("Feeder CAN 14 Applied (%)",
        () -> feeder14.getAppliedOutput() * 100.0);
    debugTab.addDouble("Feeder CAN 11 Current (A)", feeder11::getOutputCurrent);
    debugTab.addDouble("Feeder CAN 14 Current (A)", feeder14::getOutputCurrent);
  }

  public boolean isFeedAllowed() {
    return DriverStation.isEnabled();
  }

  /** Positive is the existing toward-shooter command; negative reverses it. */
  public void runFeeder(double output) {
    if (!Double.isFinite(output) || !isFeedAllowed()) {
      stopFeeder();
      return;
    }
    double safeOutput = MathUtil.clamp(output, -1.0, 1.0);
    if (safeOutput == 0.0) {
      stopFeeder();
      return;
    }
    commandedOutput = safeOutput;
    feeder11.set(safeOutput);
  }

  public void runFeeder() {
    runFeeder(FeederConstants.TEST_SPEED);
  }

  public boolean isFeeding() {
    return commandedOutput != 0.0 && isFeedAllowed();
  }

  public String getDirection() {
    if (!isFeeding()) return "STOP";
    return commandedOutput > 0.0 ? "FORWARD" : "REVERSE";
  }

  public void stopFeeder() {
    commandedOutput = 0.0;
    feeder11.stopMotor();
  }

  @Override
  public void periodic() {
    if (commandedOutput != 0.0 && !isFeedAllowed()) {
      stopFeeder();
    }
  }
}
