package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/** Feeder mechanism from FRC_Drive_Shooter_Merged.zip. */
public class FeederSubsystem extends SubsystemBase {
  private final SparkMax feeder11 =
      new SparkMax(FeederConstants.FEEDER_11_ID, MotorType.kBrushless);
  private final SparkMax feeder14 =
      new SparkMax(FeederConstants.FEEDER_14_ID, MotorType.kBrushless);

  public FeederSubsystem() {
    SparkMaxConfig feeder11Config = new SparkMaxConfig();
    feeder11Config.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
    feeder11.configure(feeder11Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig feeder14Config = new SparkMaxConfig();
    feeder14Config.follow(feeder11, true).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
    feeder14.configure(feeder14Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void runFeeder(double output) {
    if (!Double.isFinite(output)) {
      feeder11.stopMotor();
      return;
    }
    feeder11.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  public void runFeeder() {
    runFeeder(FeederConstants.TEST_SPEED);
  }

  public void stopFeeder() {
    feeder11.stopMotor();
  }
}
