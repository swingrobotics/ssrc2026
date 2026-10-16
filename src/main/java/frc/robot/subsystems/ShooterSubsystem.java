package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/** Shooter mechanism from FRC_Drive_Shooter_Merged.zip. */
public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax shooter9 =
      new SparkMax(ShooterConstants.SHOOTER_9_ID, MotorType.kBrushless);
  private final SparkMax shooter10 =
      new SparkMax(ShooterConstants.SHOOTER_10_ID, MotorType.kBrushless);
  private final SparkMax shooter12 =
      new SparkMax(ShooterConstants.SHOOTER_12_ID, MotorType.kBrushless);

  public ShooterSubsystem() {
    SparkMaxConfig shooter9Config = new SparkMaxConfig();
    shooter9Config.idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    shooter9.configure(shooter9Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig shooter10Config = new SparkMaxConfig();
    shooter10Config.follow(shooter9, false).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    shooter10.configure(shooter10Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig shooter12Config = new SparkMaxConfig();
    shooter12Config.follow(shooter9, true).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    shooter12.configure(shooter12Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void runShooter(double output) {
    if (!Double.isFinite(output)) {
      shooter9.stopMotor();
      return;
    }
    shooter9.set(MathUtil.clamp(output, -1.0, 1.0));
  }

  public void runShooter() {
    runShooter(ShooterConstants.TEST_SPEED);
  }

  public void stopShooter() {
    shooter9.stopMotor();
  }
}
