package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

/** Manual hood control from the ZIP with fail-safe encoder and limit checks. */
public class HoodSubsystem extends SubsystemBase {
  private final SparkMax hoodMotor =
      new SparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);
  private final DutyCycleEncoder hoodEncoder =
      new DutyCycleEncoder(HoodConstants.HOOD_ENCODER_DIO);

  public HoodSubsystem() {
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(15);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    Shuffleboard.getTab("Test").addDouble("Hood Encoder", this::getEncoderPosition);
    Shuffleboard.getTab("Test").addBoolean("Hood Encoder Connected", hoodEncoder::isConnected);
    Shuffleboard.getTab("Test").addBoolean("Hood Limits Configured", this::limitsConfigured);
    Shuffleboard.getTab("Test").addBoolean("Hood At Min", this::isAtMin);
    Shuffleboard.getTab("Test").addBoolean("Hood At Max", this::isAtMax);
    Shuffleboard.getTab("Test").addBoolean("Hood Ready", this::isHoodReady);
  }

  public double getEncoderPosition() {
    return hoodEncoder.get();
  }

  private boolean limitsConfigured() {
    return Double.isFinite(HoodConstants.MIN_POSITION)
        && Double.isFinite(HoodConstants.MAX_POSITION)
        && HoodConstants.MIN_POSITION >= 0.0
        && HoodConstants.MAX_POSITION <= 1.0
        && HoodConstants.MIN_POSITION < HoodConstants.MAX_POSITION;
  }

  public boolean isHoodReady() {
    return limitsConfigured() && hoodEncoder.isConnected()
        && Double.isFinite(getEncoderPosition());
  }

  public void setManual(double output) {
    // Stop instead of driving blind if the sensor is disconnected or configured incorrectly.
    if (!Double.isFinite(output) || !isHoodReady()) {
      hoodMotor.stopMotor();
      return;
    }

    double boundedOutput = Math.max(
        -HoodConstants.TEST_MAX_SPEED,
        Math.min(HoodConstants.TEST_MAX_SPEED, output));

    if (boundedOutput == 0.0) {
      hoodMotor.stopMotor();
      return;
    }

    double position = getEncoderPosition();
    if (!Double.isFinite(position)) {
      hoodMotor.stopMotor();
      return;
    }

    boolean movingTowardMax = HoodConstants.POSITIVE_OUTPUT_INCREASES_ENCODER
        ? boundedOutput > 0.0 : boundedOutput < 0.0;
    boolean movingTowardMin = HoodConstants.POSITIVE_OUTPUT_INCREASES_ENCODER
        ? boundedOutput < 0.0 : boundedOutput > 0.0;

    if ((movingTowardMax && position >= HoodConstants.MAX_POSITION)
        || (movingTowardMin && position <= HoodConstants.MIN_POSITION)) {
      hoodMotor.stopMotor();
      return;
    }

    hoodMotor.set(boundedOutput);
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  public boolean isAtMin() {
    return isHoodReady() && getEncoderPosition() <= HoodConstants.MIN_POSITION;
  }

  public boolean isAtMax() {
    return isHoodReady() && getEncoderPosition() >= HoodConstants.MAX_POSITION;
  }
}
