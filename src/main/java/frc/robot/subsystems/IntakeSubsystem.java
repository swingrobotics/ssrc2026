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
import frc.robot.Constants.IntakeConstants;

/**
 * Two mirrored intake rollers driven by independent Spark MAX controllers and
 * original (first-generation) brushless NEO motors.
 *
 * Positive speed: left motor positive, right motor negative. These are motor
 * shaft directions, not a verified game-piece intake direction: check the
 * actual roller motion on the assembled mechanism before regular operation.
 *
 * Hold the driver controller's left bumper to intake; the motor CAN IDs are 16/17.
 */
public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final boolean configured;
  private double commandedOutput = 0.0;

  public IntakeSubsystem() {
    int leftId = IntakeConstants.LEFT_CAN_ID;
    int rightId = IntakeConstants.RIGHT_CAN_ID;

    configured = isValidCanId(leftId)
        && isValidCanId(rightId)
        && leftId != rightId;
    ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    intakeTab.addBoolean("Intake Configured", this::isConfigured);
    intakeTab.addDouble("Intake Output (%)", () -> commandedOutput * 100.0);
    intakeTab.addDouble("Left Motor CAN ID", () -> IntakeConstants.LEFT_CAN_ID);
    intakeTab.addDouble("Right Motor CAN ID", () -> IntakeConstants.RIGHT_CAN_ID);
    intakeTab.addDouble("Left Motor Current (A)",
        () -> configured ? leftMotor.getOutputCurrent() : 0.0);
    intakeTab.addDouble("Right Motor Current (A)",
        () -> configured ? rightMotor.getOutputCurrent() : 0.0);

    if (!configured) {
      // Do not construct Spark MAX objects with placeholders or duplicate IDs.
      leftMotor = null;
      rightMotor = null;
      DriverStation.reportWarning(
          "Intake disabled: set distinct LEFT_CAN_ID and RIGHT_CAN_ID in Constants.java",
          false);
      return;
    }

    leftMotor = new SparkMax(leftId, MotorType.kBrushless);
    rightMotor = new SparkMax(rightId, MotorType.kBrushless);

    // Explicit inversion is handled by the two set() calls below, not via
    // controller configuration, to keep the motor directions easy to audit.
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.CURRENT_LIMIT_AMPS);
    leftMotor.configure(
        leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(false)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.CURRENT_LIMIT_AMPS);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    stopIntake();
  }

  private static boolean isValidCanId(int id) {
    return id >= 1 && id <= 62;
  }

  public boolean isConfigured() {
    return configured;
  }

  /**
   * Signed motor output. For positive output, left is + and right is -.
   * A negative output reverses both. Non-finite values stop both motors.
   */
  public void setIntakeSpeed(double output) {
    if (!configured || DriverStation.isDisabled() || !Double.isFinite(output)) {
      stopIntake();
      return;
    }

    commandedOutput = MathUtil.clamp(
        output, -IntakeConstants.MAX_OUTPUT, IntakeConstants.MAX_OUTPUT);
    leftMotor.set(commandedOutput);
    rightMotor.set(-commandedOutput);
  }

  /** Run the rollers using the conservative default speed. */
  public void runIntake() {
    setIntakeSpeed(IntakeConstants.DEFAULT_OUTPUT);
  }

  /** Reverse both rollers using the conservative default speed. */
  public void reverseIntake() {
    setIntakeSpeed(-IntakeConstants.DEFAULT_OUTPUT);
  }

  /** Stop both motors, including on command interruption. */
  public void stopIntake() {
    commandedOutput = 0.0;
    if (leftMotor != null) {
      leftMotor.stopMotor();
    }
    if (rightMotor != null) {
      rightMotor.stopMotor();
    }
  }

  public double getCommandedOutput() {
    return commandedOutput;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && commandedOutput != 0.0) {
      stopIntake();
    }
  }
}
