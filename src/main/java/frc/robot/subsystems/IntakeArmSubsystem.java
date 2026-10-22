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
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.Constants.IntakeConstants;

/**
 * Manual rack-and-pinion intake ARM (not the intake rollers or shooter feeder).
 *
 * D-pad UP requests positive arm output, D-pad DOWN negative.
 * The actual forward/backward direction must be checked on the mechanism.
 *
 * There is no verified absolute position or end-stop sensor yet: the NEO
 * integrated encoder is RELATIVE, so it cannot enforce travel limits after
 * reboot. This implementation uses a conservative output, current limit,
 * and a hold-to-move timer; it does not automatically drive to end positions.
 */
public class IntakeArmSubsystem extends SubsystemBase {
  private final SparkMax armMotor;
  private final RelativeEncoder armEncoder;
  private final boolean configured;
  private double commandedOutput = 0.0;
  private double moveStartTime = -1.0;
  private boolean timedOut = false;

  public IntakeArmSubsystem() {
    int id = IntakeArmConstants.MOTOR_CAN_ID;
    // CAN 1-14 are already used by drivetrain/shooter/feeder/hood in this
    // project; do not let the arm accidentally claim any of their IDs.
    configured = id >= 15 && id <= 62
        && id != IntakeConstants.LEFT_CAN_ID
        && id != IntakeConstants.RIGHT_CAN_ID;

    if (!configured) {
      armMotor = null;
      armEncoder = null;
      DriverStation.reportWarning(
          "Intake arm DISABLED: assign an unused CAN ID (15-62) "
              + "to IntakeArmConstants.MOTOR_CAN_ID and verify wiring.",
          false);
    } else {
      armMotor = new SparkMax(id, MotorType.kBrushless);
      armEncoder = armMotor.getEncoder();
      SparkMaxConfig config = new SparkMaxConfig();
      config.inverted(false)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(IntakeArmConstants.CURRENT_LIMIT_AMPS);
      armMotor.configure(config, ResetMode.kResetSafeParameters,
          PersistMode.kNoPersistParameters);
      armMotor.stopMotor();
    }

    // Intake arm and roller state share a single operator page.
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    tab.addBoolean("Arm Configured", this::isConfigured).withPosition(3, 0);
    tab.addDouble("Arm Output (%)", () -> commandedOutput * 100.0)
        .withPosition(4, 0).withSize(2, 1);
    tab.addBoolean("Arm Hold Timeout", () -> timedOut).withPosition(0, 1);
    tab.addString("Arm Status", this::getStatus).withPosition(1, 1).withSize(5, 1);

    ShuffleboardTab debugTab = Shuffleboard.getTab("Mechanism Diagnostics");
    debugTab.addDouble("Arm CAN ID", () -> IntakeArmConstants.MOTOR_CAN_ID);
    debugTab.addDouble("Arm Motor Rotations", this::getMotorRotations);
    debugTab.addDouble("Arm Current (A)", this::getMotorCurrent);
  }

  public boolean isConfigured() {
    return configured;
  }

  /** Measured motor-shaft revolutions since controller startup, NOT arm angle. */
  public double getMotorRotations() {
    return configured ? armEncoder.getPosition() : -1.0;
  }

  public double getMotorCurrent() {
    return configured ? armMotor.getOutputCurrent() : 0.0;
  }

  public String getStatus() {
    if (!configured) {
      return "DISABLED: assign unused arm CAN ID";
    }
    if (DriverStation.isDisabled()) {
      return "ROBOT DISABLED";
    }
    if (timedOut) {
      return "TIMEOUT: release D-pad before next motion";
    }
    if (commandedOutput > 0.0) {
      return "D-PAD UP: positive motor output";
    }
    if (commandedOutput < 0.0) {
      return "D-PAD DOWN: negative motor output";
    }
    return "STOPPED: no calibrated end stops / no position hold";
  }

  /**
   * Manual output in [-MAX_OUTPUT,+MAX_OUTPUT]. Stops when released or the
   * maximum continuous actuation time elapses. A timeout requires releasing
   * the D-pad; holding the button cannot automatically restart the motor.
   */
  private void setManualOutput(double request) {
    if (!configured || DriverStation.isDisabled() || !Double.isFinite(request)) {
      stopArm();
      return;
    }
    if (timedOut) {
      armMotor.stopMotor();
      commandedOutput = 0.0;
      return;
    }
    double requested = MathUtil.clamp(
        request, -IntakeArmConstants.MAX_OUTPUT, IntakeArmConstants.MAX_OUTPUT);
    if (requested == 0.0) {
      stopArm();
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (moveStartTime < 0.0) {
      moveStartTime = now;
    }
    if (now - moveStartTime >= IntakeArmConstants.MAX_CONTINUOUS_RUN_SECONDS) {
      armMotor.stopMotor();
      commandedOutput = 0.0;
      timedOut = true;
      return;
    }

    commandedOutput = IntakeArmConstants.MOTOR_DIRECTION * requested;
    armMotor.set(commandedOutput);
  }

  public void moveForward() {
    setManualOutput(IntakeArmConstants.DEFAULT_OUTPUT);
  }

  public void moveBackward() {
    setManualOutput(-IntakeArmConstants.DEFAULT_OUTPUT);
  }

  /** Stops and clears the hold timer; runEnd calls this on button release. */
  public void stopArm() {
    commandedOutput = 0.0;
    moveStartTime = -1.0;
    timedOut = false;
    if (armMotor != null) {
      armMotor.stopMotor();
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled() && (commandedOutput != 0.0 || timedOut)) {
      stopArm();
    }
  }
}
