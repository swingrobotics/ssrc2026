package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

import frc.robot.Constants.HoodConstants;

/** Hood manual control with calibration mode and limit checks. */
public class HoodSubsystem extends SubsystemBase {

    private final SparkMax hoodMotor =
            new SparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

    private final DutyCycleEncoder hoodEncoder =
            new DutyCycleEncoder(HoodConstants.HOOD_ENCODER_DIO);

    // Conservative initial tuning: position feedback only, with the same
    // motor-output cap as manual movement. Tune with the mechanism supervised.
    private static final double HOLD_KP = 4.0;
    private static final double HOLD_MAX_OUTPUT = HoodConstants.TEST_MAX_SPEED;
    private final PIDController holdPid = new PIDController(HOLD_KP, 0.0, 0.0);
    private double holdTarget = Double.NaN;
    private double holdOutput = 0.0;
    private double appliedGravityAssist = 0.0;
    // Signed percent (-8 to +8). Defaults to 0 until an adult/team
    // validates which motor direction supports the hood against gravity.
    private final GenericEntry gravityAssistEntry;

    // Constructor
    public HoodSubsystem() {

        SparkMaxConfig hoodConfig = new SparkMaxConfig();

        hoodConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(15);

        hoodMotor.configure(
                hoodConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // A signed, conservative gravity-feedforward adjustment.
        // 0% is intentionally the initial value; never guess lifting direction.
        gravityAssistEntry = Shuffleboard.getTab("Test")
                .add("Hood Gravity Assist (%)", 0.0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -8.0, "max", 8.0))
                .getEntry();

        Shuffleboard.getTab("Test")
                .addDouble("Hood Encoder", this::getEncoderPosition);

        Shuffleboard.getTab("Test")
                .addBoolean("Hood Encoder Connected", hoodEncoder::isConnected);

        Shuffleboard.getTab("Test")
                .addBoolean("Hood Calibration Mode",
                        () -> HoodConstants.CALIBRATION_MODE);

        Shuffleboard.getTab("Test")
                .addBoolean("Hood Limits Configured", this::limitsConfigured);

        Shuffleboard.getTab("Test")
                .addBoolean("Hood At Min", this::isAtMin);

        Shuffleboard.getTab("Test")
                .addBoolean("Hood At Max", this::isAtMax);

        Shuffleboard.getTab("Test")
                .addBoolean("Hood Ready", this::isHoodReady);

        Shuffleboard.getTab("Test")
                .addBoolean("Hood Hold Active",
                        () -> !HoodConstants.CALIBRATION_MODE
                                && Double.isFinite(holdTarget));
        Shuffleboard.getTab("Test")
                .addDouble("Hood Hold Target",
                        () -> Double.isFinite(holdTarget) ? holdTarget : -1.0);
        Shuffleboard.getTab("Test")
                .addDouble("Hood Hold Error",
                        () -> Double.isFinite(holdTarget) && isHoodReady()
                                ? holdTarget - getEncoderPosition() : 0.0);
        Shuffleboard.getTab("Test")
                .addDouble("Hood Hold Output", () -> holdOutput);
        Shuffleboard.getTab("Test")
                .addDouble("Hood Gravity Assist Applied (%)",
                        () -> appliedGravityAssist * 100.0);
    }

    @Override
    public void periodic() {
        // Disabling removes motor torque. Do not chase a stale target on re-enable.
        if (DriverStation.isDisabled()) {
            resetHold();
        }
    }

    private void resetHold() {
        holdTarget = Double.NaN;
        holdOutput = 0.0;
        appliedGravityAssist = 0.0;
        holdPid.reset();
    }

    // Encoder position
    public double getEncoderPosition() {
        return hoodEncoder.get();
    }

    // Check limit configuration
    private boolean limitsConfigured() {

        return Double.isFinite(HoodConstants.MIN_POSITION)
                && Double.isFinite(HoodConstants.MAX_POSITION)
                && HoodConstants.MIN_POSITION >= 0.0
                && HoodConstants.MAX_POSITION <= 1.0
                && HoodConstants.MIN_POSITION < HoodConstants.MAX_POSITION;
    }

    // Check hood readiness
    public boolean isHoodReady() {

        boolean encoderReady =
                hoodEncoder.isConnected()
                && Double.isFinite(getEncoderPosition());

        if (!encoderReady) {
            return false;
        }

        // Limits are not required during calibration
        return HoodConstants.CALIBRATION_MODE || limitsConfigured();
    }

    /**
     * Right-stick control. Moving the stick overrides hold; releasing it
     * captures the current absolute-encoder position and holds that position.
     */
    public void setManual(double output) {
        if (!Double.isFinite(output) || !isHoodReady()) {
            resetHold();
            hoodMotor.stopMotor();
            return;
        }

        double position = getEncoderPosition();
        double boundedOutput = MathUtil.clamp(
                output, -HoodConstants.TEST_MAX_SPEED, HoodConstants.TEST_MAX_SPEED);

        if (boundedOutput != 0.0) {
            // Manual input always has priority. A fresh position is captured
            // when the stick returns to deadband.
            resetHold();
            runWithLimits(boundedOutput, position);
            return;
        }

        if (HoodConstants.CALIBRATION_MODE) {
            // Do not automatically move the mechanism during calibration.
            resetHold();
            hoodMotor.stopMotor();
            return;
        }

        if (!Double.isFinite(holdTarget)) {
            holdTarget = MathUtil.clamp(
                    position, HoodConstants.MIN_POSITION, HoodConstants.MAX_POSITION);
            holdPid.reset();
        }

        // PID output is positive when the encoder should INCREASE.
        // Reverse it when positive motor output actually DECREASES encoder.
        double encoderCorrection = holdPid.calculate(position, holdTarget);
        double motorCorrection = HoodConstants.POSITIVE_OUTPUT_INCREASES_ENCODER
                ? encoderCorrection : -encoderCorrection;
        // Gravity feedforward applies supporting torque immediately, while
        // PID corrects the remaining position error. Both outputs together
        // stay within the existing +/-8% motor limit.
        double assistPercent = gravityAssistEntry.getDouble(0.0);
        if (!Double.isFinite(assistPercent)) {
            assistPercent = 0.0;
        }
        appliedGravityAssist = MathUtil.clamp(
                assistPercent / 100.0, -HOLD_MAX_OUTPUT, HOLD_MAX_OUTPUT);
        double holdCommand = MathUtil.clamp(
                motorCorrection + appliedGravityAssist,
                -HOLD_MAX_OUTPUT, HOLD_MAX_OUTPUT);

        runWithLimits(holdCommand, position);
    }

    private void runWithLimits(double output, double position) {
        if (!Double.isFinite(output) || !Double.isFinite(position)) {
            holdOutput = 0.0;
            hoodMotor.stopMotor();
            return;
        }

        if (!HoodConstants.CALIBRATION_MODE) {
            boolean towardMax = HoodConstants.POSITIVE_OUTPUT_INCREASES_ENCODER
                    ? output > 0.0 : output < 0.0;
            boolean towardMin = HoodConstants.POSITIVE_OUTPUT_INCREASES_ENCODER
                    ? output < 0.0 : output > 0.0;
            if ((towardMax && position >= HoodConstants.MAX_POSITION)
                    || (towardMin && position <= HoodConstants.MIN_POSITION)) {
                holdOutput = 0.0;
                hoodMotor.stopMotor();
                return;
            }
        }

        holdOutput = output;
        if (output == 0.0) {
            hoodMotor.stopMotor();
        } else {
            hoodMotor.set(output);
        }
    }

    // Stop motor and forget the old position.
    public void stop() {
        resetHold();
        hoodMotor.stopMotor();
    }

    // Check minimum position
    public boolean isAtMin() {

        return isHoodReady()
                && limitsConfigured()
                && getEncoderPosition() <= HoodConstants.MIN_POSITION;
    }

    // Check maximum position
    public boolean isAtMax() {

        return isHoodReady()
                && limitsConfigured()
                && getEncoderPosition() >= HoodConstants.MAX_POSITION;
    }
}
