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

/** Hood manual control with calibration mode and limit checks. */
public class HoodSubsystem extends SubsystemBase {

    private final SparkMax hoodMotor =
            new SparkMax(HoodConstants.HOOD_MOTOR_ID, MotorType.kBrushless);

    private final DutyCycleEncoder hoodEncoder =
            new DutyCycleEncoder(HoodConstants.HOOD_ENCODER_DIO);

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

        // Shuffleboard
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

    // Manual motor control
    public void setManual(double output) {

        // Check input and encoder
        if (!Double.isFinite(output) || !isHoodReady()) {
            hoodMotor.stopMotor();
            return;
        }

        // Limit motor output
        double boundedOutput = Math.max(
                -HoodConstants.TEST_MAX_SPEED,
                Math.min(HoodConstants.TEST_MAX_SPEED, output)
        );

        // Stop if output is zero
        if (boundedOutput == 0.0) {
            hoodMotor.stopMotor();
            return;
        }

        // Apply software limits only in normal mode
        if (!HoodConstants.CALIBRATION_MODE) {

            double position = getEncoderPosition();

            boolean movingTowardMax =
                    HoodConstants.POSITIVE_OUTPUT_INCREASES_ENCODER
                            ? boundedOutput > 0.0
                            : boundedOutput < 0.0;

            boolean movingTowardMin =
                    HoodConstants.POSITIVE_OUTPUT_INCREASES_ENCODER
                            ? boundedOutput < 0.0
                            : boundedOutput > 0.0;

            // Stop at limits
            if ((movingTowardMax
                    && position >= HoodConstants.MAX_POSITION)
                    || (movingTowardMin
                    && position <= HoodConstants.MIN_POSITION)) {

                hoodMotor.stopMotor();
                return;
            }
        }

        // Run motor
        hoodMotor.set(boundedOutput);
    }

    // Stop motor
    public void stop() {
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
