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
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Two-motor feeder (11 leader, 14 inverted follower).
 *
 * The feeder is interlocked with a supplied shooter-ready condition:
 * the shooter must be at its target RPM before any feeder output is allowed.
 * If readiness is lost or the robot is disabled, the feeder stops.
 * The operator must still hold A to request feed; the feeder cannot start
 * solely because the shooter reaches its target speed.
 */
public class FeederSubsystem extends SubsystemBase {
  private final SparkMax feeder11 =
      new SparkMax(FeederConstants.FEEDER_11_ID, MotorType.kBrushless);
  private final SparkMax feeder14 =
      new SparkMax(FeederConstants.FEEDER_14_ID, MotorType.kBrushless);

  private final BooleanSupplier shooterReady;
  private double commandedOutput = 0.0;

  public FeederSubsystem(BooleanSupplier shooterReady) {
    this.shooterReady = Objects.requireNonNull(shooterReady, "shooterReady");

    SparkMaxConfig feeder11Config = new SparkMaxConfig();
    feeder11Config.idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    feeder11.configure(feeder11Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig feeder14Config = new SparkMaxConfig();
    feeder14Config.follow(feeder11, true).idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    feeder14.configure(feeder14Config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    stopFeeder();

    ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
    shooterTab.addBoolean("Feeder Allowed", this::isFeedAllowed);
    shooterTab.addBoolean("Feeder Command Active", this::isFeeding);
    shooterTab.addDouble("Feeder Command (%)", () -> commandedOutput * 100.0);
  }

  /** Whether a new feeder command is permitted right now. */
  public boolean isFeedAllowed() {
    return DriverStation.isEnabled() && shooterReady.getAsBoolean();
  }

  /**
   * Feed at the requested duty cycle ONLY while the shooter is at target RPM.
   * If the shooter slows below tolerance, output is stopped immediately on
   * the next command/periodic cycle and automatically resumes when ready
   * if the operator continues holding the feed button.
   */
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

  /** Stop the leader and its hardware follower; used on release/interruption. */
  public void stopFeeder() {
    commandedOutput = 0.0;
    feeder11.stopMotor();
  }

  @Override
  public void periodic() {
    // Backstop even if a caller forgets to check the shooter speed.
    if (commandedOutput != 0.0 && !isFeedAllowed()) {
      stopFeeder();
    }
  }
}
