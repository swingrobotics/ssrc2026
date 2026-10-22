// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    //public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxSpeedMetersPerSecond = 3;  //위쪽 주석을 수정함
    //public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularSpeed = 1 * Math.PI; // radians per second  위쪽 주석을 수정함


    // Keep the team's prior measured chassis dimensions separate from the
    // measured SWERVE MODULE CENTERS in PathPlanner (these differ from the
    // chassis outline and the overall bumper dimensions).
    public static final double kMeasuredChassisWidthMeters = Units.inchesToMeters(28);
    public static final double kMeasuredChassisLengthMeters = Units.inchesToMeters(27);

    // Measured module center spans from pathplanner/settings.json.
    // The rear-left X offset is 0.0005 m farther back than rear-right,
    // so kinematics below uses each exact measurement rather than symmetry.
    public static final double kTrackWidth = 0.614;
    public static final double kWheelBase = 0.55325;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(0.2765, 0.307),     // FL
        new Translation2d(0.2765, -0.307),    // FR
        new Translation2d(-0.277, 0.307),     // RL
        new Translation2d(-0.2765, -0.307));  // RR

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;  //CAD ID는 보기쉽게 수정함
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13; //우리가 모터 13T를 사용하기 때문에 13으로 설정함

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kHoodDeadband = 0.10;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


  /**
   * Two Spark MAX / original NEO intake rollers.
   *
   * CAN IDs 16 (left) and 17 (right) restored from shooter-integration.
   * Verify these are still the physical controller IDs before enabling.
   */
  public static final class IntakeConstants {
    public static final int LEFT_CAN_ID = 16;
    public static final int RIGHT_CAN_ID = 17;

    // Conservative first-power-on limits; tune only after confirming roller direction.
    public static final int CURRENT_LIMIT_AMPS = 40;
    public static final double MAX_OUTPUT = 1.00;
    public static final double DEFAULT_OUTPUT = 0.85;
  }

  /**
   * Rack-and-pinion intake arm, separate from the two intake rollers.
   * Assign the real unused CAN ID after confirming the SPARK MAX wiring.
   * Until then the subsystem will not construct a motor or drive anything.
   *
   * There is no calibrated arm-angle sensor or hardware limit switch yet,
   * so these conservative test values DO NOT provide travel-end protection.
   */
  public static final class IntakeArmConstants {
    public static final int MOTOR_CAN_ID = 15; // TODO: actual unique CAN ID (15-62).
    public static final double MOTOR_DIRECTION = -1.0; // Verify physical direction.
    public static final int CURRENT_LIMIT_AMPS = 40;
    public static final double DEFAULT_OUTPUT = 0.15;
    public static final double MAX_OUTPUT = 0.20;
    public static final double MAX_CONTINUOUS_RUN_SECONDS = 1.5;
  }

  // Mechanism constants from FRC_Drive_Shooter_Merged.zip.
  public static final class ShooterConstants {
    public static final int SHOOTER_9_ID = 9;
    public static final int SHOOTER_10_ID = 10;
    public static final int SHOOTER_12_ID = 12;

    // Test values, NOT measured match-ready shot settings.
    // Motor 9 was previously driven with negative duty cycle.
    public static final double MOTOR_DIRECTION = -1.0;
    // Original 50% driver-controlled duty cycle. Manual/uncalibrated auto
    // operation uses this, never an RPM-to-percent conversion.
    public static final double DEFAULT_OUTPUT_PERCENT = 50.0;
    // Only direct duty-cycle output control; no RPM targets or tuning.
    public static final int CURRENT_LIMIT_AMPS = 20;
    public static final double MAX_MOTOR_OUTPUT = 1.0;
  }

  public static final class FeederConstants {
    public static final int FEEDER_11_ID = 11;
    public static final int FEEDER_14_ID = 14;
    public static final double TEST_SPEED = 0.70;
  }

  /** 2026 REBUILT official (Welded) field coordinate and camera mount. */
  public static final class VisionConstants {
    public static final String CAMERA_NAME = "PC_Camera";
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Reported physical mounting (2026-10-22):
    //   - Optical axis points UP 25 degrees relative to robot +X.
    //   - Camera housing/sensor is installed UPSIDE DOWN (180 degree roll).
    //   - PhotonVision pipeline Input > Orientation is set to 180 degrees,
    //     rotating the image BEFORE AprilTag 3-D processing. Treat the
    //     output as the VIRTUAL upright camera frame, not the raw sensor.
    //   - WPILib +Y pitch tilts the optical axis DOWN: UP 25 => pitch -25.
    //
    // IMPORTANT: Do not also blindly add 180 degrees to ROBOT_TO_CAMERA
    // roll while PhotonVision applies an image rotation. Verify the
    // resulting per-tag 3-D transform with a physical AprilTag at a known
    // HEIGHT and location. If that check contradicts the virtual-frame
    // assumption, correct the convention instead of compensating aim trim.
    //
    // Position values are PREVIOUSLY REPORTED, not freshly measured today.
    // Robot frame: X forward, Y left, Z up (meters).
    public static final double CAMERA_FORWARD_M = 0.31732884884;
    public static final double CAMERA_LEFT_M = 0.0;
    public static final double CAMERA_UP_M = 0.40394608582;
    // Effective processed-frame roll; physical sensor roll is 180.
    public static final double CAMERA_ROLL_DEG = 0.0;
    public static final double CAMERA_PITCH_DEG = -25.0;
    public static final double CAMERA_YAW_DEG = 0.0;
    // Documentation of expected manual PhotonVision UI setting.
    // This value does NOT set PhotonVision through PhotonLib.
    public static final double EXPECTED_PHOTONVISION_IMAGE_ORIENTATION_DEG = 180.0;
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(CAMERA_FORWARD_M, CAMERA_LEFT_M, CAMERA_UP_M),
        new Rotation3d(Math.toRadians(CAMERA_ROLL_DEG),
            Math.toRadians(CAMERA_PITCH_DEG), Math.toRadians(CAMERA_YAW_DEG)));

    // Conservative validation for single-AprilTag estimates.
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.25;
    public static final double MAX_SINGLE_TAG_DISTANCE_M = 4.0;
    public static final double MAX_ABS_ESTIMATED_HEIGHT_M = 0.5;
    public static final double FIELD_BOUNDS_TOLERANCE_M = 0.35;
  }

  /**
   * Nominal 2026 REBUILT HUB CENTER positions in the always-blue-origin
   * WPILib FIELD coordinate frame, in meters. These approximate values
   * are taken from published field geometry; verify against the team's
   * actual Welded field/official CAD before enabling automatic motion.
   */
  public static final class HubConstants {
    public static final Translation2d BLUE_CENTER_METERS = new Translation2d(4.625, 4.033);
    public static final Translation2d RED_CENTER_METERS = new Translation2d(11.920, 4.033);
    // Adjust ONLY after measuring the hood/projectile launch point relative
    // to robot center. Zero preserves center-to-center planar distance.
    public static final Translation2d SHOOTER_OFFSET_ROBOT_METERS = new Translation2d();
    public static final double VISION_MAX_AGE_SECONDS = 0.4;
    // Low-pass each accepted camera position update; cannot reject all
    // pose jumps. Inspect source pose in Shuffleboard during calibration.
    public static final double DISTANCE_FILTER_ALPHA = 0.35;
  }

  /** Broken swerve module: HUB automatic chassis rotation is locked in code. */
  public static final class HubAimSafety {
    public static final boolean ENABLE_AUTO_ROTATION = false;
  }

  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 13;
    public static final int HOOD_ENCODER_DIO = 0;
    public static final double TEST_MAX_SPEED = 0.08;
    // Actual values in the ZIP's Constants.java (README had outdated values).

    public static final boolean CALIBRATION_MODE = false;
    
    public static final double MIN_POSITION = 0.196;
    public static final double MAX_POSITION = 0.410;
    public static final boolean POSITIVE_OUTPUT_INCREASES_ENCODER = false;
    // Encoder fractional-turn units. Tune against measured steady-state error.
    public static final double AUTO_TARGET_TOLERANCE = 0.006;
  }
}
