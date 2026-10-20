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
    public static final double kMaxSpeedMetersPerSecond = 1;  //위쪽 주석을 수정함
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
   * Assign TWO unique CAN IDs before enabling. -1 deliberately prevents motor
   * initialization, so an unfinished configuration cannot claim another device.
   */
  public static final class IntakeConstants {


  
    public static final int LEFT_CAN_ID = 16;
    public static final int RIGHT_CAN_ID = 17;

    // Conservative first-power-on limits; tune only after confirming roller direction.
    public static final int CURRENT_LIMIT_AMPS = 40;
    public static final double MAX_OUTPUT = 0.55;
    public static final double DEFAULT_OUTPUT = 0.50;
  }

  // Mechanism constants from FRC_Drive_Shooter_Merged.zip.
  public static final class ShooterConstants {
    public static final int SHOOTER_9_ID = 9;
    public static final int SHOOTER_10_ID = 10;
    public static final int SHOOTER_12_ID = 12;
    public static final double TEST_SPEED = -0.50;
  }

  public static final class FeederConstants {
    public static final int FEEDER_11_ID = 11;
    public static final int FEEDER_14_ID = 14;
    public static final double TEST_SPEED = 0.50;
  }

  /** 2026 REBUILT official (Welded) field coordinate and camera mount. */
  public static final class VisionConstants {
    public static final String CAMERA_NAME = "PC_Camera";
    public static final AprilTagFieldLayout FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Previously measured on this robot, in WPILib robot coordinates:
    // X forward, Y left, Z up. Verify the optical axis/pitch with a known tag.
    public static final double CAMERA_FORWARD_M = 0.31732884884;
    public static final double CAMERA_LEFT_M = 0.0;
    public static final double CAMERA_UP_M = 0.40394608582;
    public static final double CAMERA_ROLL_DEG = 0.0;
    public static final double CAMERA_PITCH_DEG = -25.0;
    public static final double CAMERA_YAW_DEG = 0.0;
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

  public static final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 13;
    public static final int HOOD_ENCODER_DIO = 0;
    public static final double TEST_MAX_SPEED = 0.08;
    // Actual values in the ZIP's Constants.java (README had outdated values).

    public static final boolean CALIBRATION_MODE = false;
    
    public static final double MIN_POSITION = 0.196;
    public static final double MAX_POSITION = 0.410;
    public static final boolean POSITIVE_OUTPUT_INCREASES_ENCODER = false;
  }
}
