# Shooter integration (test branch)

This branch is based on `swerve-integration`. It adds the Shooter, Feeder, and Hood
mechanisms from the user-provided `FRC_Drive_Shooter_Merged.zip` without replacing
the existing navX2, PhotonVision, PathPlanner, or NetworkTables drive code.

## Xbox controller (port 0)

| Input | Action |
| --- | --- |
| Left stick | Swerve translation |
| LT / RT | Rotate |
| **Hold A** | **Feeder output**, stop on release |
| **Hold B** | **Shooter output**, stop on release |
| **Press X** | **Toggle PhotonVision yaw-based auto-align** |
| **Press Y** | **Toggle field-relative / robot-relative drive** |
| Right stick Y | Manual Hood, limited output and encoder-position limits |
| Hold RB | Swerve X lock |
| Start | navX2 heading zero |

The `Shuffleboard` **Test** tab retains **Drive Speed (%)** (100% default),
**Shooter Speed (%)** (50% default; negative output), and
**Feeder Speed (%)** (50% default; positive output) sliders.

Mechanism CAN IDs: shooter **9, 10, 12** (10 follows same direction,
12 follows opposite), feeder **11, 14** (14 follows opposite),
Hood **13**, absolute encoder on **DIO 0**. The follower and current-limit
settings match the supplied ZIP.

## Important hardware verification BEFORE deploying

- The existing drive code still uses **SPARK MAX** for drive CAN 1–4 and
  steering CAN 5–8. The ZIP instead declares **SPARK Flex** for CAN 1–4.
  This was deliberately **not** switched without confirmation of the
  physical installed motor controllers; adjust `MAXSwerveModule.java` and
  `Configs.java` to match the hardware before deploying.
- **navX2** remains the only configured gyro in `DriveSubsystem`; the
  ZIP's ADIS16470 code was not copied.
- Hood software limits come from the ZIP's **actual `Constants.java`**:
  **0.2687 to 0.4989** rotations. Its README separately claimed
  **0.363 to 0.548**, which conflicts. Physically verify the proper range
  and the `POSITIVE_OUTPUT_INCREASES_ENCODER` direction before use.
- Hood output is now stopped if the encoder is disconnected, the reading is
  non-finite, or the limits are invalid. This is only a software safeguard;
  it does **not** replace mechanical stops and physical safe testing.
- REVLib is deliberately left at the GitHub project's **2026.0.0** instead
  of overwriting existing dependencies with the ZIP's 2026.0.5.
  Confirm dependency/API compatibility using WPILib Build Robot Code.
- The PhotonVision camera name remains `PC_Camera`. PathPlanner deploy files,
  drivetrain module telemetry, and existing dashboard topics are unchanged.

**The code has not been built on the team's development laptop or
deployed/tested on the physical robot.** Do not merge into
`swerve-integration` or deploy without confirming the hardware and building.
