package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Utils;

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

  public static final String CANIVORE = "509CANIvore";

  public static final int LIGHTS_PORT = 0;

  public static final int ledPixelCount = 25;

  // Vision-related constants.
  public static final int redAllianceSubstationTagID = 5;
  public static final int blueAllianceSubstationTagID = 4;
  public static final int[] redAllianceTargetTagIDs = { 6, 7, 8 };
  public static final int[] blueAllianceTargetTagIDs = { 3, 2, 1 };
  public static final String frontCameraId = "limelight-front";
  public static final String backCameraId = "limelight-back";

  // Control-related constants.
  public static final double stickDeadband = 0.1;
  public static final double armPivotOperatorCoefficient = 0.5;
  public static final double armExtensionOperatorCoefficient = 1.0;

  public static final double intakePercentVel = 0.74;

  // Drivetrain-related constants.
  public static final double safetyBuffer = Units.inchesToMeters(40);
  public static final double chassisLength = Units.inchesToMeters(28);
  public static final double chassisWidth = Units.inchesToMeters(28);
  public static final double offsetToSwerveModule = chassisLength / 2 - Units.inchesToMeters(3.25);

  // 0.3191858136047229930278045677412 meters
  
  public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;
  public static final double driveGearRatio = 6.75;
  public static final double angleGearRatio = 12.8;
  public static final double maxSpeed = 4.96824; // 5.146844360768413;
  public static final double maxAngularVelocity = maxSpeed / (Math.hypot(offsetToSwerveModule, offsetToSwerveModule));

  public static final double kS = 0.079354 / 12.0;
  public static final double kV = 2.3277 / 12.0;
  public static final double kA = 0.26532 / 12.0;
  
  //0.67488/12
  //0.2
  public static final Utils.PIDConstants drive = new Utils.PIDConstants(0.1, 0, 0, 0);
  public static final Utils.PIDConstants steer = new Utils.PIDConstants(0.2, 0, 0, 0);
  public static final boolean closedLoopDriveVelocity = true;

  // public static final double driveKS = 0.017371;
  // public static final double driveKV = 2.3131;
  // public static final double driveKA = 0.10452;

  // public static final double TEST = 10.0 * 1023.0d / 2048.0 / maxSpeed /
  // wheelCircumference / driveGearRatio;
  // public static final double TEST2 = 1023
  // / (maxSpeed / (Math.PI * Units.inchesToMeters(4.0) * (1.0d / driveGearRatio)
  // / 2048.0 * 10));

  // Rishabh's magic
  // kF - 0.047542572
  // kP - 0.0160000324
  // kI - 0.01n

  /* 
   * The order of each vector corresponds to the index of the swerve module inside the swerveModules array.
   * 
   * module 2 (-, -) |--b--| module 1 (-, +)
   *                 |     |
   * module 3 (+, -) |--f--| module 0 (+, +)
  */
  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(+offsetToSwerveModule, +offsetToSwerveModule),
      new Translation2d(-offsetToSwerveModule, +offsetToSwerveModule),
      new Translation2d(-offsetToSwerveModule, -offsetToSwerveModule),
      new Translation2d(+offsetToSwerveModule, -offsetToSwerveModule));

  public static final class SwerveModuleConfigurations {
    public int moduleNumber;
    public int angleEncoderId;
    public int angleMotorId;
    public int driveMotorId;
    public double angleEncoderOffset;
    public Utils.PIDConstants steerPID;
    public Utils.PIDConstants drivePID;

    public SwerveModuleConfigurations(int moduleNumber, int angleEncoderId, int angleMotorId, int driveMotorId,
        double angleEncoderOffset, Utils.PIDConstants steerPID, Utils.PIDConstants drivePID) {
      this.moduleNumber = moduleNumber;
      this.angleEncoderId = angleEncoderId;
      this.angleMotorId = angleMotorId;
      this.driveMotorId = driveMotorId;
      this.angleEncoderOffset = angleEncoderOffset;
      this.steerPID = steerPID;
      this.drivePID = drivePID;
    }
  }

  public static final SwerveModuleConfigurations s_frontLeft = new SwerveModuleConfigurations(
      0,
      0,
      8,
      4,
      -315.09,
      steer,
      drive);

  public static final SwerveModuleConfigurations s_frontRight = new SwerveModuleConfigurations(
      3,
      3,
      11,
      7,
      -299.18,
      steer,
      drive);

  public static final SwerveModuleConfigurations s_backLeft = new SwerveModuleConfigurations(
      1,
    1,
      9,
      5,
      -309.28,
      steer,
      drive);

  public static final SwerveModuleConfigurations s_backRight = new SwerveModuleConfigurations(
      2,
      2,
      10,
      6,
      -156.44,
      steer,
      drive);
}