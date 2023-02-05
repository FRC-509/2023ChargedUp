package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

  public static final double stickDeadband = 0.1;

  public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;
  public static final double driveGearRatio = 6.75;
  public static final double angleGearRatio = 12.8;
  public static final double maxSpeed = 4.96824; // 5.146844360768413;
  public static final double maxAngularVelocity = 10.0;

  public static final boolean closedLoopDriveVelocity = true;

  // public static final double driveKS = 0.017371;
  // public static final double driveKV = 2.3131;
  // public static final double driveKA = 0.10452;

  public static final double chassisLength = Units.inchesToMeters(28);
  public static final double chassisWidth = Units.inchesToMeters(28);
  public static final double offsetToSwerveModule = chassisLength / 2 - Units.inchesToMeters(3.25);

  public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(+offsetToSwerveModule, +offsetToSwerveModule),
      new Translation2d(+offsetToSwerveModule, -offsetToSwerveModule),
      new Translation2d(-offsetToSwerveModule, +offsetToSwerveModule),
      new Translation2d(-offsetToSwerveModule, -offsetToSwerveModule));

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
      2,
      5,
      4,
      140.28d,
      new Utils.PIDConstants(0.2, 0, 0, 0),
      new Utils.PIDConstants(0.02, 0.00055, 0.002, 0.0012));

  public static final SwerveModuleConfigurations s_frontRight = new SwerveModuleConfigurations(
      1,
      0,
      1,
      0,
      157.40d,
      new Utils.PIDConstants(0.2, 0, 0, 0),
      new Utils.PIDConstants(0.02, 0.00055, 0.002, 0.0012));

  public static final SwerveModuleConfigurations s_backLeft = new SwerveModuleConfigurations(
      2,
      1,
      3,
      6,
      154.42d,
      new Utils.PIDConstants(0.2, 0, 0, 0),
      new Utils.PIDConstants(0.02, 0.00055, 0.002, 0.0012));

  public static final SwerveModuleConfigurations s_backRight = new SwerveModuleConfigurations(
      3,
      3,
      7,
      2,
      104.42d,
      new Utils.PIDConstants(0.2, 0, 0, 0),
      new Utils.PIDConstants(0.02, 0.00055, 0.002, 0.0012));

}