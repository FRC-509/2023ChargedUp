package frc.robot;

import com.ctre.phoenixpro.sim.ChassisReference;

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
  public static final double maxSpeed = 4.0;
  public static final double maxAngularVelocity = 10.0;

  public static final boolean closedLoopDriveVelocity = false;

  public static final double driveKS = 0;
  public static final double driveKV = 0;
  public static final double driveKA = 0;

  public static final double driveKP = 0.05;
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKF = 0.0;

  public static final double chasisLength = Units.inchesToMeters(28);
  public static final double chasisWidth = Units.inchesToMeters(28);
  public static final double offsetToSwerveModule = chasisLength / 2 - Units.inchesToMeters(3.25);

  public static final double wheelBase = 0.53; // TODO: This must be tuned to specific robot

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
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public double kNorm;

    public SwerveModuleConfigurations(int moduleNumber, int angleEncoderId, int angleMotorId, int driveMotorId,
        double angleEncoderOffset, double kP, double kI, double kD, double kF, double kNorm) {
      this.moduleNumber = moduleNumber;
      this.angleEncoderId = angleEncoderId;
      this.angleMotorId = angleMotorId;
      this.driveMotorId = driveMotorId;
      this.angleEncoderOffset = angleEncoderOffset;
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kF = kF;
      this.kNorm = kNorm;
    }
  }

  public static final SwerveModuleConfigurations s_frontLeft = new SwerveModuleConfigurations(
      0,
      2,
      5,
      4,
      140.28d,
      0.150d,
      0.092d,
      0.001d,
      0.0,
      12);

  public static final SwerveModuleConfigurations s_frontRight = new SwerveModuleConfigurations(
      1,
      0,
      1,
      0,
      157.40d,
      0.150d,
      0.092d,
      0.001d,
      0.0,
      12);

  public static final SwerveModuleConfigurations s_backLeft = new SwerveModuleConfigurations(
      2,
      1,
      3,
      6,
      154.42d,
      0.150d,
      0.092d,
      0.001d,
      0.0,
      12);

  public static final SwerveModuleConfigurations s_backRight = new SwerveModuleConfigurations(
      3,
      3,
      7,
      2,
      104.42d,
      0.15d,
      0.092d,
      0.001d,
      0.0,
      12);

}