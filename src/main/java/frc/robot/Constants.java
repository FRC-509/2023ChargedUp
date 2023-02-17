package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Util.SwerveModuleConfig;

public class Constants {
  public static String Canivore = "509CANIvore";

  public static final class Chassis {
    public static final double length = Units.inchesToMeters(28);
    public static final double width = Units.inchesToMeters(28);
    public static final double swerveModuleOffset = length / 2 - Units.inchesToMeters(3.25);
  }

  public static final class DeviceId {
    // swerve module angle encoders
    public static final int frontLeftCANCoder = 0;
    public static final int backLeftCANCoder = 1;
    public static final int backRightCANCoder = 2;
    public static final int frontRightCANCoder = 3;

    // swerve module drive motors
    public static final int frontLeftDriveMotor = 4;
    public static final int backLeftDriveMotor = 5;
    public static final int backRightDriveMotor = 6;
    public static final int frontRightDriveMotor = 7;

    // swerve module steer motors
    public static final int frontLeftSteerMotor = 8;
    public static final int backLeftSteerMotor = 9;
    public static final int backRightSteerMotor = 10;
    public static final int frontRightSteerMotor = 11;

    // gyro
    public static final int gyro = -1;

    // arm motors
    public static final int extensionMotor = 12;
    public static final int leftPivotMotor = 12;
    public static final int rightPivotMotor = 13;

    // intake motors
    public static final int primaryIntakeMotor = 14;
    public static final int secondaryIntakeMotor = 15;

    // rev hubs
    public static final int pneumaticHub = 19;

    // pneumatics
    public static final int intakeForwardChannel = 5;
    public static final int intakeReverseChannel = 7;
  }

  public static final class SwerveConfig {
    public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;
    public static final double driveGearRatio = 6.75;
    public static final double angleGearRatio = 12.8;
    public static final double maxSpeed = 5.0d;
    public static final double maxAngularSpeed = maxSpeed / (Math.sqrt(2.0d) * Chassis.swerveModuleOffset);
    public static final boolean driveClosedLoopVelocity = false;

    public static final SwerveModuleConfig[] modules = new SwerveModuleConfig[] {
        new Util.SwerveModuleConfig(
            "front-left",
            0,
            DeviceId.frontLeftCANCoder,
            DeviceId.frontLeftDriveMotor,
            DeviceId.frontLeftSteerMotor,
            -315.09,
            +Chassis.swerveModuleOffset,
            +Chassis.swerveModuleOffset),

        new Util.SwerveModuleConfig(
            "back-left",
            1,
            DeviceId.backLeftCANCoder,
            DeviceId.backLeftDriveMotor,
            DeviceId.backLeftSteerMotor,
            -299.18,
            -Chassis.swerveModuleOffset,
            +Chassis.swerveModuleOffset),

        new Util.SwerveModuleConfig(
            "back-right",
            2,
            DeviceId.backRightCANCoder,
            DeviceId.backRightDriveMotor,
            DeviceId.backRightSteerMotor,
            -299.18,
            -Chassis.swerveModuleOffset,
            -Chassis.swerveModuleOffset),

        new Util.SwerveModuleConfig(
            "front-right",
            3,
            DeviceId.frontRightCANCoder,
            DeviceId.frontRightDriveMotor,
            DeviceId.frontRightSteerMotor,
            -299.18,
            +Chassis.swerveModuleOffset,
            -Chassis.swerveModuleOffset)
    };
  }

  public static final Util.TalonFxConfig leftPivotMotor = new Util.TalonFxConfig(
      DeviceId.leftPivotMotor,
      Canivore,
      null,
      NeutralMode.Brake,
      false);

  public static final Util.TalonFxConfig rightPivotMotor = new Util.TalonFxConfig(
      DeviceId.rightPivotMotor,
      Canivore,
      null,
      NeutralMode.Brake,
      true);

  public static final Util.SparkConfig primaryIntakeMotor = new Util.SparkConfig(
      DeviceId.primaryIntakeMotor,
      MotorType.kBrushless,
      IdleMode.kCoast,
      false);

  public static final Util.SparkConfig secondaryIntakeMotor = new Util.SparkConfig(
      DeviceId.secondaryIntakeMotor,
      MotorType.kBrushless,
      IdleMode.kCoast,
      true);

  public static final Util.SparkConfig extensionMotor = new Util.SparkConfig(
      DeviceId.extensionMotor,
      MotorType.kBrushless,
      IdleMode.kBrake,
      false);

  public static final Util.SolenoidConfig intakeSolenoid = new Util.SolenoidConfig(
      DeviceId.pneumaticHub,
      PneumaticsModuleType.CTREPCM,
      DeviceId.intakeForwardChannel,
      DeviceId.intakeReverseChannel);

  public static final double intakeSpinVelocity = 0.0d;
  public static final double pivotGearRatio = 20.0d;

}
