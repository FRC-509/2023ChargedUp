package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDWrapper;

public final class Constants {
	public static class PID {
		public static final PIDWrapper extension_P = new PIDWrapper(0.5d, 0.0d, 0.0d, 0.0d);
	}

	public static class FieldData {
		public enum FieldHeight {
			ConeHigh,
			ConeLow,
			CubeHigh,
			CubeLow,
			Ground,
			HumanStation;

			public double height() {
				switch (this) {
					case ConeHigh:
					case ConeLow:
					case CubeHigh:
					case CubeLow:
					case Ground:
					case HumanStation:
					default:
						return 0.0;
				}
			}
		}
	}

	public static class Arm {
		public static final double maxExtension = 245.0;

		// TODO: populate with real data!!!
		public static double baseLength = -1.0d;

		// units per second
		public static double maxExtensionSpeed = 1.0d;
		// meters per second
		public static double maxMPSExtensionSpeed = 1.0d;
		// degrees per second
		public static double maxPivotSpeed = -1.0d;
		public static final double maxExtensionLength = -1.0d;
		public static final double minExtensionLength = -1.0d;
		public static final double minHeight = 0.02;
		public static final double pivotHeight = 1.20;
		public static final double offsetToBase = 0.57;
	}

	public static class Chassis {
		public static final double length = Units.inchesToMeters(28);
		public static final double width = Units.inchesToMeters(28);

		// TODO: populate with real data!!!
		public static final double height = -1;
	}

	public static final double Voltage = 12.0d;

	public static final int revBlinkinPort = 9;
	public static final int ledPixelCount = 25;

	// Control-related constants.
	public static final double stickDeadband = 0.1;
	public static final double armPivotOperatorCoefficient = 0.2;
	public static final double armExtensionOperatorCoefficient = 1.0;

	// Claw-related constants.
	public static final double intakePercentVel = 1.0d;

	// Arm-related constants.
	public static final double pivotGearRatio = 227.556;
	public static final double extensionGearRatio = 64.0d;

	// Drivetrain-related constants.

	public static final double offsetToSwerveModule = Chassis.length / 2 - Units.inchesToMeters(3.25);

	public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI; // 0.3192 meters
	public static final double driveGearRatio = 6.75; // exact value is 425.0d / 63.0d
	public static final double angleGearRatio = 12.8;
	public static final double maxSpeed = 4.96824;
	public static final double maxAngularVelocity = maxSpeed / (Math.hypot(offsetToSwerveModule, offsetToSwerveModule));

	public static final double kS = 0.079354 / 12.0;
	public static final double kV = 2.3277 / 12.0;
	public static final double kA = 0.26532 / 12.0;

	public static final PIDWrapper drive = new PIDWrapper(0.012, 0, 0.5, 0.0455);
	public static final PIDWrapper steer = new PIDWrapper(0.2, 0, 0, 0);
	public static final boolean closedLoopDriveVelocity = false;

	/*
	 * The order of each vector corresponds to the index of the swerve module inside
	 * the swerveModules array.
	 * 
	 * module 2 (-, -) |--b--| module 1 (-, +)
	 * | |
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
		public PIDWrapper steerPID;
		public PIDWrapper drivePID;

		public SwerveModuleConfigurations(int moduleNumber, int angleEncoderId, int angleMotorId, int driveMotorId,
				double angleEncoderOffset, PIDWrapper steerPID, PIDWrapper drivePID) {
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