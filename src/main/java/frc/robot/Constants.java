package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDWrapper;

public final class Constants {
	public static class PID {
		public static final PIDWrapper extension_P = new PIDWrapper(0.1d, 0.001d, 0.0d, 0.0d);
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
		public static double maxExtensionSpeed = 175.0d;
		public static final double maxExtensionLength = 250.0d;
		public static final double maxExtension = 240.0d;
		public static final double minExtension = 5.0d;
		public static final double maxPivot = 110.0d;
		public static final double minPivot = 20.0d;
		public static final double clawLength = 32.0d;
		public static final double offsetToPivot = 5.0d;

		public static double baseLength = 68.5;

		// degrees per second
		public static double maxPivotSpeed = 250.0d;
		public static final double minHeight = 15.0d;
		public static final double pivotHeight = 120.0d;
		public static final double offsetToBase = 75.0d;
	}

	public static class Chassis {
		public static final double length = Units.inchesToMeters(28);
		public static final double width = Units.inchesToMeters(28);

		public static final double height = 22.0d; // cm
	}

	public static class Vision {
		public static final double highConeTargetAngle = 6.5d;
		public static final double substationTargetAngle = 7.9d;
		public static final double midConeTargetAngle = 11.8d;
	}

	public static final double rotationScale = 12.0d;

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

	public static final double minHeadingCorrectionSpeed = 0.15;

	public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI; // 0.3192 meters
	public static final double driveGearRatio = 6.75; // exact value is 425.0d / 63.0d
	public static final double angleGearRatio = 12.8;
	public static final double maxSpeed = 4.96824;
	public static final double maxAngularVelocity = maxSpeed / (Math.hypot(offsetToSwerveModule, offsetToSwerveModule));

	public static final double kS = 0.079354 / 12.0;
	public static final double kV = 2.3277 / 12.0;
	public static final double kA = 0.26532 / 12.0;

	// kf was 0.01
	public static final PIDWrapper drive = new PIDWrapper(0.01, 0.000425, 0.0, 0.00);
	public static final PIDWrapper steer = new PIDWrapper(0.2, 0, 0, 0);

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
			-308.84,
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