package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PIDConstants;
import frc.robot.util.controllers.JoystickController;
import frc.robot.util.controllers.LogitechController;

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
	public static class PID {
		public static final PIDController extension = new PIDController(0.1, 0.0, 0.0);
	}

	public static class Controller {
		public static final JoystickController leftStick = new JoystickController(1);
		public static final JoystickController rightStick = new JoystickController(0);
		public static final LogitechController controller = new LogitechController(2);
	}

	public static final String RoboRio = "rio";

	public static final double Voltage = 12.0d;

	public static final String CANIvore = "509CANIvore";

	public static final int revBlinkinPort = 9;
	public static final int ledPixelCount = 25;

	// Vision-related constants.
	public static final int redAllianceSubstationTagID = 5;
	public static final int blueAllianceSubstationTagID = 4;
	public static final int[] redAllianceTargetTagIDs = { 6, 7, 8 };
	public static final int[] blueAllianceTargetTagIDs = { 3, 2, 1 };
	public static final String limelightName = "limelight";

	// Control-related constants.
	public static final double stickDeadband = 0.1;
	public static final double armPivotOperatorCoefficient = 0.2;
	public static final double armExtensionOperatorCoefficient = 1.0;

	// Claw-related constants.
	public static final double intakePercentVel = 1.0d;

	// Arm-related constants.
	public static final double pivotGearRatio = 227.556;
	public static final double extensionGearRatio = 64.0d;
	public static final double maxExtension = 520.0;

	// Drivetrain-related constants.
	public static final double safetyBuffer = Units.inchesToMeters(40);
	public static final double chassisLength = Units.inchesToMeters(28);
	public static final double chassisWidth = Units.inchesToMeters(28);
	public static final double offsetToSwerveModule = chassisLength / 2 - Units.inchesToMeters(3.25);

	public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI; // 0.3192 meters
	public static final double driveGearRatio = 6.75; // exact value is 425.0d / 63.0d
	public static final double angleGearRatio = 12.8;
	public static final double maxSpeed = 4.96824;
	public static final double maxAngularVelocity = maxSpeed / (Math.hypot(offsetToSwerveModule, offsetToSwerveModule));

	public static final double kS = 0.079354 / 12.0;
	public static final double kV = 2.3277 / 12.0;
	public static final double kA = 0.26532 / 12.0;

	public static final PIDConstants drive = new PIDConstants(0.1, 0, 0, 0);
	public static final PIDConstants steer = new PIDConstants(0.2, 0, 0, 0);
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
		public PIDConstants steerPID;
		public PIDConstants drivePID;

		public SwerveModuleConfigurations(int moduleNumber, int angleEncoderId, int angleMotorId, int driveMotorId,
				double angleEncoderOffset, PIDConstants steerPID, PIDConstants drivePID) {
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