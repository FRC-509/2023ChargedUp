package frc.robot.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import frc.robot.util.DeviceBuilder.FalconBuilder;
import frc.robot.util.DeviceBuilder.NeoBuilder;

public class Device {
	public static class Swerve {
		public static class Module {
			public static int frontLeft = 0;
			public static int backLeft = 1;
			public static int backRight = 2;
			public static int frontRight = 3;
		}

		public static class AngleEncoder {
			public static int frontLeft = 0;
			public static int backLeft = 1;
			public static int backRight = 2;
			public static int frontRight = 3;
		}

		public static class AngleMotor {
			public static int frontLeft = 8;
			public static int backLeft = 9;
			public static int backRight = 10;
			public static int frontRight = 11;
		}

		public static class DriveMotor {
			public static int frontLeft = 4;
			public static int backLeft = 5;
			public static int backRight = 6;
			public static int frontRight = 7;
		}

		public static class EncoderOffset {
			public static double frontLeft = -315.09;
			public static double backLeft = -309.28;
			public static double backRight = -156.44;
			public static double frontRight = -299.18;
		}
	}

	public static class MotorId {
		public static int rightPivot = 20;
		public static int leftPivot = 20;
		public static int extension = 12;
		public static int intake = 14;
	}

	public static class SolenoidId {
		public static int claw = 0;
		public static int clawForward = 0;
		public static int clawReverse = 0;
	}

	public static class Motor {
		public static FalconBuilder rightPivot = new FalconBuilder(
				20,
				Constants.RoboRio,
				new PIDConstants(0.03, 0, 0, 0),
				NeutralMode.Brake,
				false);

		public static FalconBuilder leftPivot = new FalconBuilder(
				13,
				Constants.RoboRio,
				new PIDConstants(0.03, 0, 0, 0),
				NeutralMode.Brake,
				true);

		public static NeoBuilder extension = new NeoBuilder(
				12,
				IdleMode.kBrake,
				false);
	}
}