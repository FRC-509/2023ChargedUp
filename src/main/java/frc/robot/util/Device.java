package frc.robot.util;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.util.DeviceBuilder.CANCoderBuilder;
import frc.robot.util.DeviceBuilder.FalconBuilder;
import frc.robot.util.DeviceBuilder.NeoBuilder;

public class Device {
	public static final String CanBus = "509CANIvore";
	public static final String RioBus = "rio";
	public static final String limelightName = "limelight";

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

	public static class EncoderId {
		public static int pivot = 40;
		public static int extension = 50;
	}

	public static class SolenoidId {
		public static int claw = 0;
		public static int clawForward = 0;
		public static int clawReverse = 0;
	}

	public static class Encoder {
		public static CANCoderBuilder pivot = new CANCoderBuilder(
				EncoderId.pivot,
				RioBus,
				-235.458529d,
				true);
	}

	public static class Motor {
		public static FalconBuilder rightPivot = new FalconBuilder(
				20,
				RioBus,
				new PIDWrapper(0.05, 0, 0.0005, 0),
				NeutralMode.Brake,
				false, FeedbackDevice.IntegratedSensor,
				1.0,
				40);

		public static FalconBuilder leftPivot = new FalconBuilder(
				13,
				RioBus,
				new PIDWrapper(0.05, 0, 0.0005, 0),
				NeutralMode.Brake,
				true, FeedbackDevice.IntegratedSensor,
				1.0,
				40);

		public static NeoBuilder extension = new NeoBuilder(
				12,
				IdleMode.kBrake,
				false);
	}
}
