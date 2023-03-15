package frc.robot.util.drivers;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation3d;

public class PigeonWrapper extends Pigeon2 {
	private double yawOffset = 0.0d;

	public PigeonWrapper(int deviceNumber, String canbus) {
		super(deviceNumber, canbus);
	}

	/// Sets the offset to the negated current yaw
	public void resetYaw() {
		yawOffset = -super.getYaw();
	}

	/// Returns the robot yaw shifted by the offset
	public double getRelativeYaw() {
		return super.getYaw() + yawOffset;
	}

	/// Returns the absolute robot yaw
	public double getAbsoluteYaw() {
		return super.getYaw();
	}

	/**
	 * @return Acceleration vector in m/s^2
	 */
	public static Translation3d getAccelerometerData(Pigeon2 gyro) {
		short[] xyz = new short[3];
		gyro.getBiasedAccelerometer(xyz);
		return new Translation3d(xyz[0] / 16384d * 9.81d, xyz[1] / 16384d * 9.81d, xyz[2] / 16384d * 9.81d);
	}
}
