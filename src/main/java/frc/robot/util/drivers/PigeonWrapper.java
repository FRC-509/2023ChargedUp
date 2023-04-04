package frc.robot.util.drivers;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation3d;

public class PigeonWrapper extends Pigeon2 {
	private final double initialOffset;
	private double yawOffset = 0.0d;

	public PigeonWrapper(int deviceNumber, String canbus, double initialOffset) {
		super(deviceNumber, canbus);
		this.initialOffset = initialOffset;
	}

	/// Sets the offset to the negated current yaw
	public void resetYaw() {
		yawOffset = -(super.getYaw() + initialOffset);
	}

	/// Returns the robot yaw shifted by the offset
	public double getRelativeYaw() {
		return (super.getYaw() + yawOffset + initialOffset) % 360.0d;
	}

	/// Returns the absolute robot yaw
	public double getAbsoluteYaw() {
		return (super.getYaw() + initialOffset) % 360.0d;
	}

	public void setRelativeYaw(double yaw) {
		yawOffset = yaw - (super.getYaw() + initialOffset);
	}

	public double getAbsoluteZero() {
		return initialOffset;
	}

	/**
	 * @return Acceleration vector in m/s^2
	 */
	public Translation3d getAccelerometerData() {
		short[] xyz = new short[3];
		getBiasedAccelerometer(xyz);
		return new Translation3d(xyz[0] / 16384d * 9.81d, xyz[1] / 16384d * 9.81d, xyz[2] / 16384d * 9.81d);
	}
}
