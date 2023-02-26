package frc.robot.util.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

// Implemented by team 254: https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/lib/drivers/LazyTalonFX.java
/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 * overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
	protected double lastSet = Double.NaN;
	protected ControlMode lastControlMode = null;

	public LazyTalonFX(int deviceId, String canBus) {
		super(deviceId, canBus);
	}

	public LazyTalonFX(int deviceId) {
		super(deviceId, "rio");
	}

	public double getLastSet() {
		return lastSet;
	}

	@Override
	public void set(ControlMode mode, double value) {
		if (value != lastSet || mode != lastControlMode) {
			lastSet = value;
			lastControlMode = mode;
			super.set(mode, value);
		}
	}
}