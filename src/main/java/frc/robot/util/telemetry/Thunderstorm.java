package frc.robot.util.telemetry;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Thunderstorm {
	private final NetworkTable table;

	public Thunderstorm() {
		table = NetworkTableInstance.getDefault().getTable("Thunderstorm");
	}

	public void update(SwerveModuleState[] states) {
		for (int number = 0; number < states.length; number++) {
			table.getEntry("Module" + number + "Velocity").setDouble(states[number].speedMetersPerSecond);
			table.getEntry("Module" + number + "Angle").setDouble(states[number].angle.getDegrees());
		}
	}
}
