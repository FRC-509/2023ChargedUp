package frc.robot.util.telemetry;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;

public class Thunderstorm {
	private final NetworkTable table;

	public Thunderstorm() {
		table = NetworkTableInstance.getDefault().getTable("Thunderstorm");
	}

	public void update(RobotContainer container) {
		SwerveModuleState[] states = container.swerveSubsystem.getModuleStates();
		for (int number = 0; number < states.length; number++) {
			table.getEntry("Module" + number + "Velocity").setDouble(states[number].speedMetersPerSecond);
			table.getEntry("Module" + number + "Angle").setDouble(states[number].angle.getDegrees());
		}
		table.getEntry("ArmPivot").setDouble(container.armSubsystem.getPivotDegrees());
		table.getEntry("ArmExtension").setDouble(container.armSubsystem.getExtensionPosition());
	}
}
