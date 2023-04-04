package frc.robot.util;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;

public class RobotWrapper extends Robot {
	private static List<Subsystem> subsystems;
	private static boolean hasInitialized;

	public static void addSubsystem(Subsystem subsystem) {
		subsystems.add(subsystem);
	}

	public static List<Subsystem> getSubsystems() {
		return subsystems;
	}

	public static boolean isHasInitialized() {
		return hasInitialized;
	}
}
