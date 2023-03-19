package frc.robot.util.math;

public final class Utils {
	public static boolean withinDeadband(double value, double target, double deadband) {
		return Math.abs(target - value) <= deadband;
	}
}
