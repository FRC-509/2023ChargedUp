package frc.robot.util.math;

public final class Utils {
	public static boolean withinDeadband(double value, double target, double deadband) {
		return Math.abs(target - value) <= deadband;
	}

	public static double pow(double x, double p) {
		return Math.signum(x) * Math.pow(Math.abs(x), p);
	}
}
