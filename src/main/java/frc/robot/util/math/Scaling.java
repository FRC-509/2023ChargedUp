package frc.robot.util.math;

public final class Scaling {
	public static double pow(double x, double p) {
		return Math.signum(x) * Math.pow(Math.abs(x), p);
	}
}
