package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class PositionTarget {
	double target = 0.0d;
	double rate = 1.0d;
	double min = -Double.MAX_VALUE;
	double max = +Double.MAX_VALUE;

	double previousTimeStamp = 0.0;

	public PositionTarget() {
		previousTimeStamp = Timer.getFPGATimestamp();
	}

	public PositionTarget(double min, double max, double weight) {
		previousTimeStamp = Timer.getFPGATimestamp();
		this.rate = weight;
		this.min = min;
		this.max = max;
	}

	public double setPosition(double position) {
		target = MathUtil.clamp(position, min, max);
		return target;
	}

	public void setRate(double rate) {
		this.rate = rate;
	}

	public double update(double percent) {
		percent = MathUtil.clamp(percent, -1.0d, 1.0d);

		double deltaTime = Timer.getFPGATimestamp() - previousTimeStamp;
		target += rate * percent * deltaTime;
		previousTimeStamp = Timer.getFPGATimestamp();

		target = MathUtil.clamp(target, min, max);

		return target;
	}

	public double getTarget() {
		return target;
	}
}
