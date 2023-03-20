package frc.robot.util.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class PositionTarget {
	double target = 0.0d;
	double min = -Double.MAX_VALUE;
	double max = +Double.MAX_VALUE;

	double previousTimeStamp = 0.0;

	public PositionTarget() {
		previousTimeStamp = Timer.getFPGATimestamp();
	}

	public PositionTarget(double start, double min, double max) {
		previousTimeStamp = Timer.getFPGATimestamp();
		this.target = start;
		this.min = min;
		this.max = max;
	}

	public void setMin(double min) {
		this.min = min;
	}

	public void setMax(double max) {
		this.max = max;
	}

	public double setPosition(double position) {
		target = MathUtil.clamp(position, min, max);
		return target;
	}

	public double update(double percent, double rate) {
		percent = MathUtil.clamp(percent, -1.0d, 1.0d);

		double deltaTime = Timer.getFPGATimestamp() - previousTimeStamp;
		target += rate * percent * deltaTime;
		previousTimeStamp = Timer.getFPGATimestamp();

		target = MathUtil.clamp(target, min, max);

		return target;
	}

	public double update(double percent) {
		percent = MathUtil.clamp(percent, -1.0d, 1.0d);

		double deltaTime = Timer.getFPGATimestamp() - previousTimeStamp;
		target += percent * deltaTime;
		previousTimeStamp = Timer.getFPGATimestamp();

		target = MathUtil.clamp(target, min, max);

		return target;
	}

	public double getTarget() {
		return target;
	}
}
