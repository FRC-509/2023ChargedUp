package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.TimeStamp;
// import edu.wpi.first.math.interpolation.Interpolatable;

public class Interpolator {
	double scale;
	double setPoint;
	double trailingPoint;
	double position;

	TimeStamp timeStamp;

	public Interpolator(TimeStamp timeStamp, double scale) {
		this.scale = scale;
		this.timeStamp = timeStamp;
		this.setPoint = 0.0d;
		this.trailingPoint = 0.0d;
		this.position = 0.0d;
	}

	public void setPoint(double value) {
		setPoint = value;
	}

	public double getPosition() {
		return position;
	}

	public double update() {
		position += scale * (trailingPoint - position) * timeStamp.deltaTime();

		if (Utils.withinDeadband(position, trailingPoint, 0.1 * scale)) {
			trailingPoint = setPoint;
		}

		return position;
	}
}
