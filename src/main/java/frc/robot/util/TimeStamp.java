package frc.robot.util;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;

public class TimeStamp {
	double previousTime;
	double delta;

	public TimeStamp() {
		this.previousTime = Timer.getFPGATimestamp();
		this.delta = 0.0d;
	}

	public void update() {
		double currentTime = Timer.getFPGATimestamp();
		delta = currentTime - previousTime;
		previousTime = currentTime;
	}

	public double deltaTime() {
		return delta;
	}
}
