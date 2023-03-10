package frc.robot.util.controllers;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.TimeStamp;
import frc.robot.util.Interpolator;

public class InterpolatedInput {
	DoubleSupplier input;
	Interpolator interpolator;
	TimeStamp timeStamp;

	public InterpolatedInput(DoubleSupplier input) {
		this.input = input;
		this.timeStamp = new TimeStamp();
		this.interpolator = new Interpolator(timeStamp, 1);
	}
}