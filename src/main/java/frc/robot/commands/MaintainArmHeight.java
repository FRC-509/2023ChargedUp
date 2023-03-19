package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.util.math.Utils;

public class MaintainArmHeight extends CommandBase {
	Arm arm;
	double height;
	DoubleSupplier rotation;
	BooleanSupplier end;

	public MaintainArmHeight(Arm arm, DoubleSupplier rotation, BooleanSupplier end) {
		this.arm = arm;
		this.rotation = rotation;
		this.end = end;
	}

	@Override
	public void initialize() {
		double armHeight = arm.getArmLength() * Math.cos(Math.toRadians(arm.getPivotDegrees()));
	}

	@Override
	public boolean isFinished() {
		return super.isFinished();
	}

	@Override
	public void execute() {
		super.execute();
	}
}
