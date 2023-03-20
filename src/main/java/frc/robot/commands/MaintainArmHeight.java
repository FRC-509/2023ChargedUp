package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.util.math.PositionTarget;

public class MaintainArmHeight extends CommandBase {
	Arm arm;
	double height;
	DoubleSupplier rotation;
	BooleanSupplier end;
	PositionTarget pivotTarget;

	public MaintainArmHeight(Arm arm, DoubleSupplier rotation, BooleanSupplier end) {
		this.arm = arm;
		this.height = arm.getHeight();
		this.rotation = rotation;
		this.end = end;

		double minTheta = Math.toDegrees(Math.acos(height / arm.getMinArmLength()));
		double maxTheta = Math.toDegrees(Math.acos(height / arm.getMaxArmLength()));
		this.pivotTarget = new PositionTarget(arm.getPivotDegrees(), minTheta, maxTheta);
	}

	@Override
	public void execute() {
		// * move pivot slowly so the extension can easily keep up -- tune CAREFULLY!!!

		pivotTarget.update(rotation.getAsDouble(), 1.0);
		arm.setPivotDegrees(pivotTarget.getTarget());

		double extension = height / Math.cos(Math.toRadians(arm.getPivotDegrees())) - Constants.Arm.baseLength;
		arm.setExtensionLength(extension);
	}

	@Override
	public void initialize() {
		if (arm.isInChassis()) {
			end(true);
		}
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		return super.isFinished();
	}
}
