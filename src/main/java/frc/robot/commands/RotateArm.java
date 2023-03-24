package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RotateArm extends CommandBase {
	private Arm arm;
	private double targetAngleDegrees;

	public RotateArm(Arm arm, double target) {
		this.arm = arm;
		this.targetAngleDegrees = target;
	}

	@Override
	public void initialize() {
		if (!arm.isValidState(targetAngleDegrees, arm.getExtensionLength())) {
			end(true);
		} else {
			arm.setPivotDegrees(targetAngleDegrees);
		}
	}

	@Override
	public boolean isFinished() {
		return Math.abs(arm.getPivotDegrees() - targetAngleDegrees) <= 2.5;
	}
}
