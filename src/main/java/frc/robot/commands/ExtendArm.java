package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ExtendArm extends CommandBase {
	private Arm arm;
	private double targetPosition;

	public ExtendArm(Arm arm, double target) {
		this.arm = arm;
		this.targetPosition = target;
	}

	@Override
	public void initialize() {
		if (!arm.isValidState(arm.getPivotDegrees(), arm.extensionTicksToLength(targetPosition))) {
			end(true);
		} else {
			arm.setExtensionPosition(targetPosition);
		}
	}

	@Override
	public boolean isFinished() {
		return Math.abs(arm.getExtensionPosition() - targetPosition) <= 1.5;
	}
}
