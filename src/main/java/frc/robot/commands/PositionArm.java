package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.util.math.Utils;

public class PositionArm extends CommandBase {
	Arm arm;
	double targetPivot;
	double targetExtension;

	public PositionArm(Arm arm, double pivot, double extension) {
		this.arm = arm;
		this.targetPivot = pivot;
		this.targetExtension = extension;
	}

	@Override
	public void execute() {
		// if the pivot is constrained by the length
		if (!arm.isPossible(targetPivot, arm.getArmLength())) {
			// if the desired arm position is impossible given the current arm extension
			// then immediately draw the arm back as fast as possible while optimizing the
			// pivot heading to the minimum value possible given the instantaneous extension
			double maxHeight = arm.getHeightLimit();
			double pivot = Math.toDegrees(Math.acos(maxHeight / arm.getArmLength()));

			// since the placing the arm inside the robot requires a different height
			// then having it off the ground, ensure that minimizing the pivot angle
			// won't crash into the chassis, and if it does, re-optimize the pivot
			// given the chassis's height
			double finalMaxHeight = arm.getHeightLimitAt(pivot);
			if (finalMaxHeight != maxHeight) {
				pivot = Math.toDegrees(Math.acos(finalMaxHeight / arm.getArmLength()));
			}

			arm.setPivotDegrees(pivot);
			arm.setExtensionLength(targetExtension);
		}
		// if the length is constrained by the pivot
		else if (!arm.isPossible(arm.getPivotDegrees(), targetExtension)) {
			// if the desired arm position is impossible given the current pivot heading
			// then immediately rotate the arm out as fast as possible while optimizing the
			// extension length to the maximum value possible given the instantaneous pivot
			double maxExtension = arm.getHeightLimit() / Math.cos(Math.toRadians(arm.getPivotDegrees()))
					- Constants.Arm.baseLength;

			arm.setExtensionLength(maxExtension);
			arm.setPivotDegrees(targetPivot);
		}
		// if neither mechanism is constrained
		else {
			arm.setExtensionLength(targetExtension);
			arm.setPivotDegrees(targetPivot);
		}
	}

	@Override
	public void initialize() {
		if (!arm.isPossible(targetPivot, targetExtension)) {
			DriverStation.reportError("arm position isn't allowed!", null);
			end(true);
		}
	}

	@Override
	public void end(boolean wasInterrupted) {
	}

	@Override
	public boolean isFinished() {
		return Utils.withinDeadband(arm.getPivotDegrees(), targetPivot, 2.50d)
				&& Utils.withinDeadband(arm.getExtensionLength(), targetExtension, 0.05);
	}
}
