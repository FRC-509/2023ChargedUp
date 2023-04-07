package frc.robot.commands;

import javax.print.event.PrintEvent;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class PositionArmTeleop extends SequentialCommandGroup {
	public enum ArmState {
		Substation,
		ConeHigh,
		ConeMid,
		CubeHigh,
		CubeMid,
		GroudPickup,
		Home;

		public double getPivot() {
			switch (this) {
				case CubeHigh:
				case ConeHigh:
					return 100.0d;
				case CubeMid:
				case ConeMid:
					return 82.0d;
				case Substation:
					return 86.5d;
				case Home:
					return 30.0d;
				case GroudPickup:
					return 41.0d;
				default:
					return 0.0d;
			}
		}

		public double getExtension() {
			switch (this) {
				case ConeHigh:
					return 240.0d;
				case CubeHigh:
					return 100.0d;
				case ConeMid:
					return 75.0d;
				case CubeMid:
					return 0.0d;
				case Substation:
					return 10.0d;
				case Home:
					return 0.0d;
				case GroudPickup:
					return 152.0d;
				default:
					return 0.0d;
			}
		}
	}

	public PositionArmTeleop(Arm arm, ArmState state) {
		if (!arm.isValidState(state.getPivot(),
				arm.getArmLength())) {
			addCommands(
					new ExtendArm(arm, state.getExtension()),
					new RotateArm(arm, state.getPivot()));
		} else {
			addCommands(
					new RotateArm(arm, state.getPivot()),
					new ExtendArm(arm, state.getExtension()));
		}
	}
}
