package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class OneConeTeleopHigh extends SequentialCommandGroup {
	public OneConeTeleopHigh(Arm arm, boolean extend) {
		if (extend) {
			addCommands(
					new RotateArm(arm, 100),
					new ExtendArm(arm, 250));
		} else {
			addCommands(
					new RotateArm(arm, 100));
		}
	}
}
