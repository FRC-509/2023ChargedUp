package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class OneConeTeleopMid extends SequentialCommandGroup {
	public OneConeTeleopMid(Arm arm) {
		addCommands(
				new RotateArm(arm, 90),
				new ExtendArm(arm, 115));
	}
}
