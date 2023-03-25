//package frc.robot.autonomous;
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.RotateArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeTeleopHigh extends SequentialCommandGroup {
	public OneConeTeleopHigh(Arm arm) {
		addCommands(
				new RotateArm(arm, 107),
				new ExtendArm(arm, 255));
	}
}
