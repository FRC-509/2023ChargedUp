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

public class ResetArm extends SequentialCommandGroup {
	public ResetArm(Arm arm) {
		addCommands(
				new ExtendArm(arm, 0),
				new RotateArm(arm, 10));
	}
}
