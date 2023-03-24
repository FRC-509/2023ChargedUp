package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.RotateArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneCone extends SequentialCommandGroup {
	public OneCone(Arm arm, Claw claw, Swerve swerve) {
		addCommands(
				new RotateArm(arm, 110),
				new ExtendArm(arm, 255),
				new InstantCommand(() -> claw.retractClaw(), claw),
				new WaitCommand(0.2),
				new ExtendArm(arm, 0));
	}
}
