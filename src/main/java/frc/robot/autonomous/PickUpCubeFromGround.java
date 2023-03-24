package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.RotateArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PickUpCubeFromGround extends SequentialCommandGroup {
	public PickUpCubeFromGround(Arm arm, Claw claw) {
		addCommands(
				new InstantCommand(() -> claw.retractClaw(), claw),
				new WaitCommand(0.2),
				new RotateArm(arm, 42),
				new InstantCommand(() -> claw.spinIntake(true), claw),
				new ExtendArm(arm, 120),
				new InstantCommand(() -> claw.stopIntake(), claw),
				new WaitCommand(0.5));
	}
}
