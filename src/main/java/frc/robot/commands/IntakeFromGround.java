package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PositionArmTeleop.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class IntakeFromGround extends SequentialCommandGroup {
	Claw claw;
	Arm arm;

	public IntakeFromGround(Arm arm, Claw claw) {
		addCommands(
				new PositionArmTeleop(arm, ArmState.GroudPickup),
				new InstantCommand(() -> {
					claw.closeClaw();
					claw.spinIntake(true);
				}, claw));
	}
}