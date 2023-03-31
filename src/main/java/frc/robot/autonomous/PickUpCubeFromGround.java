package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.PositionArm;
import frc.robot.commands.RotateArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PickUpCubeFromGround extends SequentialCommandGroup {
	public PickUpCubeFromGround(Arm arm, Claw claw) {
		addCommands(
				new PositionArm(arm, 45, Arm.extensionTicksToLength(10.0d)/* 74 */),
				new InstantCommand(() -> claw.retractClaw(), claw),
				new InstantCommand(() -> claw.spinIntake(true), claw));
	}
}
