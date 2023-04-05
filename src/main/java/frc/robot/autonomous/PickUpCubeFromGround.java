package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.PositionArm;
import frc.robot.commands.PositionArmTeleop;
import frc.robot.commands.RotateArm;
import frc.robot.commands.PositionArmTeleop.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PickUpCubeFromGround extends SequentialCommandGroup {
	public PickUpCubeFromGround(Arm arm, Claw claw) {
		addCommands(
				new InstantCommand(() -> claw.retractClaw()),
				new ExtendArm(arm, 0),
				new InstantCommand(() -> claw.spinIntake(false), claw),
				new PositionArmTeleop(arm, ArmState.GroudPickup),
				new WaitCommand(0.2),
				new ExtendArm(arm, 20),
				new RotateArm(arm, 50),
				new InstantCommand(() -> claw.stopIntake(), claw));
	}
}
