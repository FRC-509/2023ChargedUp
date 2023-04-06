package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.PlaceCube;
import frc.robot.commands.PositionArmTeleop;
import frc.robot.commands.PositionArmTeleop.ArmState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeOneCubeRipoff extends SequentialCommandGroup {
	public OneConeOneCubeRipoff(Arm arm, Claw claw, Swerve swerve) {
		addCommands(
				// Place Cone
				new PositionArmTeleop(arm, ArmState.ConeHigh),
				// new WaitCommand(0.25),
				new InstantCommand(() -> claw.openClaw(), claw),
				new WaitCommand(0.1),
				new InstantCommand(() -> claw.spinIntake(false), claw),
				// Drive to the cube and move the arm into the ground intake position
				new ParallelCommandGroup(
						new PositionArmTeleop(arm, ArmState.GroudPickup).beforeStarting(new WaitCommand(0.2)),
						RobotContainer.followPath("line", swerve, true, 0, 0.7)),
				// Stop intaking and start driving back into the community
				new InstantCommand(() -> claw.stopIntake(), claw),
				new ParallelCommandGroup(
						new PositionArmTeleop(arm, ArmState.CubeMid),
						RobotContainer.followPath("reversedLine", swerve, false, 180, 0.7)),
				// Now place the cube!
				new InstantCommand(() -> claw.spinIntake(true), claw),
				new WaitCommand(0.1),
				new InstantCommand(() -> claw.stopIntake(), claw));
	}
}
