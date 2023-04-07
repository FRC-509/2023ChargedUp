package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ExtendArm;
import frc.robot.commands.PositionArmTeleop;
import frc.robot.commands.RotateArm;
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
				// Drive to the cube and pivot the arm into the ground intake position
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								new ExtendArm(arm, 0),
								// new WaitCommand(0.1),
								new RotateArm(arm, ArmState.GroudPickup.getPivot())),
						RobotContainer.followPath("lineNoAdjust", swerve, true, 0, 0.5)),
				// Extend the arm into the ground intake position
				new ExtendArm(arm, ArmState.GroudPickup.getExtension()),
				// Stop intaking and start driving back into the community
				new InstantCommand(() -> claw.stopIntake(), claw),
				new ParallelCommandGroup(
						new SequentialCommandGroup(
								new WaitCommand(0.5),
								new PositionArmTeleop(arm, ArmState.CubeMid)),
						RobotContainer.followPath("reversedLineHighGoal", swerve, false, 180, 0.5)),
				// Now place the cube!
				new InstantCommand(() -> claw.spinIntake(true), claw),
				new WaitCommand(0.1),
				new InstantCommand(() -> claw.stopIntake(), claw));
	}
}
