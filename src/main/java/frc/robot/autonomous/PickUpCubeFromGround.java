package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class PickUpCubeFromGround extends SequentialCommandGroup {
	public PickUpCubeFromGround(Arm arm, Claw claw) {
		addCommands(
				new InstantCommand(() -> claw.retractClaw(), claw),
				new WaitCommand(0.2),
				new FunctionalCommand(() -> {
				}, () -> arm.setPivotDegrees(38),
						(end) -> {
							arm.setPivotOutput(0);
						},
						() -> false, arm).withTimeout(0.8),
				new FunctionalCommand(() -> {
				}, () -> arm.setExtensionPosition(190),
						(end) -> {
							arm.stopExtensionMotor();
						},
						() -> false, arm).withTimeout(0.8),
				new InstantCommand(() -> claw.spinIntake(true), claw),
				new WaitCommand(0.5),
				new InstantCommand(() -> claw.stopIntake(), claw));
	}
}
