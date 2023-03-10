package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class OneCone extends SequentialCommandGroup {
	public OneCone(Arm arm, Claw claw) {
		addCommands(
				new InstantCommand(() -> claw.toggleClaw(), claw),
				new FunctionalCommand(() -> {
				}, () -> arm.setPivotDegrees(100),
						(end) -> {
							arm.setPivotOutput(0);
						},
						() -> false, arm).withTimeout(3),
				new FunctionalCommand(() -> {
				}, () -> arm.setExtensionPosition(509),
						(end) -> {
						},
						() -> false, arm).withTimeout(4),
				new InstantCommand(() -> claw.toggleClaw(), claw)

		);
		// addCommands(
		// new InstantCommand(() -> claw.toggleClaw(), claw),
		// new FunctionalCommand(
		// () -> {
		// ;
		// },
		// () -> arm.setPivotOutput(0.4),
		// () -> false,
		// arm).withTimeout(2),
		// new InstantCommand(() -> claw.toggleClaw(), claw));
		// addRequirements(arm, claw);
	}
}
