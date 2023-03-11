package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import frc.robot.RobotContainer;

public class OneCone extends SequentialCommandGroup {
	public OneCone(Arm arm, Claw claw, Swerve swerve) {
		addCommands(
				new InstantCommand(() -> claw.toggleClaw(), claw),
				new FunctionalCommand(() -> {
				}, () -> arm.setPivotDegrees(98),
						(end) -> {
							arm.setPivotOutput(0);
						},
						() -> false, arm).withTimeout(2),
				new FunctionalCommand(() -> {
				}, () -> arm.setExtensionPosition(510),
						(end) -> {
						},
						() -> false, arm).withTimeout(4),
				new InstantCommand(() -> claw.toggleClaw(), claw).withTimeout(1));

		// new DriveCommand(swerve, -1, 0, 0, false));
		// new InstantCommand(() -> (2)),
		// new DriveCommand(swerve, -1.0, 0, 0, true).withTimeout(1));
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
