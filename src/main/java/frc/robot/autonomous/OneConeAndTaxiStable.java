package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeAndTaxiStable extends SequentialCommandGroup {
	public OneConeAndTaxiStable(Arm arm, Claw claw, Swerve swerve) {
		addCommands(
				new OneCone(arm, claw, swerve),
				new DriveCommand(swerve, -0.5, 0, 0, false).withTimeout(2));
	}
}
