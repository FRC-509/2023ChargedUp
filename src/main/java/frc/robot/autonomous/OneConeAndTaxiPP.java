package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeAndTaxiPP extends SequentialCommandGroup {
	public OneConeAndTaxiPP(Arm arm, Claw claw, Swerve swerve) {
		addCommands(
				new OneCone(arm, claw, swerve));
	}
}
