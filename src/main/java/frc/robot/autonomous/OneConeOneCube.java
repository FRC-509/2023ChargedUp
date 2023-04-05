package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeOneCube extends SequentialCommandGroup {
	public OneConeOneCube(Arm arm, Claw claw, Swerve swerve) {
		addCommands(
				// new OneCone(arm, claw, swerve),
				RobotContainer.followPath("line", swerve, true, 360.0, 0.1),
				new InstantCommand(() -> {
					System.out.println("PATH IS DONE");
				})
		// new PickUpCubeFromGround(arm, claw)
		/* RobotContainer.followPath("lineCopy", swerve, true, 0.0) */ );
	}
}
