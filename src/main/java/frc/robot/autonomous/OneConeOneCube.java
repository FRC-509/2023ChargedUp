package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeOneCube extends SequentialCommandGroup {
	public OneConeOneCube(Arm arm, Claw claw, Swerve swerve) {
		// new PathConstraints(Constants.maxSpeed / 3, 3.2 / 2));
		// new PIDConstants(3.0, 0, 0),
		// new PIDConstants(1.3, 0.3, 0.1),
		addCommands(
				new OneCone(arm, claw, swerve),
				RobotContainer.followPath("line", swerve),
				new PickUpCubeFromGround(arm, claw));
	}
}
