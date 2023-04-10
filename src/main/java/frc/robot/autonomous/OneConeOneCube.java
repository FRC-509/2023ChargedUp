package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.PlaceCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeOneCube extends SequentialCommandGroup {
	public OneConeOneCube(Arm arm, Claw claw, Swerve swerve) {
		addCommands(
				new OneCone(arm, claw, swerve),
				RobotContainer.followPath("s1DriveToCube", swerve, true, 0, 0.5),
				new PickUpCubeFromGround(arm, claw),
				RobotContainer.followPath("s1DriveToCubeNode", swerve, false, 180, 0.3),
				new PlaceCube(arm),
				new InstantCommand(() -> claw.spinIntake(true), claw),
				new WaitCommand(0.1),
				new InstantCommand(() -> claw.stopIntake(), claw));
	}
}
